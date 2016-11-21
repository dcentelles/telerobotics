/*
 * ROVCamera.cpp
 *
 *  Created on: 27 oct. 2016
 *      Author: diego
 */

#include <telerobotics/ROVCamera.h>
#include <dccomms/Checksum.h>
#include <telerobotics/Constants.h>

namespace dcauv {

void defaultLastImageSentCallback(ROVCamera & rovcamera)
{
	//Nothing to do
}

void defaultOrdersReceivedCallback(ROVCamera & rovcamera)
{
	//Nothing to do
}


ROVCamera::ROVCamera(LinkType _linkType):service(this),txservice(this),rxservice(this) {
	// TODO Auto-generated constructor stub
	_SetEndianess();
	stateLength = MAX_IMG_STATE_LENGTH;
	imgTrunkInfoLength = IMG_TRUNK_INFO_SIZE;
	maxImgTrunkLength = MAX_IMG_TRUNK_LENGTH;
	maxPacketLength = MAX_PACKET_LENGTH;

	dlfcrctype = DataLinkFrame::fcsType::crc16;
	device.SetNamespace("camera");
	buffer = new uint8_t[maxPacketLength + MAX_IMG_SIZE];
	currentState = buffer;
	beginImgPtr = currentState + stateLength;
	imgInBuffer = false;
	lastImageSentCallback = &defaultLastImageSentCallback;
	ordersReceivedCallback = &defaultOrdersReceivedCallback;
	device.SetChecksumType(DataLinkFrame::crc16);

	linkType = _linkType;
	if(linkType == halfDuplex)
		service.SetWork(&ROVCamera::_Work);
	else //full duplex
	{
		txservice.SetWork(&ROVCamera::_TxWork);
		rxservice.SetWork(&ROVCamera::_RxWork);
	}
	device.SetLogName("ROVCamera:CommsDeviceService");
	SetLogName("ROVCamera");
	localAddr = 0;
	remoteAddr = 0;
}

ROVCamera::~ROVCamera() {
	// TODO Auto-generated destructor stub
	if(service.IsRunning())
		service.Stop();
	if(txservice.IsRunning())
		txservice.Stop();
	if(rxservice.IsRunning())
		rxservice.Stop();
	device.Stop();
	delete buffer;
}

void ROVCamera::SetLocalAddr(int _addr)
{
	localAddr = _addr;
	if(txdlf)
	{
		txdlf->SetSrcDir(localAddr);
	}
}

void ROVCamera::SetRemoteAddr(int _addr)
{
	remoteAddr = _addr;
	if(txdlf)
	{
		txdlf->SetDesDir(remoteAddr);
	}
}

void ROVCamera::SetMaxImageTrunkLength(int _len)
{
	maxImgTrunkLength = _len;
}

void ROVCamera::SetStateSize(int _len)
{
	stateLength = _len;
	beginImgPtr = currentState + stateLength;
}

void ROVCamera::SetLogLevel(Loggable::LogLevel _level)
{
	Loggable::SetLogLevel(_level);
	device.SetLogLevel(_level);
}
void ROVCamera::SetChecksumType(DataLinkFrame::fcsType fcs)
{
	dlfcrctype = fcs;
}

void ROVCamera::SendImage(void * _buf, unsigned int _length)
{
	//TODO: aply a hash to de img in order to check errors when reassembling trunks
	std::unique_lock<std::mutex> lock(immutex);
	while(imgInBuffer)
	{
		imgInBufferCond.wait(lock);
	}
	memcpy(beginImgPtr, _buf, _length);
	imgChksumPtr =  (uint16_t*) (beginImgPtr + _length);
	endImgPtr = ((uint8_t *) imgChksumPtr) + IMG_CHKSUM_SIZE;
	currentImgPtr = beginImgPtr;

	uint32_t imgChksum = Checksum::crc32(beginImgPtr, _length);
	if(bigEndian)
	{
		*imgChksumPtr = imgChksum;
	}
	else
	{
		Utils::IntSwitchEndian(imgChksumPtr, imgChksum);
	}

	//TEMPORAL CHECK
	uint32_t crc = Checksum::crc32(beginImgPtr, _length + IMG_CHKSUM_SIZE);
	if(crc != 0)
	{
		Log->critical("data link frame with errors before transmission");
	}


	imgInBuffer = true;

	//mutex is unlocked automatically when calling the unique_lock destructor:
	//http://www.cplusplus.com/reference/mutex/unique_lock/
}

void ROVCamera::SetLastImgSentCallback(f_notification _callback)
{
	immutex.lock();
	lastImageSentCallback = _callback;
	immutex.unlock();
}

void ROVCamera::SetOrdersReceivedCallback(f_notification _callback)
{
	statemutex.lock();
	ordersReceivedCallback = _callback;
	statemutex.unlock();
}

bool ROVCamera::SendingCurrentImage()
{
	return imgInBuffer;
}

void ROVCamera::Start()
{
	txdlf = DataLinkFrame::BuildDataLinkFrame(dlfcrctype);
	rxdlf = DataLinkFrame::BuildDataLinkFrame(dlfcrctype);

	SetLocalAddr(localAddr);
	SetRemoteAddr(remoteAddr);

	txbuffer = txdlf->GetPayloadBuffer();
	rxbuffer = rxdlf->GetPayloadBuffer();

	txStatePtr = txbuffer;
	imgTrunkInfoPtr = (uint16_t*) (txStatePtr + stateLength);
	imgTrunkPtr = ((uint8_t *)imgTrunkInfoPtr) + IMG_TRUNK_INFO_SIZE;

	currentImgPtr = beginImgPtr; //No image in buffer
	endImgPtr = currentImgPtr;   //

	rxStatePtr = rxbuffer;
	device.Start();

	if(linkType == halfDuplex)
		service.Start();
	else
	{
		txservice.Start();
		rxservice.Start();
	}
}

void ROVCamera::_Work()
{
	Log->debug("waiting for new orders...");
	_WaitForNewOrders(10000);
	immutex.lock();
	_SendPacketWithCurrentStateAndImgTrunk();
	_CheckIfEntireImgIsSent();
	immutex.unlock();
	while(device.BusyTransmitting());
}

void ROVCamera::_TxWork()
{
	immutex.lock();
	_SendPacketWithCurrentStateAndImgTrunk();
	_CheckIfEntireImgIsSent();
	immutex.unlock();
	while(device.BusyTransmitting());
}

void ROVCamera::_RxWork()
{
	Log->debug("waiting for new orders...");
	_WaitForNewOrders(10000);
}


void ROVCamera::_UpdateCurrentStateFromRxState()
{
	statemutex.lock();
	memcpy(currentState, rxStatePtr, stateLength);
	statemutex.unlock();

}
void ROVCamera::_UpdateTxStateFromCurrentState()
{
	statemutex.lock();
	memcpy(txStatePtr, currentState, stateLength);
	statemutex.unlock();
}

void ROVCamera::GetCurrentState(void * dst)
{
	statemutex.lock();
	memcpy(dst,  currentState, stateLength);
	statemutex.unlock();

}

void ROVCamera::_WaitForNewOrders(int timeout)
{
	//Wait for the next packet and call the callback
	long elapsed = 0;
	rxtimer.Reset();
	while(elapsed < timeout)
	{
		if(device.GetRxFifoSize() > 0)
		{
			while(device.GetRxFifoSize() > 0)
			{
				device >> rxdlf;
				Log->debug("New orders received!");
			}
			_UpdateCurrentStateFromRxState();
			ordersReceivedCallback(*this);
			return;
		}
		Utils::Sleep(0.5);
		elapsed = rxtimer.Elapsed();
	}
	Log->warn("Timeout when trying to receive new orders from the operator!");

}

void ROVCamera::_SendPacketWithCurrentStateAndImgTrunk()
{
	//TODO: Prepare the next packet with the next image's trunk and send it
	//unsigned long a1 = (unsigned long) endImgPtr;
	//unsigned long a0 = (unsigned long) currentImgPtr;
	if(device.BusyTransmitting())
	{
		Log->critical("BUG: Device busy transmitting...");
		return;
	}
	int bytesLeft = endImgPtr - currentImgPtr;
	int nextTrunkLength;
	uint16_t trunkInfo = 0;

	_UpdateTxStateFromCurrentState();

	if(bytesLeft > 0) // == (ImgInBuffer == True)
	{
		if(bytesLeft > maxImgTrunkLength)
		{
			nextTrunkLength = maxImgTrunkLength;
		}
		else
		{
			trunkInfo |= IMG_LAST_TRUNK_FLAG;
			nextTrunkLength = bytesLeft;
		}
		if(beginImgPtr == currentImgPtr)
			trunkInfo |= IMG_FIRST_TRUNK_FLAG;

		trunkInfo |= nextTrunkLength;

		if(bigEndian)
			*imgTrunkInfoPtr = trunkInfo;
		else
		{
			Utils::IntSwitchEndian(imgTrunkInfoPtr, trunkInfo);
		}

		memcpy(imgTrunkPtr, currentImgPtr, nextTrunkLength);
		currentImgPtr += nextTrunkLength;

		txdlf->PayloadUpdated(stateLength + imgTrunkInfoLength + nextTrunkLength);
	}
	else
	{
		*imgTrunkInfoPtr = 0;
		txdlf->PayloadUpdated(stateLength + IMG_TRUNK_INFO_SIZE);
	}
	device << txdlf;

}

void ROVCamera::_CheckIfEntireImgIsSent()
{
	//Check If it has been sent the last image trunk and call the callback
	if(imgInBuffer && currentImgPtr == endImgPtr)
	{
		imgInBuffer = false;
		currentImgPtr = beginImgPtr;
		endImgPtr = currentImgPtr;
		imgInBufferCond.notify_one();
		lastImageSentCallback(*this);
	}
}

void ROVCamera::_SetEndianess()
{
	bigEndian = DataLinkFrame::IsBigEndian();
}


} /* namespace dcauv */
