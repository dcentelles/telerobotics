/*
 * ROVCamera.cpp
 *
 *  Created on: 27 oct. 2016
 *      Author: diego
 */

#include <ROVCamera.h>
#include <Checksum.h>
#include <Constants.h>

namespace dcauv {

void defaultLastImageSentCallback(ROVCamera & rovcamera)
{
	//Nothing to do
}

void defaultOrdersReceivedCallback(ROVCamera & rovcamera)
{
	//Nothing to do
}


ROVCamera::ROVCamera():service(this) {
	// TODO Auto-generated constructor stub
	_SetEndianess();
	stateLength = MAX_IMG_STATE_LENGTH;
	imgTrunkInfoLength = IMG_TRUNK_INFO_SIZE;
	maxImgTrunkLength = MAX_IMG_TRUNK_LENGTH;
	maxPacketLength = stateLength +
				   	  imgTrunkInfoLength +
					  maxImgTrunkLength;

	dlfcrctype = DataLinkFrame::fcsType::crc16;
	device.SetNamespace("camera");
	buffer = new uint8_t[maxPacketLength + MAX_IMG_SIZE];
	currentState = buffer;
	beginImgPtr = currentState + stateLength;
	imgInBuffer = false;
	lastImageSentCallback = &defaultLastImageSentCallback;
	ordersReceivedCallback = &defaultOrdersReceivedCallback;

	service.SetWork(&ROVCamera::_Work);

}

ROVCamera::~ROVCamera() {
	// TODO Auto-generated destructor stub
	service.Stop();
	device.Stop();
	delete buffer;
}

void ROVCamera::SetChecksumType(DataLinkFrame::fcsType fcs)
{
	dlfcrctype = fcs;
}

void ROVCamera::SendImage(void * _buf, unsigned int _length)
{
	//TODO: aply a hash to de img in order to check errors when reassembling trunks
	std::unique_lock<std::mutex> lock(mutex);
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
		std::cerr << "internal error" << std::endl;
	}


	imgInBuffer = true;

	//mutex is unlocked automatically when calling the unique_lock destructor:
	//http://www.cplusplus.com/reference/mutex/unique_lock/
}

void ROVCamera::SetLastImgSentCallback(f_notification _callback)
{
	mutex.lock();
	lastImageSentCallback = _callback;
	mutex.unlock();
}

void ROVCamera::SetOrdersReceivedCallback(f_notification _callback)
{
	mutex.lock();
	ordersReceivedCallback = _callback;
	mutex.unlock();
}

bool ROVCamera::SendingCurrentImage()
{
	return imgInBuffer;
}

void ROVCamera::Start()
{
	txdlf = DataLinkFrame::BuildDataLinkFrame(dlfcrctype);
	rxdlf = DataLinkFrame::BuildDataLinkFrame(dlfcrctype);

	txbuffer = txdlf->GetPayloadBuffer();
	rxbuffer = rxdlf->GetPayloadBuffer();

	txStatePtr = txbuffer;
	imgTrunkInfoPtr = (uint16_t*) (txStatePtr + stateLength);
	imgTrunkPtr = ((uint8_t *)imgTrunkInfoPtr) + IMG_TRUNK_INFO_SIZE;

	currentImgPtr = beginImgPtr; //No image in buffer
	endImgPtr = currentImgPtr;   //

	rxStatePtr = rxbuffer;
	device.Start();
	service.Start();
}

void ROVCamera::_Work()
{
	LOG_DEBUG("waiting for new orders...");
	_WaitForNewOrders(5000);
	mutex.lock();
	_SendPacketWithCurrentStateAndImgTrunk();
	_CheckIfEntireImgIsSent();
	mutex.unlock();
}

void ROVCamera::_UpdateCurrentStateFromLastMsg()
{
	mutex.lock();
	memcpy(currentState, rxStatePtr, stateLength);
	mutex.unlock();

}

void ROVCamera::GetCurrentState(void * dst)
{
	mutex.lock();
	memcpy(dst,  currentState, stateLength);
	mutex.unlock();

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
			device >> rxdlf;
			_UpdateCurrentStateFromLastMsg();
			ordersReceivedCallback(*this);
			break;
		}
		Utils::Sleep(0.5);
		elapsed = rxtimer.Elapsed();
	}

}

void ROVCamera::_SendPacketWithCurrentStateAndImgTrunk()
{
	//TODO: Prepare the next packet with the next image's trunk and send it
	int bytesLeft = endImgPtr - currentImgPtr;
	int nextTrunkLength;
	uint16_t trunkInfo = 0;
	if(bytesLeft > maxImgTrunkLength)
	{
		nextTrunkLength = maxImgTrunkLength;
	}
	else
	{
		trunkInfo |= IMG_LAST_TRUNK_FLAG;
		nextTrunkLength = bytesLeft;
	}


	memcpy(txStatePtr, currentState, stateLength);

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
