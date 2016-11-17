/*
 * ROVOperator.cpp
 *
 *  Created on: 27 oct. 2016
 *      Author: diego
 */

#include <ROVOperator.h>
#include <Constants.h>
#include <dccomms/DataLinkFrame.h>
#include <dccomms/Checksum.h>

namespace dcauv {

using namespace dccomms;

void defaultImageReceivedCallback(ROVOperator & rovOperator)
{

}

void defaultStateReceivedCallback(ROVOperator & rovOperator)
{

}

ROVOperator::ROVOperator(LinkType _linkType):service(this),txservice(this),rxservice(this){
	rxbuffer = 0;
	imgTrunkPtr = 0;
	txbuffer = 0;
	imgTrunkInfoPtr = 0;
	currentImgPtr = 0;
	rxStatePtr = 0;
	txStatePtr = 0;

	bigEndian = DataLinkFrame::IsBigEndian();
	stateLength = MAX_IMG_STATE_LENGTH;
	imgTrunkInfoLength = IMG_TRUNK_INFO_SIZE;
	maxImgTrunkLength = MAX_IMG_TRUNK_LENGTH;
	maxPacketLength = MAX_PACKET_LENGTH;

	dlfcrctype = DataLinkFrame::fcsType::crc16;
	buffer = new uint8_t[maxPacketLength + MAX_IMG_SIZE + IMG_CHKSUM_SIZE + MAX_IMG_SIZE];
	currentState = buffer;

	_UpdateStateSize(stateLength);


	device.SetNamespace("operator");
	device.SetChecksumType(DataLinkFrame::crc16);
	lastImgSize = 0;
	imgInBuffer = false;
	linkType = _linkType;
	if(linkType == halfDuplex)
		service.SetWork(&ROVOperator::_Work);
	else //full duplex
	{
		txservice.SetWork(&ROVOperator::_TxWork);
		rxservice.SetWork(&ROVOperator::_RxWork);
	}

	imageReceivedCallback = &defaultImageReceivedCallback;
	stateReceivedCallback = &defaultStateReceivedCallback;

	SetLogName("ROVOperator");
	localAddr = 0;
	remoteAddr = 0;
}

ROVOperator::~ROVOperator() {
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

void ROVOperator::SetLocalAddr(int _addr)
{
	localAddr = _addr;
	if(txdlf)
	{
		txdlf->SetSrcDir(localAddr);
	}
}

void ROVOperator::SetRemoteAddr(int _addr)
{
	remoteAddr = _addr;
	if(txdlf)
	{
		txdlf->SetDesDir(remoteAddr);
	}
}

void ROVOperator::SetMaxImageTrunkLength(int _len)
{
	maxImgTrunkLength = _len;
}

void ROVOperator::SetStateSize(int _len)
{
	_UpdateStateSize(_len);
}

void ROVOperator::_UpdateStateSize(int _len)
{
	stateLength = _len;
	desiredState = currentState + stateLength;
	beginImgPtr = desiredState + stateLength;
	beginLastImgPtr = beginImgPtr + MAX_IMG_SIZE;
}
void ROVOperator::SetLogLevel(Loggable::LogLevel _level)
{
	Loggable::SetLogLevel(_level);
	device.SetLogLevel(_level);
}

int ROVOperator::GetLastReceivedImage(void * data)
{
	int imgSize;
	immutex.lock();
	memcpy(data, beginLastImgPtr, lastImgSize);
	imgSize = lastImgSize;
	immutex.unlock();

	return imgSize;
}

void ROVOperator::GetLastConfirmedState(void * data)
{
	statemutex.lock();
	memcpy(data, currentState, stateLength);
	statemutex.unlock();

}

void ROVOperator::SetDesiredState(const void * _data, unsigned int _length)
{
	statemutex.lock();
	memcpy(desiredState, _data, _length);
	statemutex.unlock();
}

void ROVOperator::SetImageReceivedCallback(f_notification _callback)
{
	imageReceivedCallback = _callback;
}

void ROVOperator::SetStateReceivedCallback(f_notification _callback)
{
	stateReceivedCallback = _callback;
}

void ROVOperator::Start()
{
	txdlf = DataLinkFrame::BuildDataLinkFrame(dlfcrctype);
	rxdlf = DataLinkFrame::BuildDataLinkFrame(dlfcrctype);

	SetLocalAddr(localAddr);
	SetRemoteAddr(remoteAddr);

	txbuffer = txdlf->GetPayloadBuffer();
	rxbuffer = rxdlf->GetPayloadBuffer();

	txStatePtr = txbuffer;
	rxStatePtr = rxbuffer;

	imgTrunkInfoPtr = (uint16_t*) (rxStatePtr + stateLength);
	imgTrunkPtr = ((uint8_t *)imgTrunkInfoPtr) + IMG_TRUNK_INFO_SIZE;

	currentImgPtr = beginImgPtr;

	device.Start();
	if(linkType == halfDuplex)
		service.Start();
	else
	{
		txservice.Start();
		rxservice.Start();
	}
}

void ROVOperator::_Work()
{
	Log->debug("waiting for new state from ROV...");
	_WaitForCurrentStateAndNextImageTrunk(10000);
	_SendPacketWithDesiredState();
}

void ROVOperator::_TxWork()
{
	_SendPacketWithDesiredState();
}

void ROVOperator::_RxWork()
{
	Log->debug("waiting for new state from ROV...");
	_WaitForCurrentStateAndNextImageTrunk(10000);
}

void ROVOperator::_WaitForCurrentStateAndNextImageTrunk(int timeout)
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
				Log->debug("Received new packet with last state confirmed and next image trunk");
			}
			_UpdateLastConfirmedStateFromLastMsg();
			_UpdateImgBufferFromLastMsg();
			stateReceivedCallback(*this);
			return;
		}
		Utils::Sleep(0.5);
		elapsed = rxtimer.Elapsed();
	}
	Log->warn("Timeout when trying to receive feedback from the ROV!");
}

void ROVOperator::_UpdateImgBufferFromLastMsg()
{
	uint16_t trunkInfo = _GetTrunkInfo();
	uint16_t trunkSize = _GetTrunkSize(trunkInfo);

	if(trunkInfo != 0)
	{
		if(trunkInfo & IMG_FIRST_TRUNK_FLAG) //the received trunk is the first trunk of an image
		{
			Log->debug("the received trunk is the first trunk of an image");
			memcpy(beginImgPtr, imgTrunkPtr, trunkSize);
			currentImgPtr = beginImgPtr + trunkSize;
			if(trunkInfo & IMG_LAST_TRUNK_FLAG) //the received trunk is also the last of an image (the image only has 1 trunk)
			{
				Log->debug("the received trunk is also the last of an image (the image only has 1 trunk)");
				_LastTrunkReceived(trunkSize);
			}
		}
		else //the received trunk is not the first of an image
		{
			if(currentImgPtr != beginImgPtr) //first trunk has already been received
			{
				memcpy(currentImgPtr, imgTrunkPtr, trunkSize);
				currentImgPtr += trunkSize;
				if(trunkInfo & IMG_LAST_TRUNK_FLAG) //the received trunk is the last of an image
				{
					Log->debug("the received trunk is the last of an image");
					_LastTrunkReceived(trunkSize);
				}
			}
			else
			{
			//else, we are waiting for the first trunk of an image
				Log->debug("waiting for the first trunk of an image");
			}
		}
	}
	else
	{
		//else, packet without an image trunk
		Log->warn("packet received without an image trunk");
	}



}
void ROVOperator::_LastTrunkReceived(uint16_t trunkSize)
{
	int blockSize  = currentImgPtr - beginImgPtr;

	currentImgPtr = beginImgPtr;
	uint32_t crc = Checksum::crc32(beginImgPtr, blockSize);
	if(crc == 0)
	{
		immutex.lock();
		lastImgSize = blockSize - IMG_CHKSUM_SIZE;
		memcpy(beginLastImgPtr, beginImgPtr, lastImgSize);
		immutex.unlock();

		imageReceivedCallback(*this);
	}
	else
	{
		Log->warn("image received with errors... (some packets were lost)");
	}
}

uint16_t ROVOperator::_GetTrunkInfo()
{
	uint16_t result;
	if(bigEndian)
	{
		result = *imgTrunkInfoPtr;
	}
	else
	{
		Utils::IntSwitchEndian(&result, *imgTrunkInfoPtr);
	}
	return result;
}

uint16_t ROVOperator::_GetTrunkSize(uint16_t rawInfo)
{
	return rawInfo & 0x3fff;
}

void ROVOperator::_SendPacketWithDesiredState()
{
	if(!device.BusyTransmitting())
	{
		statemutex.lock();
		memcpy(txStatePtr, desiredState, stateLength);
		statemutex.unlock();

		txdlf->PayloadUpdated(stateLength);
		Log->debug("Sending packet with new orders...");
		device << txdlf;
		while(device.BusyTransmitting());
	}
	else
		Log->critical("Device busy transmitting after wating for the current rov state");
}

void ROVOperator::_UpdateLastConfirmedStateFromLastMsg()
{
	statemutex.lock();
	memcpy(currentState, rxStatePtr, stateLength);
	statemutex.unlock();
}

} /* namespace dcauv */
