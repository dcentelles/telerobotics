/*
 * ROVOperator.cpp
 *
 *  Created on: 27 oct. 2016
 *      Author: diego
 */

#include <ROVOperator.h>
#include <Constants.h>
#include <DataLinkFrame.h>
#include <Checksum.h>

namespace dcauv {

using namespace dccomms;

void defaultImageReceivedCallback(ROVOperator & rovOperator)
{

}

void defaultStateReceivedCallback(ROVOperator & rovOperator)
{

}

ROVOperator::ROVOperator():service(this){
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
	//minPacketLength = stateLength;
	maxPacketLength = stateLength +
				   	  imgTrunkInfoLength +
					  maxImgTrunkLength;

	dlfcrctype = DataLinkFrame::fcsType::crc16;
	buffer = new uint8_t[maxPacketLength + stateLength + MAX_IMG_SIZE + IMG_CHKSUM_SIZE + MAX_IMG_SIZE];
	currentState = buffer;
	desiredState = currentState + stateLength;
	beginImgPtr = desiredState + stateLength;
	beginLastImgPtr = beginImgPtr + MAX_IMG_SIZE;
	device.SetNamespace("operator");
	device.SetChecksumType(DataLinkFrame::crc16);
	lastImgSize = 0;
	imgInBuffer = false;
	service.SetWork(&ROVOperator::_Work);

	imageReceivedCallback = &defaultImageReceivedCallback;
	stateReceivedCallback = &defaultStateReceivedCallback;

	SetLogName("ROVOperator");
}

ROVOperator::~ROVOperator() {
	// TODO Auto-generated destructor stub
	service.Stop();
	device.Stop();
	delete buffer;
}

void ROVOperator::SetLogLevel(Loggable::LogLevel _level)
{
	Loggable::SetLogLevel(_level);
	device.SetLogLevel(_level);
}

int ROVOperator::GetLastReceivedImage(void * data)
{
	int imgSize;
	mutex.lock();
	memcpy(data, beginLastImgPtr, lastImgSize);
	imgSize = lastImgSize;
	mutex.unlock();

	return imgSize;
}

void ROVOperator::GetLastConfirmedState(void * data)
{
	mutex.lock();
	memcpy(data, currentState, stateLength);
	mutex.unlock();

}

void ROVOperator::SetDesiredState(const void * _data, unsigned int _length)
{
	mutex.lock();
	memcpy(desiredState, _data, _length);
	mutex.unlock();
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

	txbuffer = txdlf->GetPayloadBuffer();
	rxbuffer = rxdlf->GetPayloadBuffer();

	txStatePtr = txbuffer;
	rxStatePtr = rxbuffer;

	imgTrunkInfoPtr = (uint16_t*) (rxStatePtr + stateLength);
	imgTrunkPtr = ((uint8_t *)imgTrunkInfoPtr) + IMG_TRUNK_INFO_SIZE;

	currentImgPtr = beginImgPtr;

	device.Start();
	service.Start();
}

void ROVOperator::_Work()
{
	Log->debug("waiting for new state from ROV...");
	_WaitForCurrentStateAndNextImageTrunk(10000);
	if(!device.BusyTransmitting())
		_SendPacketWithDesiredState();
	else
		Log->critical("Device busy transmitting after wating for the current rov state");
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
		mutex.lock();
		lastImgSize = blockSize - IMG_CHKSUM_SIZE;
		memcpy(beginLastImgPtr, beginImgPtr, lastImgSize);
		mutex.unlock();

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
	mutex.lock();
	memcpy(txStatePtr, desiredState, stateLength);
	mutex.unlock();

	txdlf->PayloadUpdated(stateLength);
	Log->debug("Sending packet with new orders...");
	device << txdlf;
	while(device.BusyTransmitting());
}

void ROVOperator::_UpdateLastConfirmedStateFromLastMsg()
{
	mutex.lock();
	memcpy(currentState, rxStatePtr, stateLength);
	mutex.unlock();
}

} /* namespace dcauv */
