/*
 * ROVOperator.cpp
 *
 *  Created on: 27 oct. 2016
 *      Author: diego
 */

#include <telerobotics/ROVOperator.h>
#include <telerobotics/Constants.h>
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
	imgTrunkInfoLength = IMG_TRUNK_INFO_SIZE;
	maxImgTrunkLength = MAX_IMG_TRUNK_LENGTH;
	maxPacketLength = MAX_PACKET_LENGTH;

	dlfcrctype = DataLinkFrame::fcsType::crc16;
	buffer = new uint8_t[maxPacketLength + MAX_IMG_SIZE + IMG_CHKSUM_SIZE + MAX_IMG_SIZE];

    currentRxState = buffer;
    _UpdateRxStateSize(MAX_IMG_STATE_LENGTH);
    _UpdateTxStateSize(MAX_IMG_STATE_LENGTH);


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

	device.SetLogName("ROVOperator:CommsDeviceService");
	SetLogName("ROVOperator");
	localAddr = 0;
	remoteAddr = 0;

    desiredStateSet = false;
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

void ROVOperator::SetRxStateSize(int _len)
{
    _UpdateRxStateSize(_len);
}

void ROVOperator::SetTxStateSize(int _len)
{
    _UpdateTxStateSize(_len);
}

void ROVOperator::_UpdateRxStateSize(int _len)
{
    rxStateLength = _len;
    desiredState = currentRxState + rxStateLength;
    beginImgPtr = desiredState + txStateLength;
	beginLastImgPtr = beginImgPtr + MAX_IMG_SIZE;
    Log->debug("Set a new Rx-State length: {} bytes", _len);
}


void ROVOperator::_UpdateTxStateSize(int _len)
{
    txStateLength = _len;
    beginImgPtr = desiredState + txStateLength;
    beginLastImgPtr = beginImgPtr + MAX_IMG_SIZE;
    Log->debug("Set a new Tx-State length: {} bytes", _len);
}

void ROVOperator::SetLogLevel(Loggable::LogLevel _level)
{
	Loggable::SetLogLevel(_level);
	device.SetLogLevel(_level);
}

void ROVOperator::SetLogName(string name)
{
    Loggable::SetLogName(name);
    device.SetLogName(name + ":CommsDeviceService");
}

void ROVOperator::LogToConsole(bool c)
{
    Loggable::LogToConsole(c);
    device.LogToConsole(c);
}

void ROVOperator::LogToFile(const string &filename)
{
    Loggable::LogToFile(filename);
    device.LogToFile(filename + "_service");
}

void ROVOperator::FlushLog()
{
    Loggable::FlushLog();
    device.FlushLog();
}

void ROVOperator::FlushLogOn(LogLevel level)
{
    Loggable::FlushLogOn(level);
    device.FlushLogOn(level);
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
    rxstatemutex.lock();
    memcpy(data, currentRxState, rxStateLength);
    rxstatemutex.unlock();

}

void ROVOperator::SetDesiredState(const void * _data)
{
    txstatemutex.lock();
    memcpy(desiredState, _data, txStateLength);
    txstatemutex.unlock();
    desiredStateSet = true;
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

    auto txdlbuffer = txdlf->GetPayloadBuffer();
    auto rxdlbuffer = rxdlf->GetPayloadBuffer();

    txtrp = TransportPDU::BuildTransportPDU(0,txdlbuffer);
    rxtrp = TransportPDU::BuildTransportPDU(0,rxdlbuffer);

    txbuffer = txtrp->GetPayloadBuffer();
    rxbuffer = rxtrp->GetPayloadBuffer();

	txStatePtr = txbuffer;
	rxStatePtr = rxbuffer;

    imgTrunkInfoPtr = (uint16_t*) (rxStatePtr + rxStateLength);
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
	device.WaitForDeviceReadyToTransmit();
	_SendPacketWithDesiredState();
}

void ROVOperator::_TxWork()
{
    Log->debug("TX: Waiting for device ready to transmit...");
	device.WaitForDeviceReadyToTransmit();
	_SendPacketWithDesiredState();
}

void ROVOperator::_RxWork()
{
    Log->debug("RX: waiting for new state from ROV...");
	_WaitForCurrentStateAndNextImageTrunk(10000);
}

void ROVOperator::_WaitForCurrentStateAndNextImageTrunk(int timeout)
{
	//Wait for the next packet and call the callback
    Log->debug("RX: waiting for frames...");
	if(device.WaitForFramesFromRxFifo(timeout))
	{
        Log->debug("RX: new frames in RX FIFO ({} frames).", device.GetRxFifoSize());
		while(device.GetRxFifoSize() > 0)
		{
			device >> rxdlf;
            Log->debug("RX: received new packet with last state confirmed and next image trunk (FS: {}).", rxdlf->GetFrameSize());
		}
		_UpdateLastConfirmedStateFromLastMsg();
		_UpdateImgBufferFromLastMsg();
		stateReceivedCallback(*this);
	}
	else
	{
        Log->warn("RX: timeout when trying to receive feedback from the ROV!");
    }
}

void ROVOperator::_UpdateImgBufferFromLastMsg()
{
	uint16_t trunkInfo = _GetTrunkInfo();
	uint16_t trunkSize = _GetTrunkSize(trunkInfo);

	if(trunkInfo != 0)
	{
		if(trunkInfo & IMG_FIRST_TRUNK_FLAG) //the received trunk is the first trunk of an image
		{
            Log->debug("RX: the received trunk is the first trunk of an image");
			memcpy(beginImgPtr, imgTrunkPtr, trunkSize);
			currentImgPtr = beginImgPtr + trunkSize;
			if(trunkInfo & IMG_LAST_TRUNK_FLAG) //the received trunk is also the last of an image (the image only has 1 trunk)
			{
                Log->debug("RX: the received trunk is also the last of an image (the image only has 1 trunk)");
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
                    Log->debug("RX: the received trunk is the last of an image");
					_LastTrunkReceived(trunkSize);
				}
			}
			else
			{
			//else, we are waiting for the first trunk of an image
                Log->debug("RX: waiting for the first trunk of an image");
			}
		}
	}
	else
	{
		//else, packet without an image trunk
        Log->warn("RX: packet received without an image trunk");
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
        Log->warn("RX: image received with errors... (some packets were lost)");
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
    if(desiredStateSet)
    {
        if(!device.BusyTransmitting())
        {
            txstatemutex.lock();
            memcpy(txStatePtr, desiredState, txStateLength);
            txstatemutex.unlock();

            txdlf->PayloadUpdated(TransportPDU::OverheadSize+txStateLength);
            Log->debug("TX: sending packet with new orders...");
            device << txdlf;
            while(device.BusyTransmitting());
        }
        else
            Log->critical("TX: device busy transmitting after wating for the current rov state");
    }
    else
    {
        Log->warn("TX: desired state is not set yet");
        Utils::Sleep(1000);
    }
}

void ROVOperator::_UpdateLastConfirmedStateFromLastMsg()
{
    rxstatemutex.lock();
    memcpy(currentRxState, rxStatePtr, rxStateLength);
    rxstatemutex.unlock();
}

} /* namespace dcauv */
