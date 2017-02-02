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
    rxStateLength = MAX_IMG_STATE_LENGTH;
    txStateLength = MAX_IMG_STATE_LENGTH;
	imgTrunkInfoLength = IMG_TRUNK_INFO_SIZE;
	maxImgTrunkLength = MAX_IMG_TRUNK_LENGTH;
	maxPacketLength = MAX_PACKET_LENGTH;

	dlfcrctype = DataLinkFrame::fcsType::crc16;
	device.SetNamespace("camera");
    buffer = new uint8_t[maxPacketLength + MAX_IMG_STATE_LENGTH*2 + MAX_IMG_SIZE]; //buffer max size is orientative...
    currentRxState = buffer;
    currentTxState = currentRxState + rxStateLength;
    beginImgPtr = currentTxState + txStateLength;
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
    txStateSet = false;
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
    Log->debug("Set a new maximum image trunk length: {} bytes", _len);
}

void ROVCamera::SetTxStateSize(int _len)
{
    txStateLength = _len;
    beginImgPtr = currentTxState + txStateLength;
    Log->debug("Set a new Tx-State length: {} bytes", _len);
}
void ROVCamera::SetRxStateSize(int _len)
{
    rxStateLength = _len;
    currentTxState = currentRxState + rxStateLength;
    SetTxStateSize(txStateLength);
    Log->debug("Set a new Rx-State length: {} bytes", _len);
}

void ROVCamera::SetLogLevel(Loggable::LogLevel _level)
{
	Loggable::SetLogLevel(_level);
	device.SetLogLevel(_level);
}

void ROVCamera::SetLogName(string name)
{
    Loggable::SetLogName(name);
    device.SetLogName(name + ":CommsDeviceService");
}

void ROVCamera::LogToConsole(bool c)
{
    Loggable::LogToConsole(c);
    device.LogToConsole(c);
}

void ROVCamera::LogToFile(const string &filename)
{
    Loggable::LogToFile(filename);
    device.LogToFile(filename + "_service");
}

void ROVCamera::FlushLog()
{
    Loggable::FlushLog();
    device.FlushLog();
}

void ROVCamera::FlushLogOn(LogLevel level)
{
    Loggable::FlushLogOn(level);
    device.FlushLogOn(level);
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
    Log->debug("New image available to transmit ({} bytes).", _length);

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
    rxstatemutex.lock();
	ordersReceivedCallback = _callback;
    rxstatemutex.unlock();
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

    auto txdlbuffer = txdlf->GetPayloadBuffer();
    auto rxdlbuffer = rxdlf->GetPayloadBuffer();

    txtrp = TransportPDU::BuildTransportPDU(0,txdlbuffer);
    rxtrp = TransportPDU::BuildTransportPDU(0,rxdlbuffer);

    txbuffer = txtrp->GetPayloadBuffer();
    rxbuffer = rxtrp->GetPayloadBuffer();

	txStatePtr = txbuffer;
    imgTrunkInfoPtr = (uint16_t*) (txStatePtr + txStateLength);
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
	device.WaitForDeviceReadyToTransmit();
}

void ROVCamera::_TxWork()
{
	immutex.lock();
	_SendPacketWithCurrentStateAndImgTrunk();
	_CheckIfEntireImgIsSent();
	immutex.unlock();
	device.WaitForDeviceReadyToTransmit();
}

void ROVCamera::_RxWork()
{
    Log->debug("RX: waiting for new orders...");
	_WaitForNewOrders(10000);
}


void ROVCamera::_UpdateCurrentRxStateFromRxState()
{
    rxstatemutex.lock();
    memcpy(currentRxState, rxStatePtr, rxStateLength);
    rxstatemutex.unlock();

}
void ROVCamera::_UpdateTxStateFromCurrentTxState()
{
    txstatemutex.lock();
    memcpy(txStatePtr, currentTxState, txStateLength);
    txstatemutex.unlock();
}

void ROVCamera::GetCurrentRxState(void * dst)
{
    rxstatemutex.lock();
    memcpy(dst,  currentRxState, rxStateLength);
    rxstatemutex.unlock();
}

void ROVCamera::SetCurrentTxState(void * src)
{
    txstatemutex.lock();
    memcpy(currentTxState, src, txStateLength);
    txstatemutex.unlock();
    txStateSet = true;
}

void ROVCamera::_WaitForNewOrders(int timeout)
{
	//Wait for the next packet and call the callback
	if(device.WaitForFramesFromRxFifo(timeout))
	{
			while(device.GetRxFifoSize() > 0)
			{
				device >> rxdlf;
                Log->debug("RX: new orders received! (Seq: {}) (FS: {})",
                           rxtrp->GetSeqNum (),
                           rxdlf->GetFrameSize());
			}
            _UpdateCurrentRxStateFromRxState();
			ordersReceivedCallback(*this);
	}
	else
	{
        Log->warn("RX: timeout when trying to receive new orders from the operator!");
    }
}

void ROVCamera::_SendPacketWithCurrentStateAndImgTrunk()
{
	//TODO: Prepare the next packet with the next image's trunk and send it
	//unsigned long a1 = (unsigned long) endImgPtr;
	//unsigned long a0 = (unsigned long) currentImgPtr;
	if(device.BusyTransmitting())
	{
        Log->critical("TX: possible bug: device busy transmitting at init..");
		return;
	}
    if(txStateSet)
    {
        int bytesLeft = endImgPtr - currentImgPtr;
        int nextTrunkLength;
        uint16_t trunkInfo = 0;

        _UpdateTxStateFromCurrentTxState();

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

            txdlf->PayloadUpdated(TransportPDU::OverheadSize + txStateLength + imgTrunkInfoLength + nextTrunkLength);
            Log->debug("TX: transmitting packet with the current state and an image trunk (Seq: {}) (FS: {})",
                       txtrp->GetSeqNum (),
                       txdlf->GetFrameSize());
        }
        else
        {
            *imgTrunkInfoPtr = 0;
            txdlf->PayloadUpdated(TransportPDU::OverheadSize + txStateLength + IMG_TRUNK_INFO_SIZE);
            Log->debug("TX: transmitting packet without an image trunk (only the current state (Seq: {}) (FS: {})",
                       txtrp->GetSeqNum (),
                       txdlf->GetFrameSize());
        }
        device << txdlf;
        txtrp->IncSeqNum ();
    }
    else
    {
        Log->warn("TX: current state is not set yet");
        Utils::Sleep(1000);
    }

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
        Log->debug("TX: image transmission completed");
	}
}

void ROVCamera::_SetEndianess()
{
	bigEndian = DataLinkFrame::IsBigEndian();
}


} /* namespace dcauv */
