/*
 * ROVCamera.cpp
 *
 *  Created on: 27 oct. 2016
 *      Author: diego
 */

#include <ROVCamera.h>

namespace dcauv {

ROVCamera::ROVCamera():service(this) {
	// TODO Auto-generated constructor stub
	_SetEndianess();
	stateLength = 40;
	imgTrunkInfoLength = IMG_TRUNK_INFO_SIZE;
	maxImgTrunkLength = 200;
	maxPacketLength = stateLength +
				   	  imgTrunkInfoLength +
					  maxImgTrunkLength;

	dlfcrctype = DataLinkFrame::fcsType::crc32;
	device.SetNamespace("camera");
	buffer = new uint8_t[maxPacketLength + MAX_IMG_SIZE];
	currentState = buffer;
	beginImgPtr = currentState + stateLength;
	imgInBuffer = false;

}

ROVCamera::~ROVCamera() {
	// TODO Auto-generated destructor stub
	delete buffer;
}

void ROVCamera::SendImage(void * _buf, unsigned int _length)
{
	while(imgInBuffer){}
	//TODO: aply a hash to de img in order to check errors when reassembling trunks
	memcpy(beginImgPtr, _buf, _length);
	endImgPtr = beginImgPtr + _length;
	currentImgPtr = beginImgPtr;
	imgInBuffer = true;
}

void ROVCamera::SetOrdersReceivedCallback(std::function<void(void*, unsigned int)> _callback)
{
	ordersReceivedCallback = _callback;
}

void ROVCamera::Start()
{
	txdlf = DataLinkFrame::BuildDataLinkFrame(dlfcrctype);
	rxdlf = DataLinkFrame::BuildDataLinkFrame(dlfcrctype);

	txbuffer = txdlf->GetPayloadBuffer();
	rxbuffer = rxdlf->GetPayloadBuffer();

	txStatePtr = txbuffer;
	imgTrunkInfoPtr = txStatePtr + stateLength;
	imgTrunkPtr = ((uint8_t *)imgTrunkInfoPtr) + IMG_TRUNK_INFO_SIZE;

	rxStatePtr = rxbuffer;

	device.Start();
}

void ROVCamera::_Work()
{
	_WaitForNewOrders(2000);
	_SendPacketWithCurrentStateAndImgTrunk();
	_CheckIfEntireImgIsSent();
}

void ROVCamera::_UpdateCurrentStateFromLastMsg()
{
	_mutex.lock();
	memcpy(currentState, rxStatePtr, stateLength);
	_mutex.unlock();

}

void ROVCamera::_WaitForNewOrders(int timeout)
{
	//Wait for the next packet and call the callback
	unsigned int elapsed = 0;
	rxtimer.Reset();
	while(elapsed < timeout)
	{
		if(device.GetRxFifoSize() > 0)
		{
			rxdlf << device;
			_UpdateCurrentStateFromLastMsg();
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
	if(bytesLeft > maxImgTrunkLength)
	{
		nextTrunkLength = maxImgTrunkLength;
	}
	else
		nextTrunkLength = bytesLeft;

	memcpy(txStatePtr, currentState, stateLength);

	uint16_t trunkInfo = 0;
	if(beginImgPtr == currentImgPtr)
		trunkInfo = IMG_FIRST_TRUNK_FLAG;

	trunkInfo |= bytesLeft;

    if(bigEndian)
    	*imgTrunkInfoPtr = trunkInfo;
    else
    {
    	*(uint8_t*)imgTrunkInfoPtr = (uint8_t)(trunkInfo >> 8);
    	*(((uint8_t*)imgTrunkInfoPtr)+1) = (uint8_t)(trunkInfo & 0xff);
    }

	memcpy(imgTrunkPtr, currentImgPtr, nextTrunkLength);
	currentImgPtr += nextTrunkLength;

}

void ROVCamera::_CheckIfEntireImgIsSent()
{
	//Check If it has been sent the last image trunk and call the callback
	if(imgInBuffer && currentImgPtr == endImgPtr)
	{
		imgInBuffer = false;
		lastImageSent();
	}
}

void ROVCamera::_SetEndianess()
{
	bigEndian = DataLinkFrame::IsBigEndian();
}


} /* namespace dcauv */
