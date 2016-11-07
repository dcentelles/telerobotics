/*
 * ROVOperator.cpp
 *
 *  Created on: 27 oct. 2016
 *      Author: diego
 */

#include <ROVOperator.h>
#include <Constants.h>
#include <DataLinkFrame.h>

namespace dcauv {

using namespace dccomms;

ROVOperator::ROVOperator():service(this){
	// TODO Auto-generated constructor stub
	bigEndian = DataLinkFrame::IsBigEndian();
	stateLength = 40;
	imgTrunkInfoLength = IMG_TRUNK_INFO_SIZE;
	maxImgTrunkLength = 200;
	maxPacketLength = stateLength +
				   	  imgTrunkInfoLength +
					  maxImgTrunkLength;

	dlfcrctype = DataLinkFrame::fcsType::crc16;
	buffer = new uint8_t[maxPacketLength + MAX_IMG_SIZE];
	currentState = buffer;
	beginImgPtr = currentState + stateLength;
	device.SetNamespace("operator");
	imgInBuffer = false;
}

ROVOperator::~ROVOperator() {
	// TODO Auto-generated destructor stub
}

int ROVOperator::GetLastReceivedImage(void * data)
{
	return 0;
}

void ROVOperator::GetLastConfirmedState(void * data)
{

}

void ROVOperator::SetDesiredState(const void * _data, unsigned int _length)
{

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
}

} /* namespace dcauv */
