/*
 * ROVOperator.h
 *
 *  Created on: 27 oct. 2016
 *      Author: diego
 */

#ifndef ROVOPERATOR_H_
#define ROVOPERATOR_H_

#include <functional>
#include <iostream>
#include <Utils.h>
#include <CommsDeviceService.h>
#include <mutex>
#include <Loggable.h>

namespace dcauv {

using namespace dccomms;

class ROVOperator: public Loggable {
public:
	ROVOperator();
	virtual ~ROVOperator();
	void SetDesiredState(const void * data, unsigned int length);

	//http://stackoverflow.com/questions/2298242/callback-functions-in-c
	/*
	//mode 1:
	typedef void (*f_data)(void*,unsigned int);
	void SetDataReceivedCallback(f_data);
	*/
	//mode 2:
	typedef std::function<void(ROVOperator &)> f_notification;

	void SetImageReceivedCallback(f_notification);
	void SetStateReceivedCallback(f_notification);

	int GetLastReceivedImage(void *);
	void GetLastConfirmedState(void *);

	void Start();
private:
	f_notification imageReceivedCallback;
	f_notification stateReceivedCallback;

	std::mutex mutex;
	uint8_t * buffer;
	uint8_t * currentState, * desiredState, * beginImgPtr, * beginLastImgPtr;
	uint16_t lastImgSize;

	CommsDeviceService device;
	DataLinkFramePtr txdlf;
	DataLinkFramePtr rxdlf;

	ServiceThread<ROVOperator> service;

	DataLinkFrame::fcsType dlfcrctype;

	///// TX ////
	uint8_t * txStatePtr, * txbuffer;

	///// RX ////
	uint8_t * rxStatePtr,     *imgTrunkPtr,  *rxbuffer,
		    * currentImgPtr;

	uint16_t * imgTrunkInfoPtr;

	int stateLength;
	int imgTrunkInfoLength;
	int maxImgTrunkLength;
	int maxPacketLength;
	//int minPacketLength;

	bool imgInBuffer;
	bool bigEndian;
	Timer rxtimer;

	void _WaitForCurrentStateAndNextImageTrunk(int millis_timeout);
	void _UpdateImgBufferFromLastMsg();
	void _SendPacketWithDesiredState();
	void _UpdateLastConfirmedStateFromLastMsg();
	void _Work();

	uint16_t _GetTrunkInfo();
	uint16_t _GetTrunkSize(uint16_t rawInfo);
	void _LastTrunkReceived(uint16_t trunkSize);
};

} /* namespace dcauv */

#endif /* ROVOPERATOR_H_ */
