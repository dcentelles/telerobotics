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

namespace dcauv {

using namespace dccomms;

class ROVOperator {
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
	typedef std::function<void(void)> f_notification;

	void SetImageReceivedCallback(f_notification);
	void SetStateReceivedCallback(f_notification);

	int GetLastReceivedImage(void *);
	void GetLastConfirmedState(void *);

	void Start();
private:
	f_notification imageReceivedCallback;
	f_notification stateReceivedCallback;

	uint8_t * buffer;
	uint8_t * currentState;

	CommsDeviceService device;
	DataLinkFramePtr txdlf;
	DataLinkFramePtr rxdlf;

	ServiceThread<ROVOperator> service;

	DataLinkFrame::fcsType dlfcrctype;

	///// RX ////
	uint8_t * beginImgPtr;

	int stateLength;
	int imgTrunkInfoLength;
	int maxImgTrunkLength;
	int maxPacketLength;

	bool imgInBuffer;
	bool bigEndian;
	Timer rxtimer;

	void _lockStateMutex();
	void _unlockStateMutex();
};

} /* namespace dcauv */

#endif /* ROVOPERATOR_H_ */
