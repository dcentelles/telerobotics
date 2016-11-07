/*
 * ROVCamera.h
 *
 *  Created on: 27 oct. 2016
 *      Author: diego
 */

#ifndef ROVCAMERA_H_
#define ROVCAMERA_H_

#include <iostream>
#include <functional>
#include <CommsDeviceService.h>
#include <DataLinkFrame.h>
#include <Utils.h>
#include <mutex>
#include <condition_variable>

namespace dcauv {

using namespace dccomms;

class ROVCamera {
public:
	ROVCamera();
	virtual ~ROVCamera();
	void SendImage(void *, unsigned int);
	void SetState(void * data, unsigned int length);

	//typedef std::function<void(void*, unsigned int)> f_data;
	typedef std::function<void(void)> f_notification;

	void SetOrdersReceivedCallback(f_notification);//f_data);
	void SetLastImgSentCallback(f_notification);

	void GetCurrentState(void * dst);

	bool SendingCurrentImage();

	void SetChecksumType(DataLinkFrame::fcsType fcs);
	void Start();
private:
	void _WaitForNewOrders(int millis_timeout);
	void _SendPacketWithCurrentStateAndImgTrunk();
	void _CheckIfEntireImgIsSent();

	void _UpdateCurrentStateFromLastMsg();

	void _SetEndianess();

	void _Work();
	std::mutex mutex;
	condition_variable imgInBufferCond;

	//f_data ordersReceivedCallback;
	f_notification ordersReceivedCallback;
	f_notification lastImageSentCallback;

	uint8_t * buffer;
	uint8_t * currentState;

	CommsDeviceService device;
	DataLinkFramePtr txdlf;
	DataLinkFramePtr rxdlf;
	ServiceThread<ROVCamera> service;

	//// TX /////
	uint8_t * txbuffer;
	uint8_t * txStatePtr;
	uint16_t * imgTrunkInfoPtr;
	uint8_t * imgTrunkPtr;

	uint8_t * beginImgPtr;
	uint8_t * currentImgPtr;
	uint16_t * imgChksumPtr;
	uint8_t * endImgPtr;

	//// RX /////
	uint8_t * rxbuffer;
	uint8_t * rxStatePtr;
	DataLinkFrame::fcsType dlfcrctype;


	int stateLength;
	int imgTrunkInfoLength;
	int maxImgTrunkLength;
	int maxPacketLength;

	bool imgInBuffer;
	bool bigEndian;
	Timer rxtimer;
};

} /* namespace dcauv */

#endif /* ROVCAMERA_H_ */
