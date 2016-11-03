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

namespace dcauv {

#define IMG_TRUNK_INFO_SIZE 2
#define IMG_FIRST_TRUNK_FLAG 0x8000
#define MAX_IMG_SIZE 32767 //(2^15-1)

using namespace dccomms;

class ROVCamera {
public:
	ROVCamera();
	virtual ~ROVCamera();
	void SendImage(void *, unsigned int);
	void SetState(void * data, unsigned int length);

	typedef std::function<void(void*, unsigned int)> f_data;
	typedef std::function<void(void)> f_notification;

	void SetOrdersReceivedCallback(f_data);
	void SetLastImgSentCallback(f_notification);

	void SetChecksumType(DataLinkFrame::fcsType fcs);
	void Start();
private:
	void _WaitForNewOrders(int millis_timeout);
	void _SendPacketWithCurrentStateAndImgTrunk();
	void _CheckIfEntireImgIsSent();

	void _UpdateCurrentStateFromLastMsg();

	void _SetEndianess();

	void _Work();
	mutex _mutex;
	f_data ordersReceivedCallback;
	f_notification lastImageSent;

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
