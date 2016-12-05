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
#include <dccomms/Utils.h>
#include <dccomms/CommsDeviceService.h>
#include <mutex>
#include <Loggable.h>
#include <telerobotics/Constants.h>

namespace dcauv {

using namespace dccomms;
using namespace cpplogging;

class ROVOperator: public Loggable {
public:
	ROVOperator(LinkType = halfDuplex);
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

	void SetLocalAddr(int);
	void SetRemoteAddr(int);

	void Start();

	virtual void SetLogLevel(Loggable::LogLevel);

	void SetMaxImageTrunkLength(int);
        void SetRxStateSize(int);
        void SetTxStateSize(int);
private:
        void _UpdateRxStateSize(int);
        void _UpdateTxStateSize(int);

	f_notification imageReceivedCallback;
	f_notification stateReceivedCallback;

	LinkType linkType;
        std::mutex immutex, txstatemutex, rxstatemutex;
	uint8_t * buffer;
        uint8_t * currentRxState,
                * desiredState,  // == currentTxState
                * beginImgPtr,
                * beginLastImgPtr;
	uint16_t lastImgSize;

	CommsDeviceService device;
	DataLinkFramePtr txdlf;
	DataLinkFramePtr rxdlf;

	//for halfDuplex
	ServiceThread<ROVOperator> service;

	//for fullDuplex
	ServiceThread<ROVOperator> txservice;
	ServiceThread<ROVOperator> rxservice;

	DataLinkFrame::fcsType dlfcrctype;

	///// TX ////
	uint8_t * txStatePtr, * txbuffer;

	///// RX ////
	uint8_t * rxStatePtr,     *imgTrunkPtr,  *rxbuffer,
		    * currentImgPtr;

	uint16_t * imgTrunkInfoPtr;

        int rxStateLength,
            txStateLength;
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

	void _Work(); //for half duplex

	void _RxWork(); //for full duplex
	void _TxWork(); //for full duplex

	uint16_t _GetTrunkInfo();
	uint16_t _GetTrunkSize(uint16_t rawInfo);
	void _LastTrunkReceived(uint16_t trunkSize);

	int localAddr, remoteAddr;
        bool desiredStateSet;
};

} /* namespace dcauv */

#endif /* ROVOPERATOR_H_ */
