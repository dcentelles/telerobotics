/*
 * ROVOperator.h
 *
 *  Created on: 27 oct. 2016
 *      Author: diego
 */

#ifndef WIRELESS_ARDUSUB_OPERATOR_H_
#define WIRELESS_ARDUSUB_OPERATOR_H_

#include <cpplogging/Loggable.h>
#include <dccomms/Utils.h>
#include <telerobotics/WAFrame.h>
#include <functional>
#include <iostream>
#include <mutex>
#include <telerobotics/Constants.h>

namespace telerobotics {

using namespace dccomms;
using namespace cpplogging;
using namespace std;
using namespace dccomms_packets;

class Operator : public Loggable {
public:
  Operator();
  virtual ~Operator();
  void SetTxState(const void *data, uint32_t length);
  void SetComms(Ptr<CommsDevice> comms);

  // http://stackoverflow.com/questions/2298242/callback-functions-in-c
  /*
      //mode 1:
      typedef void (*f_data)(void*,unsigned int);
      void SetDataReceivedCallback(f_data);
      */
  // mode 2:
  typedef std::function<void(Operator &)> f_notification;

  void SetImageReceivedCallback(f_notification);
  void SetStateReceivedCallback(f_notification);

  int GetLastReceivedImage(void *);
  void GetLastConfirmedState(void *);

  void Start();
  void DisableTransmission();
  void EnableTransmission();
  void SetEnableSrcAddrCheck(bool enable);

private:
  bool _canTransmit;
  f_notification imageReceivedCallback;
  f_notification stateReceivedCallback;

  std::mutex immutex, txstatemutex, rxstatemutex;
  uint8_t *buffer;
  uint8_t *rxStateBegin,
      *txStateBegin, *beginImgPtr;
  uint16_t lastImgSize;

  Ptr<CommsDevice> _comms;
  Ptr<WAFrame> txdlf;
  Ptr<WAFrame> rxdlf;

  ServiceThread<Operator> txservice;
  ServiceThread<Operator> rxservice;

  FCS dlfcrctype;

  ///// TX ////
  uint8_t *txStatePtr, *txbuffer, *txFlags;

  ///// RX ////
  uint8_t *rxStatePtr, *imgTrunkPtr, *rxbuffer, *currentImgPtr;

  uint8_t *rxMsgInfoPtr;

  int rxStateLength, txStateLength;
  int imgTrunkInfoLength;
  int imageTrunkLength;
  int maxPacketLength;
  // int minPacketLength;

  bool imgInBuffer;
  bool bigEndian;
  Timer rxtimer;

  void _WaitForCurrentStateAndNextImageTrunk(int millis_timeout);
  void _UpdateImgBufferFromLastMsg();
  void _SendPacketWithDesiredState();
  void _UpdateLastConfirmedStateFromLastMsg();

  void _Work(); // for half duplex

  void _RxWork(); // for full duplex
  void _TxWork(); // for full duplex

  uint8_t _GetStateSize(uint8_t rawInfo);
  void _LastTrunkReceived(uint8_t trunkSize);

  uint8_t msgInfo, trunkSize;
  bool desiredStateSet;

  bool _HaveImgTrunk(int i);
  void _MarkLastImgTrunk(int i);
  void _MarkImgTrunk(int i);
  int _ImgReceptionCompleted();
  int _GetReqImgSeq();
  void _ChangeImgSeq();
  void _UpdateTrunkSeq();
  bool baseTrunkSizeSet = false;
  int baseTrunkSize = 0;
  int lastTrunkReceived = false;
  int lastTrunkSize = 0;
  uint64_t receivedTrunksFlags = 0;
  bool cancelling;
  uint32_t _pktSeq;
  bool _checkSrcAddr;
  bool _rxError = false;
};

} /* namespace dcauv */

#endif /* ROVOPERATOR_H_ */
