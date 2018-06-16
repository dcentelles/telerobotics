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
#include <dccomms_packets/VariableLengthPacket.h>
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
  Ptr<VariableLengthPacket> txdlf;
  Ptr<VariableLengthPacket> rxdlf;

  ServiceThread<Operator> txservice;
  ServiceThread<Operator> rxservice;

  FCS dlfcrctype;

  ///// TX ////
  uint8_t *txStatePtr, *txbuffer;

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

  uint8_t _GetMsgInfo();
  uint8_t _GetStateSize(uint8_t rawInfo);
  void _LastTrunkReceived(uint8_t trunkSize);

  uint8_t msgInfo, trunkSize;
  bool desiredStateSet;
};

} /* namespace dcauv */

#endif /* ROVOPERATOR_H_ */
