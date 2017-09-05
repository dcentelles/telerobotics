/*
 * ROVCamera.h
 *
 *  Created on: 27 oct. 2016
 *      Author: diego
 */

#ifndef ROVCAMERA_H_
#define ROVCAMERA_H_

#include <condition_variable>
#include <cpplogging/Loggable.h>
#include <dccomms/CommsDeviceService.h>
#include <dccomms/DataLinkFrame.h>
#include <dccomms/TransportPDU.h>
#include <dccomms/Utils.h>
#include <functional>
#include <iostream>
#include <mutex>
#include <telerobotics/Constants.h>

namespace dcauv {

using namespace dccomms;
using namespace cpplogging;

class ROVCamera : public Loggable {
public:
  ROVCamera(LinkType = halfDuplex);
  virtual ~ROVCamera();
  void SendImage(void *, unsigned int);

  // typedef std::function<void(void*, unsigned int)> f_data;
  typedef std::function<void(ROVCamera &)> f_notification;

  void SetOrdersReceivedCallback(f_notification); // f_data);
  void SetLastImgSentCallback(f_notification);

  void GetCurrentRxState(void *dst);
  void SetCurrentTxState(void *src);

  bool SendingCurrentImage();

  void SetLocalAddr(int);
  void SetRemoteAddr(int);

  void SetChecksumType(DataLinkFrame::fcsType fcs);
  void Start();

  virtual void SetLogLevel(cpplogging::LogLevel);
  virtual void SetLogName(string name);
  virtual void FlushLog();
  virtual void FlushLogOn(LogLevel);
  virtual void LogToConsole(bool);
  virtual void LogToFile(const string &filename);

  void SetMaxImageTrunkLength(int);
  void SetRxStateSize(int);
  void SetTxStateSize(int);

private:
  void _WaitForNewOrders(int millis_timeout);
  void _SendPacketWithCurrentStateAndImgTrunk();
  void _CheckIfEntireImgIsSent();

  void _UpdateCurrentRxStateFromRxState();
  void _UpdateTxStateFromCurrentTxState();
  void _SetEndianess();

  void _Work(); // for half duplex

  void _RxWork(); // for full duplex
  void _TxWork(); // for full duplex

  LinkType linkType;
  std::mutex immutex, rxstatemutex, txstatemutex;
  condition_variable imgInBufferCond;

  // f_data ordersReceivedCallback;
  f_notification ordersReceivedCallback;
  f_notification lastImageSentCallback;

  uint8_t *buffer;
  uint8_t *currentRxState, *currentTxState;

  CommsDeviceService device;
  DataLinkFramePtr txdlf;
  DataLinkFramePtr rxdlf;
  TransportPDUPtr txtrp;
  TransportPDUPtr rxtrp;

  // for halfDuplex
  ServiceThread<ROVCamera> service;

  // for fullDuplex
  ServiceThread<ROVCamera> txservice;
  ServiceThread<ROVCamera> rxservice;

  //// TX /////
  uint8_t *txbuffer;
  uint8_t *txStatePtr;
  uint16_t *imgTrunkInfoPtr;
  uint8_t *imgTrunkPtr;

  uint8_t *beginImgPtr;
  uint8_t *currentImgPtr;
  uint16_t *imgChksumPtr;
  uint8_t *endImgPtr;

  //// RX /////
  uint8_t *rxbuffer;
  uint8_t *rxStatePtr;
  DataLinkFrame::fcsType dlfcrctype;

  int rxStateLength, txStateLength;
  int imgTrunkInfoLength;
  int maxImgTrunkLength;
  int maxPacketLength;

  bool imgInBuffer;
  bool bigEndian;
  Timer rxtimer;

  int localAddr, remoteAddr;
  bool txStateSet;

  unsigned int _timeout, _minTimeout, _timeoutInc;
};

} /* namespace dcauv */

#endif /* ROVCAMERA_H_ */
