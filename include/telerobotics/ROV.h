/*
 * ROVCamera.h
 *
 *  Created on: 27 oct. 2016
 *      Author: diego
 */

#ifndef WIRELESS_ARDUSUB_ROV_H_
#define WIRELESS_ARDUSUB_ROV_H_

#include <condition_variable>
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
using namespace dccomms_packets;

class ROV : public Loggable {
public:
  ROV();
  virtual ~ROV();

  void SetComms(Ptr<CommsDevice> _comms);

  void SendImage(void *, unsigned int);

  // typedef std::function<void(void*, unsigned int)> f_data;
  typedef std::function<void(ROV &)> f_notification;

  void SetOrdersReceivedCallback(f_notification); // f_data);
  void SetLastImgSentCallback(f_notification);

  void GetCurrentRxState(void *dst);
  void SetCurrentTxState(void *src, uint32_t length);

  bool SendingCurrentImage();
  void CancelLastImage();

  void SetChecksumType(FCS fcs);
  void Start();

  void SetImageTrunkLength(int);

  void HoldChannel(bool);
  bool HoldingChannel() { return _holdChannel; }

  void SetEnsureImgDelivery(bool v);
  bool EnsureDelivery();
  void SetEnableSrcAddrCheck(bool enable);

private:
  void _ReinitImageFlags();
  void _WaitForNewOrders();
  void
  _SendPacketWithCurrentStateAndImgTrunk(std::unique_lock<std::mutex> &lock,
                                         bool block = false);
  void _UpdateCurrentRxStateFromRxState();
  void _UpdateTxStateFromCurrentTxState();
  void _SetEndianess();

  void _Work(); // for full duplex
  void _HoldChannelWork();
  void _UpdateTxStateSizeOnMsgInfo();
  void _UpdateTrunkFlagsOnMsgInfo(uint8_t flags);
  uint8_t _GetTxStateSizeFromMsgInfo();
  int _GetRequestedImgSeq();
  int _GetRequestedImgTrunkSeq();
  bool _LastImageCancelled();

  bool _holdChannel, _ensureDelivery;
  std::mutex _holdChannel_mutex;
  condition_variable _holdChannel_cond;

  std::mutex _immutex, _rxstatemutex, _txstatemutex;
  condition_variable _imgInBufferCond;

  // f_data ordersReceivedCallback;
  f_notification _ordersReceivedCallback;
  f_notification _lastImageSentCallback;

  uint8_t *_buffer;
  uint8_t *_rxStateBegin, *_txStateBegin;

  Ptr<CommsDevice> _comms;

  ServiceThread<ROV> _commsWorker, _holdChannelCommsWorker;

  Ptr<VariableLengthPacket> _txdlf;
  Ptr<VariableLengthPacket> _rxdlf;
  //// TX /////
  uint8_t *_txbuffer;
  uint8_t *_txStatePtr;
  uint8_t *_txMsgInfoPtr;
  uint8_t *_imgTrunkPtr;

  uint8_t *_beginImgPtr;
  uint16_t *_imgChksumPtr;
  uint8_t *_endImgPtr;

  //// RX /////
  uint8_t *_rxbuffer, *_rxflags;
  uint8_t *_rxStatePtr;
  FCS _dlfcrctype;

  int _rxStateLength, _txStateLength;
  int _imgTrunkInfoLength;
  int _imgTrunkLength;
  int _maxPacketLength;

  bool _imgInBuffer;
  bool _bigEndian;
  Timer _rxtimer;

  bool _txStateSet;
  bool _cancelLastImage;
  uint32_t _pktSeq;
  bool _checkSrcAddr;
  int _holdChannelImgTrunkSeq, _holdChannelImgSeq;
};

} // namespace telerobotics

#endif /* ROVCAMERA_H_ */
