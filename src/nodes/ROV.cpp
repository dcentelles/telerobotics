/*
 * ROVCamera.cpp
 *
 *  Created on: 27 oct. 2016
 *      Author: diego
 */

#include <dccomms/Checksum.h>
#include <telerobotics/LogUtils.h>
#include <telerobotics/ROV.h>

namespace telerobotics {

using namespace dccomms;
void defaultLastImageSentCallback(ROV &rovcamera) {
  // Nothing to do
}

void defaultOrdersReceivedCallback(ROV &rovcamera) {
  // Nothing to do
}

ROV::ROV() : _commsWorker(this), _holdChannelCommsWorker(this) {
  _SetEndianess();
  _rxStateLength = MAX_NODE_STATE_LENGTH;
  _txStateLength = MAX_NODE_STATE_LENGTH;
  _imgTrunkInfoLength = MSG_INFO_SIZE;
  _imgTrunkLength = MAX_IMG_TRUNK_LENGTH;
  _maxPacketLength = MAX_PACKET_LENGTH;

  _dlfcrctype = CRC16;
  _buffer = new uint8_t[_maxPacketLength * 2 + MAX_IMG_SIZE + IMG_CHKSUM_SIZE +
                        MAX_IMG_SIZE]; // buffer max size is orientative...
  _rxStateBegin = _buffer;
  _txStateBegin = _rxStateBegin + MAX_PACKET_LENGTH;
  _beginImgPtr = _txStateBegin + MAX_PACKET_LENGTH;
  _imgInBuffer = false;
  _lastImageSentCallback = &defaultLastImageSentCallback;
  _ordersReceivedCallback = &defaultOrdersReceivedCallback;

  _commsWorker.SetWork(&ROV::_Work);
  _holdChannelCommsWorker.SetWork(&ROV::_HoldChannelWork);
  SetLogName("ROV");
  _txStateSet = false;
  SetEnsureImgDelivery(true);
  _cancelLastImage = false;
  _pktSeq = 0;
  _checkSrcAddr = true;
}

ROV::~ROV() {
  // TODO Auto-generated destructor stub
  if (_commsWorker.IsRunning())
    _commsWorker.Stop();
  delete _buffer;
}

void ROV::SetEnableSrcAddrCheck(bool enable) { _checkSrcAddr = enable; }

void ROV::SetImageTrunkLength(int _len) {
  _len = _len <= MAX_IMG_TRUNK_LENGTH ? _len : MAX_IMG_TRUNK_LENGTH;
  _imgTrunkLength = _len;
  Log->debug("Set a new maximum image trunk length: {} bytes", _len);
}

void ROV::SetChecksumType(FCS fcs) { _dlfcrctype = fcs; }

void ROV::SendImage(void *_buf, unsigned int _length) {
  // TODO: aply a hash to de img in order to check errors when reassembling
  // trunks
  Log->info("TX IMG REQ");
  std::unique_lock<std::mutex> lock(_immutex);
  while (_imgInBuffer) {
    _imgInBufferCond.wait(lock);
  }
  memcpy(_beginImgPtr, _buf, _length);
  _imgChksumPtr = (uint16_t *)(_beginImgPtr + _length);
  _endImgPtr = ((uint8_t *)_imgChksumPtr) + IMG_CHKSUM_SIZE;

  uint16_t imgChksum = Checksum::crc16(_beginImgPtr, _length);
  if (_bigEndian) {
    *_imgChksumPtr = imgChksum;
  } else {
    Utils::IntSwitchEndian(_imgChksumPtr, imgChksum);
  }

  // TEMPORAL CHECK
  uint32_t crc = Checksum::crc16(_beginImgPtr, _length + IMG_CHKSUM_SIZE);
  if (crc != 0) {
    Log->critical("image corrupted before transmission?");
  }

  _imgInBuffer = true;
  lock.unlock();
  _imgInBufferCond.notify_all();
  Log->info("TX IMG {}", _length);

  // mutex is unlocked automatically when calling the unique_lock destructor:
  // http://www.cplusplus.com/reference/mutex/unique_lock/
}

void ROV::SetLastImgSentCallback(f_notification _callback) {
  _immutex.lock();
  _lastImageSentCallback = _callback;
  _immutex.unlock();
}

void ROV::SetOrdersReceivedCallback(f_notification _callback) {
  _rxstatemutex.lock();
  _ordersReceivedCallback = _callback;
  _rxstatemutex.unlock();
}

bool ROV::SendingCurrentImage() { return _imgInBuffer; }

void ROV::CancelLastImage() {
  std::unique_lock<std::mutex> lock(_immutex);
  Log->warn("CANCEL LAST IMAGE");
  _cancelLastImage = true;
  _ReinitImageFlags();
}

void ROV::SetComms(Ptr<CommsDevice> comms) { _comms = comms; }

void ROV::Start() {
  _txdlf = CreateObject<WAFrame>();
  _rxdlf = CreateObject<WAFrame>();

  _txbuffer = _txdlf->GetPayloadBuffer();
  _rxbuffer = _rxdlf->GetPayloadBuffer();

  _txMsgInfoPtr = _txbuffer;
  _txStatePtr = _txMsgInfoPtr + MSG_INFO_SIZE;
  _rxflags = _rxbuffer;
  _rxStatePtr = _rxflags + 1;

  _endImgPtr = _beginImgPtr; // No image in buffer

  _holdChannel = false;

  _commsWorker.Start();
  _holdChannelCommsWorker.Start();
}

void ROV::_WaitForNewOrders() {
  int lastImgSeq = _GetRequestedImgSeq();
  _comms >> _rxdlf;
  if (_rxdlf->IsOk()) {
    auto srcAddr = _rxdlf->GetSrc();
    Log->info("RX FROM {} SEQ {} SIZE {}", srcAddr, _rxdlf->GetSeq(),
              _rxdlf->GetPacketSize());
    info_packet(Log, _rxdlf);
    if (srcAddr == 2 || !_checkSrcAddr) {
      auto psize = _rxdlf->GetPayloadSize();
      if (psize < 1) {
        Log->critical(
            "rx payload must be greater than 1 (first byte = rx flags)");
        return;
      }
      int reqImgSeq = _GetRequestedImgSeq();
      if (_LastImageCancelled()) {
        Log->warn("LAST IMAGE CANCELLED");
        _cancelLastImage = false;
      }
      Log->info("RX IMSEQ {} (L. {})", reqImgSeq, lastImgSeq);
      if (lastImgSeq !=
          reqImgSeq) { // This means last image was successfully received
        Log->info("RX IMG OK");
        _lastImageSentCallback(*this);
        _ReinitImageFlags();
      }
      _rxStateLength = psize - 1;
      _UpdateCurrentRxStateFromRxState();
      _ordersReceivedCallback(*this);
    }
  } else {
    Log->warn("ERR PKT {}", _rxdlf->GetPacketSize());
    info_packet(Log, _rxdlf);
  }
}

int ROV::_GetRequestedImgSeq() { return *_rxflags & OP_IMG_SEQ ? 1 : 0; }

bool ROV::_LastImageCancelled() { return *_rxflags & OP_IMG_CANCEL; }

int ROV::_GetRequestedImgTrunkSeq() {
  return *_rxflags & OP_IMG_REQTRUNKSEQ_MASK;
}

void ROV::_Work() {
  _WaitForNewOrders();
  std::unique_lock<std::mutex> lock(_immutex);
  _SendPacketWithCurrentStateAndImgTrunk(lock);
}

void ROV::HoldChannel(bool v) {
  _immutex.lock();
  _ReinitImageFlags();
  _holdChannel = v;
  _holdChannel_cond.notify_one();
  if (_holdChannel) {
    _holdChannelImgSeq = _GetRequestedImgSeq();
    _holdChannelImgTrunkSeq = _GetRequestedImgTrunkSeq();
  }
  _immutex.unlock();
}

void ROV::_HoldChannelWork() {
  std::unique_lock<std::mutex> holdChanneLock(_holdChannel_mutex);
  while (!_holdChannel)
    _holdChannel_cond.wait(holdChanneLock);

  std::unique_lock<std::mutex> lock(_immutex);
  while (!_imgInBuffer) {
    _imgInBufferCond.wait(lock);
  }
  if (_holdChannel) {
    _SendPacketWithCurrentStateAndImgTrunk(lock, true);
  }
}
void ROV::_UpdateCurrentRxStateFromRxState() {
  _rxstatemutex.lock();
  memcpy(_rxStateBegin, _rxStatePtr, _rxStateLength);
  _rxstatemutex.unlock();
}
void ROV::_UpdateTxStateFromCurrentTxState() {
  _txstatemutex.lock();
  _UpdateTxStateSizeOnMsgInfo();
  memcpy(_txStatePtr, _txStateBegin, _txStateLength);
  _imgTrunkPtr = _txStatePtr + _txStateLength;
  _txstatemutex.unlock();
}

void ROV::_UpdateTrunkFlagsOnMsgInfo(uint8_t flags) {
  *_txMsgInfoPtr = (*_txMsgInfoPtr & MSG_STATE_SIZE_MASK) | flags;
}

void ROV::_UpdateTxStateSizeOnMsgInfo() {
  *_txMsgInfoPtr = (*_txMsgInfoPtr & ~MSG_STATE_SIZE_MASK) | _txStateLength;
}

uint8_t ROV::_GetTxStateSizeFromMsgInfo() {
  return *_txMsgInfoPtr & MSG_STATE_SIZE_MASK;
}

void ROV::GetCurrentRxState(void *dst) {
  _rxstatemutex.lock();
  memcpy(dst, _rxStateBegin, _rxStateLength);
  _rxstatemutex.unlock();
}

void ROV::SetCurrentTxState(void *src, uint32_t length) {
  _txstatemutex.lock();
  _txStateLength = length;
  memcpy(_txStateBegin, src, _txStateLength);
  _txstatemutex.unlock();
  _txStateSet = true;
}

void ROV::SetEnsureImgDelivery(bool v) {
  _ReinitImageFlags();
  _ensureDelivery = v;
}

bool ROV::EnsureDelivery() { return _ensureDelivery; }

void ROV::_SendPacketWithCurrentStateAndImgTrunk(
    std::unique_lock<std::mutex> &lock, bool block) {
  if (_comms->BusyTransmitting()) {
    if (!_holdChannel)
      Log->critical("TX: possible bug: device busy transmitting before next "
                    "packet build");
    return;
  }
  uint8_t trunkFlags = 0;
  if (_txStateSet) {
    if (!_imgInBuffer) {
      Log->info("TX NO IMG IN BUFFER");
      _imgInBufferCond.wait_for(lock, 100ms);
    }
    _UpdateTxStateFromCurrentTxState();

    int payloadSize = 0;
    int nextTrunkLength = 0;

    *_imgTrunkPtr = 0;

    if (_cancelLastImage) {
      Log->info("SET CANCEL IMG FLAG");
      *_imgTrunkPtr |= IMG_CANCEL;
    } else {
      *_imgTrunkPtr &= ~IMG_CANCEL;
    }

    int reqImgTrunkSeq;
    reqImgTrunkSeq =
        _holdChannel ? _holdChannelImgTrunkSeq : _GetRequestedImgTrunkSeq();
    auto imgTrunkSeq = reqImgTrunkSeq;
    int imgSize = _endImgPtr - _beginImgPtr;
    // check if requested sequence number is possible
    int imgPrefix = _imgTrunkLength * imgTrunkSeq;
    if (imgPrefix >= imgSize) {
      // if not, we send the last trunk. This will notify the receiver that
      // he was requesting a trunk sequence soo high (this may happen if the
      // image
      // size is reduced)
      imgTrunkSeq = imgSize / _imgTrunkLength;
      if (imgTrunkSeq * _imgTrunkLength == imgSize)
        imgTrunkSeq -= 1;
    }

    uint8_t *imgTrunkPtr = imgTrunkSeq * _imgTrunkLength + _beginImgPtr;

    if (_imgInBuffer) {

      int bytesLeft = _endImgPtr - imgTrunkPtr;
      if (bytesLeft > _imgTrunkLength)
        nextTrunkLength = _imgTrunkLength;
      else {
        trunkFlags |= IMG_LAST_TRUNK_FLAG;
        nextTrunkLength = bytesLeft;
      }
      if (imgTrunkSeq == 0) {
        trunkFlags |= IMG_FIRST_TRUNK_FLAG;
      }
      Log->info("TX IMGSIZE {} : IMGSEQ {} (R. {}) : TSIZE {}", imgSize,
                imgTrunkSeq, reqImgTrunkSeq, nextTrunkLength);

      memcpy(_imgTrunkPtr + 1, imgTrunkPtr, nextTrunkLength);

      if (!_holdChannel)
        *_imgTrunkPtr |= (_GetRequestedImgSeq() ? IMG_SEQ : 0);
      else
        *_imgTrunkPtr |= (_holdChannelImgSeq ? IMG_SEQ : 0);

      *_imgTrunkPtr |= IMG_TRUNK_SEQ_MASK & imgTrunkSeq;

      if (_holdChannel) {
        if (trunkFlags & IMG_LAST_TRUNK_FLAG) {
          _imgInBuffer = false;
          _holdChannelImgSeq = _holdChannelImgSeq ? 0 : 1;
          _endImgPtr = _beginImgPtr;
          _lastImageSentCallback(*this);
          _imgInBufferCond.notify_all();
          _holdChannelImgTrunkSeq = 0;
        } else {
          _holdChannelImgTrunkSeq++;
        }
      }
    }
    payloadSize =
        MSG_INFO_SIZE + _GetTxStateSizeFromMsgInfo() + 1 + nextTrunkLength;

    _UpdateTrunkFlagsOnMsgInfo(trunkFlags);
    _txdlf->PayloadUpdated(payloadSize);
    _txdlf->SetSrc(1);
    _txdlf->SetDst(2);
    _txdlf->SetSeq(_pktSeq++);
    _txdlf->UpdateFCS();
    Log->info("TX TO 2 SEQ {} SIZE {}", _txdlf->GetSeq(),
              _txdlf->GetPacketSize());
    *_comms << _txdlf;
    info_packet(Log, _txdlf);
    if (block) {
      auto lastPktSize = _txdlf->GetPacketSize();
      lock.unlock();
      unsigned long nanos = lastPktSize * 8 / 1850. * 1e9;
      std::this_thread::sleep_for(chrono::nanoseconds(nanos));
      lock.lock();
    }
  } else {
    Log->warn("TX: current state is not set yet");
    Utils::Sleep(1000);
  }
}

void ROV::_ReinitImageFlags() {
  _imgInBuffer = false;
  _endImgPtr = _beginImgPtr;
  _holdChannelImgTrunkSeq = 0;
  _holdChannelImgSeq = 0;
  _imgInBufferCond.notify_all();
}

void ROV::_SetEndianess() { _bigEndian = DataLinkFrame::IsBigEndian(); }

} // namespace telerobotics
