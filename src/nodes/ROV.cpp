/*
 * ROVCamera.cpp
 *
 *  Created on: 27 oct. 2016
 *      Author: diego
 */

#include <dccomms/Checksum.h>
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
}

ROV::~ROV() {
  // TODO Auto-generated destructor stub
  if (_commsWorker.IsRunning())
    _commsWorker.Stop();
  delete _buffer;
}

void ROV::SetImageTrunkLength(int _len) {
  _len = _len <= MAX_IMG_TRUNK_LENGTH ? _len : MAX_IMG_TRUNK_LENGTH;
  _imgTrunkLength = _len;
  Log->debug("Set a new maximum image trunk length: {} bytes", _len);
}

void ROV::SetChecksumType(FCS fcs) { _dlfcrctype = fcs; }

void ROV::SendImage(void *_buf, unsigned int _length) {
  // TODO: aply a hash to de img in order to check errors when reassembling
  // trunks
  std::unique_lock<std::mutex> lock(_immutex);
  while (_imgInBuffer) {
    _imgInBufferCond.wait(lock);
  }
  memcpy(_beginImgPtr, _buf, _length);
  _imgChksumPtr = (uint16_t *)(_beginImgPtr + _length);
  _endImgPtr = ((uint8_t *)_imgChksumPtr) + IMG_CHKSUM_SIZE;
  _currentImgPtr = _beginImgPtr;

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
  Log->debug("New image available to transmit ({} bytes).", _length);

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
  _ReinitImageFlags();
}

void ROV::SetComms(Ptr<CommsDevice> comms) { _comms = comms; }

void ROV::Start() {
  _txdlf = CreateObject<VariableLengthPacket>();
  _rxdlf = CreateObject<VariableLengthPacket>();

  _txbuffer = _txdlf->GetPayloadBuffer();
  _rxbuffer = _rxdlf->GetPayloadBuffer();

  _txMsgInfoPtr = _txbuffer;
  _txStatePtr = _txMsgInfoPtr + MSG_INFO_SIZE;
  _rxStatePtr = _rxbuffer;

  _currentImgPtr = _beginImgPtr; // No image in buffer
  _endImgPtr = _currentImgPtr;   //

  _holdChannel = false;

  _commsWorker.Start();
  _holdChannelCommsWorker.Start();
}

void ROV::_WaitForNewOrders() {
  _comms >> _rxdlf;
  if (_rxdlf->PacketIsOk()) {
    Log->info("Packet received ({} bytes)", _rxdlf->GetPacketSize());
    _rxStateLength = _rxdlf->GetPayloadSize();
    _UpdateCurrentRxStateFromRxState();
    _ordersReceivedCallback(*this);
  } else {
    Log->warn("Packet received with errors ({} bytes)",
              _rxdlf->GetPacketSize());
  }
}
void ROV::_Work() {
  _WaitForNewOrders();
  _immutex.lock();
  _SendPacketWithCurrentStateAndImgTrunk();
  _CheckIfEntireImgIsSent();
  _immutex.unlock();
}

void ROV::HoldChannel(bool v) {
  _immutex.lock();
  _ReinitImageFlags();
  _holdChannel = v;
  _holdChannel_cond.notify_one();
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
    _SendPacketWithCurrentStateAndImgTrunk(true);
    _CheckIfEntireImgIsSent();
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

void ROV::_SendPacketWithCurrentStateAndImgTrunk(bool block) {
  if (_comms->BusyTransmitting()) {
    if (!_holdChannel)
      Log->critical("TX: possible bug: device busy transmitting before next "
                    "packet build");
    return;
  }
  uint8_t trunkFlags = 0;
  if (_txStateSet) {
    int bytesLeft = _endImgPtr - _currentImgPtr;
    int nextTrunkLength = 0;

    _UpdateTxStateFromCurrentTxState();

    if (bytesLeft > 0) // == (ImgInBuffer == True)
    {
      if (bytesLeft > _imgTrunkLength) {
        nextTrunkLength = _imgTrunkLength;
      } else {
        trunkFlags |= IMG_LAST_TRUNK_FLAG;
        nextTrunkLength = bytesLeft;
      }
      if (_beginImgPtr == _currentImgPtr)
        trunkFlags |= IMG_FIRST_TRUNK_FLAG;

      memcpy(_imgTrunkPtr, _currentImgPtr, nextTrunkLength);
      _currentImgPtr += nextTrunkLength;
    }
    _UpdateTrunkFlagsOnMsgInfo(trunkFlags);
    _txdlf->PayloadUpdated(MSG_INFO_SIZE + _GetTxStateSizeFromMsgInfo() +
                           nextTrunkLength);
    _txdlf->UpdateFCS();
    Log->info("Sending packet ({} bytes)", _txdlf->GetPacketSize());
    *_comms << _txdlf;
    if (block) {
      auto lastPktSize = _txdlf->GetPacketSize();
      auto nanos = (uint32_t)(lastPktSize * 8 / 1850. * 1e9);
      std::this_thread::sleep_for(chrono::nanoseconds(nanos));
    }
  } else {
    Log->warn("TX: current state is not set yet");
    Utils::Sleep(1000);
  }
}

void ROV::_ReinitImageFlags() {
  _imgInBuffer = false;
  _currentImgPtr = _beginImgPtr;
  _endImgPtr = _currentImgPtr;
  _imgInBufferCond.notify_all();
}
void ROV::_CheckIfEntireImgIsSent() {
  // Check If it has been sent the last image trunk and call the callback
  if (_imgInBuffer && _currentImgPtr == _endImgPtr) {
    _ReinitImageFlags();
    _lastImageSentCallback(*this);
    Log->debug("TX: image transmission completed");
  }
}

void ROV::_SetEndianess() { _bigEndian = DataLinkFrame::IsBigEndian(); }

} /* namespace dcauv */
