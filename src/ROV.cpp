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
  _imgTrunkInfoLength = IMG_TRUNK_INFO_SIZE;
  _maxImgTrunkLength = MAX_IMG_TRUNK_LENGTH;
  _maxPacketLength = MAX_PACKET_LENGTH;

  _dlfcrctype = CRC16;
  _buffer = new uint8_t[_maxPacketLength + MAX_NODE_STATE_LENGTH * 2 +
                        MAX_IMG_SIZE]; // buffer max size is orientative...
  _currentRxState = _buffer;
  _currentTxState = _currentRxState + _rxStateLength;
  _beginImgPtr = _currentTxState + _txStateLength;
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

void ROV::SetMaxImageTrunkLength(int _len) {
  _len = _len <= MAX_IMG_TRUNK_LENGTH ? _len : MAX_IMG_TRUNK_LENGTH;
  _maxImgTrunkLength = _len;
  Log->debug("Set a new maximum image trunk length: {} bytes", _len);
}

void ROV::SetTxStateSize(int _len) {
  _len = _len <= MAX_NODE_STATE_LENGTH ? _len : MAX_NODE_STATE_LENGTH;
  _txStateLength = _len;
  _beginImgPtr = _currentTxState + _txStateLength;
  Log->debug("Set a new Tx-State length: {} bytes", _len);
}
void ROV::SetRxStateSize(int _len) {
  _len = _len <= MAX_NODE_STATE_LENGTH ? _len : MAX_NODE_STATE_LENGTH;
  _rxStateLength = _len;
  _currentTxState = _currentRxState + _rxStateLength;
  SetTxStateSize(_txStateLength);
  Log->debug("Set a new Rx-State length: {} bytes", _len);
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
    Log->critical("data link frame with errors before transmission");
  }

  _imgInBuffer = true;
  lock.unlock();
  _imgInBufferCond.notify_one();
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

uint32_t ROV::GetTxPacketSize() {
  return _txStateLength + _imgTrunkInfoLength + _maxImgTrunkLength;
}

uint32_t ROV::GetRxPacketSize() { return _rxStateLength; }

void ROV::Start() {
  _txdlf = CreateObject<SimplePacket>(GetTxPacketSize(), _dlfcrctype);
  _rxdlf = CreateObject<SimplePacket>(GetRxPacketSize(), _dlfcrctype);

  _txbuffer = _txdlf->GetPayloadBuffer();
  _rxbuffer = _rxdlf->GetPayloadBuffer();

  _txStatePtr = _txbuffer;
  _imgTrunkInfoPtr = (uint16_t *)(_txStatePtr + _txStateLength);
  _imgTrunkPtr = ((uint8_t *)_imgTrunkInfoPtr) + IMG_TRUNK_INFO_SIZE;

  _currentImgPtr = _beginImgPtr; // No image in buffer
  _endImgPtr = _currentImgPtr;   //

  _rxStatePtr = _rxbuffer;

  _holdChannel = false;

  _commsWorker.Start();
  _holdChannelCommsWorker.Start();
}

void ROV::_WaitForNewOrders() {
  _comms >> _rxdlf;
  if (_rxdlf->PacketIsOk()) {
    Log->info("Packet received ({} bytes)", _rxdlf->GetPacketSize());
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
  _holdChannel = v;
  _holdChannel_cond.notify_one();
}

void ROV::_HoldChannelWork() {
  std::unique_lock<std::mutex> holdChanneLock(_holdChannel_mutex);
  while (!_holdChannel)
    _holdChannel_cond.wait(holdChanneLock);

  std::unique_lock<std::mutex> lock(_immutex);
  while (!_imgInBuffer) {
    _imgInBufferCond.wait(lock);
  }
  _SendPacketWithCurrentStateAndImgTrunk();
  _CheckIfEntireImgIsSent();
}
void ROV::_UpdateCurrentRxStateFromRxState() {
  _rxstatemutex.lock();
  memcpy(_currentRxState, _rxStatePtr, _rxStateLength);
  _rxstatemutex.unlock();
}
void ROV::_UpdateTxStateFromCurrentTxState() {
  _txstatemutex.lock();
  memcpy(_txStatePtr, _currentTxState, _txStateLength);
  _txstatemutex.unlock();
}

void ROV::GetCurrentRxState(void *dst) {
  _rxstatemutex.lock();
  memcpy(dst, _currentRxState, _rxStateLength);
  _rxstatemutex.unlock();
}

void ROV::SetCurrentTxState(void *src) {
  _txstatemutex.lock();
  memcpy(_currentTxState, src, _txStateLength);
  _txstatemutex.unlock();
  _txStateSet = true;
}

void ROV::_SendPacketWithCurrentStateAndImgTrunk() {
  if (_comms->BusyTransmitting()) {
    if (!_holdChannel)
      Log->critical("TX: possible bug: device busy transmitting before next packet build");
    return;
  }
  if (_txStateSet) {
    int bytesLeft = _endImgPtr - _currentImgPtr;
    int nextTrunkLength;
    uint16_t trunkInfo = 0;

    _UpdateTxStateFromCurrentTxState();

    if (bytesLeft > 0) // == (ImgInBuffer == True)
    {
      if (bytesLeft > _maxImgTrunkLength) {
        nextTrunkLength = _maxImgTrunkLength;
      } else {
        trunkInfo |= IMG_LAST_TRUNK_FLAG;
        nextTrunkLength = bytesLeft;
      }
      if (_beginImgPtr == _currentImgPtr)
        trunkInfo |= IMG_FIRST_TRUNK_FLAG;

      trunkInfo |= nextTrunkLength;

      if (_bigEndian)
        *_imgTrunkInfoPtr = trunkInfo;
      else {
        Utils::IntSwitchEndian(_imgTrunkInfoPtr, trunkInfo);
      }

      memcpy(_imgTrunkPtr, _currentImgPtr, nextTrunkLength);
      _currentImgPtr += nextTrunkLength;

      _txdlf->UpdateFCS();
    } else {
      *_imgTrunkInfoPtr = 0;
      _txdlf->UpdateFCS();
    }
    Log->info("Sending packet ({} bytes)", _txdlf->GetPacketSize());
    *_comms << _txdlf;
  } else {
    Log->warn("TX: current state is not set yet");
    Utils::Sleep(1000);
  }
}

void ROV::_ReinitImageFlags() {
  _imgInBuffer = false;
  _currentImgPtr = _beginImgPtr;
  _endImgPtr = _currentImgPtr;
  _imgInBufferCond.notify_one();
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
