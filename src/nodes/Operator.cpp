/*
 * ROVOperator.cpp
 *
 *  Created on: 27 oct. 2016
 *      Author: diego
 */

#include <dccomms/Checksum.h>
#include <dccomms/DataLinkFrame.h>
#include <telerobotics/Operator.h>

namespace telerobotics {

using namespace dccomms;

void defaultImageReceivedCallback(Operator &rovOperator) {}

void defaultStateReceivedCallback(Operator &rovOperator) {}

Operator::Operator() : txservice(this), rxservice(this) {
  rxbuffer = 0;
  imgTrunkPtr = 0;
  txbuffer = 0;
  rxMsgInfoPtr = 0;
  currentImgPtr = 0;
  rxStatePtr = 0;
  txStatePtr = 0;

  bigEndian = DataLinkFrame::IsBigEndian();
  imgTrunkInfoLength = MSG_INFO_SIZE;
  imageTrunkLength = MAX_IMG_TRUNK_LENGTH;
  maxPacketLength = MAX_PACKET_LENGTH;

  dlfcrctype = CRC16;
  buffer = new uint8_t[maxPacketLength * 2 + MAX_IMG_SIZE + IMG_CHKSUM_SIZE];

  rxStateBegin = buffer;
  txStateBegin = rxStateBegin + MAX_PACKET_LENGTH;
  beginImgPtr = txStateBegin + MAX_PACKET_LENGTH;

  currentImgPtr = beginImgPtr;

  lastImgSize = 0;
  imgInBuffer = false;
  txservice.SetWork(&Operator::_TxWork);
  rxservice.SetWork(&Operator::_RxWork);

  imageReceivedCallback = &defaultImageReceivedCallback;
  stateReceivedCallback = &defaultStateReceivedCallback;

  desiredStateSet = false;
  _canTransmit = true;
}

Operator::~Operator() {
  if (txservice.IsRunning())
    txservice.Stop();
  if (rxservice.IsRunning())
    rxservice.Stop();
  delete buffer;
}

void Operator::SetComms(Ptr<CommsDevice> comms) { _comms = comms; }

int Operator::GetLastReceivedImage(void *data) {
  int imgSize;
  immutex.lock();
  memcpy(data, beginImgPtr, lastImgSize);
  imgSize = lastImgSize;
  immutex.unlock();

  return imgSize;
}

void Operator::GetLastConfirmedState(void *data) {
  rxstatemutex.lock();
  memcpy(data, rxStateBegin, rxStateLength);
  rxstatemutex.unlock();
}

void Operator::SetTxState(const void *_data, uint32_t length) {
  txstatemutex.lock();
  txStateLength = length;
  memcpy(txStateBegin, _data, txStateLength);
  txstatemutex.unlock();
  desiredStateSet = true;
}

void Operator::SetImageReceivedCallback(f_notification _callback) {
  imageReceivedCallback = _callback;
}

void Operator::SetStateReceivedCallback(f_notification _callback) {
  stateReceivedCallback = _callback;
}

void Operator::Start() {
  txdlf = CreateObject<VariableLengthPacket>();
  rxdlf = CreateObject<VariableLengthPacket>();

  auto txdlbuffer = txdlf->GetPayloadBuffer();
  auto rxdlbuffer = rxdlf->GetPayloadBuffer();

  txbuffer = txdlf->GetPayloadBuffer();
  rxbuffer = rxdlf->GetPayloadBuffer();

  rxMsgInfoPtr = rxbuffer;
  rxStatePtr = rxMsgInfoPtr + MSG_INFO_SIZE;

  txStatePtr = txbuffer;

  txservice.Start();
  rxservice.Start();
}

void Operator::DisableTransmission() { _canTransmit = false; }
void Operator::EnableTransmission() { _canTransmit = true; }
void Operator::_Work() {}

void Operator::_TxWork() {
  while (!desiredStateSet) {
    std::this_thread::sleep_for(chrono::milliseconds(750));
  }
  if (_canTransmit) {
    _SendPacketWithDesiredState();
    auto lastPktSize = txdlf->GetPacketSize();
    auto nanos = (uint32_t) (lastPktSize * 8 / 200. * 1e9);
    std::this_thread::sleep_for(chrono::nanoseconds(nanos));
  } else
    std::this_thread::sleep_for(chrono::milliseconds(50));
}

void Operator::_RxWork() {
  Log->debug("RX: waiting for new state from ROV...");
  _WaitForCurrentStateAndNextImageTrunk(0);
}

void Operator::_WaitForCurrentStateAndNextImageTrunk(int timeout) {
  // Wait for the next packet and call the callback
  Log->debug("RX: waiting for frames...");
  *_comms >> rxdlf;
  if (rxdlf->PacketIsOk()) {
    Log->info("RX PKT {}", rxdlf->GetPacketSize());

    msgInfo = *rxMsgInfoPtr;
    rxStateLength = _GetStateSize(msgInfo);
    trunkSize = rxdlf->GetPayloadSize() - rxStateLength - MSG_INFO_SIZE;

    imgTrunkPtr = rxStatePtr + rxStateLength;

    _UpdateLastConfirmedStateFromLastMsg();
    _UpdateImgBufferFromLastMsg();
    stateReceivedCallback(*this);
  } else {
    Log->warn("ERR PKT {}", rxdlf->GetPacketSize());
  }
}

void Operator::_UpdateImgBufferFromLastMsg() {
  if (msgInfo != 0 && trunkSize > 0) {
    if (msgInfo & IMG_FIRST_TRUNK_FLAG) // the received trunk is the first
                                        // trunk of an image
    {
      Log->debug("RX: the received trunk is the first trunk of an image");
      memcpy(beginImgPtr, imgTrunkPtr, trunkSize);
      currentImgPtr = beginImgPtr + trunkSize;
      if (msgInfo & IMG_LAST_TRUNK_FLAG) // the received trunk is also the
                                         // last of an image (the image only
                                         // has 1 trunk)
      {
        Log->debug("RX: the received trunk is also the last of an image (the "
                   "image only has 1 trunk)");
        _LastTrunkReceived(trunkSize);
      }
    } else // the received trunk is not the first of an image
    {
      if (currentImgPtr != beginImgPtr) // first trunk has already been received
      {
        memcpy(currentImgPtr, imgTrunkPtr, trunkSize);
        currentImgPtr += trunkSize;
        if (msgInfo &
            IMG_LAST_TRUNK_FLAG) // the received trunk is the last of an image
        {
          Log->debug("RX: the received trunk is the last of an image");
          _LastTrunkReceived(trunkSize);
        }
      } else {
        // else, we are waiting for the first trunk of an image
        Log->debug("RX: waiting for the first trunk of an image");
      }
    }
  } else {
    // else, packet without an image trunk
    Log->debug("RX: packet received without an image trunk");
  }
}
void Operator::_LastTrunkReceived(uint8_t trunkSize) {
  int blockSize = currentImgPtr - beginImgPtr;

  currentImgPtr = beginImgPtr;
  uint16_t crc = Checksum::crc16(beginImgPtr, blockSize);
  if (crc == 0) {
    immutex.lock();
    lastImgSize = blockSize - IMG_CHKSUM_SIZE;
    memcpy(beginImgPtr, beginImgPtr, lastImgSize);
    immutex.unlock();

    imageReceivedCallback(*this);
  } else {
    Log->warn("ERR IMG");
  }
}

uint8_t Operator::_GetStateSize(uint8_t rawInfo) {
  return rawInfo & MSG_STATE_SIZE_MASK;
}

void Operator::_SendPacketWithDesiredState() {
  if (desiredStateSet) {
    if (!_comms->BusyTransmitting()) {
      txstatemutex.lock();
      memcpy(txStatePtr, txStateBegin, txStateLength);
      txstatemutex.unlock();

      txdlf->PayloadUpdated(txStateLength);
      txdlf->UpdateFCS();
      Log->info("TX PKT {}", txdlf->GetPacketSize());
      *_comms << txdlf;
      while (_comms->BusyTransmitting())
        ;
    }
  } else {
    Log->warn("TX: desired state is not set yet");
    Utils::Sleep(1000);
  }
}

void Operator::_UpdateLastConfirmedStateFromLastMsg() {
  rxstatemutex.lock();
  memcpy(rxStateBegin, rxStatePtr, rxStateLength);
  rxstatemutex.unlock();
}

} /* namespace dcauv */
