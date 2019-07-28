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
  cancelling = false;
  _pktSeq = 0;
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

  txFlags = txbuffer;
  txStatePtr = txFlags + 1;

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
    auto nanos = (uint32_t)(lastPktSize * 8 / 50. * 1e9);
    std::this_thread::sleep_for(chrono::nanoseconds(nanos));
  } else
    std::this_thread::sleep_for(chrono::milliseconds(50));
}

void Operator::_RxWork() {
  Log->debug("RX: waiting for new state from ROV...");
  _WaitForCurrentStateAndNextImageTrunk(0);
}

bool Operator::_HaveImgTrunk(int i) { return receivedTrunksFlags & (1 << i); }

void Operator::_MarkImgTrunk(int i) { receivedTrunksFlags |= (uint64_t)1 << i; }

int Operator::_ImgReceptionCompleted() {
  // returns number of trunks if completed, otherwise returns 0
  int i;
  for (i = 0; i < 64; i++) {
    if (!_HaveImgTrunk(i))
      break;
  }

  if (i > 0) {
    for (int b = i; b < 64; b++) {
      if (_HaveImgTrunk(b))
        return 0; // todavia hay 0's entre 1's
      return i;
    }
  } else
    return 0;
}

void Operator::_MarkLastImgTrunk(int l) {
  receivedTrunksFlags &= ~0 >> (63 - l);
}

int Operator::_GetReqImgSeq() { return *txFlags & OP_IMG_SEQ ? 1 : 0; }

void Operator::_ChangeImgSeq() {
  if (_GetReqImgSeq())
    *txFlags &= ~OP_IMG_SEQ;
  else
    *txFlags |= OP_IMG_SEQ;
}

void Operator::_UpdateTrunkSeq() {
  *txFlags &= ~OP_IMG_REQTRUNKSEQ_MASK;
  for (uint64_t i = 0; i < 64; i++) {
    if (!(receivedTrunksFlags & ((uint64_t)1 << i))) {
      *txFlags |= i;
      break;
    }
  }
}

void Operator::_WaitForCurrentStateAndNextImageTrunk(int timeout) {
  // Wait for the next packet and call the callback
  Log->debug("RX: waiting for frames...");
  *_comms >> rxdlf;
  if (rxdlf->PacketIsOk()) {
    auto srcAddr = rxdlf->GetSrcAddr();
    Log->info("RX FROM {} SEQ {} SIZE {}", srcAddr, rxdlf->GetSeq(),
              rxdlf->GetPacketSize());
    if (srcAddr == 1) {
      Log->info("RX PKT {}", rxdlf->GetPacketSize());

      msgInfo = *rxMsgInfoPtr;
      rxStateLength = _GetStateSize(msgInfo);

      _UpdateLastConfirmedStateFromLastMsg();

      uint8_t *rxTrunkInfo = rxStatePtr + rxStateLength;
      if (*rxTrunkInfo & IMG_CANCEL) {
        Log->info("CANCEL LAST RECEIVED IMG");
        // Reinit image slots
        if (!cancelling) {
          Log->info("CANCELLING...");
          receivedTrunksFlags = 0;
          _UpdateTrunkSeq();
          _ChangeImgSeq();
          *txFlags |= OP_IMG_CANCEL;
          cancelling = true;
        }
      } else {
        cancelling = false;
        *txFlags &= ~OP_IMG_CANCEL;
        int receivedImgSeq = *rxTrunkInfo & IMG_SEQ ? 1 : 0;
        int imgSeq = _GetReqImgSeq();
        if (imgSeq == receivedImgSeq) {
          int receivedImgTrunkSeq = *rxTrunkInfo & IMG_TRUNK_SEQ_MASK;
          trunkSize =
              rxdlf->GetPayloadSize() - rxStateLength - MSG_INFO_SIZE - 1;
          if (trunkSize > 0) {
            imgTrunkPtr = rxStatePtr + rxStateLength + 1;
            int lastReqTrunkSeq = *txFlags & OP_IMG_REQTRUNKSEQ_MASK;
            bool firstTrunk, lastTrunk;
            if (firstTrunk = msgInfo & IMG_FIRST_TRUNK_FLAG) {
              baseTrunkSize = trunkSize;
              baseTrunkSizeSet = true;
            }
            if (baseTrunkSizeSet) {
              memcpy(beginImgPtr + (receivedImgTrunkSeq * baseTrunkSize),
                     imgTrunkPtr, trunkSize);
              _MarkImgTrunk(receivedImgTrunkSeq);
              _UpdateTrunkSeq();
            }
            if (lastTrunk = msgInfo & IMG_LAST_TRUNK_FLAG) {
              lastTrunkReceived = true;
              lastTrunkSize = trunkSize;
              _MarkLastImgTrunk(receivedImgTrunkSeq);
            }
            if (firstTrunk) {
              if (lastTrunk) {
                Log->info(
                    "RX FIRSTLAST TRUNK {} ; IMSEQ {} (E. {}) ; SEQ {} (E. {})",
                    trunkSize, receivedImgSeq, imgSeq, receivedImgTrunkSeq,
                    lastReqTrunkSeq);
              } else {
                Log->info(
                    "RX FIRST TRUNK {} ; IMSEQ {} (E. {}) ; SEQ {} (E. {})",
                    trunkSize, receivedImgSeq, imgSeq, receivedImgTrunkSeq,
                    lastReqTrunkSeq);
              }
            } else if (lastTrunk) {
              Log->info("RX LAST TRUNK {} ; IMSEQ {} (E. {}) ; SEQ {} (E. {})",
                        trunkSize, receivedImgSeq, imgSeq, receivedImgTrunkSeq,
                        lastReqTrunkSeq);
            } else {
              Log->info("RX INTER TRUNK {} ; IMSEQ {} (E. {}) ; SEQ {} (E. {})",
                        trunkSize, receivedImgSeq, imgSeq, receivedImgTrunkSeq,
                        lastReqTrunkSeq);
            }
            if (lastTrunkReceived) {
              int ntrunks = _ImgReceptionCompleted();
              if (ntrunks > 0) {
                int blockSize = (ntrunks - 1) * baseTrunkSize + lastTrunkSize;
                uint16_t crc = Checksum::crc16(beginImgPtr, blockSize);
                if (crc == 0) {
                  immutex.lock();
                  lastImgSize = blockSize - IMG_CHKSUM_SIZE;
                  Log->info("RX IMG {}", lastImgSize);
                  memcpy(beginImgPtr, beginImgPtr, lastImgSize);
                  immutex.unlock();

                  imageReceivedCallback(*this);
                } else {
                  Log->warn("ERROR ON DECODING IMAGE. THIS CAN ONLY HAVE "
                            "HAPPENED IF THE SIZE OF THE IMAGE HAS CHANGED. "
                            "RESETTING TRUNK FLAGS... {}",
                            lastImgSize);
                }
                baseTrunkSizeSet = false;
                baseTrunkSize = 0;
                lastTrunkReceived = false;
                lastTrunkSize = 0;
                receivedTrunksFlags = 0;
                _ChangeImgSeq();
                _UpdateTrunkSeq();
              }
            }
          } else {
            Log->info("RX NO IMG");
          }
        }
      }
      stateReceivedCallback(*this);
    } else {
      // Data packet received from another node
    }

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
    Log->info("RX IMG {}", lastImgSize);
    memcpy(beginImgPtr, beginImgPtr, lastImgSize);
    immutex.unlock();

    imageReceivedCallback(*this);
  } else {
    Log->warn("ERR IMG {}", lastImgSize);
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

      txdlf->PayloadUpdated(1 + txStateLength);
      txdlf->SetSrcAddr(2);
      txdlf->SetDestAddr(1);
      txdlf->SetSeq(_pktSeq++);
      txdlf->UpdateFCS();
      Log->info("TX TO 0 SEQ {} SIZE {}", txdlf->GetSeq(),
                txdlf->GetPacketSize());
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

} // namespace telerobotics
