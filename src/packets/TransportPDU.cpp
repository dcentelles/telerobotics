#include <cstring>
#include <telerobotics/TransportPDU.h>

namespace telerobotics {

int TransportPDU::OverheadSize = 1;

TransportPDU::TransportPDU() { _Init(); }

void TransportPDU::_Init() { _InitPointers(); }

void TransportPDU::_InitPointers() {
  _nseq = GetBuffer();
  _payload = _nseq + OverheadSize;
}

uint8_t TransportPDU::GetSeqNum() { return *_nseq; }

void TransportPDU::SetSeqNum(uint8_t seq) { *_nseq = seq; }

void TransportPDU::IncSeqNum() { *_nseq = *_nseq + 1; }

void TransportPDU::DoCopyFromRawBuffer(void *buffer) {}

inline uint8_t *TransportPDU::GetPayloadBuffer() { return _payload; }
inline uint32_t TransportPDU::GetPayloadSize() { return _payloadSize; }
inline int TransportPDU::GetPacketSize() { return _payloadSize + OverheadSize; }
void TransportPDU::Read(Stream *comms) {}

uint32_t TransportPDU::SetPayload(uint8_t *payload, uint32_t size) {
  _payloadSize = size;
  memcpy(_payload, payload, _payloadSize);
  return _payloadSize;
}

PacketPtr TransportPDU::Create() {
  return cpputils::CreateObject<TransportPDU>();
}
void TransportPDU::BufferUpdated() { _InitPointers(); }
void TransportPDU::SetBuffer(void *buffer) {
  _SetBuffer(buffer);
  BufferUpdated();
}
TransportPDUPtr TransportPDU::BuildTransportPDU() {
  return TransportPDUPtr(new TransportPDU());
}
} // namespace telerobotics
