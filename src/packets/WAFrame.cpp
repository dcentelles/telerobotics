#include <class_loader/multi_library_class_loader.hpp>
#include <telerobotics/WAFrame.h>

namespace telerobotics {
WAFrame::WAFrame() {
  _headerSize = PRE_SIZE + PAYLOAD_SIZE_FIELD;
  _overheadSize = _headerSize + FCS_SIZE;
  _maxPacketSize = _overheadSize + MAX_PAYLOAD_SIZE;
  _AllocBuffer(_maxPacketSize);
  _Init();
}

void WAFrame::_Init() {
  _pre = GetBuffer();
  memset(_pre, 0x55, PRE_SIZE);
  _payloadSize = _pre + PRE_SIZE;
  *_payloadSize = 0;
  _payload = _payloadSize + 1;
  _fcs = _payload + *_payloadSize;
}

void WAFrame::DoCopyFromRawBuffer(void *buffer) {
  uint8_t payloadSize = *((uint8_t *)buffer + PRE_SIZE);
  memcpy(GetBuffer(), buffer, payloadSize + _overheadSize);
}

inline uint8_t *WAFrame::GetPayloadBuffer() { return _payload; }

inline uint32_t WAFrame::GetPayloadSize() { return *_payloadSize; }

inline int WAFrame::GetPacketSize() { return _overheadSize + *_payloadSize; }

void WAFrame::Read(Stream *stream) {
  stream->WaitFor(_pre, PRE_SIZE);
  stream->Read(_payloadSize, 1);
  stream->Read(_payload, *_payloadSize + FCS_SIZE);
}

void WAFrame::PayloadUpdated(uint32_t payloadSize) {
  *_payloadSize = payloadSize;
  _fcs = _payload + *_payloadSize;
  UpdateFCS();
}

void WAFrame::GetPayload(void *copy, int size) {
  auto copySize = *_payloadSize < size ? *_payloadSize : size;
  memcpy(copy, _payload, copySize);
}

uint32_t WAFrame::SetPayload(uint8_t *data, uint32_t size) {
  auto copySize = MAX_PAYLOAD_SIZE < size ? MAX_PAYLOAD_SIZE : size;
  *_payloadSize = size;
  memcpy(_payload, data, copySize);
  _fcs = _payload + *_payloadSize;
  return copySize;
}

void WAFrame::UpdateFCS() {
  uint16_t crc = Checksum::crc16(GetBuffer(), _headerSize + *_payloadSize);
  *_fcs = (uint8_t)(crc >> 8);
  *(_fcs + 1) = (uint8_t)(crc & 0xff);
}

bool WAFrame::_CheckFCS() {
  uint16_t crc =
      Checksum::crc16(GetBuffer(), _headerSize + *_payloadSize + FCS_SIZE);
  return crc == 0;
}
bool WAFrame::IsOk() { return _CheckFCS(); }
PacketPtr WAFrame::Create() { return CreateObject<WAFrame>(); }
CLASS_LOADER_REGISTER_CLASS(WAFrameBuilder, IPacketBuilder)
} // namespace telerobotics
