#ifndef TELEROBOTICS_TRANSPORTPDU_H
#define TELEROBOTICS_TRANSPORTPDU_H

#include <dccomms/Packet.h>
#include <cpputils/Object.h>

namespace telerobotics {

using namespace dccomms;
class TransportPDU;

typedef cpputils::Ptr<TransportPDU> TransportPDUPtr;

class TransportPDU : public Packet {
public:
  static int OverheadSize;
  static TransportPDUPtr BuildTransportPDU();

  TransportPDU();
  uint8_t GetSeqNum();
  void SetSeqNum(uint8_t seq);
  void IncSeqNum();

  void SetBuffer(void *);
  void BufferUpdated();
  uint32_t SetPayload(uint8_t *pyload, uint32_t size);
  PacketPtr Create();

  void DoCopyFromRawBuffer(void *buffer);
  uint8_t *GetPayloadBuffer();
  uint32_t GetPayloadSize();
  int GetPacketSize();
  void Read(Stream *comms);
  void PayloadUpdated(uint32_t payloadSize) { _payloadSize = payloadSize; }

private:
  uint8_t *_nseq, *_payload;

  uint32_t _payloadSize;

  void _Init();
  void _InitPointers();
};
}

#endif // TELEROBOTICS_TRANSPORTPDU_H
