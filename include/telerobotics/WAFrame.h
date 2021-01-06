/*
 * This packet is a copy of wireless_ardusub::SimplePacket (wireless_ardusub
 * project)
 */

#ifndef TELEROBOTICS_WAFRAME_H_
#define TELEROBOTICS_WAFRAME_H_

#include <dccomms/dccomms.h>
#include <dccomms_packets/types.h>

using namespace dccomms;
namespace telerobotics {

class WAFrame : public dccomms::Packet {
public:
  WAFrame();
  void DoCopyFromRawBuffer(void *buffer);
  uint8_t *GetPayloadBuffer();
  uint32_t GetPayloadSize();
  int GetPacketSize();
  void Read(Stream *comms);
  void PayloadUpdated(uint32_t payloadSize);

  bool IsOk();
  PacketPtr Create();

  void GetPayload(void *copy, int size);
  uint32_t SetPayload(uint8_t *data, uint32_t size);

  void UpdateFCS();

  static const int MAX_PAYLOAD_SIZE = UINT8_MAX, PAYLOAD_SIZE_FIELD = 1,
                   FCS_SIZE = 2; // CRC16
  static const int PRE_SIZE = 3;

private:

  uint8_t *_pre;
  uint8_t *_payloadSize;
  uint8_t *_payload;
  uint8_t *_fcs;
  int _maxPacketSize;
  int _overheadSize;
  void _Init();
  bool _CheckFCS();
};

class WAFrameBuilder : public IPacketBuilder {
public:
  dccomms::PacketPtr CreateFromBuffer(void *buffer) {
    auto pkt = dccomms::CreateObject<WAFrame>();
    pkt->CopyFromRawBuffer(buffer);
    return pkt;
  }
  dccomms::PacketPtr Create() { return dccomms::CreateObject<WAFrame>(); }
};
}

#endif // TELEROBOTICS_WAFRAME_H_
