/*
 * HROVOrders.h
 *
 *  Created on: 30 nov. 2016
 *      Author: centelld
 */

#ifndef INCLUDE_TELEROBOTICS_HROVMessageV2V2_H_
#define INCLUDE_TELEROBOTICS_HROVMessageV2V2_H_

#include <telerobotics/Constants.h>

namespace telerobotics {

class HROVMessageV2;

typedef std::shared_ptr<HROVMessageV2> HROVMessageV2Ptr;

class HROVMessageV2 {
public:
  const static uint8_t MessageLength;
  HROVMessageV2();
  HROVMessageV2(uint8_t *);
  virtual ~HROVMessageV2();

  static HROVMessageV2Ptr BuildHROVMessageV2() {
    return HROVMessageV2Ptr(new HROVMessageV2());
  }
  static HROVMessageV2Ptr BuildHROVMessageV2(uint8_t *_buffer) {
    return HROVMessageV2Ptr(new HROVMessageV2(_buffer));
  }

  void UpdateFromBuffer(uint8_t *);
  void GetBufferCopy(uint8_t *);

  uint8_t *GetBuffer() { return buffer; }

  static uint8_t GetNextOrderSeqNumber(uint8_t sq);

  bool Ready();
  void Ready(bool);
  uint8_t GetExpectedOrderSeqNumber(); // returns 1 or 0

  // void SetExpectedOrderSeqNumber(uint8_t);
  void IncExpectedOrderSeqNumber();
  void SetHeading(uint16_t);
  uint16_t GetHeading();
  int16_t GetX();
  void SetX(double);
  int16_t GetY();
  void SetY(double);
  int16_t GetZ();
  void SetZ(double);
  int16_t GetRoll();
  void SetRoll(double);
  int16_t GetPitch();
  void SetPitch(double);

  void LastOrderCancelledFlag(bool);
  bool LastOrderCancelledFlag();

  void KeepingHeadingFlag(bool);
  bool KeepingHeadingFlag();

  ARDUSUB_NAV_MODE GetNavMode();
  void SetNavMode(ARDUSUB_NAV_MODE);

  void Armed(bool);
  bool Armed();
  uint32_t GetMsgSize();

private:
  void _Init();
  uint8_t buffer[MAX_HROVSTATE_LENGHT];

  const static uint8_t READY_FLAG = 0x80, NEXT_ORDER_SEQ_FLAG = 0x40,
                       LAST_ORDER_CANCELLED_FLAG = 0x20,
                       KEEPING_HEADING_FLAG = 0x10, ARMED_FLAG = 0x8;

  const static uint8_t NAV_MODE_MASK = 0x7;

  uint8_t *flags, *orientation, *position;
  int16_t *x, *y, *z;

  bool bigEndian;
};

} /* namespace merbots */

#endif /* MERBOTS_LIB_INCLUDE_MERBOTS_HROVMessageV2_H_ */
