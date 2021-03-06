/*
 * HROVOrders.cpp
 *
 *  Created on: 30 nov. 2016
 *      Author: centelld
 */

#include <cmath>
#include <cstring>
#include <dccomms/Utils.h>
#include <telerobotics/HROVMessageV2.h>

namespace telerobotics {

const uint8_t HROVMessageV2::MessageLength = 1 + // flags
                                             6 + // position
                                             5   // orientation
    ;

HROVMessageV2::HROVMessageV2() { _Init(); }

HROVMessageV2::HROVMessageV2(uint8_t *_buffer) {
  _Init();
  UpdateFromBuffer(_buffer);
}

HROVMessageV2::~HROVMessageV2() {
  // TODO Auto-generated destructor stub
}

void HROVMessageV2::_Init() {
  bigEndian = dccomms::Utils::IsBigEndian();

  flags = buffer;
  position = flags + 1;
  x = (int16_t *)position;
  y = x + 1;
  z = y + 1;
  orientation = (uint8_t *)(z + 1);

  *flags = 0;
}

void HROVMessageV2::UpdateFromBuffer(uint8_t *_stateb) {
  memcpy(buffer, _stateb, MessageLength);
}

void HROVMessageV2::GetBufferCopy(uint8_t *_stateb) {
  memcpy(_stateb, buffer, MessageLength);
}

bool HROVMessageV2::LastOrderCancelledFlag() {
  return *flags & LAST_ORDER_CANCELLED_FLAG;
}

void HROVMessageV2::KeepingHeadingFlag(bool v) {
  *flags = v ? *flags | KEEPING_HEADING_FLAG : *flags & ~KEEPING_HEADING_FLAG;
}
bool HROVMessageV2::KeepingHeadingFlag() {
  return *flags & KEEPING_HEADING_FLAG;
}

void HROVMessageV2::LastOrderCancelledFlag(bool v) {
  *flags = v ? *flags | LAST_ORDER_CANCELLED_FLAG
             : *flags & ~LAST_ORDER_CANCELLED_FLAG;
}

bool HROVMessageV2::Ready() { return *flags & READY_FLAG; }

void HROVMessageV2::Ready(bool v) {
  *flags = v ? *flags | READY_FLAG : *flags & ~READY_FLAG;
}
uint8_t HROVMessageV2::GetExpectedOrderSeqNumber() {
  return *flags & NEXT_ORDER_SEQ_FLAG ? 1 : 0;
}

uint8_t HROVMessageV2::GetNextOrderSeqNumber(uint8_t seq) {
  return seq ? 0 : 1;
}

void HROVMessageV2::IncExpectedOrderSeqNumber() {
  *flags = *flags & NEXT_ORDER_SEQ_FLAG ? *flags & ~NEXT_ORDER_SEQ_FLAG
                                        : *flags | NEXT_ORDER_SEQ_FLAG;
}

void HROVMessageV2::SetX(double _X) {
  int16_t X = (int16_t)std::round(_X);
  *x = X;
}

int16_t HROVMessageV2::GetX() { return *x; }

void HROVMessageV2::SetY(double _Y) {
  int16_t Y = (int16_t)std::round(_Y);
  *y = Y;
}

int16_t HROVMessageV2::GetY() { return *y; }

void HROVMessageV2::SetZ(double _Z) {
  int16_t Z = (int16_t)std::round(_Z);
  *z = Z;
}

int16_t HROVMessageV2::GetZ() { return *z; }

void HROVMessageV2::SetHeading(uint16_t _yaw) {
  uint8_t *ptr = orientation;

  // encoded with 9 bits (max value: 360)
  orientation[0] = 0;
  orientation[1] &= 0x7f;
  *ptr |= (_yaw & 0x01fe) >> 1;    // 8
  *(ptr + 1) |= (_yaw & 0x1) << 7; // +1
}

uint16_t HROVMessageV2::GetHeading() {
  uint8_t *ptr = orientation;
  uint16_t res;

  res = *ptr << 1; // 8
  ptr++;
  res |= (*ptr & 0x80) >> 7;
  return res;
}

void HROVMessageV2::SetRoll(double _X) {
  // encoded with 13 bits (up to 819.2 m)
  uint8_t *ptr = orientation + 1;
  int16_t X = (int16_t)std::round(_X);
  *ptr &= 0x80;
  *ptr |= (X & 0x1fc0) >> 6; // 7
  ptr++;
  *ptr &= 0x03;
  *ptr |= (X & 0x3f) << 2; // +6
}

int16_t HROVMessageV2::GetRoll() {
  uint8_t *ptr = orientation + 1;
  int16_t res;
  res = (*ptr & 0x7f) << 6;
  ptr++;
  res |= *ptr >> 2;

  if (res & 0x1000) {
    res |= 0xfe00;
  }

  return res;
}

void HROVMessageV2::SetPitch(double _Y) {
  uint8_t *ptr = orientation + 2;
  int16_t Y = (int16_t)std::round(_Y);
  *ptr &= 0xfc;
  *ptr |= (Y & 0x1800) >> 11; // 2
  ptr++;
  *ptr = (Y & 0x7f8) >> 3; //+8
  ptr++;
  *ptr &= 0x1f;
  *ptr |= (Y & 0x7) << 5; //+3
}

int16_t HROVMessageV2::GetPitch() {
  uint8_t *ptr = orientation + 2;
  int16_t res;
  res = (*ptr & 0x03) << 11;
  ptr++;
  res |= *ptr << 3;
  ptr++;
  res |= *ptr >> 5;

  if (res & 0x1000) {
    res |= 0xfe00;
  }

  return res;
}

void HROVMessageV2::SetNavMode(ARDUSUB_NAV_MODE mode) {
  *flags = (*flags & ~NAV_MODE_MASK | mode);
}

ARDUSUB_NAV_MODE HROVMessageV2::GetNavMode() {
  unsigned int mode = *flags & NAV_MODE_MASK;
  return mode >= 0 && mode < NAV_UNKNOWN ? (ARDUSUB_NAV_MODE)mode
                                         : ARDUSUB_NAV_MODE::NAV_UNKNOWN;
}

bool HROVMessageV2::Armed() { return *flags & ARMED_FLAG; }

void HROVMessageV2::Armed(bool v) {
  *flags = v ? *flags | ARMED_FLAG : *flags & ~ARMED_FLAG;
}

uint32_t HROVMessageV2::GetMsgSize() { return HROVMessageV2::MessageLength; }
} /* namespace merbots */
