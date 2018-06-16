/*
 * HROVOrders.cpp
 *
 *  Created on: 30 nov. 2016
 *      Author: centelld
 */

#include <dccomms/dccomms.h>
#include <telerobotics/OperatorMessageV2.h>
namespace telerobotics {

const uint8_t OperatorMessageV2::MessageLength =
    HROVSettingsV2::SettingsSize + 1 // flags
    ;

OperatorMessageV2::OperatorMessageV2() {
  _Init();
  SetNoOrder();
}

OperatorMessageV2::OperatorMessageV2(uint8_t *_buf) {
  _Init();
  UpdateFromBuffer(_buf);
}

void OperatorMessageV2::_Init() {
  _bigEndian = dccomms::Utils::IsBigEndian();

  messageInfo = buffer;
  orderBuffer = messageInfo + 1;

  *messageInfo = 0;

  x = (int16_t *)orderBuffer;
  y = x + 1;
  z = y + 1;
  heading = z + 1;
}

OperatorMessageV2::~OperatorMessageV2() {
  // TODO Auto-generated destructor stub
}

void OperatorMessageV2::SetOrderOrderIncomming() {
  _SetOrderType(OrderType::OrderIncomming);
}

void OperatorMessageV2::SetGoToOrder(int16_t x_, int16_t y_, int16_t z_,
                                     int16_t heading_) {
  *x = x_;
  *y = y_;
  *z = z_;
  *heading = heading_;
  _SetOrderType(OrderType::GoTo);
}

void OperatorMessageV2::GetGoToOrder(int &x_, int &y_, int &z_, int &heading_) {
  x_ = *x;
  y_ = *y;
  z_ = *z;
  heading_ = *heading;
}

uint32_t OperatorMessageV2::GetOrderSize(OrderType otype) {
  uint32_t msgSize = 0;
  switch (otype) {
  case Move: {
    msgSize += TeleopOrder::Size;
    break;
  }
  case NoOrder: {
    break;
  }
  case KeepOrientation: {
    msgSize += 2;
    break;
  }
  case HoldChannel: {
    msgSize += 1;
    break;
  }
  case DisableKeepOrientation: {
    break;
  }
  case UpdateImageSettings: {
    msgSize += HROVSettingsV2::SettingsSize;
    break;
  }
  case GoTo: {
    msgSize += 8;
    break;
  }
  case OrderIncomming: {
    break;
  }
  case OtherNotImplemented: {
    break;
  }
  }
  return msgSize;
}

uint32_t OperatorMessageV2::GetMsgSize() {
  OrderType otype = GetOrderType();
  return GetOrderSize(otype) + 1;
}

uint32_t OperatorMessageV2::GetMsgSize(uint8_t *src) {
  OrderType otype = GetOrderType(*src);
  return GetOrderSize(otype) + 1;
}

uint8_t OperatorMessageV2::GetHoldChannelDuration() { return *orderBuffer; }

void OperatorMessageV2::SetHoldChannelOrder(uint8_t v) {
  _SetOrderType(OrderType::HoldChannel);
  *orderBuffer = v;
}

HROVSettingsV2Ptr OperatorMessageV2::GetImageSettingsOrderCopy() {
  auto settings = HROVSettingsV2::Build();
  settings->UpdateFromBuffer(orderBuffer);
  return settings;
}

void OperatorMessageV2::SetUpdateImageSettingsOrder(
    HROVSettingsV2Ptr _settings) {
  _SetOrderType(OrderType::UpdateImageSettings);
  _settings->GetBufferCopy(orderBuffer);
}

TeleopOrderPtr OperatorMessageV2::GetMoveOrderCopy() {
  auto moveOrder = TeleopOrder::Build();
  moveOrder->BuildFromBuffer(orderBuffer);
  return moveOrder;
}

void OperatorMessageV2::SetMoveOrder(TeleopOrderPtr _moveOrder) {
  _SetOrderType(OrderType::Move);
  memcpy(orderBuffer, _moveOrder->GetBuffer(), TeleopOrder::Size);
}

void OperatorMessageV2::SetEnableKeepOrientationOrder(uint16_t orientation) {
  _SetOrderType(OrderType::KeepOrientation);
  if (_bigEndian) {
    *(uint16_t *)orderBuffer = orientation;
  } else {
    dccomms::Utils::IntSwitchEndian(orderBuffer, orientation);
  }
}

uint16_t OperatorMessageV2::GetKeepOrientationValue() {
  uint16_t value = *(uint16_t *)orderBuffer;
  uint16_t res;
  if (_bigEndian) {
    res = *(uint16_t *)orderBuffer;
  } else {
    dccomms::Utils::IntSwitchEndian(&res, *(uint16_t *)orderBuffer);
  }
  return res;
}

void OperatorMessageV2::SetDisableKeepOrientationOrder() {
  _SetOrderType(OrderType::DisableKeepOrientation);
}

OperatorMessageV2::OrderType OperatorMessageV2::GetOrderType() {
  return GetOrderType(*messageInfo);
}

OperatorMessageV2::OrderType OperatorMessageV2::GetOrderType(uint8_t src) {

  unsigned int type = src & ORDER_TYPE_MASK;
  return type >= 0 && type < OtherNotImplemented ? (OrderType)type
                                                 : OtherNotImplemented;
}

void OperatorMessageV2::SetNoOrder() { _SetOrderType(NoOrder); }
void OperatorMessageV2::_SetOrderType(OrderType orderType) {
  *messageInfo = (*messageInfo & ~ORDER_TYPE_MASK) | orderType;
}

uint8_t OperatorMessageV2::GetOrderSeqNumber() {
  return (*messageInfo & ORDER_SEQ_FLAG) ? 1 : 0;
}

void OperatorMessageV2::SetOrderSeqNumber(uint8_t seq) {
  *messageInfo =
      seq ? *messageInfo | ORDER_SEQ_FLAG : *messageInfo & ~ORDER_SEQ_FLAG;
}

void OperatorMessageV2::CancelLastOrder(bool v) {
  *messageInfo = v ? *messageInfo | CANCEL_LAST_ORDER_FLAGH
                   : *messageInfo & ~CANCEL_LAST_ORDER_FLAGH;
}

bool OperatorMessageV2::CancelLastOrder() {
  return *messageInfo & CANCEL_LAST_ORDER_FLAGH;
}

void OperatorMessageV2::UpdateFromBuffer(uint8_t *_stateb) {
  memcpy(buffer, _stateb, GetMsgSize(_stateb));
}

void OperatorMessageV2::GetBufferCopy(uint8_t *_stateb) {
  memcpy(_stateb, buffer, GetMsgSize());
}

} /* namespace merbots */
