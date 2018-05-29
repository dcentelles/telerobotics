/*
 * HROVOrders.h
 *
 *  Created on: 30 nov. 2016
 *      Author: centelld
 */

#ifndef INCLUDE_TELEROBOTICS_OPERATORMESSAGEV2_H_
#define INCLUDE_TELEROBOTICS_OPERATORMESSAGEV2_H_

#include <telerobotics/Constants.h>
#include <telerobotics/HROVSettingsV2.h>
#include <telerobotics/TeleopOrder.h>

namespace telerobotics {

class OperatorMessageV2;

typedef std::shared_ptr<OperatorMessageV2> OperatorMessageV2Ptr;

class OperatorMessageV2 {
public:
  const static uint8_t MessageLength;
  OperatorMessageV2();
  OperatorMessageV2(uint8_t *);
  virtual ~OperatorMessageV2();

  static OperatorMessageV2Ptr Build() {
    return OperatorMessageV2Ptr(new OperatorMessageV2());
  }

  void UpdateFromBuffer(uint8_t *);
  void GetBufferCopy(uint8_t *);
  uint8_t *GetBuffer() { return buffer; }

  uint8_t GetOrderSeqNumber(); // returns 1 or 0
  void SetOrderSeqNumber(uint8_t);
  //  uint8_t IncOrderSeqNumber();

  void CancelLastOrderFlag(bool);
  bool CancelLastOrderFlag();

  enum OrderType {
    NoOrder = 0,
    Move,
    KeepOrientation,
    HoldChannel,
    UpdateImageSettings,
    DisableKeepOrientation,
    OtherNotImplemented
  };
  OrderType GetOrderType();
  TeleopOrderPtr GetMoveOrderCopy();
  void SetMoveOrder(TeleopOrderPtr);
  void SetNoOrder();
  uint8_t GetHoldChannelDuration();
  void SetHoldChannelOrder(uint8_t);
  void SetEnableKeepOrientationOrder(uint16_t orientation);
  uint16_t GetKeepOrientationValue();
  void SetDisableKeepOrientationOrder();

  HROVSettingsV2Ptr GetImageSettingsOrderCopy();
  void SetUpdateImageSettingsOrder(HROVSettingsV2Ptr);

private:
  void _SetOrderType(OrderType);
  void _Init();
  uint8_t buffer[MAX_HROVSTATE_LENGHT];

  const static uint8_t ORDER_SEQ_FLAG = 0x80, CANCEL_LAST_ORDER_FLAGH = 0x40,
                       ORDER_TYPE_MASK = 0xf;
  uint8_t *messageInfo;
  uint8_t *orderBuffer;

  bool _bigEndian;
};

} /* namespace merbots */

#endif /* MERBOTS_LIB_INCLUDE_MERBOTS_OPERATORMESSAGE_H_ */