/*
 * HROVOrders.h
 *
 *  Created on: 30 nov. 2016
 *      Author: centelld
 */

#ifndef INCLUDE_TELEROBOTICS_OPERATORMESSAGE_H_
#define INCLUDE_TELEROBOTICS_OPERATORMESSAGE_H_

#include <telerobotics/Constants.h>
#include <telerobotics/HROVMoveOrder.h>
#include <telerobotics/HROVSettings.h>

namespace telerobotics {

class OperatorMessage;

typedef std::shared_ptr<OperatorMessage> OperatorMessagePtr;

class OperatorMessage {
public:
  const static uint8_t MessageLength;
  OperatorMessage();
  OperatorMessage(uint8_t *);
  virtual ~OperatorMessage();

  static OperatorMessagePtr BuildOperatorMessage() {
    return OperatorMessagePtr(new OperatorMessage());
  }

  void UpdateFromBuffer(uint8_t *);
  void GetBufferCopy(uint8_t *);
  uint8_t *GetBuffer() { return buffer; }

  uint8_t GetOrderSeqNumber(); // returns 1 or 0
  void SetOrderSeqNumber(uint8_t);
  //  uint8_t IncOrderSeqNumber();

  void CancelLastOrderFlag(bool);
  bool CancelLastOrderFlag();

  enum OrderType { NoOrder = 0, Move, OtherNotImplemented };
  OrderType GetOrderType();
  void SetOrderType(OrderType);
  HROVMoveOrderPtr GetMoveOrderCopy();
  void SetMoveOrder(HROVMoveOrderPtr);

  HROVSettingsPtr GetSettingsCopy();
  void SetSettings(HROVSettingsPtr);

private:
  void _Init();
  uint8_t buffer[MAX_HROVSTATE_LENGHT];

  const static uint8_t ORDER_SEQ_FLAG = 0x80, CANCEL_LAST_ORDER_FLAGH = 0x40,
                       ORDER_TYPE_MASK = 0x3;
  uint8_t *messageInfo;
  uint8_t *settingsBuffer, *moveOrderBuffer;
};

} /* namespace merbots */

#endif /* MERBOTS_LIB_INCLUDE_MERBOTS_OPERATORMESSAGE_H_ */
