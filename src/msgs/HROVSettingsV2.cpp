/*
 * HROVState.cpp
 *
 *  Created on: 30 nov. 2016
 *      Author: centelld
 */

#include <dccomms/Utils.h>
#include <telerobotics/HROVSettingsV2.h>
#include <cstring>

namespace telerobotics {

HROVSettingsV2::HROVSettingsV2() { _Init(); }

//HROVSettingsV2::HROVSettingsV2(
//    const merbots_whrov_msgs::hrov_settings::ConstPtr &cstate) {
//  _Init();
//  SetFromROSMsg(cstate);
//}

HROVSettingsV2::~HROVSettingsV2() {
  // TODO Auto-generated destructor stub
}

void HROVSettingsV2::_Init() {
  bigEndian = dccomms::Utils::IsBigEndian();

  // IMAGE
  roi = (uint8_t *)buffer;

  roi_shift = (uint8_t *)(roi + 5); //+5
  flags = roi_shift;

  // roi shift ocupa 4 bits

  img_size = (uint16_t *)(flags + 1); //+2

  SetROIConf(false);
}
void HROVSettingsV2::UpdateFromBuffer(uint8_t *_stateb) {
  memcpy(buffer, _stateb, SettingsSize);
}

void HROVSettingsV2::EncodeMonoVersion(bool v) {
  if (v)
    *flags |= 0x1;
  else
    *flags &= 0xfe;
}

bool HROVSettingsV2::EncodeMonoVersion() { return *flags & 0x1; }

void HROVSettingsV2::GetBufferCopy(uint8_t *_stateb) {
  memcpy(_stateb, buffer, SettingsSize);
}

//void HROVSettingsV2::SetFromROSMsg(
//    const merbots_whrov_msgs::hrov_settings::ConstPtr &cstate) {
//  _SetROIConf(cstate->image_config.roi_x0, cstate->image_config.roi_y0,
//              cstate->image_config.roi_x1, cstate->image_config.roi_y1,
//              cstate->image_config.roi_shift);

//  EncodeMonoVersion(cstate->image_config.encode_mono);
//  _SetImgSize(cstate->image_config.size);
//}

//merbots_whrov_msgs::hrov_settings::Ptr HROVSettingsV2::GetROSMsg() {
//  merbots_whrov_msgs::hrov_settings::Ptr cstate(
//      new merbots_whrov_msgs::hrov_settings());
//  cstate.reset(new merbots_whrov_msgs::hrov_settings());
//  cstate->image_config.size = _GetImgSize();
//  cstate->image_config.roi_x0 = _GetROIX0();
//  cstate->image_config.roi_y0 = _GetROIY0();
//  cstate->image_config.roi_x1 = _GetROIX1();
//  cstate->image_config.roi_y1 = _GetROIY1();
//  cstate->image_config.roi_shift = _GetROIShift();
//  cstate->image_config.encode_mono = EncodeMonoVersion();

//  return cstate;
//}

void HROVSettingsV2::SetSettings(uint16_t x0, uint16_t y0, uint16_t x1,
                                 uint16_t y1, uint8_t shift,
                                 uint16_t img_size) {
  SetROIConf(x0, y0, x1, y1, shift);
  SetImgSize(img_size);
}

void HROVSettingsV2::SetROIConf(uint16_t x0, uint16_t y0, uint16_t x1,
                                 uint16_t y1, uint8_t shift) {
  *roi_shift &= 0x1f;
  *roi_shift |= (shift & 0xf) << 4;
  roi[0] = 0;
  roi[1] = 0;
  roi[2] = 0;
  roi[3] = 0;
  roi[4] = 0;

  uint8_t *ptr = roi;

  // x0
  *ptr |= (x0 & 0x03fc) >> 2; // 8
  ptr++;
  *ptr |= (x0 & 0x3) << 6; //+2

  // y0
  *ptr |= (y0 & 0x03f0) >> 4; // 6
  ptr++;
  *ptr |= (y0 & 0xf) << 4; //+4

  // x1
  *ptr |= (x1 & 0x03c0) >> 6; // 4
  ptr++;
  *ptr |= (x1 & 0x3f) << 2; //+6

  // y1
  *ptr |= (y1 & 0x0300) >> 8; // 2
  ptr++;
  *ptr |= y1 & 0xff; //+8
}

void HROVSettingsV2::GetROIConf(int16_t &x0, int16_t &y0, int16_t &x1,
                                 int16_t &y1, uint8_t &shift) {
  shift = GetROIShift();
  x0 = GetROIX0();
  y0 = GetROIY0();
  x1 = GetROIX1();
  y1 = GetROIY1();
}

uint16_t HROVSettingsV2::GetROIX0() {
  uint8_t *ptr = roi;
  uint16_t res;

  res = (*ptr & 0xff) << 2;        // 8
  res |= (*(ptr + 1) & 0xc0) >> 6; //+2

  return res;
}

uint16_t HROVSettingsV2::GetROIY0() {
  uint8_t *ptr = roi + 1;
  uint16_t res;

  res = (*ptr & 0x3f) << 4;        // 6
  res |= (*(ptr + 1) & 0xf0) >> 4; //+4

  return res;
}

uint16_t HROVSettingsV2::GetROIX1() {
  uint8_t *ptr = roi + 2;
  uint16_t res;

  res = (*ptr & 0x0f) << 6;        // 4
  res |= (*(ptr + 1) & 0xfc) >> 2; //+6

  return res;
}

uint16_t HROVSettingsV2::GetROIY1() {
  uint8_t *ptr = roi + 3;
  uint16_t res;

  res = (*ptr & 0x03) << 8; // 2
  res |= *(ptr + 1);        //+8

  return res;
}

uint8_t HROVSettingsV2::GetROIShift() { return *roi_shift >> 4; }

void HROVSettingsV2::SetImgSize(uint16_t size) {
  if (bigEndian) {
    *img_size = size;
  } else {
    dccomms::Utils::IntSwitchEndian(img_size, size);
  }
}

uint16_t HROVSettingsV2::GetImgSize() {
  uint16_t res;
  if (bigEndian) {
    res = *img_size;
  } else {
    dccomms::Utils::IntSwitchEndian(&res, *img_size);
  }
  return res;
}

} /* namespace merbots */
