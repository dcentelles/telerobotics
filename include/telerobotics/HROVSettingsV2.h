/*
 * HROVState.h
 *
 *  Created on: 30 nov. 2016
 *      Author: centelld
 */

#ifndef INCLUDE_TELEROBOTICS_HROVSETTINGSV2_H_
#define INCLUDE_TELEROBOTICS_HROVSETTINGSV2_H_

#include <telerobotics/Constants.h>
#include <memory>

namespace telerobotics {

class HROVSettingsV2;

typedef std::shared_ptr<HROVSettingsV2> HROVSettingsV2Ptr;

class HROVSettingsV2 {
public:
  const static uint8_t SettingsSize = 8;
  HROVSettingsV2();
  virtual ~HROVSettingsV2();

  static HROVSettingsV2Ptr Build() {
    return HROVSettingsV2Ptr(new HROVSettingsV2());
  }

  void SetSettings(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                   uint8_t shift, uint16_t img_size);

  void UpdateFromBuffer(uint8_t *);
  void GetBufferCopy(uint8_t *);

  uint8_t *GetBuffer() { return buffer; }
  bool EncodeMonoVersion();
  void EncodeMonoVersion(bool);

  void SetROIConf(uint16_t x0 = 0, uint16_t y0 = 0, uint16_t x1 = 0,
                   uint16_t y1 = 0, uint8_t shift = 0);
  void SetImgSize(uint16_t _size);

  void GetROIConf(int16_t &x0, int16_t &y0, int16_t &x1, int16_t &y1,
                   uint8_t &shift);
  uint16_t GetROIX0();
  uint16_t GetROIY0();
  uint16_t GetROIX1();
  uint16_t GetROIY1();
  uint8_t GetROIShift();

  uint16_t GetImgSize();

private:
  void _Init();

  uint8_t buffer[MAX_HROVSTATE_LENGHT];

  uint8_t *roi;
  uint8_t *roi_shift;
  uint8_t *flags;
  uint16_t *img_size; //, *img_width, *img_height;

  bool bigEndian;
};

} /* namespace merbots */

#endif /* MERBOTS_LIB_INCLUDE_MERBOTS_HROVSETTINGS_H_ */
