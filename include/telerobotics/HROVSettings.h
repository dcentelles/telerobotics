/*
 * HROVState.h
 *
 *  Created on: 30 nov. 2016
 *      Author: centelld
 */

#ifndef INCLUDE_TELEROBOTICS_HROVSETTINGS_H_
#define INCLUDE_TELEROBOTICS_HROVSETTINGS_H_

#include <memory>
#include <telerobotics/Constants.h>

namespace telerobotics {

class HROVSettings;

typedef std::shared_ptr<HROVSettings> HROVSettingsPtr;

class HROVSettings {
public:
  const static uint8_t SettingsSize = 10;
  HROVSettings();
  virtual ~HROVSettings();

  static HROVSettingsPtr BuildHROVSettings() {
    return HROVSettingsPtr(new HROVSettings());
  }

  void UpdateFromBuffer(uint8_t *);
  void GetBufferCopy(uint8_t *);

  uint8_t *GetBuffer() { return buffer; }

  void SetROIConf(uint16_t x0 = 0, uint16_t y0 = 0, uint16_t x1 = 0,
                  uint16_t y1 = 0, uint8_t shift = 0);
  void SetImgSize(uint16_t _size);
  void SetImgResolution(uint16_t resolution);
  void SetMaxPacketLength(uint16_t _maxPacketLength);

  void GetROIConf(int16_t &x0, int16_t &y0, int16_t &x1, int16_t &y1,
                  uint8_t &shift);
  uint16_t GetROIX0();
  uint16_t GetROIY0();
  uint16_t GetROIX1();
  uint16_t GetROIY1();
  uint8_t GetROIShift();
  uint16_t GetImgSize();
  uint16_t GetMaxPacketLength();
  uint8_t GetImgResolution();

private:
  void _Init();

  uint8_t buffer[MAX_HROVSTATE_LENGHT];

  uint8_t *roi;
  uint8_t *roi_shift;
  uint16_t *img_size;      //, *img_width, *img_height;
  uint8_t *img_resolution; //, * roi_enabled

  uint16_t *max_packet_length;

  bool bigEndian;
};

} /* namespace merbots */

#endif /* MERBOTS_LIB_INCLUDE_MERBOTS_HROVSETTINGS_H_ */
