/*
 * @Description: define eeprom layout
 * @Author: yutong.pan@horizon.ai
 * @Date: 2019-12-21
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */

#ifndef INCLUDE_EEPROM_LAYOUT_H_
#define INCLUDE_EEPROM_LAYOUT_H_

#define CAM_CALIB_START_ADDR (0x0)
#define CAM_CALIB_DATA_LENGTH (72)

struct EEPROMHeader {
  char* SN;
  char* md5;
  char* version;
  char* length;
  char* data;
};

#endif  // INCLUDE_EEPROM_LAYOUT_H_
