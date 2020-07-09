/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2019 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef INCLUDE_X2_CAMERA_COMMON_H_
#define INCLUDE_X2_CAMERA_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "x2_camera.h"
#define CAM_MAX_NUM 6
#define GPIO_NUMBER 6
#define DESERIAL_NUMBER 2
#define MAX_NUM_LENGTH 128
#define ENTRY_NUM 4

#define RET_OK 0
#define RET_ERROR 1
#define HB_CAM_PARSE_BOARD_CFG_FAIL 2
#define HB_CAM_PARSE_MIPI_CFG_FAIL 3
#define HB_CAM_DLOPEN_LIBRARY_FAIL 4
#define HB_CAM_INIT_FAIL 5
#define HB_CAM_DEINIT_FAIL 6
#define HB_CAM_START_FAIL 7
#define HB_CAM_STOP_FAIL 8
#define HB_CAM_I2C_WRITE_FAIL 9
#define HB_CAM_I2C_WRITE_BYTE_FAIL 10
#define HB_CAM_I2C_WRITE_BLOCK_FAIL 11
#define HB_CAM_I2C_READ_BLOCK_FAIL 12
#define HB_CAM_CONTROL_ISP_FAIL 13
#define HB_CAM_DYNAMIC_SWITCH_FAIL 14
#define HB_CAM_DYNAMIC_SWITCH_FPS_FAIL 15
#define HB_CAM_PARSE_SERIAL_CFG_FAIL 16
#define HB_CAM_S954_POWERON_FAIL 17
#define HB_CAM_S954_CONFIG_FAIL 18
#define HB_CAM_S954_STREAM_ON_FAIL 19
#define HB_CAM_S954_STREAM_OFF_FAIL 20
#define HB_CAM_SENSOR_POWERON_FAIL 21
#define HB_CAM_START_PHYSICAL_FAIL 22
#define HB_CAM_SPI_WRITE_BLOCK_FAIL 23
#define HB_CAM_SPI_READ_BLOCK_FAIL 24

/* for interface type */
#define INTERFACE_MIPI "mipi"
#define INTERFACE_BT "bt"
#define INTERFACE_DVP "dvp"
#define INTERFACE_SDIO "sdio"
#define INTERFACE_NET "net"

#define CAMERA_EVENT_AE_FACE_INFO (1UL << 0)
#define CAMERA_EVENT_EEPROM_CALIBRATION (1UL << 1)
#define CAMERA_EVENT_GET_AGC_GAIN (1UL << 2)
#define CAMERA_EVENT_GET_VERSION (1UL << 3)
#define CAMERA_EVENT_GET_AVERAGE_LUMA (1UL << 4)
#define CAMERA_EVENT_SET_MIRRIOR (1UL << 5)

/* not use now*/
enum GPIO_DEF_VALUE {
  ISP_RESET,
  SENSOR_POWER_ON,
  SENSOR_RESET,
  S954_POWER_ON,
  S954_RESET
};

typedef struct spi_data {
  int spi_mode;
  int spi_cs;
  uint32_t spi_speed;
} spi_data_t;

struct rect_t {
  int32_t start_x;
  int32_t start_y;
  int32_t width;
  int32_t height;
};

struct face_track_info {
  // face track id
  int32_t face_type;  // 0: rgb  1: nir
  int32_t track_id;
  struct rect_t face_rio;
};
struct face_info_t {
  int face_rio_size;                   // face rio size, none set to 0
  struct face_track_info *face_track;  // face track info
};

typedef struct camera_calib_eeprom_s {
#define EEPROM_RD_MODE 0
#define EEPROM_WR_MODE 1
  int wr_mode;  // 0 : for read, 1 for write
  int addr;
  int length;
  char *buff;
} calib_eeprom_t;

typedef struct sensor_info_s {
  int port;
  int bus_type;
  int bus_num;
  int isp_addr;
  int sensor_addr;
  int sensor1_addr;
  int serial_addr;
  int serial_addr1;
  int imu_addr;
  int eeprom_addr;
  int power_mode;
  int sensor_mode;
  int entry_num;
  int reg_width;
  int gpio_num;
  int gpio_pin[GPIO_NUMBER];
  int gpio_level[GPIO_NUMBER];
  int fps;
  int width;
  int height;
  int format;
  int resolution;
  int extra_mode;
  int power_delay;
  int deserial_index;
  int deserial_port;
  char *sensor_name;
  char *config_path;
  void *sensor_ops;
  void *sensor_fd;
  void *deserial_info;
  int stream_control;
  int config_index;
  spi_data_t spi_info;
} sensor_info_t;

typedef struct deserial_info_s {
  int bus_type;
  int bus_num;
  int deserial_addr;
  int power_mode;
  int physical_entry;
  int gpio_num;
  int gpio_pin[GPIO_NUMBER];
  int gpio_level[GPIO_NUMBER];
  char *deserial_name;
  char *deserial_config_path;
  void *deserial_ops;
  void *deserial_fd;
  void *sensor_info[CAM_MAX_NUM];
  int init_state;
} deserial_info_t;

typedef struct board_info_s {
  int config_number;
  char *board_name;
  char *interface_type;
  int deserial_num;
  int port_number;
  deserial_info_t deserial_info[DESERIAL_NUMBER];
  sensor_info_t sensor_info[CAM_MAX_NUM];
} board_info_t;

typedef struct {
  const char *module;
  int (*init)(sensor_info_t *sensor_info);
  int (*deinit)(sensor_info_t *sensor_info);
  int (*start)(sensor_info_t *sensor_info);
  int (*stop)(sensor_info_t *sensor_info);
  int (*process)(sensor_info_t *sensor_info, int event, void *arg);
  int (*power_on)(sensor_info_t *sensor_info);
  int (*power_off)(sensor_info_t *sensor_info);
  int (*power_reset)(sensor_info_t *sensor_info);
  int (*spi_read)(sensor_info_t *sensor_info, uint32_t reg_addr, char *buffer,
                  int sizee);
  int (*spi_write)(sensor_info_t *sensor_info, uint32_t reg_addr, char *buffer,
                   int sizee);
  int (*set_awb)(int i2c_bus, int sensor_addr, float rg_gain, float b_gain);
  int (*set_ex_gain)(int i2c_bus, int sensor_addr, uint32_t exposure_setting,
                     uint32_t gain_setting_0, uint16_t gain_setting_1);
  int (*dynamic_switch_fps)(sensor_info_t *sensor_info, uint32_t fps);
} sensor_module_t;

typedef struct {
  const char *module;
  int (*init)(deserial_info_t *deserial_info);
  int (*stream_on)(deserial_info_t *deserial_info, int port);
  int (*stream_off)(deserial_info_t *deserial_info, int port);
  int (*deinit)(deserial_info_t *deserial_info);
  int (*start_physical)(deserial_info_t *deserial_info);
  int (*reset)(deserial_info_t *deserial_info);
} deserial_module_t;

extern int hb_cam_htoi(char s[]);

#ifdef __cplusplus
}
#endif

#endif  // INCLUDE_X2_CAMERA_COMMON_H_
