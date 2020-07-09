/***************************************************************************
 *   Copyright (C) 2019 by horizon.                                        *
 *   you@horizon.ai                                                  *
 *                                                                         *
 *   iar header file.                                                     *
 *                                                                         *
 ***************************************************************************/

#ifndef INCLUDE_HB_SYS_IAR_INTERFACE_H_
#define INCLUDE_HB_SYS_IAR_INTERFACE_H_

#define MAX_FRAME_BUF_SIZE (1920 * 1080 * 4)

typedef struct _frame_info_t {
  void *addr;
  unsigned int size;
} frame_info_t;

enum mem_mode_e {
  NORMAL_MODE,
  DMA_MODE,
};

enum format_yuv_e {
  FORMAT_YUV422_UYVY = 0,
  FORMAT_YUV422_VYUY = 1,
  FORMAT_YUV422_YVYU = 2,
  FORMAT_YUV422_YUYV = 3,
  FORMAT_YUV422SP_UV = 4,
  FORMAT_YUV422SP_VU = 5,
  FORMAT_YUV420SP_UV = 6,
  FORMAT_YUV420SP_VU = 7,
  FORMAT_YUV422P_UV = 8,
  FORMAT_YUV422P_VU = 9,
  FORMAT_YUV420P_UV = 10,
  FORMAT_YUV420P_VU = 11,
};

enum format_rgb_e {
  FORMAT_8BPP = 0,
  FORMAT_RGB565 = 1,
  FORMAT_RGB888 = 2,
  FORMAT_RGB888P = 3,
  FORMAT_ARGB8888 = 4,
  FORMAT_RGBA8888 = 5,
};

enum output_channel_e {
  OUTPUT_CHANNEL0 = 1,
  OUTPUT_CHANNEL1 = 2,
  OUTPUT_CHANNEL2 = 4,
  OUTPUT_CHANNEL3 = 8,
};

enum output_mode_e {
  OUTPUT_MIPI = 0,
  OUTPUT_BT1120 = 1,
  OUTPUT_RGB888 = 2,
};

int hb_disp_init(void);
int hb_disp_close(void);
int hb_disp_start(void);
int hb_disp_stop(void);
int hb_disp_set_lcd_backlight(unsigned int backlight_level);
int hb_disp_layer_on(unsigned int layer_number);
int hb_disp_layer_off(unsigned int layer_number);
int hb_disp_video_set_channel(unsigned int channel_number);
int iar_write_framedata(frame_info_t *frame1, frame_info_t *frame2,
                        frame_info_t *frame3, frame_info_t *frame4);
int iar_update_frame(unsigned int ch_set);
char *iar_mmap(unsigned int ch);
int iar_parser_configfile(void *root);
int iar_mmap_channel0(void);
void iar_munmap_channel0(void);

#endif  // INCLUDE_HB_SYS_IAR_INTERFACE_H_
