//===-------- hbdk_sim_ipu.h - IPU simulator interface ------*- C -*-===//
//
//                     The HBDK Simulator Infrastructure
//
// This file is subject to the terms and conditions defined in file
// 'LICENSE.txt', which is part of this source code package.
//
//===----------------------------------------------------------------===//

#ifndef HBDK_SIM_IPU_H_
#define HBDK_SIM_IPU_H_
#pragma once

#include "hbdk_config.h"
#include "hbdk_error.h"
#include "hbdk_march.h"
#include "hbdk_type.h"

#ifdef __cplusplus
#include <cstdint>
#include <cstring>
#else
#include <stdint.h>
#include <string.h>
#endif  // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { HBSIM_YUV420SP = 0x1, HBSIM_YUV422SP = 0x2, HBSIM_YUV444 = 0x3 } hbsim_img_format;

/* members of the following config structures hold the same names as VIO config file */

typedef struct {
  hbsim_img_format input_format; /* bernoulli/bernoulli2 support YUV444 and YUV422SP input */
  uint32_t input_image_height;
  uint32_t input_image_width;
  uint32_t st_w;    /* start W of the ROI for crop */
  uint32_t st_h;    /* start H of the ROI for crop */
  uint32_t ed_w;    /* end W of the ROI for crop */
  uint32_t ed_h;    /* end H of the ROI for crop */
  bool crop_enable; /* bypass crop module if set as false, output will be same as input */
} hbsim_crop_config;

typedef struct {
  uint32_t st_w;        /* width of the input image */
  uint32_t st_h;        /* height of the input image */
  uint32_t ed_w;        /* width of the output image */
  uint32_t ed_h;        /* height of the output image */
  uint32_t pre_scale_x; /* factor for pre-scale in x-axis */
  uint32_t pre_scale_y; /* factor for pre-scale in y-axis */
  /* When scaler input image size is larger than or equal to twice of the dest size,
   * we suggest pre-down scale the image to 1/2^(pre_scale_x) of the original size by set pre_scale_x/y.
   * the hardware implement the pre-down scale by taking average value of multiple lines/columns
   */
  union {
    char reserve[256];
    struct {
      bool bypass_x; /* if true, skip bilinear interpolation on x-axis */
      bool bypass_y; /* if true, skip bilinear interpolation on y-axis */
    } x2;
    struct {
      bool scale_enable;
    } x2a;
  } platform_config;
} hbsim_scaler_config;

typedef struct {
  uint32_t roi_l;  /* minimum x coordinate of ROI. must be even and inside the image */
  uint32_t roi_t;  /* minimum y coordinate of ROI. must be even and inside the image */
  uint32_t src_w;  /* width of ROI. ROI can not exceed image */
  uint32_t src_h;  /* height of ROI. ROI can not exceed image */
  uint32_t factor; /* scale factor = 64 / (64 + factor) when scale factor = 0, this layer is disabled. */
  /* ROI layer width = ((src_width - 1) * 64 / (64 + factor) +1 >> 1) << 1 */
  /* ROI layer height = ((src_height - 1) * 64 / (64 + factor) +1 >> 1) << 1 */
} hbsim_pyramid_roi_layer_config;

typedef struct {
  uint32_t src_w;        /* width of the input image */
  uint32_t src_h;        /* height of the input image */
  uint32_t ds_layer_en;  /* number of down sample layers, 4~23 */
  uint32_t ds_uv_bypass; /* if true, skip uv channel when down sampling */
  uint32_t us_layer_en;  /* number of up sample layers, 0~6 */
  uint32_t us_uv_bypass; /* if true, skip uv channel when up sampling */
} hbsim_pyramid_control_config;

typedef struct {
  /* ROI layer 1 2 3 is base on layer 0 */
  hbsim_pyramid_roi_layer_config roi_layer1;
  hbsim_pyramid_roi_layer_config roi_layer2;
  hbsim_pyramid_roi_layer_config roi_layer3;
  /* ROI layer 5 6 7 is base on layer 4 */
  hbsim_pyramid_roi_layer_config roi_layer5;
  hbsim_pyramid_roi_layer_config roi_layer6;
  hbsim_pyramid_roi_layer_config roi_layer7;
  /* ROI layer 9 10 11 is base on layer 8 */
  hbsim_pyramid_roi_layer_config roi_layer9;
  hbsim_pyramid_roi_layer_config roi_layer10;
  hbsim_pyramid_roi_layer_config roi_layer11;
  /* ROI layer 13 14 15 is base on layer 12 */
  hbsim_pyramid_roi_layer_config roi_layer13;
  hbsim_pyramid_roi_layer_config roi_layer14;
  hbsim_pyramid_roi_layer_config roi_layer15;
  /* ROI layer 17 18 19 is base on layer 16 */
  hbsim_pyramid_roi_layer_config roi_layer17;
  hbsim_pyramid_roi_layer_config roi_layer18;
  hbsim_pyramid_roi_layer_config roi_layer19;
  /* ROI layer 21 22 23 is base on layer 20 */
  hbsim_pyramid_roi_layer_config roi_layer21;
  hbsim_pyramid_roi_layer_config roi_layer22;
  hbsim_pyramid_roi_layer_config roi_layer23;
} hbsim_pyramid_down_sample_config;

typedef struct {
  hbsim_pyramid_roi_layer_config us_layer0;
  /* bernoulli hardware us layer 0 factor is pre-set as 50. (up-scale by 1.28) */
  hbsim_pyramid_roi_layer_config us_layer1;
  /* bernoulli hardware us layer 0 factor is pre-set as 40. (up-scale by 1.60) */
  hbsim_pyramid_roi_layer_config us_layer2;
  /* bernoulli hardware us layer 0 factor is pre-set as 32. (up-scale by 2.00) */
  hbsim_pyramid_roi_layer_config us_layer3;
  /* bernoulli hardware us layer 0 factor is pre-set as 25. (up-scale by 2.56) */
  hbsim_pyramid_roi_layer_config us_layer4;
  /* bernoulli hardware us layer 0 factor is pre-set as 20. (up-scale by 3.20) */
  hbsim_pyramid_roi_layer_config us_layer5;
  /* bernoulli hardware us layer 0 factor is pre-set as 16. (up-scale by 4.00) */
} hbsim_pyramid_up_sample_config;

/* members of the above config structures hold the same names as VIO config file */

typedef struct {
  const uint8_t *yuv420sp_y;
  const uint8_t *yuv420sp_uv;
  uint32_t width;
  uint32_t height;
  uint32_t step;
  /* Each line of the output may occupy more bytes than the width needed due to bus alignment.
   * Assume the result width = W, height = H and step = S.
   * Results returned by the APIs in this header only guarantee the data inside HxW is same with hardware.
   * Padding value is platform-determined and is not clear in the C model. Please do not rely on these
   * values in any case.
   */
} hbsim_yuv420sp_output;

/* This API can only parse VIO configuration file for X2J2 at current version */
HBDK_PUBLIC hbrt_error_t hbsimParseConfigJson(hbsim_crop_config *crop_config, hbsim_scaler_config *scaler_config,
                                              hbsim_pyramid_control_config *pym_control_config,
                                              hbsim_pyramid_down_sample_config *pym_down_sample_config,
                                              hbsim_pyramid_up_sample_config *pym_up_sample_config,
                                              const char *config_filename, hbrt_march_t march);

/* IPU scaler related API.
 * Call hbsimScalerInit() once at beginning.
 * Call hbsimScalerSetInput() -> hbsimScalerProcess() -> hbsimScalerGetResult() for each data frame.
 * Call hbsimScalerRelease() to release memory if no more data need to be processed.
 */
typedef void *hbsim_scaler_handle;
HBDK_PUBLIC hbrt_error_t hbsimScalerInit(hbsim_scaler_handle *scaler_handle, hbsim_crop_config crop_config,
                                         hbsim_scaler_config scaler_config, hbrt_march_t march);
HBDK_PUBLIC hbrt_error_t hbsimScalerSetInput(hbsim_scaler_handle scaler_handle, const uint8_t *input_y,
                                             const uint8_t *input_u, const uint8_t *input_v);
HBDK_PUBLIC hbrt_error_t hbsimScalerProcess(hbsim_scaler_handle scaler_handle);
HBDK_PUBLIC hbrt_error_t hbsimScalerGetResult(hbsim_scaler_handle scaler_handle, hbsim_yuv420sp_output *output);
HBDK_PUBLIC hbrt_error_t hbsimScalerRelease(hbsim_scaler_handle scaler_handle);

typedef struct {
  hbsim_yuv420sp_output down_sample_layer_output[24];
  hbsim_yuv420sp_output up_sample_layer_output[6];
} hbsim_pyramid_output;

/* IPU pyramid related API.
 * Call hbsimPyramidInit() once at beginning.
 * Call hbsimPyramidSetInput() -> hbsimPyramidProcess() -> hbsimPyramidGetResult() for each data frame.
 * Call hbsimPyramidRelease() to release memory if no more data need to be processed.
 */
typedef void *hbsim_pyramid_handle;
HBDK_PUBLIC hbrt_error_t hbsimPyramidInit(hbsim_pyramid_handle *pym_handle, hbsim_pyramid_control_config control_config,
                                          hbsim_pyramid_down_sample_config down_sample_config,
                                          hbsim_pyramid_up_sample_config up_sample_config, hbrt_march_t march);

HBDK_PUBLIC hbrt_error_t hbsimPyramidSetInput(hbsim_pyramid_handle pym_handle, const uint8_t *input_y,
                                              const uint8_t *input_uv);

HBDK_PUBLIC hbrt_error_t hbsimPyramidProcess(hbsim_pyramid_handle pym_handle);

HBDK_PUBLIC hbrt_error_t hbsimPyramidGetResult(hbsim_pyramid_handle pym_handle, hbsim_pyramid_output *output);

HBDK_PUBLIC hbrt_error_t hbsimPyramidRelease(hbsim_pyramid_handle pym_handle);

/* IPU utility functions. */
HBDK_PUBLIC hbrt_error_t hbsimUvConvertYuv444ToYuv422sp(uint8_t *output_422sp_u, uint8_t *output_422sp_v,
                                                        uint32_t height, uint32_t width, const uint8_t *input_yuv444_u,
                                                        const uint8_t *input_yuv444_v);

HBDK_PUBLIC hbrt_error_t hbsimUvConvertYuv422spToYuv420sp(uint8_t *output_420sp_u, uint8_t *output_420sp_v,
                                                          uint32_t height, uint32_t width,
                                                          const uint8_t *input_yuv420sp_u,
                                                          const uint8_t *input_yuv420sp_v);

/* Ayers Pyramid Related API */

#define HBSIM_AYERS_DS_LAYER_NUM (6U)
#define HBSIM_AYERS_US_LAYER_NUM (1U)
#define HBSIM_AYERS_BASIC_LAYER_NUM (5U)

typedef enum {
  HBSIM_MIRROR_101 = 0,
  HBSIM_EXTEND,
  HBSIM_CONSTANT,
} hbsim_pad_mode;

typedef struct {
  int32_t vertical[5]; /* vertical 5x1 template coefficient, 6-bit signed */
  int32_t horizon[5];  /* horizon  1x5 template coefficient, 6-bit signed */
  uint32_t right_shift;
  /* filter on vertical axis first, then on horizon axis. this shift is for the final result */
  /*
   * For gaussian basic layer, assume the output is stored in g[][], and source is stored in s[][]
   * g[x][y] = ((sigma(j=0, 4)((sigma(i=0, 4)(s[x-2+i][y-2+j])*vertical[i]))*horizon[j]) + 1 << (right_shift -1))
   *           >> right_shift
   */
} hbsim_gaussian_filter_coef;

typedef struct {
  /* for ds layers, step = factor + 65536, scale = 65536 / (factor + 65536) */
  /* for us layers, step = factor, scale = 65536 / factor */
  uint32_t factor_vertical;
  uint32_t factor_horizon;
  int32_t left; /* starting horizontal coordinate in source image */
  int32_t top;  /* starting vertical coordinate in source image */
  /* initial phase, signed 9 bits for us layers, unsigned 8 bits for ds layers */
  /* us layers phase must be in range [16384, 65536) */
  /* ROI is for input */
  int32_t phase_vertical;
  int32_t phase_horizon;
  uint32_t out_w;
  uint32_t out_h;
  uint32_t basic_sel;
  /* source layer flag, 0:source, 1:bilinear, 2:gaussian, only for down scale layers */
  uint32_t basic_layer;
  /* source basic layer index, only for down scale */
  int32_t stride; /* output line stride, must be aligned to 16 */
} hbsim_ayers_roi_layer_config;

typedef struct {
  int32_t start_h;
  int32_t start_w;
  int32_t output_length_h;
  int32_t output_length_w;
  /* ROI is for output */
  int32_t stride; /* output line stride, must be aligned to 16 */
} hbsim_ayers_basic_layer_config;

typedef struct {
  hbsim_img_format input_format;
  uint32_t input_width;
  uint32_t input_height;
  uint32_t bilinear_basic_layer_num;
  uint32_t gaussian_basic_layer_num;

  int32_t bilinear_out_layer_enable;
  /* output enable for each bilinear basic layer, bit mask, must be in range [0, (1 << 5)) */
  int32_t bilinear_out_uv_bypass;
  /* bypass uv flag for each bilinear basic layer, bit mask, must be in range [0, (1 << 5)) */
  int32_t gaussian_out_layer_enable;
  /* output enable for each gaussian basic layer, bit mask, must be in range [0, (1 << 5)) */
  int32_t gaussian_out_uv_bypass;
  /* bypass uv flag for each gaussian basic layer, bit mask, must be in range [0, (1 << 5)) */

  uint32_t ds_layer_en;
  /* output enable for each down sample layer, bit mask, must be in range [0, (1 << 6)) */
  uint32_t ds_uv_bypass;
  /* bypass uv flag for each down sample layer, bit mask, must be in range [0, (1 << 6)) */
  uint32_t us_layer_en;
  /* output enable for each up sample layer, bit mask, must be in range [0, (1 << 6)) */
  uint32_t us_uv_bypass;
  /* bypass uv flag for each up sample layer, bit mask, must be in range [0, (1 << 6)) */
  hbsim_pad_mode pad_mode;
  uint8_t pad_const_value;
  hbsim_gaussian_filter_coef gaussian_filter_coef_y;
  hbsim_gaussian_filter_coef gaussian_filter_coef_uv;
  hbsim_ayers_roi_layer_config ds_roi_config[HBSIM_AYERS_DS_LAYER_NUM];
  hbsim_ayers_roi_layer_config us_roi_config[HBSIM_AYERS_US_LAYER_NUM];
  hbsim_ayers_basic_layer_config bilinear_basic_config[HBSIM_AYERS_BASIC_LAYER_NUM];
  hbsim_ayers_basic_layer_config gaussian_basic_config[HBSIM_AYERS_BASIC_LAYER_NUM];
  hbsim_ayers_basic_layer_config source_layer_config;
} hbsim_ayers_pyramid_config;

typedef struct {
  hbsim_yuv420sp_output bilinear_basic_layer[HBSIM_AYERS_BASIC_LAYER_NUM];
  hbsim_yuv420sp_output gaussian_basic_layer[HBSIM_AYERS_BASIC_LAYER_NUM];
  hbsim_yuv420sp_output ds_roi_layer[HBSIM_AYERS_DS_LAYER_NUM];
  hbsim_yuv420sp_output us_roi_layer[HBSIM_AYERS_US_LAYER_NUM];
} hbsim_ayers_pyramid_output;

/* IPU pyramid related API.
 * Call hbsimPyramidInitAyers() once at beginning.
 * Call hbsimPyramidSetInput() -> hbsimPyramidProcess() -> hbsimPyramidGetResultAyers() for each data frame.
 * Call hbsimPyramidRelease() to release memory if no more data need to be processed.
 * hbsimPyramidSetInput(), hbsimPyramidProcess() and hbsimPyramidRelease() reuse former interfaces.
 */
HBDK_PUBLIC hbrt_error_t hbsimPyramidInitAyers(hbsim_pyramid_handle *pym_handle,
                                               const hbsim_ayers_pyramid_config *config);

HBDK_PUBLIC hbrt_error_t hbsimPyramidGetResultAyers(hbsim_pyramid_handle pym_handle,
                                                    hbsim_ayers_pyramid_output *output);

#ifdef __cplusplus
}
#endif

#endif  // HBDK_SIM_IPU_H_
