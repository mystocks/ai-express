/*
 *  Copyright (c) 2019 by Horizon
 * \file bpu_predict.h
 * \brief BPU predict API for Horizon BPU Platform.
 */

#ifndef INCLUDE_BPU_PREDICT_BPU_PREDICT_EXTENSION_H_
#define INCLUDE_BPU_PREDICT_BPU_PREDICT_EXTENSION_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

// typedef uint64_t size_t;

typedef struct hb_BPU_MEMORY_S {
  uint64_t phyAddr;
  void *virAddr;
  size_t memSize;
} BPU_MEMORY_S;

typedef enum hb_BPU_OP_TYPE_E {
  BPU_OP_TYPE_CONV,
  BPU_OP_TYPE_UNKNOWN,
} BPU_OP_TYPE_E;

typedef enum hb_BPU_LAYOUT_E {
  BPU_LAYOUT_NONE = 0,
  BPU_LAYOUT_NHWC,
  BPU_LAYOUT_NCHW,
} BPU_LAYOUT_E;

#define BPU_MODEL_MAX_SHAPE_DIM (8)
typedef struct hb_BPU_DATA_SHAPE_S {
  BPU_LAYOUT_E layout;
  int ndim;
  int d[BPU_MODEL_MAX_SHAPE_DIM];
} BPU_DATA_SHAPE_S;

typedef struct hb_BPU_RUN_CTRL_S {
  int core_id;
} BPU_RUN_CTRL_S;

typedef enum hb_BPU_DATA_TYPE_E {
  // IMG type is uint8
  BPU_TYPE_IMG_Y,
  BPU_TYPE_IMG_YUV_NV12,
  BPU_TYPE_IMG_YUV444,
  BPU_TYPE_IMG_BGR,
  BPU_TYPE_IMG_BGRP,
  BPU_TYPE_IMG_RGB,
  BPU_TYPE_IMG_RGBP,
  BPU_TYPE_TENSOR_U8,   // for uint8 tensor
  BPU_TYPE_TENSOR_S8,   // for signed int8
  BPU_TYPE_TENSOR_F32,  // for float32
  BPU_TYPE_TENSOR_S32,  // for int32
  BPU_TYPE_TENSOR_U32,  // for uint32
  BPU_TYPE_MAX,
} BPU_DATA_TYPE_E;

typedef struct hb_BPU_MODEL_NODE_S {
  BPU_OP_TYPE_E op_type;  // only used for output node
  BPU_DATA_TYPE_E data_type;
  BPU_DATA_SHAPE_S shape;
  BPU_DATA_SHAPE_S aligned_shape;
  const char *name;
  uint8_t *shifts;  // only used for output node
  int shift_len;    // only used for output node
} BPU_MODEL_NODE_S;

#define BPU_MODEL_MAX_NODE_NUM (16)
typedef struct hb_BPU_MODEL_S {
  void *handle;
  int input_num;
  int output_num;
  BPU_MODEL_NODE_S inputs[BPU_MODEL_MAX_NODE_NUM];
  BPU_MODEL_NODE_S outputs[BPU_MODEL_MAX_NODE_NUM];
} BPU_MODEL_S;

typedef struct hb_BPU_TENSOR_S {
  BPU_DATA_TYPE_E data_type;
  BPU_DATA_SHAPE_S data_shape;
  BPU_MEMORY_S data0;
  BPU_MEMORY_S data1;
  BPU_MEMORY_S data2;
} BPU_TENSOR_S;

typedef enum hb_BPU_GLOBAL_CONFIG_E {
  BPU_GLOBAL_CONFIG_MAX_TASK_NUM,
  BPU_GLOBAL_CONFIG_MAX_MEM_POOL_SIZE,
  BPU_GLOBAL_CONFIG_DEBUG,
} BPU_GLOBAL_CONFIG_E;

/*
 * \brief load model from address
 * return 0 for everything is ok, error code for encounter a problem.
 */
int HB_BPU_loadModel(const void *model_data,
                     int model_size,
                     BPU_MODEL_S *model);

/*
 * \brief release model
 *  return 0 for everything is ok, error code for encounter a problem.
 */
int HB_BPU_releaseModel(BPU_MODEL_S *model);

/*
 * \brief get error Name from error code.
 */
const char *HB_BPU_getErrorName(int error_code);

/*
 * \brief set global config by k-v way.
 */
int HB_BPU_setGlobalConfig(BPU_GLOBAL_CONFIG_E config_key,
                           const char *config_value);

typedef void *BPU_TASK_HANDLE;

/*
 * \brief there are two ways to run model, synchronous and asynchronous.
 * When in synchronous mode, calling this interface will block the current
 * thread until the model runs to completion. Otherwise, calling this interface
 * will return immediately, after which you should use HB_BPU_waitModelDone with
 * task_handle wait to the end.
 */
int HB_BPU_runModel(const BPU_MODEL_S *model,
                    const BPU_TENSOR_S input_data[],
                    const int input_num,
                    const BPU_TENSOR_S output_data[],
                    const int output_num,
                    const BPU_RUN_CTRL_S *run_ctrl,
                    bool is_sync,
                    BPU_TASK_HANDLE *task_handle);

/*
 * \brief wait for the run to end when in asynchronous mode.
 */
int HB_BPU_waitModelDone(BPU_TASK_HANDLE *task_handle);

/*
 * \brief release task when in asynchronous mode.
 */
int HB_BPU_releaseTask(BPU_TASK_HANDLE *task_handle);

// memory api
/*
 * \brief alloc bpu memory for input and output of model,
 * flag cachable indicates whether a cache is required.
 */
int HB_SYS_bpuMemAlloc(const char *name,
                       size_t alloc_mem_size,
                       bool cachable,
                       BPU_MEMORY_S *mem);

#define HB_SYS_MEM_CACHE_INVALIDATE (1)  // flush memory to CPU
#define HB_SYS_MEM_CACHE_CLEAN (2)       // flush CPU to memory

/*
 * \brief flush cache with flag.
 * HB_SYS_MEM_CACHE_INVALIDATE should be used  after write,
 * and HB_SYS_MEM_CACHE_CLEAN should be used before read .
 */
int HB_SYS_flushMemCache(const BPU_MEMORY_S *mem, int flag);

/*
 * \brief query memory whether is cachable or not.
 */
int HB_SYS_isMemCachable(const BPU_MEMORY_S *mem);

/*
 * \brief free bpu mem.
 */
int HB_SYS_bpuMemFree(BPU_MEMORY_S *mem);

typedef struct hb_BPU_ADDR_INFO_S {
  uint16_t width;
  uint16_t height;
  uint16_t step;
  uint64_t y_paddr;
  uint64_t c_paddr;
  uint64_t y_vaddr;
  uint64_t c_vaddr;
} BPU_ADDR_INFO_S;

// for ds|us
#define DOWN_SCALE_MAIN_MAX 6
#define DOWN_SCALE_MAX 24
#define UP_SCALE_MAX 6

typedef struct hb_BPU_IMG_INFO_S {
  int slot_id;        // getted slot buff
  int frame_id;       // for x2 may be 0 - 0xFFFF or 0x7FFF
  int64_t timestamp;  // BT from Hisi; mipi & dvp from kernel time
  int img_format;     // now only support yuv420sp
  int ds_pym_layer;   // get down scale layers
  int us_pym_layer;   // get up scale layers
  BPU_ADDR_INFO_S *down_scale[DOWN_SCALE_MAX];
  BPU_ADDR_INFO_S *up_scale[UP_SCALE_MAX];
  BPU_ADDR_INFO_S *down_scale_main[DOWN_SCALE_MAIN_MAX];
  int cam_id;
} BPU_IMG_INFO_S;

typedef struct hb_BPU_PYRAMID_RESULT_S {
  BPU_IMG_INFO_S result_info;
} BPU_PYRAMID_RESULT_S;

typedef struct hb_BPU_CAMERA_IMAGE_INFO_S {
  int frame_id;       // for x2 may be 0 - 0xFFFF or 0x7FFF
  int64_t timestamp;  // BT from Hisi; mipi & dvp from kernel time
  int img_format;     // now only support yuv420sp
  BPU_ADDR_INFO_S src_img;
  int cam_id;
} BPU_CAMERA_IMAGE_INFO_S;

/*
 * \brief convert layout.
 */

typedef enum hb_BPU_CONVERT_LAYOUT_METHOD_E {
  BPU_1HW1_CONVERT_METHOD = 0,
  BPU_111C_CONVERT_METHOD,
  BPU_1111_CONVERT_METHOD,
  BPU_NHWC_CONVERT_METHOD
} BPU_CONVERT_LAYOUT_METHOD_E;
/*
 * \brief convert layout.
 */
int HB_BPU_convertLayout(const BPU_MODEL_S *model,
                         void *to_data,
                         const void *from_data,
                         BPU_CONVERT_LAYOUT_METHOD_E convert_method,
                         uint32_t output_index,
                         uint32_t n_index,
                         uint32_t h_index,
                         uint32_t w_index,
                         uint32_t c_index);

typedef enum hb_BPU_RESIZE_TYPE_E {
  BPU_RESIZE_TYPE_BILINEAR
} BPU_RESIZE_TYPE_E;

typedef struct hb_BPU_RESIZE_CTRL_S {
  BPU_RESIZE_TYPE_E resize_type;
} BPU_RESIZE_CTRL_S;

typedef struct hb_BPU_ROI_S {
  int x1;
  int y1;
  int x2;
  int y2;
} BPU_ROI_S;

/*
 * \brief resize input image, layout of result is 8w4c with padding.
 */
int HB_BPU_resize(const BPU_TENSOR_S *src,
                  BPU_TENSOR_S *dest,
                  const BPU_RESIZE_CTRL_S *ctrl_param);

/*
 * \brief roi resize, layout of result is 8w4c with padding.
 */
int HB_BPU_cropAndResize(const BPU_TENSOR_S *src,
                         const BPU_ROI_S *input_roi,
                         BPU_TENSOR_S *dest,
                         const BPU_RESIZE_CTRL_S *ctrl_param);

/*
 * \brief get nhwc layout of resize result.
 */
int HB_BPU_getResizeResultWithoutPadding(const BPU_MEMORY_S *src,
                                         const BPU_DATA_TYPE_E data_type,
                                         const BPU_DATA_SHAPE_S *shape,
                                         void *dest,
                                         const int dest_size);

int HB_BPU_getImageAlignedShape(const BPU_DATA_SHAPE_S *shape,
                                int *aligned_size);
#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // INCLUDE_BPU_PREDICT_BPU_PREDICT_EXTENSION_H_
