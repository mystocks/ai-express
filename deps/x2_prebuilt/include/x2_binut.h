#ifndef __MATRIX_BINUT_H__
#define __MATRIX_BINUT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "3rd_party_lib/plat_cnn.h"
#include "hbdk/hbrt_layout.h"

#define MAX_MODEL_INPUT_NUM 48
#define MAX_MODEL_OUTPUT_NUM 48
#define MAX_MODEL_NUM 32
#define MAX_SEGMENT_NUMBER (32)
#define MAX_AUGMENTATION_PARAM_NUMBER (5)
#define MAX_SCALE_VEC_NUM 2048
#define MAX_SEG_INPUT_NUM 10
#define CNN_PLAT_VERSION ("0.9.X")

/*!
 * @brief model brief info structure
 */
typedef struct x2_model_info_s {
  uint32_t ins_len;
  uint32_t ins_addr_off;
  uint32_t param_addr_off;
  uint32_t dyn_ddr_size;
  uint32_t intermediate_feature_size;
  const char *compiler_flags;
} x2_model_info_t;


/*!
 * @brief model output ddr address type
 */
typedef enum x2_addr_type_s { X2_ADDR_UNKNOWN, X2_ADDR_OFFSET, X2_ADDR_IMMEDIATE } x2_addr_type_t;

/*!
 * @brief model input data type
 */
typedef enum x2_input_source_s {
  X2_INPUT_SOURCE_UNKONWN,
  X2_INPUT_SOURCE_DDR,  // input from DDR data
  X2_INPUT_SOURCE_RSZ,  // input from resizer
  X2_INPUT_SOURCE_PYM   // input from pyramid
} x2_input_source_t;

typedef enum x2_output_type_s {
  X2_OUTPUT_TYPE_UNDEFINED,
  X2_OUTPUT_TYPE_PARSINGPP,
  X2_OUTPUT_TYPE_NMS,
  X2_OUTPUT_TYPE_CHANNEL_SWAPPED,
  X2_OUTPUT_TYPE_SHIFT_ALIGNED,
  X2_OUTPUT_TYPE_RPP,
  X2_OUTPUT_TYPE_ANCHOR,
} x2_output_type_t;

typedef enum x2_memory_region_type_e {
  MEMORY_REGION_INPUT,
  MEMORY_REGION_OUTPUT,
  MEMORY_REGION_INTERMEDIATE,
} x2_memory_region_type_t;

/*!
 * @brief model input/output data description structure
 */
typedef struct x2_feature_info_s {
  uintptr_t ddr_addr;
  uint32_t ddr_base_reg;
  uint32_t byte_size;
  uint32_t aligned_nhwc[4];
  uint32_t padding_nhwc[4];
  const uint32_t *p_shift;
  x2_addr_type_t addr_type;
  hbrt_layout_t layout_flag;
  x2_input_source_t input_source;
  x2_output_type_t output_type;
  uint32_t elem_size;  // 1: int8; 4: int32
  x2_memory_region_type_t memory_region_type;
  int32_t nms_top_n;
  const char *p_desc;
  const char *name;
} x2_feature_info_t;

typedef struct {
  uint32_t aligned_dims[4];
  uint32_t real_dims[4];
  hbrt_layout_t layout;
  x2_memory_region_type_t region;
  uint32_t offset;
} segment_feature_info_t;

typedef struct {
  uint32_t middle_layer_id;
  uint32_t base_img_scale;
  uint32_t roi_fd_num;
  uint32_t roi_fd_stride_h[MAX_MODEL_INPUT_NUM];
  uint32_t roi_fd_stride_w[MAX_MODEL_INPUT_NUM];
  uint16_t clip_box_flag;
  uint16_t augmentation_flag;
  float augmentation_param[MAX_AUGMENTATION_PARAM_NUMBER];
  int32_t max_pooling_num;
  int32_t force_pooling_num;
  int32_t pooling_method;  // 0 for max, 1 for average
  int32_t roi_align_output_h;
  int32_t roi_align_output_w;
  uint32_t padding_mode;
} roi_align_info_t;

typedef struct {
  uint32_t orig_img_h_;
  uint32_t orig_img_w_;
  uint32_t nms_threshold_;
  uint32_t exp_shift_num_;
  uint32_t input_shift_num_;
  uint32_t post_nms_top_n_;
  uint32_t pre_nms_top_n_;

  uint32_t fdin_num_;
  int32_t anchor_off_list[MAX_MODEL_INPUT_NUM];
  int32_t fd_stride_list[MAX_MODEL_INPUT_NUM];
  int32_t class_num_list[MAX_MODEL_INPUT_NUM];
  int32_t class_off_list[MAX_MODEL_INPUT_NUM];

  int32_t anchor_table_number;
  int32_t *p_anchor_table;
  int32_t exp_table_number;
  int32_t *p_exp_table;
} sortnms_info_t;

typedef struct {
  uint32_t orig_img_h_;
  uint32_t orig_img_w_;
  uint32_t class_num_;
  uint32_t nms_top_n_;
  float nms_threshold_;
  float score_threshold_;

  float bbox_delta_mean[4];  // type: float
  float bbox_delta_std[4];   // type: float
} rcnn_postprocess_info_t;

typedef struct {
  uint32_t scale_fp32_vec_number;
  float scale_fp32_vec[MAX_SCALE_VEC_NUM];
} i32tof32convert_info_t;

#define SEGMENT_INPUT_FEATURE_MAX_NUMBER (8)
#define SEGMENT_OUTPUT_FEATURE_MAX_NUMBER (32)
#define SEGMENT_INT32_PARAM_MAX_NUMBER (16)
typedef struct _model_segment_info_t {
  const char *name;
  uint32_t operator_id;  // 0 stands for BPU
  uint32_t input_num;
  uint32_t output_num;
  x2_feature_info_t input_features[SEGMENT_INPUT_FEATURE_MAX_NUMBER];
  x2_feature_info_t output_features[SEGMENT_OUTPUT_FEATURE_MAX_NUMBER];
  // cpu only
  int32_t int32_params[SEGMENT_INT32_PARAM_MAX_NUMBER];
  // bpu only
  uint32_t ins_len;
  uint32_t ins_addr_off;
  uint32_t param_addr_off;
  uint32_t dyn_ddr_size;
  uint32_t pe_num;
  // roialign info
  roi_align_info_t roi_align_info;
  sortnms_info_t sortnms_info;
  rcnn_postprocess_info_t rcnn_info;
  int32_t seg_id;
  uint32_t input_seg_num;
  int32_t input_seg_idx[MAX_SEG_INPUT_NUM];

  // i32tof32convert info
  i32tof32convert_info_t i32tof32convert_info;

  int use_cpu_buffer;
  int is_roialign_segment;
  int is_i32tof32convert_segment;
  int is_rpp_segment;
  bpu_addr_t code_cache_addr;
  uint32_t code_cache_size;
  void *cpu_buffer;
} model_segment_info_t;

extern int cnn_init(const char *hbm_path);

extern uint32_t binut_getModelNumber();

extern const char **binut_getModelNames();

extern uint32_t binut_getModelHandle(const char *model_name);

extern const x2_model_info_t *binut_getModelInfo(uint32_t handle);

extern uint32_t binut_getInputLayerNum(uint32_t handle);

extern const x2_feature_info_t *binut_getModelInputInfo(uint32_t handle);

extern uint32_t binut_getOutputLayerNum(uint32_t handle);

extern const x2_feature_info_t *binut_getModelOutputInfo(uint32_t handle);

extern uint32_t binut_getOutputSize(uint32_t handle);

extern const char *binut_getSolutionVersion();

extern int32_t binut_getBinutVersion();

extern uint16_t binut_getMachineType();

extern uint32_t binut_getHBCCVersion();

extern uint32_t binut_getSegmentNum(uint32_t handle);

extern const model_segment_info_t *binut_getSegmentInfos(uint32_t handle);
extern const model_segment_info_t *binut_getSegmentInfoByName(const char *segment_name);

/**
 * get crc32 table
 * @param num [out] number of tuple-4
 * @return pointer of array tuple(type, addr, size, crc32)
 * @note type: 0x20 is inst, 0x40 is param
 *       addr: model_segment_info_t.ins_addr_off or model_segment_info_t.param_addr_off
 *       size: data size to calc crc32
 *       crc32: crc32 value
 */
extern uint32_t *binut_getCrc32Table(uint32_t *num);
/**
 * get crc32 type of inst
 * @return
 */
extern uint32_t binut_getCrc32Type_inst();
/**
 * get crc32 type of param
 * @return
 */
extern uint32_t binut_getCrc32Type_param();
/**
 * check crc32 by given type and base addr
 * @param type crc32 type, you can use like this 'type1 | type2'
 * @param addr_base base address
 * @return 0 if same, otherwise not 0
 */
extern uint32_t binut_checkCrc32(uint32_t type, const void *addr_base);
/**
 * calculate crc32 value from memory
 * @param start address
 * @param size data size
 * @return crc32 value
 */
extern uint32_t binut_Crc32ForMemory(const void *start, uint32_t size);

/**
 * Following APIs are prepared for RUNTIME code use only.
 * Users should never call these APIs explicitly.
 */


typedef struct {
  int16_t left;
  int16_t top;
  int16_t right;
  int16_t bottom;
  uint8_t score;
  uint8_t class_label;
  int16_t padding[3];
} x2_nms_output_t;

typedef struct {
  float roi_left;
  float roi_top;
  float roi_right;
  float roi_bottom;
  float roi_score;
  float roi_class_id;
} x2_rcnnpostprocess_output_t;

typedef struct {
  int8_t class_label;
  int8_t score;
} x2_segmentation_output_t;

extern uint32_t binut_getNativeOutputSize(const x2_feature_info_t *output_info, const void *source_ptr,
                                          int32_t shift_enable);

extern void binut_ConvertOutputToNativeMXNetFormat(const x2_feature_info_t *output_info, const void *source_ptr,
                                                   void *dest_ptr, int enable_shift);

extern int binut_dumpModelInput(uint32_t handle, const x2_feature_info_t *input_info, bpu_addr_t dest_ptr);

extern int binut_dumpModelInputToFile(uint32_t model_handle);

extern int binut_dumpModelOutputToFile(uint32_t model_handle, bpu_addr_t output_cpu_addr);

extern int binut_dumpHardwarePerfInfo(bpu_addr_t dest_ptr, uint32_t inst_num);

extern void rtut_getModelOutputNHWCElemSize(const char *model_name, uint32_t *output_nhwc);

typedef struct {
  int8_t argmax_id;
  int8_t max_val;
} x2_parsing_pp_output_t;

extern void cnn_printFunccall(const void *funccall);

extern enum cnn_core_type binut_getModelStartCoreType(uint32_t handle);

extern enum cnn_start_method binut_getModelStartMethod(uint32_t handle);

extern void cnn_exit();

#ifdef __cplusplus
}
#endif

#endif  //__MATRIX_BINUT_H__
