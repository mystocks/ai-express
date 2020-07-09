/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef HB_X2A_VIO_HB_VIO_INTERFACE_H
#define HB_X2A_VIO_HB_VIO_INTERFACE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <sys/time.h>

#define HB_VIO_BUFFER_MAX_PLANES 3
#define HB_VIO_BUFFER_PREPARE 4

// callback enable when bit set 1 else disable the cb
#define HB_VIO_IPU_DS0_CB_MSG	0x001
#define HB_VIO_IPU_DS1_CB_MSG	0x002
#define HB_VIO_IPU_DS2_CB_MSG	0x004
#define HB_VIO_IPU_DS3_CB_MSG	0x008
#define HB_VIO_IPU_DS4_CB_MSG	0x010
#define HB_VIO_IPU_US_CB_MSG	0x020
#define HB_VIO_PYM_CB_MSG		0x040
#define HB_VIO_DIS_CB_MSG		0x080


typedef enum VIO_INFO_TYPE_S {
	HB_VIO_CALLBACK_ENABLE = 0,
	HB_VIO_CALLBACK_DISABLE,
	HB_VIO_IPU_SIZE_INFO,	//reserve for ipu size setting in dis
	HB_VIO_IPU_US_IMG_INFO,
	HB_VIO_IPU_DS0_IMG_INFO,
	HB_VIO_IPU_DS1_IMG_INFO,
	HB_VIO_IPU_DS2_IMG_INFO,
	HB_VIO_IPU_DS3_IMG_INFO,
	HB_VIO_IPU_DS4_IMG_INFO,
	HB_VIO_INFO_MAX
} VIO_INFO_TYPE_E;

typedef enum VIO_CALLBACK_TYPE {
	HB_VIO_IPU_DS0_CALLBACK = 0,
	HB_VIO_IPU_DS1_CALLBACK,
	HB_VIO_IPU_DS2_CALLBACK,
	HB_VIO_IPU_DS3_CALLBACK,
	HB_VIO_IPU_DS4_CALLBACK,
	HB_VIO_IPU_US_CALLBACK,
	HB_VIO_PYM_CALLBACK,	// 6
	HB_VIO_DIS_CALLBACK,
	HB_VIO_MAX_CALLBACK
} VIO_CALLBACK_TYPE_E;

typedef enum VIO_DATA_TYPE_S {
	HB_VIO_IPU_DS0_DATA = 0,
	HB_VIO_IPU_DS1_DATA,
	HB_VIO_IPU_DS2_DATA,
	HB_VIO_IPU_DS3_DATA,
	HB_VIO_IPU_DS4_DATA,
	HB_VIO_IPU_US_DATA,	// 5
	HB_VIO_PYM_FEEDBACK_SRC_DATA,	// for debug
	HB_VIO_PYM_DATA,
	HB_VIO_SIF_FEEDBACK_SRC_DATA,
	HB_VIO_SIF_RAW_DATA,
	HB_VIO_SIF_YUV_DATA,
	HB_VIO_ISP_YUV_DATA,	// for debug, a process result for raw feedback
	HB_VIO_GDC_DATA,
	HB_VIO_IARWB_DATA,
	HB_VIO_DATA_TYPE_MAX
} VIO_DATA_TYPE_E;

typedef enum buffer_state {
	BUFFER_AVAILABLE,
	BUFFER_PROCESS,
	BUFFER_DONE,
	BUFFER_REPROCESS,
	BUFFER_USER,
	BUFFER_INVALID
} buffer_state_e;

/* info :
 * y,uv             2 plane
 * raw              1 plane
 * raw, raw         2 plane(dol2)
 * raw, raw, raw    3 plane(dol3)
 **/
typedef struct address_info_s {
	uint16_t width;
	uint16_t height;
	uint16_t stride_size;
	char *addr[HB_VIO_BUFFER_MAX_PLANES];
	uint64_t paddr[HB_VIO_BUFFER_MAX_PLANES];
} address_info_t;

typedef struct image_info_s {
	uint16_t sensor_id;
	uint16_t pipeline_id;
	uint32_t frame_id;
	uint64_t time_stamp;	//HW time stamp
	struct timeval tv;  //system time of hal get buf
	int buf_index;
	int img_format;
	int fd[HB_VIO_BUFFER_MAX_PLANES];	//ion buf fd
	uint32_t size[HB_VIO_BUFFER_MAX_PLANES];
	uint32_t planeCount;
	VIO_DATA_TYPE_E data_type;
	buffer_state_e state;
} image_info_t;

/*
 * buf type  fd[num]                        size[num]                   addr[num]
 *
 * sif buf : fd[0(raw)]                     size[0(raw)]                addr[0(raw)]
 * sif dol2: fd[0(raw),1(raw)]              size[0(raw),1(raw)]         addr[0(raw),1(raw)]
 * sif dol3: fd[0(raw),1(raw),2(raw)]       size[0(raw),1(raw),2(raw)]  addr[0(raw),1(raw),2(raw)]
 * ipu buf : fd[0(y),1(c)]                  size[0(y),1(c)]             addr[0(y),1(c)]
 * pym buf : fd[0(all channel)]             size[0(all output)]         addr[0(y),1(c)] * 24
 * */

// normal capture buffer type, one image data output
typedef struct hb_vio_buffer_s {
	image_info_t img_info;
	address_info_t img_addr;
} hb_vio_buffer_t;

// special buffer type, multi image data output,
// but all channel output alloc one ion buffer avoid buf fragment

typedef struct pym_buffer_s {
	image_info_t pym_img_info;
	address_info_t pym[6];
	address_info_t pym_roi[6][3];
	address_info_t us[6];
} pym_buffer_t;

typedef struct osd_box_s {
	uint8_t osd_en;
	uint8_t overlay_mode;
	uint16_t start_x;
	uint16_t start_y;
	uint16_t width;
	uint16_t height;
} osd_box_t;

// for osd draw, Y info sta
typedef struct osd_sta_box_s {
	uint8_t sta_en;
	uint16_t start_x;
	uint16_t start_y;
	uint16_t width;
	uint16_t height;
} osd_sta_box_t;

typedef struct ipu_img_info_s {
	uint16_t width;
	uint16_t height;
	uint16_t format;
	uint16_t buf_count;
} ipu_img_info_t;

enum Format {
	HB_RGB,
	HB_RAW,
	HB_YUV422,
	HB_YUV420SP		// only support yuv420sp
};

#define HB_VIO_X2COMP_SUPPORT

#ifdef HB_VIO_X2COMP_SUPPORT

/* for get info type */
#define HB_VIO_SRC_INFO			1
#define HB_VIO_PYM_INFO			2
#define HB_VIO_SIF_INFO			3
#define HB_VIO_IPU_STATE_INFO	4
#define HB_VIO_FRAME_START_INFO	5
#define HB_VIO_PYM_MULT_INFO	6
#define HB_VIO_SRC_MULT_INFO	7
#define HB_VIO_FEEDBACK_SRC_INFO 8
#define HB_VIO_FEEDBACK_FLUSH 9
#define HB_VIO_FEEDBACK_SRC_MULT_INFO 10
#define HB_VIO_PYM_INFO_CONDITIONAL 11

/* for ds|us */
#define DOWN_SCALE_MAIN_MAX 	6
#define DOWN_SCALE_MAX	24
#define UP_SCALE_MAX	6

typedef struct addr_info_s {
	uint16_t width;
	uint16_t height;
	uint16_t step;
	uint64_t y_paddr;
	uint64_t c_paddr;
	uint64_t y_vaddr;
	uint64_t c_vaddr;
} addr_info_t;

typedef struct src_img_info_s {
	int cam_id;
	int slot_id;
	int img_format;
	int frame_id;
	int64_t timestamp;
	addr_info_t src_img;
	addr_info_t scaler_img;
} src_img_info_t;

typedef struct img_info_s {
	int slot_id;					// getted slot buff
	int frame_id;					// for x2 may be 0 - 0xFFFF or 0x7FFF
	int64_t timestamp;				// BT from Hisi; mipi & dvp from kernel time
	int img_format;					// now only support yuv420sp
	int ds_pym_layer;				// get down scale layers
	int us_pym_layer;				// get up scale layers
	addr_info_t src_img;			// for x2 src img = crop img
	addr_info_t down_scale[DOWN_SCALE_MAX];
	addr_info_t up_scale[UP_SCALE_MAX];
	addr_info_t down_scale_main[DOWN_SCALE_MAIN_MAX];
	int cam_id;
} img_info_t;

int hb_vio_start(void);
int hb_vio_stop(void);
int hb_vio_get_info(uint32_t info_type, void *data);
int hb_vio_set_info(uint32_t info_type, void *data);
int hb_vio_free_info(uint32_t info_type, void *data);
int hb_vio_get_info_conditional(uint32_t info_type, void *data, int time);

int hb_vio_pym_process(src_img_info_t *src_img_info);
int hb_vio_src_free(src_img_info_t *src_img_info);
#endif

typedef void (*data_cb) (uint32_t pipe_id, uint32_t event, void *data,
			 void *userdata);

int hb_vio_init(const char *cfg_file);
int hb_vio_deinit();
int hb_vio_start_pipeline(uint32_t pipeline_id);
int hb_vio_stop_pipeline(uint32_t pipeline_id);

// pipeline means software workflow instance,
// one sensor maps one software instance
int hb_vio_set_callbacks(uint32_t pipeline_id, VIO_CALLBACK_TYPE_E type,
			 data_cb cb);

int hb_vio_set_param(uint32_t pipeline_id, VIO_INFO_TYPE_E info_type,
		     void *info);
int hb_vio_get_param(uint32_t pipeline_id, VIO_INFO_TYPE_E info_type,
		     void *info);

int hb_vio_get_data(uint32_t pipeline_id, VIO_DATA_TYPE_E data_type,
		    void *data);
int hb_vio_get_data_conditional(uint32_t pipeline_id, VIO_DATA_TYPE_E data_type,
				void *data, int time);

/*ipu channel    osd
 *   ds 0      ==> osd 0
 *   ds 1      ==> osd 1
 *   us 0      ==> osd 2
 */

int hb_vio_init_osd_layer(uint32_t pipeline_id, uint32_t osd_layer,
		osd_box_t osd_data[3]);//ipu channel with osd(ds 0, 1 us 0)
int hb_vio_deinit_osd_layer(uint32_t pipeline_id, uint32_t osd_layer);
int hb_vio_get_osd_addr(uint32_t pipeline_id, uint32_t osd_layer,
			address_info_t(*osd_map_buf)[3]);
int hb_vio_update_osd(uint32_t pipeline_id, uint32_t osd_layer, uint32_t enable,
		      uint32_t colour_invert_enable);
int hb_vio_set_osd_sta(uint32_t pipeline_id, uint32_t osd_layer,
		       uint8_t osd_sta_level[3], osd_sta_box_t osd_sta[8]);
int hb_vio_get_osd_sta(uint32_t pipeline_id, uint32_t osd_layer,
		       uint16_t osd_sta_bin_value[8][4]);

int hb_vio_free_ipubuf(uint32_t pipeline_id, hb_vio_buffer_t * dst_img_info);
int hb_vio_free_gdcbuf(uint32_t pipeline_id, hb_vio_buffer_t * dst_img_info);
int hb_vio_free_pymbuf(uint32_t pipeline_id, VIO_DATA_TYPE_E data_type,
		    void *img_info);
int hb_vio_run_pym(uint32_t pipeline_id, hb_vio_buffer_t * src_img_info);
int hb_vio_run_gdc(uint32_t pipeline_id, hb_vio_buffer_t * src_img_info,
		       hb_vio_buffer_t * dst_img_info, int rotate);

void vio_dis_crop_set(uint32_t pipe_id, uint32_t info, void *data,
							 void *userdata);


// for debug
int hb_vio_raw_dump(uint32_t pipeline_id, hb_vio_buffer_t * raw_img,
		hb_vio_buffer_t * yuv);
int hb_vio_raw_feedback(uint32_t pipeline_id, hb_vio_buffer_t * feedback_src,
						hb_vio_buffer_t * isp_dst_yuv);

#ifdef __cplusplus
}
#endif

#endif //HB_X2A_VIO_HB_VIO_INTERFACE_H
