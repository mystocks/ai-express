/*!
 * Copyright (c) 2016-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     venc_client.h
 * \Author   ronghui.zhang
 * \Version  1.0.0.0
 * \Date     2020/5/12
 * \Brief    implement of api header
 */

#ifndef INCLUDE_UVCPLUGIN_UVC_SERVER_H_
#define INCLUDE_UVCPLUGIN_UVC_SERVER_H_
#include "video_queue.h"
#include "uvc/uvc_gadget.h"
#include <string>
#include <thread>
#include "hb_vio_interface.h"
#include "hb_vps_api.h"

#include "hb_comm_venc.h"
#include "hb_venc.h"
#include "hb_vdec.h"
#include "uvcplugin_config.h"

namespace horizon {
namespace vision {
namespace xproto {
namespace Uvcplugin {
/* pix format supported */
enum uvc_fourcc_format {
	UVC_FORMAT_YUY2 = 0,
	UVC_FORMAT_NV12,
	UVC_FORMAT_MJPEG,
	UVC_FORMAT_H264,
	UVC_FORMAT_H265,
};

#define interval_cost_trace_in() \
	struct timespec t1, t2; \
	static long t1_last = 0; \
 \
	clock_gettime(CLOCK_MONOTONIC, &t1); \
	long t1_ms = t1.tv_sec * 1000 + t1.tv_nsec / (1000 * 1000); \
	long intv = t1_ms - t1_last;

#define interval_cost_trace_out() \
	clock_gettime(CLOCK_MONOTONIC, &t2); \
 \
	long t2_ms = t2.tv_sec * 1000 + t2.tv_nsec / (1000 * 1000); \
	long dur = t2_ms - t1_ms; \
	printf("fill buffer(%d bytes) t1_ms(%ldms) t2_ms(%ldms) " \
		"interval (%ldms), and cost (%ldms)\n", \
		*buf_len, t1_ms, t2_ms, intv, dur); \
	t1_last = t1_ms;



typedef enum {
	VENC_INVALID = 0,
	VENC_H264,
	VENC_H265,
	VENC_MJPEG,
} venc_type;

struct media_info {
	enum uvc_fourcc_format format;
	venc_type	vtype;
	int		vch;
	uint32_t		width;
	uint32_t		height;

	/* store one encoded video stream */
	VIDEO_STREAM_S vstream;
};

struct single_buffer {
	VIDEO_STREAM_S vstream;

	int used;
	pthread_mutex_t mutex;
};

/* structure declare */
struct uvc_params {
	unsigned int width;
	unsigned int height;
	int format;
	int io_method;
	int bulk_mode;
	int nbufs;
	int mult;
	int burst;
	int speed;
	int h264_quirk;
};


class UvcServer {
   friend class VencClient;
 public:
   UvcServer();
   ~UvcServer();
   int Init(int width, int height, UvcConfig::VideoType type);
   int Start();
   int Stop();
   int DeInit();
/* function declare */
 public:
    void uvc_gadget_user_params_init(struct uvc_params *params);
    int uvc_gadget_init(struct uvc_context **pctx, char *uvc_devname,
                    char *v4l2_devname, struct uvc_params *user_params);
    void uvc_set_streamon_handler(struct uvc_context *ctx,
                                uvc_streamon_callback_fn cb_fn,
                                void *userdata);
	void uvc_set_prepare_data_handler(struct uvc_context *ctx,
				  uvc_prepare_buffer_callback_fn cb_fn,
				  void *userdata);
    void uvc_set_release_data_handler(struct uvc_context *ctx,
                uvc_release_buffer_callback_fn cb_fn, void *userdata);
    int uvc_gadget_start(struct uvc_context *ctx);
    int uvc_gadget_stop(struct uvc_context *ctx);
    void uvc_gadget_deinit(struct uvc_context *ctx);

    /* helper function */
    static unsigned int uvc_format_to_fcc(enum uvc_fourcc_format format);

 private:
    char *udc_find_video_device(const char *udc, const char *function);
    void uvc_loop(void *arg);
    char *path_join(const char *dirname, const char *name);

    static void uvc_streamon_off(struct uvc_context *ctx, int is_on, void *userdata);

    static int uvc_get_frame(struct uvc_context *ctx,
				void **buf_to, int *buf_len, void **entity,
				void *userdata);
                
    static void uvc_release_frame(struct uvc_context *ctx,
				     void **entity, void *userdata);

    int uvc_init_with_params(venc_type type, int vechn, int width, int height);

 public:
   static struct single_buffer *mono_q;
   int uvc_stream_on ;

 private:
   struct uvc_params params;
   static struct uvc_context *uvc_ctx;
   char *uvc_devname;       /* uvc Null, lib will find one */

   char *v4l2_devname;  

   static uint64_t width;
   static uint64_t height;
   venc_type vtype_;

   std::shared_ptr<std::thread> worker_;
};
}  // namespace Uvcplugin
}  // namespace xproto
}  // namespace vision
}  // namespace horizon
#endif  /* _UVC_GADGET_API_H_ */
