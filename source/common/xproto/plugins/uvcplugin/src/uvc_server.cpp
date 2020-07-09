/*!
 * Copyright (c) 2016-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     uvc_server.cpp
 * \Author   ronghui.zhang
 * \Version  1.0.0.0
 * \Date     2020.5.12
 * \Brief    implement of api file
 */
#include "uvc_server.h"
#include <dirent.h>
#include <string>
#include <stdio.h>
#include <pthread.h>
#include "hobotlog/hobotlog.hpp"

namespace horizon {
namespace vision {
namespace xproto {
namespace Uvcplugin {

// #define UVC_DEBUG 1

uint64_t UvcServer::height = 1080;
uint64_t UvcServer::width = 1920;
struct single_buffer* UvcServer::mono_q = nullptr;
struct uvc_context* UvcServer::uvc_ctx = nullptr;

using std::chrono::milliseconds;

UvcServer::UvcServer() {
  v4l2_devname = nullptr;
  uvc_devname = nullptr;
  uvc_stream_on = 0;
}

UvcServer::~UvcServer() {

}

int UvcServer::Init(int width_, int height_, UvcConfig::VideoType type) {
  width = width_;
  height = height_;

  if (type == UvcConfig::VIDEO_H264) {
    vtype_ = VENC_H264;
  } else if (type == UvcConfig::VIDEO_MJPEG) {
    vtype_ = VENC_MJPEG;
  } 

  uvc_init_with_params(vtype_, 0, width_, height_);

  LOGI << "UvcServer::Init";
  return 0;
}

int UvcServer::DeInit() {
    // TODO
    return 0;
}

int UvcServer::Start() {

    return 0;
}

int UvcServer::Stop() {
    uvc_gadget_stop(uvc_ctx);
    uvc_gadget_deinit(uvc_ctx);

    return 0;
}

void UvcServer::uvc_streamon_off(struct uvc_context *ctx, int is_on, void *userdata)
{
	struct media_info *info = (struct media_info *)userdata;
	struct uvc_device *dev;
	unsigned int width, height, fcc = 0;

	if (!ctx || !ctx->udev)
		return;

	dev = ctx->udev;
	width = dev->width;
	height = dev->height;

	LOGD << "##TRY STREAMON(" << is_on << ")## " << fcc_to_string(fcc)
         << "(" << width << "x" << height << ")";

	if (dev->fcc == uvc_format_to_fcc(info->format) &&
	    width == info->width && height == info->height) {
		LOGD << "##REAL STREAMON(" << is_on << ")## "
             << fcc_to_string(fcc) << "(" << width << "x" << height << ")";
		/* stream off, do flush here.
		 * stream on is handled in fill 1st uvc buffer!!
		 */
		if (is_on) {
			LOGI << "strean on!! signal to video encoder start...";

		} else {
			LOGI << "stream off!! flush the queue!";
		}
	}
}


int UvcServer::uvc_get_frame(struct uvc_context *ctx,
				void **buf_to, int *buf_len, void **entity,
				void *userdata)
{
    LOGI << "uvc_get_frame";
	struct media_info *info = (struct media_info *)userdata;
//	int r;

	if (!info) {
		return -EINVAL;
    } else if (!ctx) {
        return -EINVAL;
    } else if (!ctx->udev) {
        return -EINVAL;
    }

#ifdef UVC_DEBUG
	interval_cost_trace_in();
#endif

	struct uvc_device *dev = ctx->udev;
	unsigned int width = dev->width;
	unsigned int height = dev->height;

	/* check request stream format */
	if (dev->fcc != uvc_format_to_fcc(info->format) ||
	    width != info->width || height != info->height)
		return -EFAULT;

	/* check mono_q */
	if (!mono_q) {
		return -EFAULT;
    }

	pthread_mutex_lock(&mono_q->mutex);
	if (!mono_q->used) {
		pthread_mutex_unlock(&mono_q->mutex);
		return -EFAULT;
	}

	/* get mono_q vstream and apply frame to *buf_to */
	*buf_to = mono_q->vstream.pstPack.vir_ptr;
	*buf_len = mono_q->vstream.pstPack.size;
	*entity = mono_q;

	pthread_mutex_unlock(&mono_q->mutex);

#ifdef UVC_DEBUG
	interval_cost_trace_out();
#endif

	return 0;
#if 0
error:
	return -EFAULT;
#endif
}

void UvcServer::uvc_release_frame(struct uvc_context *ctx,
				     void **entity, void *userdata)
{
	struct media_info *info = (struct media_info *)userdata;
	if (!ctx || !entity || !(*entity))
		return;

	pthread_mutex_lock(&mono_q->mutex);

	HB_VENC_ReleaseStream(info->vch, &mono_q->vstream);
	memset(&mono_q->vstream, 0, sizeof(VIDEO_STREAM_S));
	mono_q->used = 0;
	*entity = 0;

	pthread_mutex_unlock(&mono_q->mutex);

	return;
}

/* function definitions */
void UvcServer::uvc_gadget_user_params_init(struct uvc_params *params)
{
    /* default user params
     * YUY2, VGA360p, io_method_userptr, isoc mode, ping-pong buffers,
     * usb2.0 high speed...
     */
    params->width = width;
    params->height = height;
    if (vtype_ == VENC_MJPEG) {
      params->format = UVC_FORMAT_MJPEG;
      params->h264_quirk = 0;  /* h264_quirk: 0 - default, 1 - combine sps, pps with IDR */
    } else if (vtype_ == VENC_H264) {
      params->format = UVC_FORMAT_H264;
      params->h264_quirk = 1;  /* h264_quirk: 0 - default, 1 - combine sps, pps with IDR */
    }
    params->io_method = IO_METHOD_MMAP;
    params->bulk_mode = 0;
    params->nbufs = 2;   /* Ping-Pong buffers */
    params->mult = 0;
    params->burst = 0;
    params->speed = 1;   /* high speed as default */
}

int UvcServer::uvc_init_with_params(venc_type type, int vechn, int width, int height)
{
	static struct media_info info;
	struct uvc_params params;
	char *uvc_devname = NULL;	/* uvc Null, lib will find one */
	char *v4l2_devname = NULL;	/* v4l2 Null is Null... */
	int ret;

	LOGD << "uvc gadget int with params in";

	/* init media info */
	info.vtype = type;
	info.vch  = vechn;
	info.width = width;
	info.height = height;
	switch (type) {
	case VENC_MJPEG:
		info.format = UVC_FORMAT_MJPEG;
        break;
	case VENC_H264:
		info.format = UVC_FORMAT_H264;
        break;
	default:
		info.format = UVC_FORMAT_MJPEG;
        break;
	}

	/* init uvc user params, just use default params and overlay
	 * some specify params.
	 */
	uvc_gadget_user_params_init(&params);
	params.width = width;
	params.height = height;
	params.format = info.format;
	params.bulk_mode = 1;	/* isoc mode still hung, use bulk mode instead */

	ret = uvc_gadget_init(&uvc_ctx, uvc_devname,
			v4l2_devname, &params);
	if (ret < 0) {
		LOGE << "uvc_gadget_init error!";
		return ret;
	}

	uvc_set_streamon_handler(uvc_ctx, uvc_streamon_off, &info);
	/* prepare/release buffer with video queue */
	uvc_set_prepare_data_handler(uvc_ctx, uvc_get_frame, &info);
	uvc_set_release_data_handler(uvc_ctx, uvc_release_frame, &info);

	ret = uvc_gadget_start(uvc_ctx);
	if (ret < 0) {
		LOGE << "uvc_gadget_start error!";
	}

	LOGD << "uvc gadget int with params out ret(" << ret << ")";

	return ret;
}

void UvcServer::uvc_set_prepare_data_handler(struct uvc_context *ctx,
				  uvc_prepare_buffer_callback_fn cb_fn,
				  void *userdata)
{
	if (!ctx || !ctx->udev)
		return;

	struct uvc_device *udev = ctx->udev;

	udev->prepare_cb.cb = cb_fn;
	udev->prepare_cb.userdata = userdata;

	return;
}

int UvcServer::uvc_gadget_init(struct uvc_context **pctx, char *uvc_devname,
                char *v4l2_devname, struct uvc_params *user_params)
{
	struct uvc_device *udev;
	struct v4l2_device *vdev;
	struct v4l2_format fmt;
	struct uvc_params params;
	struct uvc_context *ctx = NULL;
	char *devname = (char *)"/dev/video0";
	char *udc_name = (char *)"b2000000.usb";
	char *function1 = (char *)"g_webcam";
	char *function2 = (char *)"g_comp";
	int ret;

	ctx = (uvc_context*)calloc(1, sizeof(*ctx));
	if (!ctx)
		return -1;

	if (uvc_devname) {
		devname = uvc_devname;
	} else {
		LOGD << "udc_find_video_device " << function1;
		devname = udc_find_video_device(udc_name, function1);
		if (!devname) {
			LOGD << "udc_find_video_device " << function2;
			devname = udc_find_video_device(udc_name, function2);
			if (!devname) {
				LOGE << "no matched uvc device(" << function1
                     << " or " << function2 << ") found!";
				return -1;
			}
		}
	}

	LOGD << "using uvc device: " << devname;

	/* if no user_params, use default params */
	uvc_gadget_user_params_init(&params);
	if (user_params) {
		memcpy(&params, user_params, sizeof(params));
	}

	LOGD << "###uvc_gadget_init###";
	LOGD << "using uvc device: " << devname;
	LOGD << "width: " << params.width;
	LOGD << "height: " << params.height;
	LOGD << "format: " << params.format;
	LOGD << "io_method: " << params.io_method;
	LOGD << "bulk_mode: " << params.bulk_mode;
	LOGD << "nbufs: " << params.nbufs;
	LOGD << "mult: " << params.mult;
	LOGD << "burst: " << params.burst;
	LOGD << "speed: " << params.speed;

	if (v4l2_devname) {
		/*
		 * Try to set the default format at the V4L2 video capture
		 * device as requested by the user.
		 */
		CLEAR(fmt);
		fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		fmt.fmt.pix.width = params.width;
		fmt.fmt.pix.height = params.height;
		fmt.fmt.pix.field = V4L2_FIELD_ANY;
		switch (params.format) {
		case UVC_FORMAT_YUY2:
			fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
			fmt.fmt.pix.sizeimage =
			    params.width * params.height * 2;
			break;
		case UVC_FORMAT_NV12:
			fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
			fmt.fmt.pix.sizeimage =
			    params.width * params.height * 1.5;
			break;
		case UVC_FORMAT_MJPEG:
			fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
			fmt.fmt.pix.sizeimage =
			    params.width * params.height * 1.5 / 2.0;
			break;
		case UVC_FORMAT_H264:
		case UVC_FORMAT_H265:
			fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
			fmt.fmt.pix.sizeimage =
			    params.width * params.height * 1.5 / 2.0;
			break;
		}

		/* Open the V4L2 device. */
		ret = v4l2_open(&vdev, v4l2_devname, &fmt);
		if (vdev == NULL || ret < 0)
			goto error1;
	}

	/* Open the UVC device. */
	ret = uvc_open(&udev, devname);
	if (udev == NULL || ret < 0)
		goto error2;

	udev->uvc_devname = devname;
	udev->parent = ctx;

	if (v4l2_devname) {
		vdev->v4l2_devname = v4l2_devname;
		/* Bind UVC and V4L2 devices. */
		udev->vdev = vdev;
		vdev->udev = udev;
	}

	/* Set parameters as passed by user. */
	switch (params.format) {
	case UVC_FORMAT_YUY2:
		udev->fcc = V4L2_PIX_FMT_YUYV;
		udev->imgsize = udev->width * udev->height * 2;
		break;
	case UVC_FORMAT_NV12:
		udev->fcc = V4L2_PIX_FMT_NV12;
		udev->imgsize = udev->width * udev->height * 1.5;
		break;
	case UVC_FORMAT_MJPEG:
		udev->fcc = V4L2_PIX_FMT_MJPEG;
		udev->imgsize = udev->width * udev->height * 1.5 / 2.0;
		break;
	case UVC_FORMAT_H264:
		udev->fcc = V4L2_PIX_FMT_H264;
		udev->imgsize = udev->width * udev->height * 1.5 / 2.0;
		break;
	case UVC_FORMAT_H265:
		udev->fcc = V4L2_PIX_FMT_H265;
		udev->imgsize = udev->width * udev->height * 1.5 / 2.0;
		break;

	}
	udev->width = params.width;
	udev->height = params.height;
	udev->io = (io_method)params.io_method;
	udev->bulk = params.bulk_mode;
	udev->nbufs = params.nbufs;
	udev->mult = params.mult;
	udev->burst = params.burst;
	udev->speed = (usb_device_speed)params.speed;
    udev->h264_quirk = params.h264_quirk;

	if (!v4l2_devname) {
		/* UVC standalone setup. */
		udev->run_standalone = 1;
	}

	if (v4l2_devname) {
		/* UVC - V4L2 integrated path */
		vdev->nbufs = params.nbufs;

		/*
		 * IO methods used at UVC and V4L2 domains must be
		 * complementary to avoid any memcpy from the CPU.
		 */
		switch (params.io_method) {
		case IO_METHOD_MMAP:
			vdev->io = IO_METHOD_USERPTR;
			break;

		case IO_METHOD_USERPTR:
		default:
			vdev->io = IO_METHOD_MMAP;
			break;
		}
	}

	switch (params.speed) {
	case USB_SPEED_FULL:
		/* Full Speed. */
		if (params.bulk_mode)
			udev->maxpkt = 64;
		else
			udev->maxpkt = 1023;
		break;

	case USB_SPEED_HIGH:
		/* High Speed. */
		if (params.bulk_mode)
			udev->maxpkt = 512;
		else
			udev->maxpkt = 1024;
		break;

	case USB_SPEED_SUPER:
	default:
		/* Super Speed. */ 
		if (params.bulk_mode)
			udev->maxpkt = 1024;
		else
			udev->maxpkt = 1024;
		break;
	}

	if (v4l2_devname && (IO_METHOD_MMAP == vdev->io)) {
		/*
		 * Ensure that the V4L2 video capture device has already some
		 * buffers queued.
		 */
		v4l2_reqbufs(vdev, vdev->nbufs);
	}

	/* Init UVC events. */
	uvc_events_init(udev);

	ctx->udev = udev;
	if (v4l2_devname)
		ctx->vdev = vdev;
	else
		ctx->vdev = NULL;

	ctx->uvc_pid = 0;
	ctx->exit = 0;

	*pctx = ctx;

	LOGI << "uvc_gadget_init succeed";

	return 0;

error2:
	if (v4l2_devname)
		v4l2_close(vdev);
error1:
	if (ctx)
		free(ctx);

	return -1;
}

void UvcServer::uvc_set_release_data_handler(struct uvc_context *ctx,
            uvc_release_buffer_callback_fn cb_fn, void *userdata)
{
    if (!ctx)
        return;

    if (!ctx->udev)
        return;

    struct uvc_device *udev = ctx->udev;

    udev->release_cb.cb = cb_fn;
    udev->release_cb.userdata = userdata;

    return;
}

void UvcServer::uvc_set_streamon_handler(struct uvc_context *ctx,
            uvc_streamon_callback_fn cb_fn, void *userdata)
{
    if (!ctx)
        return;

    if (!ctx->udev)
        return;

    struct uvc_device *udev = ctx->udev;

    udev->streamon_cb.cb = cb_fn;
    udev->streamon_cb.userdata = userdata;

    return;
}

int UvcServer::uvc_gadget_start(struct uvc_context *ctx)
{
    LOGI << "UvcServer::uvc_gadget_start";
    if (!ctx)
        return -1;
#if 0
    ret = pthread_create(&ctx->uvc_pid, NULL, uvc_loop, ctx);
    if (ret < 0) {
        fprintf(stderr, "uvc_loop thread launch failed\n");
        return -1;
    }
#endif
    worker_ = std::make_shared<std::thread>(&UvcServer::uvc_loop, this, ctx);
    LOGI << "uvc_gadget_start succeed.";

    return 0;
}

int UvcServer::uvc_gadget_stop(struct uvc_context *ctx)
{
    struct uvc_device *udev;
    struct v4l2_device *vdev;

    if (!ctx)
        return -1;

    udev = ctx->udev;
    vdev = ctx->vdev;

    ctx->exit = 1;
    if (pthread_join(ctx->uvc_pid, NULL) < 0) {
        fprintf(stderr, "phtread join failed\n");
        return -1;
    }

    if (vdev && vdev->is_streaming) {
        /* Stop V4L2 streaming... */
        v4l2_stop_capturing(vdev);
        v4l2_uninit_device(vdev);
        v4l2_reqbufs(vdev, 0);
        vdev->is_streaming = 0;
    }

    if (udev->is_streaming) {
        /* ... and now UVC streaming.. */
        uvc_video_stream(udev, 0);
        uvc_uninit_device(udev);
        uvc_video_reqbufs(udev, 0);
        udev->is_streaming = 0;
    }

    LOGI << "uvc_gadget_stop succeed.";

    return 0;
}

void UvcServer::uvc_gadget_deinit(struct uvc_context *ctx)
{
    struct uvc_device *udev;
    struct v4l2_device *vdev;

    if (!ctx)
        return;

    udev = ctx->udev;
    vdev = ctx->vdev;

    if (vdev)
        v4l2_close(vdev);

    uvc_close(udev);

    free(ctx);
    ctx = NULL;

    LOGI << "uvc_gadget_deinit succeed.";
}

/* uvc main loop */
//void *UvcServer::uvc_loop(void *arg)
void UvcServer::uvc_loop(void *arg)
{
    LOGI << "UvcServer::uvc_loop";
    struct uvc_device *udev;
    struct v4l2_device *vdev;
    struct timeval tv;
    struct uvc_context *ctx = (struct uvc_context *)arg;

    fd_set fdsv, fdsu;
    int ret, nfds;

    if (!ctx) {
        fprintf(stderr, "no uvc context.\n");
      //  return (void *)0;
         return ;
    }

    udev = ctx->udev;
    vdev = ctx->vdev;

    while (!ctx->exit) {
    //    LOGI << "UvcServer::uvc_loop LOOP!!!!! ";
        if (vdev) {
            LOGI << "vdev is ok ";
            FD_ZERO(&fdsv);
        }

        FD_ZERO(&fdsu);

        /* We want both setup and data events on UVC interface.. */
        FD_SET(udev->uvc_fd, &fdsu);

        fd_set efds = fdsu;
        fd_set dfds = fdsu;

        /* ..but only data events on V4L2 interface */
        if (vdev)
            FD_SET(vdev->v4l2_fd, &fdsv);

        /* Timeout. */
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        if (vdev) {
            nfds = max_c(vdev->v4l2_fd, udev->uvc_fd);
            ret = select(nfds + 1, &fdsv, &dfds, &efds, &tv);
        } else {
            ret = select(udev->uvc_fd + 1, NULL, &dfds, &efds, NULL);
        }

        if (-1 == ret) {
            LOGE << "select error " << errno << ", " << strerror(errno);
            if (EINTR == errno)
                continue;

            break;
        }

        if (0 == ret) {
            LOGD << "select timeout";
            break;
        }
        if (udev->is_streaming) {
           uvc_stream_on = 1;
        } else {
           uvc_stream_on = 0;
        }
        if (FD_ISSET(udev->uvc_fd, &efds)) {

            uvc_events_process(udev);
        }
        if (FD_ISSET(udev->uvc_fd, &dfds)) {
            uvc_video_process(udev);
        } else {
        //    LOGI << "!!!!!NOT UVC VIDEO PROCESS";
        }
        if (vdev)
            if (FD_ISSET(vdev->v4l2_fd, &fdsv))
                v4l2_process_data(vdev);
    //    LOGI << "UvcServer::uvc_loop select END!!!!  ";
        std::this_thread::sleep_for(milliseconds(10));
    }

    LOGI << "uvc_loop exit.";
}

char *UvcServer::udc_find_video_device(const char *udc, const char *function)
{
    LOGI << "UvcServer::udc_find_video_device";
    DIR *dir;
    struct dirent *dirent;
    char fpath[128];
    char vpath[128];
    char func_name[64];
    char *video = NULL;
    int found = 0;
    int fd, ret;

    snprintf(fpath, 128, "/sys/class/udc/%s/function", udc);
    snprintf(vpath, 128, "/sys/class/udc/%s/device/gadget/video4linux/", udc);

    fd = open(fpath, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "open %s failed\n", fpath);
        goto error;
    }

    ret = read(fd, func_name, 64);
    if (ret <= 0) {
        fprintf(stderr, "read %s failed\n", fpath);
        goto no_func;
    }

    func_name[ret] = '\0';
    close(fd);

    if (strncmp(func_name, function, strlen(function)) != 0) {
        fprintf(stderr, "function name not matched. %s:%s\n",
                    func_name, function);
        goto error;
    }

    dir = opendir(vpath);
    if (!dir) {
        fprintf(stderr, "opendir %s failed\n", vpath);
        goto error;
    } else {
        while ((dirent = readdir(dir)) != NULL) {
            if (!strncmp(dirent->d_name, "video", 5)) {
                found = 1;
                break;
            }
        }

        if (found) {
            // TODO: video devname memleak, caller not free currently
            video = path_join("/dev", dirent->d_name);
        }

        closedir(dir);
    }

    return video;

no_func:
    close(fd);

error:
    return NULL;
}

char *UvcServer::path_join(const char *dirname, const char *name)
{
    char *path;
    int ret;

    /* path is malloc in heap, return string to caller, who needs to free it... */
    ret = asprintf(&path, "%s/%s", dirname, name);

    if (ret < 0)
        path = NULL;

    return path;
}

unsigned int UvcServer::uvc_format_to_fcc(enum uvc_fourcc_format format)
{
	unsigned int fcc;
	switch (format) {
	case UVC_FORMAT_YUY2:
		fcc = V4L2_PIX_FMT_YUYV;
		break;
	case UVC_FORMAT_MJPEG:
		fcc = V4L2_PIX_FMT_MJPEG;
		break;
	case UVC_FORMAT_H264:
		fcc = V4L2_PIX_FMT_H264;
		break;
	default:
		fcc = 0;
		break;
	}

	return fcc;
}


}  // namespace Uvcplugin
}  // namespace xproto
}  // namespace vision
}  // namespace horizon
