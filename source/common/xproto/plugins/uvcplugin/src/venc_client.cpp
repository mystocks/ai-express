/*!
 * Copyright (c) 2016-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     venc_client.cpp
 * \Author   ronghui.zhang
 * \Version  1.0.0.0
 * \Date     2020.5.12
 * \Brief    implement of api file
 */
#include "venc_client.h"
#include "hobotlog/hobotlog.hpp"

namespace horizon {
namespace vision {
namespace xproto {
namespace Uvcplugin {

VencClient::VencClient() {
  g_exit = 0;

  venc_param_.type = 3;
  venc_param_.veChn = 0;
  venc_param_.width = 1920;
  venc_param_.height = 1080;
  venc_param_.vpsGrp = 0;
  venc_param_.vpsChn = 5;
  venc_param_.bitrate = 2000;
}

VencClient::~VencClient() {
}

int VencClient::Init(int width, int height, UvcConfig::VideoType type) {
  venc_param_.width = width;
  venc_param_.height = height;

  if (type == UvcConfig::VIDEO_H264) {
    venc_param_.type = 1;
  } else if (type == UvcConfig::VIDEO_MJPEG) {
    venc_param_.type = 3;
  }

  VencCommonInit();
  VencInit(venc_param_.veChn, venc_param_.type, venc_param_.width,
    venc_param_.height, venc_param_.bitrate);
  VencStart(venc_param_.veChn);
  return 0;
}

int VencClient::Start() {
  if (nullptr == venc_pid_) {
    venc_pid_ = std::make_shared<std::thread>(
        &VencClient::VencToUvcThread, this, &venc_param_);
  }
  return 0;
}

int VencClient::DeInit() {
    int s32Ret = 0;

    s32Ret = HB_VENC_DestroyChn(venc_param_.veChn);
    if (s32Ret != 0) {
        LOGE << "HB_VENC_DestroyChn failed";
        return -1;
    }

    return 0;
}

int VencClient::Stop() {
    int s32Ret = 0;
    if (venc_pid_ && venc_pid_->joinable()) {
        g_exit = 1;
        venc_pid_->join();
        venc_pid_ = nullptr;
        LOGI << "VencClient stop worker";
    }

    s32Ret = HB_VENC_StopRecvFrame(venc_param_.veChn);
    if (s32Ret != 0) {
        LOGE << "HB_VENC_StopRecvFrame failed";
        return -1;
    }

    return 0;
}

void VencClient::VencToUvcThread(void *vencpram) {
    vencParam *vparam = (vencParam *) vencpram;
	int r;
	int veChn;

	LOGD << "pym_venc_to_uvc_thread in";

	if (!vparam)
		return ;

	veChn = vparam->veChn;

	UvcServer::mono_q = (single_buffer*)malloc(sizeof(struct single_buffer));
	if (!UvcServer::mono_q)
		return ;

	UvcServer::mono_q->used = 0;
	memset(&(UvcServer::mono_q->vstream), 0, sizeof(VIDEO_STREAM_S));
	pthread_mutex_init(&UvcServer::mono_q->mutex, NULL);

    while (!g_exit) {
		if (vparam->quit == 1)
			break;

		if (!UvcServer::mono_q)
			break;
		/* check uvc single buffer */
		pthread_mutex_lock(&UvcServer::mono_q->mutex);
		if (UvcServer::mono_q->used) {
			pthread_mutex_unlock(&UvcServer::mono_q->mutex);
			usleep(5*1000);		/* sleep 5ms and re-check mono_q */
			continue;
		}

		r = HB_VENC_GetStream(veChn, &UvcServer::mono_q->vstream, 300);
		if (r < 0) {
			pthread_mutex_unlock(&UvcServer::mono_q->mutex);
			usleep(3*1000);		/* sleep 3ms and re-encode */
			continue;
		} else
        {
            LOGI << "get venc ok";
        }
        
		UvcServer::mono_q->used = 1;
		pthread_mutex_unlock(&UvcServer::mono_q->mutex);
	}

	LOGD << "pym_venc_to_uvc_thread out";
	return ;
}

int VencClient::VencChnAttrInit(VENC_CHN_ATTR_S *pVencChnAttr, PAYLOAD_TYPE_E p_enType,
            int p_Width, int p_Height, PIXEL_FORMAT_E pixFmt) {
    memset(pVencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    pVencChnAttr->stVencAttr.enType = p_enType;

    pVencChnAttr->stVencAttr.u32PicWidth = p_Width;
    pVencChnAttr->stVencAttr.u32PicHeight = p_Height;

    pVencChnAttr->stVencAttr.enMirrorFlip = DIRECTION_NONE;
    pVencChnAttr->stVencAttr.enRotation = CODEC_ROTATION_0;
    pVencChnAttr->stVencAttr.stCropCfg.bEnable = HB_FALSE;

    if (p_enType == PT_JPEG || p_enType == PT_MJPEG) {
        pVencChnAttr->stVencAttr.enPixelFormat = pixFmt;
        pVencChnAttr->stVencAttr.u32BitStreamBufferCount = 1;
        pVencChnAttr->stVencAttr.u32FrameBufferCount = 2;
        pVencChnAttr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
        pVencChnAttr->stVencAttr.stAttrJpeg.dcf_enable = HB_FALSE;
        pVencChnAttr->stVencAttr.stAttrJpeg.quality_factor = 0;
        pVencChnAttr->stVencAttr.stAttrJpeg.restart_interval = 0;
    } else {
        pVencChnAttr->stVencAttr.enPixelFormat = pixFmt;
        pVencChnAttr->stVencAttr.u32BitStreamBufferCount = 5;
        pVencChnAttr->stVencAttr.u32FrameBufferCount = 5;
        pVencChnAttr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
    }

    if (p_enType == PT_H265) {
        pVencChnAttr->stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
        pVencChnAttr->stRcAttr.stH265Vbr.bQpMapEnable = HB_TRUE;
        pVencChnAttr->stRcAttr.stH265Vbr.u32IntraQp = 20;
        pVencChnAttr->stRcAttr.stH265Vbr.u32IntraPeriod = 20;
        pVencChnAttr->stRcAttr.stH265Vbr.u32FrameRate = 25;
    }
    if (p_enType == PT_H264) {
        pVencChnAttr->stRcAttr.enRcMode = VENC_RC_MODE_H264VBR;
        pVencChnAttr->stRcAttr.stH264Vbr.bQpMapEnable = HB_TRUE;
        pVencChnAttr->stRcAttr.stH264Vbr.u32IntraQp = 20;
        pVencChnAttr->stRcAttr.stH264Vbr.u32IntraPeriod = 20;
        pVencChnAttr->stRcAttr.stH264Vbr.u32FrameRate = 30;
    }

    pVencChnAttr->stGopAttr.u32GopPresetIdx = 6;
    pVencChnAttr->stGopAttr.s32DecodingRefreshType = 2;

    return 0;
}

int VencClient::VencCommonInit() {
    int s32Ret;
    LOGI << "sample_venc_common_init==begin======";
    s32Ret = HB_VENC_Module_Init();
    if (s32Ret) {
        LOGW << "HB_VENC_Module_Init: " << s32Ret;
    }
    LOGI << "sample_venc_common_init=end=======";
    return s32Ret;
}

int VencClient::VencCommonDeinit() {
    int s32Ret;

    s32Ret = HB_VENC_Module_Uninit();
    if (s32Ret) {
        LOGW << "HB_VENC_Module_Init: " << s32Ret;
    }

    return s32Ret;
}

int VencClient::VencInit(int VeChn, int type, int width, int height, int bits) {
    int s32Ret;
    VENC_CHN_ATTR_S vencChnAttr;
    VENC_RC_ATTR_S *pstRcParam;
    PAYLOAD_TYPE_E ptype;

    if (type == 1) {
        ptype = PT_H264;
    } else if (type == 2) {
        ptype = PT_H265;
    } else if (type == 3) {
		ptype = PT_MJPEG;
	} else {
        ptype = PT_JPEG;
    }

    // if (VeChn == 2) {
    //     int w = width;
    //     width = height;
    //     height = w;
    //     printf("chn1 need rotate widht: %d, height: %d\n", width, height);
    // }

    VencChnAttrInit(&vencChnAttr, ptype, width, height, HB_PIXEL_FORMAT_NV12);

    s32Ret = HB_VENC_CreateChn(VeChn, &vencChnAttr);
    if (s32Ret != 0) {
        LOGE << "HB_VENC_CreateChn " << VeChn << " failed, " << s32Ret;
        return -1;
    }
    // HB_VENC_Module_Uninit();
    if (ptype == PT_H264) {
        pstRcParam = &(vencChnAttr.stRcAttr);
        vencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
        s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
        if (s32Ret != 0) {
            LOGE << "HB_VENC_GetRcParam failed.";
            return -1;
        }

        LOGD << " vencChnAttr.stRcAttr.enRcMode = "
             << vencChnAttr.stRcAttr.enRcMode << " mmmmmmmmmmmmmmmmmm   ";
        LOGD << " u32VbvBufferSize = "
             << vencChnAttr.stRcAttr.stH264Cbr.u32VbvBufferSize
             << " mmmmmmmmmmmmmmmmmm   ";

        pstRcParam->stH264Cbr.u32BitRate = bits;
        pstRcParam->stH264Cbr.u32FrameRate = 30;
        pstRcParam->stH264Cbr.u32IntraPeriod = 60;
        pstRcParam->stH264Cbr.u32VbvBufferSize = 3000;
    } else if (ptype == PT_H265) {
        pstRcParam = &(vencChnAttr.stRcAttr);
        vencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
        s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
        if (s32Ret != 0) {
            LOGE << "HB_VENC_GetRcParam failed.";
            return -1;
        }
        LOGD << " m_VencChnAttr.stRcAttr.enRcMode = "
             << vencChnAttr.stRcAttr.enRcMode << " mmmmmmmmmmmmmmmmmm   ";
        LOGD << " u32VbvBufferSize = "
             << vencChnAttr.stRcAttr.stH265Cbr.u32VbvBufferSize
             << " mmmmmmmmmmmmmmmmmm   ";

        pstRcParam->stH265Cbr.u32BitRate = bits;
        pstRcParam->stH265Cbr.u32FrameRate = 30;
        pstRcParam->stH265Cbr.u32IntraPeriod = 30;
        pstRcParam->stH265Cbr.u32VbvBufferSize = 3000;
    }  else if (ptype == PT_MJPEG) {
        pstRcParam = &(vencChnAttr.stRcAttr);
        vencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_MJPEGFIXQP;
        s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
        if (s32Ret != 0) {
            LOGE << "HB_VENC_GetRcParam failed.";
            return -1;
        }
    }

    s32Ret = HB_VENC_SetChnAttr(VeChn, &vencChnAttr);  // config
    if (s32Ret != 0) {
        LOGE << "HB_VENC_SetChnAttr failed";
        return -1;
    }

#ifdef VENC_BIND
    LOGI << " system bind ";
    if (VeChn == 1) {
      struct HB_SYS_MOD_S src_mod, dst_mod;
      src_mod.enModId = HB_ID_VPS;
      src_mod.s32DevId = 0;
      src_mod.s32ChnId = 5;
      dst_mod.enModId = HB_ID_VENC;
      dst_mod.s32DevId = 0;
      dst_mod.s32ChnId = VeChn;
      s32Ret = HB_SYS_Bind(&src_mod, &dst_mod);
      if (s32Ret != 0) {
        LOGE << "HB_SYS_Bind failed";
        return -1;
      }
    } else {
      struct HB_SYS_MOD_S src_mod, dst_mod;
      src_mod.enModId = HB_ID_VPS;
      src_mod.s32DevId = 0;
      src_mod.s32ChnId = VeChn;
      dst_mod.enModId = HB_ID_VENC;
      dst_mod.s32DevId = 0;
      dst_mod.s32ChnId = VeChn;
      s32Ret = HB_SYS_Bind(&src_mod, &dst_mod);
      if (s32Ret != 0) {
        LOGE << "HB_SYS_Bind failed";
        return -1;
      }
    }
#endif

    return 0;
}

int VencClient::VencStart(int VeChn) {
    int s32Ret = 0;

    VENC_RECV_PIC_PARAM_S pstRecvParam;
    pstRecvParam.s32RecvPicNum = 0;  // unchangable

    s32Ret = HB_VENC_StartRecvFrame(VeChn, &pstRecvParam);
    if (s32Ret != 0) {
        LOGE << "HB_VENC_StartRecvFrame failed";
        return -1;
    }

    // s32Ret = HB_VENC_EnableIDR(VeChn, HB_FALSE);
    // printf("HB_VENC_EnableIDR: %x\n", s32Ret);
    // s32Ret = HB_VENC_EnableIDR(VeChn, HB_TRUE);
    // printf("HB_VENC_EnableIDR %x\n", s32Ret);
    return 0;
}

int VencClient::VencStop(int VeChn) {
    int s32Ret = 0;

    s32Ret = HB_VENC_StopRecvFrame(VeChn);
    if (s32Ret != 0) {
        LOGE << "HB_VENC_StopRecvFrame failed";
        return -1;
    }

    return 0;
}

int VencClient::VencDeinit(int VeChn) {
    int s32Ret = 0;

    s32Ret = HB_VENC_DestroyChn(VeChn);
    if (s32Ret != 0) {
        LOGE << "HB_VENC_DestroyChn failed";
        return -1;
    }

    return 0;
}

int VencClient::VencReinit(int Vechn, int type, int width, int height, int bits) {
    int s32Ret = 0;

    s32Ret = HB_VENC_StopRecvFrame(Vechn);
    if (s32Ret != 0) {
        LOGE << "HB_VENC_StopRecvFrame " << Vechn << " failed";
        return -1;
    }

    s32Ret = HB_VENC_DestroyChn(Vechn);
    if (s32Ret != 0) {
        LOGE << "HB_VENC_DestroyChn " << Vechn << " failed";
        return -1;
    }

    s32Ret = VencInit(Vechn, type, width, height, bits);
    if (s32Ret != 0) {
        LOGE << "sample_venc_init failed";
        return -1;
    }

    s32Ret = VencStart(Vechn);
    if (s32Ret != 0) {
        LOGE << "sample_venc_start failed";
        return -1;
    }

    return 0;
}
}  // namespace Uvcplugin
}  // namespace xproto
}  // namespace vision
}  // namespace horizon