/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: yong.wu
 * @Mail: yong.wu@horizon.ai
 * @Date: 2020-06-29
 * @Version: v0.0.1
 * @Brief: implemenation of iot media codec .
 */

#include <string.h>
#include <unistd.h>
#include "hobotlog/hobotlog.hpp"
#include "media_codec/media_codec_manager.h"
#include "./hb_vp_api.h"

#define JPEG_QUALITY_FACTOR_DEFAULT_VALUE   50
#define MAX_POOL_CNT 32

namespace horizon {
namespace vision {

int MediaCodecManager::SetDefaultChnAttr(VENC_CHN_ATTR_S *pVencChnAttr,
        PAYLOAD_TYPE_E type, int width, int height, PIXEL_FORMAT_E pix_fmt) {
    memset(pVencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    pVencChnAttr->stVencAttr.enType = type;

    pVencChnAttr->stVencAttr.u32PicWidth = width;
    pVencChnAttr->stVencAttr.u32PicHeight = height;

    pVencChnAttr->stVencAttr.enMirrorFlip = DIRECTION_NONE;
    pVencChnAttr->stVencAttr.enRotation = CODEC_ROTATION_0;
    pVencChnAttr->stVencAttr.stCropCfg.bEnable = HB_FALSE;

    if (type == PT_JPEG || type == PT_MJPEG) {
        pVencChnAttr->stVencAttr.enPixelFormat = pix_fmt;
        pVencChnAttr->stVencAttr.u32BitStreamBufferCount = 1;
        pVencChnAttr->stVencAttr.u32FrameBufferCount = 2;
        pVencChnAttr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
        pVencChnAttr->stVencAttr.stAttrJpeg.dcf_enable = HB_FALSE;
        pVencChnAttr->stVencAttr.stAttrJpeg.quality_factor =
            JPEG_QUALITY_FACTOR_DEFAULT_VALUE;
        pVencChnAttr->stVencAttr.stAttrJpeg.restart_interval = 0;
    } else {
        pVencChnAttr->stVencAttr.enPixelFormat = pix_fmt;
        pVencChnAttr->stVencAttr.u32BitStreamBufferCount = 5;
        pVencChnAttr->stVencAttr.u32FrameBufferCount = 5;
        pVencChnAttr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
    }

    if (type == PT_H265) {
#ifdef H265_LOSSLESS
      m_VencChnAttr.stVencAttr.stAttrH265.lossless_mode = 1;
      m_VencChnAttr.stVencAttr.stAttrH265.transform_skip_enabled_flag = 1;
#endif
        pVencChnAttr->stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
        pVencChnAttr->stRcAttr.stH265Vbr.bQpMapEnable = HB_TRUE;
        pVencChnAttr->stRcAttr.stH265Vbr.u32IntraQp = 20;
        pVencChnAttr->stRcAttr.stH265Vbr.u32IntraPeriod = 20;
        pVencChnAttr->stRcAttr.stH265Vbr.u32FrameRate = 25;
    }
    if (type == PT_H264) {
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

int MediaCodecManager::SetDefaultRcParams(int chn, PAYLOAD_TYPE_E type,
        VENC_CHN_ATTR_S *attr) {
    int ret;

    if (type == PT_H264)  {
        attr->stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
        VENC_RC_ATTR_S *pstRcParam = &(attr->stRcAttr);
        ret = HB_VENC_GetRcParam(chn, pstRcParam);
        if (ret) {
            LOGE << "H264 HB_VENC_GetRcParam " << chn << " failed: " << ret;
            return ret;
        }
        pstRcParam->stH264Cbr.u32BitRate = 3000;
        pstRcParam->stH264Cbr.u32FrameRate = 30;
        pstRcParam->stH264Cbr.u32IntraPeriod = 30;
        pstRcParam->stH264Cbr.u32VbvBufferSize = 3000;
    } else if (type == PT_H265) {
        attr->stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
        VENC_RC_ATTR_S *pstRcParam = &(attr->stRcAttr);
        ret = HB_VENC_GetRcParam(chn, pstRcParam);
        if (ret) {
            LOGE << "H265 HB_VENC_GetRcParam " << chn << " failed: " << ret;
            return ret;
        }
        pstRcParam->stH265Cbr.u32BitRate = 3000;
        pstRcParam->stH265Cbr.u32FrameRate = 30;
        pstRcParam->stH265Cbr.u32IntraPeriod = 30;
        pstRcParam->stH265Cbr.u32VbvBufferSize = 3000;
    } else if (type == PT_MJPEG) {
        attr->stRcAttr.enRcMode = VENC_RC_MODE_MJPEGFIXQP;
        VENC_RC_ATTR_S *pstRcParam = &(attr->stRcAttr);
        ret = HB_VENC_GetRcParam(chn, pstRcParam);
        if (ret) {
            LOGE << "MJPG HB_VENC_GetRcParam " << chn << " failed: " << ret;
            return ret;
        }
    }

    return 0;
}

int MediaCodecManager::SetUserCbrParams(int chn, int bit_rate, int fps,
        int intra, int buf_size) {
    int ret;
    PAYLOAD_TYPE_E type = chn_type_[chn];
    VENC_CHN_ATTR_S *attr = &venc_chn_attr_[chn];
    VENC_RC_ATTR_S *pstRcParam = nullptr;

    if (type == PT_H264)  {
        attr->stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
        pstRcParam = &(attr->stRcAttr);
        ret = HB_VENC_GetRcParam(chn, pstRcParam);
        if (ret) {
            LOGE << "H264 HB_VENC_GetRcParam " << chn << " failed: " << ret;
            return ret;
        }
        pstRcParam->stH264Cbr.u32BitRate = bit_rate;
        pstRcParam->stH264Cbr.u32FrameRate = fps;
        pstRcParam->stH264Cbr.u32IntraPeriod = intra;
        pstRcParam->stH264Cbr.u32VbvBufferSize = buf_size;
    } else if (type == PT_H265) {
        attr->stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
        pstRcParam = &(attr->stRcAttr);
        ret = HB_VENC_GetRcParam(chn, pstRcParam);
        if (ret) {
            LOGE << "H265 HB_VENC_GetRcParam " << chn << " failed: " << ret;
            return ret;
        }
        pstRcParam->stH265Cbr.u32BitRate = bit_rate;
        pstRcParam->stH265Cbr.u32FrameRate = fps;
        pstRcParam->stH265Cbr.u32IntraPeriod = intra;
        pstRcParam->stH265Cbr.u32VbvBufferSize = buf_size;
    } else {
        LOGE << "not support chn type: " << type;
        return -1;
    }

    ret = HB_VENC_SetRcParam(chn, pstRcParam);
    if (ret) {
        LOGE << "HB_VENC_SetRcParam " << chn << " failed: " << ret;
        return ret;
    }


    return 0;
}

int MediaCodecManager::SetUserQfactorParams(int chn, int value) {
    int ret;
    VENC_JPEG_PARAM_S *jpg_params = &venc_chn_jpg_params_[chn];

    ret = HB_VENC_GetJpegParam(chn, jpg_params);
    if (ret) {
        LOGE << "set qfactor, HB_VENC_GetJpegParam " << chn
             << " failed: " << ret;
        return ret;
    }
    jpg_params->u32Qfactor = value;
    ret = HB_VENC_SetJpegParam(chn, jpg_params);
    if (ret) {
        LOGE << "set qfactor, HB_VENC_SetJpegParam " << chn
             << " failed: " << ret;
        return ret;
    }

    return 0;
}

int MediaCodecManager::EncodeChnInit(int chn, PAYLOAD_TYPE_E type,
        int width, int height, PIXEL_FORMAT_E pix_fmt) {
    int ret;
    VENC_CHN_ATTR_S VencChnAttr;

    /* 1. Set Venc chn Attr */
    SetDefaultChnAttr(&VencChnAttr, type, width, height, pix_fmt);

    /* 2. Create Venc chn */
    ret = HB_VENC_CreateChn(chn, &VencChnAttr);
    if (ret) {
        LOGE << "HB_VENC_CreateChn, chn: " << chn << " failed: " << ret;
        return ret;
    }

    /* 3. Set default rc params */
    ret = SetDefaultRcParams(chn, type, &VencChnAttr);
    if (ret) {
        LOGE << "SetDefaultCbrParams, chn: " << chn << " failed: " << ret;
        return ret;
    }

    /* 4. Set encoder chn attr */
    ret = HB_VENC_SetChnAttr(chn, &VencChnAttr);
    if (ret) {
        LOGE << "HB_VENC_SetChnAttr, chn " << chn << " failed: " << ret;
        return ret;
    }
    venc_chn_attr_[chn] = VencChnAttr;
    chn_type_[chn] = type;

    return 0;
}

int MediaCodecManager::EncodeChnDeInit(int chn) {
    int ret;

    ret = HB_VENC_DestroyChn(chn);
    if (ret) {
        LOGE << "HB_VENC_DestroyChn, chn " << chn << " failed: " << ret;
        return ret;
    }

    return 0;
}

int MediaCodecManager::EncodeChnStart(int chn) {
    int ret;

    VENC_RECV_PIC_PARAM_S pstRecvParam;
    pstRecvParam.s32RecvPicNum = 0;

    ret = HB_VENC_StartRecvFrame(chn, &pstRecvParam);
    if (ret) {
        LOGE << "HB_VENC_StartRecvFrame, chn " << chn << " failed: " << ret;
        return ret;
    }

    return 0;
}

int MediaCodecManager::EncodeChnStop(int chn) {
    int ret;

    ret = HB_VENC_StopRecvFrame(chn);
    if (ret) {
        LOGE << "HB_VENC_StopRecvFrame, chn " << chn << " failed: " << ret;
        return ret;
    }

    return 0;
}

int MediaCodecManager::EncodeYuvToJpg(int chn, iot_venc_src_buf_t *src_buf,
        iot_venc_stream_buf_t **stream_buf) {
    int ret;
    VIDEO_FRAME_S pstFrame = { 0 };
    iot_venc_stream_buf_t *venc_buf;


    if (vb_stream_buf_queue_[chn].size() == 0) {
        LOGE << "vb stream buf queue size is zero, error!";
        return -1;
    }
    venc_buf = vb_stream_buf_queue_[chn].front();
    vb_stream_buf_queue_[chn].pop();
    if (venc_buf == nullptr) {
        LOGE << "vb buf is nullptr";
        return -1;
    }
    memset(venc_buf, 0x00, sizeof(iot_venc_stream_buf_t));

    pstFrame.stVFrame = src_buf->frame_info;
    ret = HB_VENC_SendFrame(chn, &pstFrame, 3000);
    if (ret < 0) {
        LOGE << "HB_VENC_SendFrame, chn " << chn << " failed: " << ret;
    }

    ret = HB_VENC_GetStream(chn, &venc_buf->stream_info, 3000);
    if (ret < 0) {
        LOGE << "HB_VENC_SendFrame, chn " << chn << " failed: " << ret;
    }
    venc_buf->chn = chn;
    venc_buf->src_width = src_buf->frame_info.width;
    venc_buf->src_height = src_buf->frame_info.height;
    venc_buf->src_pix_format = src_buf->frame_info.pix_format;
    venc_buf->src_pts = src_buf->frame_info.pts;
    *stream_buf = venc_buf;

    return 0;
}

int MediaCodecManager::FreeStream(int chn, iot_venc_stream_buf_t *stream_buf) {
    int ret;

    HOBOT_CHECK(stream_buf != nullptr) << "stream buf is null";

    ret = HB_VENC_ReleaseStream(chn, &stream_buf->stream_info);
    if (ret < 0) {
        LOGE << "HB_VENC_ReleaseStream, chn " << chn << " failed: " << ret;
    }

    vb_stream_buf_queue_[chn].push(stream_buf);  // push again for cycle queue

    return 0;
}

int MediaCodecManager::AllocVbBuf2Lane(int index, void *buf,
        int size_y, int size_uv) {
    int ret;
    iot_venc_src_buf_t *buffer = reinterpret_cast<iot_venc_src_buf_t*>(buf);
    uint64_t phy_ptr0, phy_ptr1;

    if (buffer == nullptr) {
        LOGE << "buffer is nullptr error!";
        return -1;
    }

    ret = HB_SYS_Alloc(&phy_ptr0,
            reinterpret_cast<void**>(&buffer->frame_info.vir_ptr[0]), size_y);
    if (ret) {
        LOGE << "index: " << index << "alloc vb buffer error, ret: " << ret;
        return ret;
    }
    buffer->frame_info.phy_ptr[0] = static_cast<uint32_t>(phy_ptr0);

    ret = HB_SYS_Alloc(&phy_ptr1,
            reinterpret_cast<void**>(&buffer->frame_info.vir_ptr[1]), size_uv);
    if (ret) {
        LOGE << "index: " << index << "alloc vb buffer error, ret: " << ret;
        return ret;
    }
    buffer->frame_info.phy_ptr[1] = static_cast<uint32_t>(phy_ptr1);

    LOGD << "mmzAlloc index: " << index;
    LOGD << "vio_buf_addr: "   << buffer;
    LOGD << "buf_y_paddr: "    << buffer->frame_info.phy_ptr[0];
    LOGD << "buf_y_vaddr "     << buffer->frame_info.vir_ptr[0];
    LOGD << "buf_uv_paddr: "   << buffer->frame_info.phy_ptr[1];
    LOGD << "buf_uv_vaddr: "   << buffer->frame_info.vir_ptr[1];

    return 0;
}

int MediaCodecManager::VbBufInit(int chn, int width, int height, int stride,
        int size, int vb_num) {
    int ret;
    uint32_t size_y, size_uv;
    iot_venc_src_buf_t *src_buf;
    iot_venc_stream_buf_t *stream_buf;

    vb_num_[chn] = vb_num;
    size_y = stride * height;
    size_uv = size - size_y;

    VP_CONFIG_S vp_config;
    memset(&vp_config, 0x00, sizeof(VP_CONFIG_S));
    vp_config.u32MaxPoolCnt = MAX_POOL_CNT;
    ret = HB_VP_SetConfig(&vp_config);
    if (ret) {
        if (ret == HB_ERR_VP_BUSY) {
            LOGW << "hb vp have already config, ret: " << ret;
        } else {
            LOGE << "hb vp config failed, need return, ret: " << ret;
            return ret;
        }
    }

    ret = HB_VP_Init();
    if (ret) {
        if (ret == HB_ERR_VP_BUSY) {
            LOGW << "hb vp have already init, ret: " << ret;
        } else {
            LOGE << "hb vp init failed, need return, ret: " << ret;
            return ret;
        }
    }

    for (int i = 0; i < vb_num; i++) {
        src_buf = reinterpret_cast<iot_venc_src_buf_t *>\
                             (malloc(sizeof(iot_venc_src_buf_t)));
        HOBOT_CHECK(src_buf != nullptr) << "malloc src buf failed";
        stream_buf = reinterpret_cast<iot_venc_stream_buf_t *>\
                    (malloc(sizeof(iot_venc_stream_buf_t)));
        HOBOT_CHECK(stream_buf != nullptr) << "malloc stream buf failed";
        memset(src_buf, 0x00, sizeof(iot_venc_src_buf_t));
        memset(stream_buf, 0x00, sizeof(iot_venc_stream_buf_t));
        ret = AllocVbBuf2Lane(i, src_buf, size_y, size_uv);
        if (ret) {
            LOGE << "alloc vb buffer 2 land failed, ret: " << ret;
            return ret;
        }
        src_buf->frame_info.size = size;
        src_buf->frame_info.width = width;
        src_buf->frame_info.height = height;
        src_buf->frame_info.stride = stride;
        vb_src_buf_queue_[chn].push(src_buf);
        vb_stream_buf_queue_[chn].push(stream_buf);
    }

    return 0;
}

int MediaCodecManager::GetVbBuf(int chn, iot_venc_src_buf_t **buf) {
    iot_venc_src_buf_t *vb_buf;

    if (vb_src_buf_queue_[chn].size() == 0) {
        LOGE << "vb src buf queue size is zero, error!";
        return -1;
    }
    vb_buf = vb_src_buf_queue_[chn].front();
    vb_src_buf_queue_[chn].pop();
    if (vb_buf == nullptr) {
        LOGE << "vb buf is nullptr";
        return -1;
    }
    *buf = vb_buf;

    return 0;
}

int MediaCodecManager::FreeVbBuf(int chn, iot_venc_src_buf_t *buf) {
    int ret = 0;

    if (buf == nullptr) {
        LOGE << "buffer is nullptr error!";
        ret = -1;
        return ret;
    }

    vb_src_buf_queue_[chn].push(buf);  // push again for cycle queue

    return ret;
}

int MediaCodecManager::VbBufDeInit(int chn) {
    int ret, cnt;
    iot_venc_src_buf_t *buffer;
    iot_venc_stream_buf_t *stream_buffer;

    cnt = vb_src_buf_queue_[chn].size();
    for (int i = 0; i < cnt; i++) {
        buffer = vb_src_buf_queue_[chn].front();
        vb_src_buf_queue_[chn].pop();
        if (buffer == nullptr) {
            LOGE << "vb buf is nullptr";
            return -1;
        }
        ret = HB_SYS_Free(buffer->frame_info.phy_ptr[0],
                buffer->frame_info.vir_ptr[0]);
        if (ret == 0) {
            ret = HB_SYS_Free(buffer->frame_info.phy_ptr[1],
                    buffer->frame_info.vir_ptr[1]);
            if (ret == 0) {
                LOGD << "mmzFree index: "  << i;
                LOGD << "vio_buf_addr: "   << buffer;
                LOGD << "buf_y_paddr: "    << buffer->frame_info.phy_ptr[0];
                LOGD << "buf_y_vaddr "     << buffer->frame_info.vir_ptr[0];
                LOGD << "buf_uv_paddr: "   << buffer->frame_info.phy_ptr[1];
                LOGD << "buf_uv_vaddr: "   << buffer->frame_info.vir_ptr[1];
            } else {
                LOGE << "hb sys free uv vio buf: " << i
                     << "failed, ret: " << ret;
                return ret;
            }
        } else {
            LOGE << "hb sys free y vio buf: " << i << "failed, ret: " << ret;
            return ret;
        }
        free(buffer);
    }

    int stream_buf_cnt = vb_stream_buf_queue_[chn].size();
    for (int i = 0; i < stream_buf_cnt; i++) {
        stream_buffer = vb_stream_buf_queue_[chn].front();
        vb_stream_buf_queue_[chn].pop();
        if (stream_buffer == nullptr) {
            LOGE << "stream buf is nullptr";
            return -1;
        }
        free(stream_buffer);
    }

    ret = HB_VP_Exit();
    if (ret == 0) {
        LOGI << "vp exit ok!";
    } else {
        LOGE << "vp exit error!";
        return ret;
    }

    return 0;
}

}  // namespace vision
}  // namespace horizon
