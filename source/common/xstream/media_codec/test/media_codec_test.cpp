/**
 * * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: media_codec_test.cpp
 * @Brief: media codec unit test
 * @Author: yong.wu
 * @Email: yong.wu@horizon.ai
 * @Date: 2020-07-01
 * @Last Modified by: yong.wu
 * @Last Modified time: 2020-07-01
 */

#include <stdio.h>
#include <string.h>
#include <iostream>
#include "hobotlog/hobotlog.hpp"
#include "media_codec/media_codec_manager.h"

#define MAX_FILE_NAME_LEN 128
#define MAX_FILE_NUM 100
using horizon::vision::MediaCodecManager;

int main(int argc, char *argv[]) {
    int chn, ret, i;
    int pic_w = 1920;
    int pic_h = 1080;
    int pic_size = pic_w * pic_h * 3 / 2;
    int y_pic_size = pic_w * pic_h;
    int uv_pic_size = pic_w * pic_h / 2;
    int vb_num = 5;
    int y_readsize, uv_readsize;
    FILE *inFile = NULL;
    FILE *outFile[MAX_FILE_NUM] = { NULL };
    const char *inputFileName = "./data/input_nv12_1080p.yuv";
    const char *outputFileNameBak = "./data/output_stream_1080p";
    char outputFileName[MAX_FILE_NAME_LEN];
    int frame_buf_depth = 5;
    int out_stream_num = 5;
    iot_venc_src_buf_t *frame_buf = nullptr;
    iot_venc_stream_buf_t *stream_buf = nullptr;

    SetLogLevel(HOBOT_LOG_INFO);
    inFile = fopen(inputFileName, "rb");
    HOBOT_CHECK(inFile != NULL);

    /* 1. media codec init */
    /* 1.1 get media codec manager and module init */
    MediaCodecManager &manager = MediaCodecManager::Get();
    ret = manager.ModuleInit();
    HOBOT_CHECK(ret == 0);
    /* 1.2 get media codec venc chn */
    chn = manager.GetEncodeChn();
    HOBOT_CHECK(ret == 0);
    /* 1.3 media codec venc chn init */
    ret = manager.EncodeChnInit(chn, PT_JPEG, pic_w, pic_h,
            frame_buf_depth, HB_PIXEL_FORMAT_NV12);
    HOBOT_CHECK(ret == 0);
    /* 1.4 set media codec venc jpg chn qfactor params */
    ret = manager.SetUserQfactorParams(chn, 80);
    HOBOT_CHECK(ret == 0);
    /* 1.5 set media codec venc jpg chn qfactor params */
    ret = manager.EncodeChnStart(chn);
    HOBOT_CHECK(ret == 0);
    /* 1.6 alloc media codec vb buffer init */
    ret = manager.VbBufInit(chn, pic_w, pic_h, pic_w, pic_size, vb_num);
    HOBOT_CHECK(ret == 0);

    /* 2. start encode yuv to jpeg */
    HOBOT_CHECK(out_stream_num < MAX_FILE_NUM);
    for (i = 0; i < out_stream_num; i++) {
        memset(outputFileName, 0x00, MAX_FILE_NAME_LEN);
        snprintf(outputFileName, MAX_FILE_NAME_LEN, "%s_%d.jpg",
                outputFileNameBak, i);
        outFile[i] = fopen(outputFileName, "wb");
        HOBOT_CHECK(outFile[i] != NULL);
        /* 2.1 get media codec vb buf for store src yuv data */
        ret = manager.GetVbBuf(chn, &frame_buf);
        LOGE << "frame_buf: " << frame_buf;
        HOBOT_CHECK(ret == 0);
        /* 2.2 get src yuv data */
        y_readsize = fread(frame_buf->frame_info.vir_ptr[0], 1,
                y_pic_size, inFile);
        uv_readsize = fread(frame_buf->frame_info.vir_ptr[1], 1,
                uv_pic_size, inFile);
        rewind(inFile);
        LOGI << "y_readsize: " << y_readsize << "uv_readsize: " << uv_readsize;
        /* 2.3. encode yuv data to jpg */
        ret = manager.EncodeYuvToJpg(chn, frame_buf, &stream_buf);
        HOBOT_CHECK(ret == 0);
        fwrite(stream_buf->stream_info.pstPack.vir_ptr,
                stream_buf->stream_info.pstPack.size, 1, outFile[i]);
        /* 2.4 free jpg stream buf */
        ret = manager.FreeStream(chn, stream_buf);
        HOBOT_CHECK(ret == 0);
        /* 2.5 free media codec vb buf */
        ret = manager.FreeVbBuf(chn, frame_buf);
        HOBOT_CHECK(ret == 0);
    }

    /* 3. media codec deinit */
    /* 3.1 media codec chn stop */
    ret = manager.EncodeChnStop(chn);
    HOBOT_CHECK(ret == 0);
    /* 3.2 media codec chn deinit */
    ret = manager.EncodeChnDeInit(chn);
    HOBOT_CHECK(ret == 0);
    /* 3.3 media codec vb buf deinit */
    ret = manager.VbBufDeInit(chn);
    HOBOT_CHECK(ret == 0);
    /* 3.4 media codec module deinit */
    ret = manager.ModuleDeInit();
    HOBOT_CHECK(ret == 0);

    if (inFile) fclose(inFile);
    for (i = 0; i < out_stream_num; i++) {
        if (outFile[i])
            fclose(outFile[i]);
    }

    return 0;
}

