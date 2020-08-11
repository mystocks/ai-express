/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file votmodule.cpp
 * @brief vot module
 * @author kairui.wang
 * @email kairui.wang@horizon.ai
 * @date 2020/07/22
 */
#include "./votmodule.h"

#include <fstream>
#include <string.h>

#include "./hb_vot.h"
#include "hobotlog/hobotlog.hpp"
#include "opencv2/opencv.hpp"

#include <sys/stat.h>

namespace horizon {
namespace vision {

VotModule::VotModule() : group_id_(-1), timeout_(40) { ; }

VotModule::~VotModule() { ; }

int getAllAttribute() {
  int ret = 0;
  VOT_PUB_ATTR_S stPubAttr = {};
  VOT_VIDEO_LAYER_ATTR_S stLayerAttr = {};
  VOT_CSC_S stCsc = {};
  VOT_UPSCALE_ATTR_S stUpScale = {};
  VOT_CHN_ATTR_S stChnAttr = {};
  VOT_CHN_ATTR_EX_S stChnAttrEx = {};
  VOT_CROP_INFO_S stCrop = {};
  POINT_S stPoint = {};
  int i = 0;

  do {
    // dev
    ret = HB_VOT_GetPubAttr(0, &stPubAttr);
    if (ret) {
      printf("HB_VOT_GetPubAttr failed.\n");
      //   break;
    }
    printf("stPubAttr output mode :%d\n", stPubAttr.enOutputMode);
    printf("stPubAttr bgcolor :0x%x\n", stPubAttr.u32BgColor);
    printf("stPubAttr intfsync :%d\n", stPubAttr.enIntfSync);
    printf("stPubAttr syncinfo hbp :%d\n", stPubAttr.stSyncInfo.hbp);
    printf("stPubAttr syncinfo hfp :%d\n", stPubAttr.stSyncInfo.hfp);
    printf("stPubAttr syncinfo hs :%d\n", stPubAttr.stSyncInfo.hs);
    printf("stPubAttr syncinfo vbp :%d\n", stPubAttr.stSyncInfo.vbp);
    printf("stPubAttr syncinfo vfp :%d\n", stPubAttr.stSyncInfo.vfp);
    printf("stPubAttr syncinfo vs :%d\n", stPubAttr.stSyncInfo.vs);
    printf("stPubAttr syncinfo vfp_cnt :%d\n", stPubAttr.stSyncInfo.vfp_cnt);

#if 1
    // get/set videolayer
    ret = HB_VOT_GetVideoLayerAttr(0, &stLayerAttr);
    if (ret) {
      printf("HB_VOT_GetVideoLayerAttr failed.\n");
      //   break;
    }
    printf("stLayer width:%d\n", stLayerAttr.stImageSize.u32Width);
    printf("stLayer height:%d\n", stLayerAttr.stImageSize.u32Height);

    ret = HB_VOT_GetVideoLayerCSC(0, &stCsc);
    if (ret) {
      printf("HB_VOT_GetVideoLayerCSC failed.\n");
      //   break;
    }
    printf("stCsc luma :%d\n", stCsc.u32Luma);
    printf("stCsc contrast :%d\n", stCsc.u32Contrast);
    printf("stCsc hue :%d\n", stCsc.u32Hue);
    printf("stCsc satuature :%d\n", stCsc.u32Satuature);

    ret = HB_VOT_GetVideoLayerUpScale(0, &stUpScale);
    if (ret) {
      printf("HB_VOT_GetVideoLayerUpScale failed.\n");
      //   break;
    }
    printf("stUpScale src width :%d\n", stUpScale.src_width);
    printf("stUpScale src height :%d\n", stUpScale.src_height);
    printf("stUpScale tgt width :%d\n", stUpScale.tgt_width);
    printf("stUpScale tgt height :%d\n", stUpScale.tgt_height);
    printf("stUpScale pos x :%d\n", stUpScale.pos_x);
    printf("stUpScale pos y :%d\n", stUpScale.pos_y);

    // set/get chn
    for (i = 0; i < 4; i++) {
      ret = HB_VOT_GetChnAttr(0, i, &stChnAttr);
      if (ret) {
        printf("HB_VOT_GetChnAttr %d failed.\n", i);
        //   break;
      }
      printf("stChnAttr priority %d :%d\n", i, stChnAttr.u32Priority);
      printf("stChnAttr src width %d :%d\n", i, stChnAttr.u32SrcWidth);
      printf("stChnAttr src height %d :%d\n", i, stChnAttr.u32SrcHeight);
      printf("stChnAttr s32X %d :%d\n", i, stChnAttr.s32X);
      printf("stChnAttr s32Y %d :%d\n", i, stChnAttr.s32Y);
      printf("stChnAttr u32DstWidth %d :%d\n", i, stChnAttr.u32DstWidth);
      printf("stChnAttr u32DstHeight %d :%d\n", i, stChnAttr.u32DstHeight);

      ret = HB_VOT_GetChnCrop(0, i, &stCrop);
      if (ret) {
        printf("HB_VOT_GetChnCrop %d failed.\n", i);
        // break;
      }
      printf("stCrop width %d :%d\n", i, stCrop.u32Width);
      printf("stCrop height %d :%d\n", i, stCrop.u32Height);

      ret = HB_VOT_GetChnDisplayPosition(0, i, &stPoint);
      if (ret) {
        printf("HB_VOT_GetChnDisplayPosition %d failed.\n", i);
        // break;
      }
      printf("stPoint s32x %d :%d\n", i, stPoint.s32X);
      printf("stPoint s32y %d :%d\n", i, stPoint.s32Y);

      ret = HB_VOT_GetChnAttrEx(0, i, &stChnAttrEx);
      if (ret) {
        printf("HB_VOT_GetChnAttrEx %d failed.\n", i);
        // break;
      }
      printf("stChnAttrEx format %d :%d\n", i, stChnAttrEx.format);
      printf("stChnAttrEx alpha_en %d :%d\n", i, stChnAttrEx.alpha_en);
      printf("stChnAttrEx alpha_sel %d :%d\n", i, stChnAttrEx.alpha_sel);
      printf("stChnAttrEx alpha %d :%d\n", i, stChnAttrEx.alpha);
      printf("stChnAttrEx keycolor %d :%d\n", i, stChnAttrEx.keycolor);
      printf("stChnAttrEx ov_mode %d :%d\n", i, stChnAttrEx.ov_mode);
    }
#endif
  } while (0);

  // ret = HB_VOT_Disable(0);
  // if (ret) printf("HB_VOT_Disable failed.\n");
  return 0;
}

int getsetAttribute() {
  int ret = 0;
  VOT_PUB_ATTR_S stPubAttr = {};
  VOT_VIDEO_LAYER_ATTR_S stLayerAttr = {};
  VOT_CSC_S stCsc = {};
  VOT_UPSCALE_ATTR_S stUpScale = {};
  VOT_CHN_ATTR_S stChnAttr = {};
  VOT_CHN_ATTR_EX_S stChnAttrEx = {};
  VOT_CROP_INFO_S stCrop = {};
  POINT_S stPoint = {};

  do {
    // dev
    ret = HB_VOT_GetPubAttr(0, &stPubAttr);
    if (ret) {
      printf("HB_VOT_GetPubAttr failed.\n");
      //   break;
    }
    printf("stPubAttr output mode :%d\n", stPubAttr.enOutputMode);
    printf("stPubAttr bgcolor :%d\n", stPubAttr.u32BgColor);
    stPubAttr.enOutputMode = HB_VOT_OUTPUT_BT1120;  // HB_VOT_OUTPUT_MIPI;
    stPubAttr.u32BgColor = 0xFF7F88;
    ret = HB_VOT_SetPubAttr(0, &stPubAttr);
    if (ret) {
      printf("HB_VOT_SetPubAttr failed.\n");
      //   break;
    }
    ret = HB_VOT_Enable(0);
    if (ret) {
      printf("HB_VOT_Enable failed.\n");
      //   break;
    }
#if 1
    // get/set videolayer
    ret = HB_VOT_GetVideoLayerAttr(0, &stLayerAttr);
    if (ret) {
      printf("HB_VOT_GetVideoLayerAttr failed.\n");
      //   break;
    }
    printf("stLayer width:%d\n", stLayerAttr.stImageSize.u32Width);
    printf("stLayer height:%d\n", stLayerAttr.stImageSize.u32Height);
    stLayerAttr.stImageSize.u32Width = 1920;
    stLayerAttr.stImageSize.u32Height = 1080;
    ret = HB_VOT_SetVideoLayerAttr(0, &stLayerAttr);
    if (ret) {
      printf("HB_VOT_SetVideoLayerAttr failed.\n");
      //   break;
    }

    ret = HB_VOT_GetVideoLayerCSC(0, &stCsc);
    if (ret) {
      printf("HB_VOT_GetVideoLayerCSC failed.\n");
      //   break;
    }
    printf("stCsc luma :%d\n", stCsc.u32Luma);
    printf("stCsc contrast :%d\n", stCsc.u32Contrast);
    printf("stCsc hue :%d\n", stCsc.u32Hue);
    printf("stCsc satuature :%d\n", stCsc.u32Satuature);
    stCsc.u32Luma = 60;
    stCsc.u32Contrast = 60;
    stCsc.u32Hue = 60;
    stCsc.u32Satuature = 60;
    ret = HB_VOT_SetVideoLayerCSC(0, &stCsc);

    ret = HB_VOT_GetVideoLayerUpScale(0, &stUpScale);
    if (ret) {
      printf("HB_VOT_GetVideoLayerUpScale failed.\n");
      //   break;
    }
    printf("stUpScale src width :%d\n", stUpScale.src_width);
    printf("stUpScale src height :%d\n", stUpScale.src_height);
    printf("stUpScale tgt width :%d\n", stUpScale.tgt_width);
    printf("stUpScale tgt height :%d\n", stUpScale.tgt_height);
    printf("stUpScale pos x :%d\n", stUpScale.pos_x);
    printf("stUpScale pos y :%d\n", stUpScale.pos_y);
    stUpScale.src_width = 1280;
    stUpScale.src_height = 720;
    stUpScale.tgt_width = 1920;
    stUpScale.tgt_height = 1080;
    ret = HB_VOT_SetVideoLayerUpScale(0, &stUpScale);
    if (ret) {
      printf("HB_VOT_SetVideoLayerUpScale failed.\n");
      //   break;
    }

    ret = HB_VOT_EnableVideoLayer(0);
    if (ret) {
      printf("HB_VOT_EnableVideoLayer failed.\n");
      //   break;
    }

    // set/get chn
    ret = HB_VOT_GetChnAttr(0, 0, &stChnAttr);
    if (ret) {
      printf("HB_VOT_GetChnAttr failed.\n");
      //   break;
    }
    printf("stChnAttr priority :%d\n", stChnAttr.u32Priority);
    printf("stChnAttr src width :%d\n", stChnAttr.u32SrcWidth);
    printf("stChnAttr src height :%d\n", stChnAttr.u32SrcHeight);
    printf("stChnAttr s32X :%d\n", stChnAttr.s32X);
    printf("stChnAttr s32Y :%d\n", stChnAttr.s32Y);
    printf("stChnAttr u32DstWidth :%d\n", stChnAttr.u32DstWidth);
    printf("stChnAttr u32DstHeight :%d\n", stChnAttr.u32DstHeight);
    stChnAttr.u32Priority = 0;
    stChnAttr.u32SrcWidth = 1920;
    stChnAttr.u32SrcHeight = 1080;
    stChnAttr.s32X = 0;
    stChnAttr.s32Y = 0;
    stChnAttr.u32DstWidth = 1920;
    stChnAttr.u32DstHeight = 1080;
    ret = HB_VOT_SetChnAttr(0, 0, &stChnAttr);
    if (ret) {
      printf("HB_VOT_SetChnAttr failed.\n");
      //   break;
    }

    ret = HB_VOT_EnableChn(0, 0);
    if (ret) {
      printf("HB_VOT_EnableChn failed.\n");
      //   break;
    }

    ret = HB_VOT_GetChnCrop(0, 0, &stCrop);
    if (ret) {
      printf("HB_VOT_GetChnCrop failed.\n");
      // break;
    }
    printf("stCrop width :%d\n", stCrop.u32Width);
    printf("stCrop height :%d\n", stCrop.u32Height);
    stCrop.u32Width = 1280;
    stCrop.u32Height = 720;
    ret = HB_VOT_SetChnCrop(0, 0, &stCrop);
    if (ret) {
      printf("HB_VOT_SetChnCrop failed.\n");
      // break;
    }

    ret = HB_VOT_GetChnDisplayPosition(0, 0, &stPoint);
    if (ret) {
      printf("HB_VOT_GetChnDisplayPosition failed.\n");
      // break;
    }
    printf("stPoint s32x :%d\n", stPoint.s32X);
    printf("stPoint s32y :%d\n", stPoint.s32Y);
    stPoint.s32X = 200;
    stPoint.s32Y = 200;
    ret = HB_VOT_SetChnDisplayPosition(0, 0, &stPoint);
    if (ret) {
      printf("HB_VOT_SetChnDisplayPosition failed.\n");
      // break;
    }

    ret = HB_VOT_GetChnAttrEx(0, 0, &stChnAttrEx);
    if (ret) {
      printf("HB_VOT_GetChnAttrEx failed.\n");
      // break;
    }
    printf("stChnAttrEx format :%d\n", stChnAttrEx.format);
    printf("stChnAttrEx alpha_en :%d\n", stChnAttrEx.alpha_en);
    printf("stChnAttrEx alpha_sel :%d\n", stChnAttrEx.alpha_sel);
    printf("stChnAttrEx alpha :%d\n", stChnAttrEx.alpha);
    printf("stChnAttrEx keycolor :%d\n", stChnAttrEx.keycolor);
    printf("stChnAttrEx ov_mode :%d\n", stChnAttrEx.ov_mode);
    // stChnAttrEx.format = 1;
    stChnAttrEx.alpha_en = 1;
    stChnAttrEx.alpha_sel = 0;
    stChnAttrEx.alpha = 30;
    stChnAttrEx.keycolor = 0x7F88;
    stChnAttrEx.ov_mode = 1;
    ret = HB_VOT_SetChnAttrEx(0, 0, &stChnAttrEx);
    if (ret) {
      printf("HB_VOT_SetChnAttrEx failed.\n");
      // break;
    }
#endif
  } while (0);

  ret = HB_VOT_DisableChn(0, 0);
  if (ret)
    printf("HB_VOT_DisableChn failed.\n");

  ret = HB_VOT_DisableVideoLayer(0);
  if (ret)
    printf("HB_VOT_DisableVideoLayer failed.\n");

  ret = HB_VOT_Disable(0);
  if (ret)
    printf("HB_VOT_Disable failed.\n");

  return 0;
}

uint32_t get_file(const char *path, char **buff) {
  FILE *file = NULL;
  struct stat statbuf;

  file = fopen(path, "r");
  if (NULL == file) {
    printf("file %s open failed", path);
    return 0;
  }
  stat(path, &statbuf);
  if (0 == statbuf.st_size) {
    printf("read file size error");
    fclose(file);
    return 0;
  }
  *buff = static_cast<char *>(malloc(statbuf.st_size));
  if (NULL == *buff) {
    printf("file buff malloc failed");
    fclose(file);
    return 0;
  }
  fread(*buff, statbuf.st_size, 1, file);
  fclose(file);
  return statbuf.st_size;
}

int VotModule::Init(uint32_t group_id, const PipeModuleInfo *module_info) {
  int ret = 0;
  image_height_ = 540;
  image_width_ = 960;
  // char *framebuf[4];
  // int framesize[4];
  // VOT_FRAME_INFO_S stFrame = {};
  VOT_VIDEO_LAYER_ATTR_S stLayerAttr;
  VOT_CHN_ATTR_S stChnAttr;
  // VOT_WB_ATTR_S stWbAttr;
  VOT_CROP_INFO_S cropAttrs;
  // hb_vio_buffer_t iar_buf = {0};
  VOT_PUB_ATTR_S devAttr;
  // iar_mmap_channel0();

  devAttr.enIntfSync = VOT_OUTPUT_1920x1080;
  devAttr.u32BgColor = 0x108080;
  devAttr.enOutputMode = HB_VOT_OUTPUT_BT1120;
  ret = HB_VOT_SetPubAttr(0, &devAttr);
  if (ret) {
    printf("HB_VOT_SetPubAttr failed\n");
    return -1;
  }
  ret = HB_VOT_Enable(0);
  if (ret)
    printf("HB_VOT_Enable failed.\n");

  ret = HB_VOT_GetVideoLayerAttr(0, &stLayerAttr);
  if (ret) {
    printf("HB_VOT_GetVideoLayerAttr failed.\n");
  }
  // memset(&stLayerAttr, 0, sizeof(stLayerAttr));
  stLayerAttr.stImageSize.u32Width = 1920;
  stLayerAttr.stImageSize.u32Height = 1080;

  stLayerAttr.panel_type = 0;
  stLayerAttr.rotate = 0;
  stLayerAttr.dithering_flag = 0;
  stLayerAttr.dithering_en = 0;
  stLayerAttr.gamma_en = 0;
  stLayerAttr.hue_en = 0;
  stLayerAttr.sat_en = 0;
  stLayerAttr.con_en = 0;
  stLayerAttr.bright_en = 0;
  stLayerAttr.theta_sign = 0;
  stLayerAttr.contrast = 0;
  stLayerAttr.theta_abs = 0;
  stLayerAttr.saturation = 0;
  stLayerAttr.off_contrast = 0;
  stLayerAttr.off_bright = 0;
  stLayerAttr.user_control_disp = 0;
  stLayerAttr.big_endian = 0;
  stLayerAttr.display_addr_type = 2;
  stLayerAttr.display_addr_type_layer1 = 2;
  // stLayerAttr.display_addr_type = 0;
  // stLayerAttr.display_addr_type_layer1 = 0;
  ret = HB_VOT_SetVideoLayerAttr(0, &stLayerAttr);
  if (ret)
    printf("HB_VOT_SetVideoLayerAttr failed.\n");

  ret = HB_VOT_EnableVideoLayer(0);
  if (ret)
    printf("HB_VOT_EnableVideoLayer failed.\n");

  stChnAttr.u32Priority = 2;
  stChnAttr.s32X = 0;
  stChnAttr.s32Y = 0;
  stChnAttr.u32SrcWidth = 1920;
  stChnAttr.u32SrcHeight = 1080;
  stChnAttr.u32DstWidth = 1920;
  stChnAttr.u32DstHeight = 1080;
  ret = HB_VOT_SetChnAttr(0, 0, &stChnAttr);
  printf("HB_VOT_SetChnAttr 0: %d\n", ret);
  // stChnAttr.s32X = 960;
  // stChnAttr.s32Y = 0;
  // ret = HB_VOT_SetChnAttr(0, 1, &stChnAttr);
  // printf("HB_VOT_SetChnAttr 0: %d\n", ret);
  // stChnAttr.s32X = 0;
  // stChnAttr.s32Y = 540;
  // ret = HB_VOT_SetChnAttr(0, 2, &stChnAttr);
  // printf("HB_VOT_SetChnAttr 0: %d\n", ret);
  // stChnAttr.s32X = 960;
  // stChnAttr.s32Y = 540;
  // ret = HB_VOT_SetChnAttr(0, 3, &stChnAttr);
  // printf("HB_VOT_SetChnAttr 0: %d\n", ret);

  cropAttrs.u32Width = stChnAttr.u32DstWidth;
  cropAttrs.u32Height = stChnAttr.u32DstHeight;
  ret = HB_VOT_SetChnCrop(0, 0, &cropAttrs);
  printf("HB_VOT_EnableChn: %d\n", ret);
  // ret = HB_VOT_SetChnCrop(0, 1, &cropAttrs);
  // printf("HB_VOT_EnableChn: %d\n", ret);
  // stChnAttr.u32Priority = 1;
  // ret = HB_VOT_SetChnCrop(0, 2, &cropAttrs);
  // printf("HB_VOT_EnableChn: %d\n", ret);
  // ret = HB_VOT_SetChnCrop(0, 3, &cropAttrs);
  // printf("HB_VOT_EnableChn: %d\n", ret);

  ret = HB_VOT_EnableChn(0, 0);
  printf("HB_VOT_EnableChn: %d\n", ret);
  // ret = HB_VOT_EnableChn(0, 1);
  // printf("HB_VOT_EnableChn: %d\n", ret);
  // ret = HB_VOT_EnableChn(0, 2);
  // printf("HB_VOT_EnableChn: %d\n", ret);
  // ret = HB_VOT_EnableChn(0, 3);
  // printf("HB_VOT_EnableChn: %d\n", ret);

  buffer_ =
      static_cast<char *>(malloc(4 * image_height_ * image_width_ * 3 / 2));
  return ret;
}

int VotModule::Input(void *data) {
  int ret = 0;
  VotData *vot_data = static_cast<VotData *> (data);
  VOT_FRAME_INFO_S stFrame = {};
  if (vot_data->channel == 0) {
    for (uint32_t i = 0; i < 540; ++i) {
      memcpy(buffer_ + i * 1920, vot_data->y_virtual_addr + i * image_width_,
             image_width_);
    }
    for (uint32_t i = 0; i < 540 / 2; ++i) {
      memcpy(buffer_ + (i + 1080) * 1920,
             vot_data->uv_virtual_addr + i * image_width_, image_width_);
    }
    cv::Mat yuv_I420(1080, 1920, CV_8UC1, buffer_);
    for (uint32_t i = 0; i < vot_data->boxes.size(); ++i) {
      int x1 = vot_data->boxes[i][0];
      int y1 = vot_data->boxes[i][1];
      int x2 = vot_data->boxes[i][2];
      int y2 = vot_data->boxes[i][3];
      cv::rectangle(yuv_I420, cv::Point(x1, y1), cv::Point(x2, y2),
                    cv::Scalar(255));
    }

  } else if (vot_data->channel == 1) {
    for (uint32_t i = 0; i < 540; ++i) {
      memcpy(buffer_ + i * 1920 + 960,
             vot_data->y_virtual_addr + i * image_width_, image_width_);
    }
    for (uint32_t i = 0; i < 540 / 2; ++i) {
      memcpy(buffer_ + (i + 1080) * 1920 + 960,
             vot_data->uv_virtual_addr + i * image_width_, image_width_);
    }
    cv::Mat yuv_I420(1080, 1920, CV_8UC1, buffer_);
    for (uint32_t i = 0; i < vot_data->boxes.size(); ++i) {
      int x1 = vot_data->boxes[i][0] + 960;
      int y1 = vot_data->boxes[i][1];
      int x2 = vot_data->boxes[i][2] + 960;
      int y2 = vot_data->boxes[i][3];
      cv::rectangle(yuv_I420, cv::Point(x1, y1), cv::Point(x2, y2),
                    cv::Scalar(255));
    }
  } else if (vot_data->channel == 2) {
    for (uint32_t i = 0; i < 540; ++i) {
      memcpy(buffer_ + (i + 540) * 1920,
             vot_data->y_virtual_addr + i * image_width_, image_width_);
    }
    for (uint32_t i = 0; i < 540 / 2; ++i) {
      memcpy(buffer_ + (i + 1080 + 540 / 2) * 1920,
             vot_data->uv_virtual_addr + i * image_width_, image_width_);
    }
    cv::Mat yuv_I420(1080, 1920, CV_8UC1, buffer_);
    for (uint32_t i = 0; i < vot_data->boxes.size(); ++i) {
      int x1 = vot_data->boxes[i][0];
      int y1 = vot_data->boxes[i][1] + 540;
      int x2 = vot_data->boxes[i][2];
      int y2 = vot_data->boxes[i][3] + 540;
      cv::rectangle(yuv_I420, cv::Point(x1, y1), cv::Point(x2, y2),
                    cv::Scalar(255));
    }
  } else if (vot_data->channel == 3) {
    for (uint32_t i = 0; i < 540; ++i) {
      memcpy(buffer_ + (i + 540) * 1920 + 960,
             vot_data->y_virtual_addr + i * image_width_, image_width_);
    }
    for (uint32_t i = 0; i < 540 / 2; ++i) {
      memcpy(buffer_ + (i + 1080 + 540 / 2) * 1920 + 960,
             vot_data->uv_virtual_addr + i * image_width_, image_width_);
    }
    cv::Mat yuv_I420(1080, 1920, CV_8UC1, buffer_);
    for (uint32_t i = 0; i < vot_data->boxes.size(); ++i) {
      int x1 = vot_data->boxes[i][0] + 960;
      int y1 = vot_data->boxes[i][1] + 540;
      int x2 = vot_data->boxes[i][2] + 960;
      int y2 = vot_data->boxes[i][3] + 540;
      cv::rectangle(yuv_I420, cv::Point(x1, y1), cv::Point(x2, y2),
                    cv::Scalar(255));
    }
  }
  // memcpy(buffer_[vot_data->channel], vot_data->y_virtual_addr,
  // image_width_*image_height_); memcpy(buffer_[vot_data->channel] +
  // image_width_*image_height_, vot_data->uv_virtual_addr,
  // image_width_*image_height_/2); cv::Mat yuv_I420(image_height_,
  // image_width_, CV_8UC1, buffer_[vot_data->channel]); for (uint32_t i = 0; i
  // < vot_data->boxes.size(); ++i) {
  //   int x1 = vot_data->boxes[i][0];
  //   int y1 = vot_data->boxes[i][1];
  //   int x2 = vot_data->boxes[i][2];
  //   int y2 = vot_data->boxes[i][3];
  //   cv::rectangle(yuv_I420, cv::Point(x1, y1), cv::Point(x2, y2),
  //   cv::Scalar(255));
  // }
  stFrame.addr = buffer_;
  stFrame.size = 1920 * 1080 * 3 / 2;
  // static int count= 0;
  // std::ofstream outfile;
  // outfile.open("image_pym" + std::to_string(count++) + ".yuv", std::ios::ate
  // | std::ios::out | std::ios::binary); outfile.write(buffer,
  // width*height*3/2); outfile.close();
  ret = HB_VOT_SendFrame(0, vot_data->channel, &stFrame, -1);

  // pym_buffer_t *pym_buffer = (pym_buffer_t *)data;
  // VOT_FRAME_INFO_S stFrame = {};
  // // int width = 1920;
  // // int height = 1080;
  // // memcpy(buffer, pym_buffer->pym[0].addr[0], width*height);
  // // memcpy(buffer + width*height, pym_buffer->pym[0].addr[1],
  // width*height/2); int width = 960; int height = 540; char *buffer = (char
  // *)malloc(width*height*3/2); memcpy(buffer, pym_buffer->pym[1].addr[0],
  // width*height); memcpy(buffer + width*height, pym_buffer->pym[1].addr[1],
  // width*height/2); cv::Mat yuv_I420(height, width, CV_8UC1, buffer);
  // cv::rectangle(yuv_I420, cv::Point(10, 10), cv::Point(200, 200),
  // cv::Scalar(255)); stFrame.addr = buffer; stFrame.size = width*height*3/2;
  // static int count= 0;
  // std::ofstream outfile;
  // outfile.open("image_pym" + std::to_string(count++) + ".yuv", std::ios::ate
  // | std::ios::out | std::ios::binary);
  // // printf("pym layer 4 h:%d  w:%d\n", pym_buf_.pym[1].height,
  // pym_buf_.pym[1].width); outfile.write(buffer, width*height*3/2);
  // // outfile.write(reinterpret_cast<char *>(pym_buf_.pym[1].addr[0]),
  // pym_buf_.pym[1].height*pym_buf_.pym[1].width);
  // // outfile.write(reinterpret_cast<char *>(pym_buf_.pym[1].addr[1]),
  // pym_buf_.pym[1].height*pym_buf_.pym[1].width/2); outfile.close(); ret =
  // HB_VOT_SendFrame(0, 0, &stFrame, -1);
  return ret;
}

int VotModule::Output(void **data) {
  int ret = 0;
  // uint32_t index = buffer_index_ % frameDepth_;
  // ret = HB_VPS_GetChnFrame(group_id_, 6, &buffers_[index], timeout_);
  // if (ret != 0) {
  //   LOGW << "HB_VPS_GetChnFrame Failed. ret = " << ret;
  //   data = nullptr;
  //   return ret;
  // }
  // buffer_index_++;
  // *data = &buffers_[index];
  // // static int count= 0;
  // // std::ofstream outfile;
  // // outfile.open("image_pym" + std::to_string(count++) + ".yuv",
  // std::ios::ate | std::ios::out | std::ios::binary);
  // // printf("pym layer 4 h:%d  w:%d\n", pym_buf_.pym[1].height,
  // pym_buf_.pym[1].width);
  // // outfile.write(reinterpret_cast<char *>(pym_buf_.pym[1].addr[0]),
  // pym_buf_.pym[1].height*pym_buf_.pym[1].width);
  // // outfile.write(reinterpret_cast<char *>(pym_buf_.pym[1].addr[1]),
  // pym_buf_.pym[1].height*pym_buf_.pym[1].width/2);
  // // outfile.close();
  return ret;
}

int VotModule::OutputBufferFree(void *data) {
  int ret = 0;
  // if (data != nullptr) {
  //   ret = HB_VPS_ReleaseChnFrame(group_id_, 6, (pym_buffer_t*)data);
  //   if (ret != 0) {
  //     LOGE << "HB_VPS_ReleaseChnFrame Failed. ret = " << ret;
  //     return ret;
  //   }
  //   return ret;
  // } else {
  //   return -1;
  // }
  return ret;
}

int VotModule::Start() {
  int ret = 0;
  // ret = HB_VPS_StartGrp(group_id_);
  // if (ret) {
  //   LOGE << "HB_VPS_StartGrp Failed. ret = " << ret;
  //   return ret;
  // }
  return ret;
}

int VotModule::Stop() {
  int ret = 0;
  ret = HB_VOT_DisableChn(0, 0);
  if (ret)
    printf("HB_VOT_DisableChn failed.\n");
  // ret = HB_VOT_DisableChn(0, 1);
  // if (ret) printf("HB_VOT_DisableChn failed.\n");
  // ret = HB_VOT_DisableChn(0, 0);
  // if (ret) printf("HB_VOT_DisableChn failed.\n");

  ret = HB_VOT_DisableVideoLayer(0);
  if (ret)
    printf("HB_VOT_DisableVideoLayer failed.\n");

  ret = HB_VOT_Disable(0);
  if (ret)
    printf("HB_VOT_Disable failed.\n");
  return ret;
}

int VotModule::DeInit() {
  int ret = 0;
  free(buffer_);
  // free(buffer_[1]);
  // free(buffer_[2]);
  // free(buffer_[3]);
  // ret = HB_VPS_DestroyGrp(group_id_);
  // if (ret) {
  //   LOGE << "HB_VPS_DestroyGrp Failed. ret = " << ret;
  //   return ret;
  // }
  return ret;
}

}  // namespace vision
}  // namespace horizon
