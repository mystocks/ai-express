/*!
 * Copyright (c) 2016-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     uvcplugin.cpp
 * \Author   ronghui.zhang
 * \Version  1.0.0.0
 * \Date     2020.5.12
 * \Brief    implement of api file
 */

#include "uvcplugin/uvcplugin.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include "hobotlog/hobotlog.hpp"
#include "xproto/message/pluginflow/msg_registry.h"
#include "xproto_msgtype/vioplugin_data.h"
#include "xproto_msgtype/smartplugin_data.h"

namespace horizon {
namespace vision {
namespace xproto {
namespace Uvcplugin {
using horizon::vision::xproto::XPluginErrorCode;
using horizon::vision::xproto::basic_msgtype::VioMessage;
using horizon::vision::xproto::basic_msgtype::SmartMessage;
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_UVC_MESSAGE)
using std::chrono::milliseconds;
bool write_flag = true;

UvcPlugin::UvcPlugin(std::string config_file) {
  config_file_ = config_file;
  LOGI << "UwsPlugin smart config file:" << config_file_;
  stop_flag_ = false;
  Reset();
}

UvcPlugin::~UvcPlugin() {
//  config_ = nullptr;
}

int UvcPlugin::Init() {
  LOGI << "UvcPlugin Init";

  config_ = std::make_shared<UvcConfig>(config_file_);
  if (!config_ || !config_->LoadConfig()) {
    LOGE << "failed to load config file";
    return -1;
  }

  uvc_server_ = std::make_shared<UvcServer>();
  if (uvc_server_->Init(config_->image_width_, config_->image_height_, config_->video_type_)) {
    LOGE << "UwsPlugin Init uWS server failed";
    return -1;
  }

  venc_client_ = std::make_shared<VencClient>();
  if (venc_client_->Init(config_->image_width_, config_->image_height_, config_->video_type_)) {
    LOGE << "UvcPlugin Init Uvc server failed";
    return -1;
  }

  hid_manager_ = std::make_shared<HidManager>(config_file_);
  if (hid_manager_->Init()) {
    LOGE << "UvcPlugin Init HidManager failed";
    return -1;
  }

  RegisterMsg(TYPE_IMAGE_MESSAGE,
    std::bind(&UvcPlugin::FeedVideo, this, std::placeholders::_1));
  RegisterMsg(TYPE_DROP_IMAGE_MESSAGE,
    std::bind(&UvcPlugin::FeedVideoDrop, this, std::placeholders::_1)); 
  RegisterMsg(TYPE_SMART_MESSAGE,
    std::bind(&UvcPlugin::FeedSmart, this, std::placeholders::_1));
  // 调用父类初始化成员函数注册信息
  XPluginAsync::Init();
  return 0;
}

int UvcPlugin::Reset() {
  LOGI << __FUNCTION__ << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
  return 0;
}

int UvcPlugin::Start() {
  uvc_server_->Start();
  venc_client_->Start();
  hid_manager_->Start();
  return 0;
}

int UvcPlugin::Stop() {
  LOGI << "UvcPlugin::Stop()";
  stop_flag_ = true;
  uvc_server_->DeInit();
  venc_client_->Stop();
  venc_client_->DeInit();
  hid_manager_->Stop();
  return 0;
}

int UvcPlugin::FeedSmart(XProtoMessagePtr msg) {
  return hid_manager_->FeedSmart(msg);
}

int UvcPlugin::FeedVideo(XProtoMessagePtr msg) {
  LOGI << "UvcPlugin Feedvideo";
  if (!uvc_server_->uvc_stream_on) {
      return 0;
  }
  VIDEO_FRAME_S pstFrame;
  memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
  auto vio_msg = std::dynamic_pointer_cast<VioMessage>(msg);
  int level = config_->layer_;
#ifdef X2
  auto image = vio_msg->image_[0]->image;
  img_info_t *src_img = reinterpret_cast<img_info_t * >(image.data);
  auto height = src_img->down_scale[level].height;
  auto width = src_img->down_scale[level].width;
  auto y_addr = src_img->down_scale[level].y_vaddr;
  auto uv_addr = src_img->down_scale[level].c_vaddr;
  HOBOT_CHECK(height) << "width = " << width << ", height = " << height;
  auto img_y_size = height * src_img->down_scale[level].step;
  auto img_uv_size = img_y_size / 2;

  pstFrame.stVFrame.pix_format = HB_PIXEL_FORMAT_NV12;
  pstFrame.stVFrame.width = src_img->down_scale[0].width;
  pstFrame.stVFrame.height = src_img->down_scale[0].height;
  pstFrame.stVFrame.size = src_img->down_scale[0].width *
                src_img->down_scale[0].height * 3 / 2;
  pstFrame.stVFrame.phy_ptr[0] = src_img->down_scale[0].y_paddr;
  pstFrame.stVFrame.phy_ptr[1] = src_img->down_scale[0].c_paddr;
  pstFrame.stVFrame.vir_ptr[0] = src_img->down_scale[0].y_vaddr;
  pstFrame.stVFrame.vir_ptr[1] = src_img->down_scale[0].c_vaddr;
#endif
  LOGI << "uvcplugin feed video";
#ifdef X3
  auto pym_image = vio_msg->image_[0];
  pstFrame.stVFrame.pix_format = HB_PIXEL_FORMAT_NV12;
  pstFrame.stVFrame.height = pym_image->down_scale[level].height;
  pstFrame.stVFrame.width = pym_image->down_scale[level].width;
  pstFrame.stVFrame.size =
    pym_image->down_scale[level].height * pym_image->down_scale[level].width * 3/2;
  pstFrame.stVFrame.phy_ptr[0] = (uint32_t)pym_image->down_scale[level].y_paddr;
  pstFrame.stVFrame.phy_ptr[1] = (uint32_t)pym_image->down_scale[level].c_paddr;
  pstFrame.stVFrame.vir_ptr[0] = (char *)pym_image->down_scale[level].y_vaddr;
  pstFrame.stVFrame.vir_ptr[1] = (char *)pym_image->down_scale[level].c_vaddr;
  pstFrame.stVFrame.pts = vio_msg->time_stamp_;
#endif
   int ret = HB_VENC_SendFrame(0, &pstFrame, 3000);
   if (ret < 0) {
       LOGE << "HB_VENC_SendFrame 0 error!!!ret " << ret;
   }

  return 0;
}

int UvcPlugin::FeedVideoDrop(XProtoMessagePtr msg) {
  if (!uvc_server_->uvc_stream_on) {
      return 0;
  }
  VIDEO_FRAME_S pstFrame;
  memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
  auto vio_msg = std::dynamic_pointer_cast<VioMessage>(msg);
  int level = config_->layer_;
#ifdef X2
  auto image = vio_msg->image_[0]->image;
  img_info_t *src_img = reinterpret_cast<img_info_t * >(image.data);
  auto height = src_img->down_scale[level].height;
  auto width = src_img->down_scale[level].width;
  auto y_addr = src_img->down_scale[level].y_vaddr;
  auto uv_addr = src_img->down_scale[level].c_vaddr;
  HOBOT_CHECK(height) << "width = " << width << ", height = " << height;
  auto img_y_size = height * src_img->down_scale[level].step;
  auto img_uv_size = img_y_size / 2;

  pstFrame.stVFrame.pix_format = HB_PIXEL_FORMAT_NV12;
  pstFrame.stVFrame.width = src_img->down_scale[0].width;
  pstFrame.stVFrame.height = src_img->down_scale[0].height;
  pstFrame.stVFrame.size = src_img->down_scale[0].width *
                src_img->down_scale[0].height * 3 / 2;
  pstFrame.stVFrame.phy_ptr[0] = src_img->down_scale[0].y_paddr;
  pstFrame.stVFrame.phy_ptr[1] = src_img->down_scale[0].c_paddr;
  pstFrame.stVFrame.vir_ptr[0] = src_img->down_scale[0].y_vaddr;
  pstFrame.stVFrame.vir_ptr[1] = src_img->down_scale[0].c_vaddr;

#endif

#ifdef X3
  auto pym_image = vio_msg->image_[0];
  pstFrame.stVFrame.pix_format = HB_PIXEL_FORMAT_NV12;
  pstFrame.stVFrame.height = pym_image->down_scale[level].height;
  pstFrame.stVFrame.width = pym_image->down_scale[level].width;
  pstFrame.stVFrame.size =
    pym_image->down_scale[level].height * pym_image->down_scale[level].width * 3/2;
  pstFrame.stVFrame.phy_ptr[0] = (uint32_t)pym_image->down_scale[level].y_paddr;
  pstFrame.stVFrame.phy_ptr[1] = (uint32_t)pym_image->down_scale[level].c_paddr;
  pstFrame.stVFrame.vir_ptr[0] = (char *)pym_image->down_scale[level].y_vaddr;
  pstFrame.stVFrame.vir_ptr[1] = (char *)pym_image->down_scale[level].c_vaddr;
  pstFrame.stVFrame.pts = vio_msg->time_stamp_;
#endif
   int ret = HB_VENC_SendFrame(0, &pstFrame, 3000);
   if (ret < 0) {
       LOGE << "HB_VENC_SendFrame 0 error!!!ret " << ret;
   }

  return 0;
}
}  // namespace Uvcplugin
}  // namespace xproto
}  // namespace vision
}  // namespace horizon