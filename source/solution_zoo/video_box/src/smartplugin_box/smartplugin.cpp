/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-01 20:38:52
 * @Version: v0.0.1
 * @Brief: smartplugin impl based on xstream.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-09-29 05:04:11
 */

#include <fstream>
#include <functional>
#include <memory>
#include <string>

#include "hobotlog/hobotlog.hpp"
#include "xproto/message/pluginflow/flowmsg.h"
#include "xproto/message/pluginflow/msg_registry.h"
#include "xproto/plugin/xpluginasync.h"

#include "hobotxsdk/xstream_sdk.h"
#include "horizon/vision/util.h"
#include "horizon/vision_type/vision_error.h"
#include "horizon/vision_type/vision_type.hpp"
#include "smartplugin_box/convert.h"
#include "smartplugin_box/runtime_monitor.h"
#include "smartplugin_box/smart_config.h"
#include "smartplugin_box/smartplugin.h"
#include "./votmodule.h"
#include "xproto_msgtype/protobuf/x2.pb.h"
#include "xproto_msgtype/protobuf/x3.pb.h"
#include "xproto_msgtype/vioplugin_data.h"

namespace horizon {
namespace vision {
namespace xproto {
namespace smartplugin_multiplebox {

using horizon::vision::xproto::XPluginAsync;
using horizon::vision::xproto::XProtoMessage;
using horizon::vision::xproto::XProtoMessagePtr;

using horizon::vision::xproto::basic_msgtype::VioMessage;
using ImageFramePtr = std::shared_ptr<hobot::vision::ImageFrame>;
using XStreamImageFramePtr = xstream::XStreamData<ImageFramePtr>;

using xstream::InputDataPtr;
using xstream::OutputDataPtr;
using xstream::XStreamSDK;

using horizon::iot::smartplugin_multiplebox::Convertor;

enum CameraFeature {
  NoneFeature = 0,               // 无
  TrafficConditionFeature = 0b1, // 拥堵事故
  SnapFeature = 0b10,            // 抓拍
  AnomalyFeature = 0b100,        // 违法行为
  CountFeature = 0b1000,         // 车流计数
  GisFeature = 0b10000,          // 定位
};

XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_SMART_MESSAGE)

SmartPlugin::SmartPlugin(const std::string &config_file) {
  config_file_ = config_file;
  LOGI << "smart config file:" << config_file_;
  monitor_.reset(new RuntimeMonitor());
  Json::Value cfg_jv;
  std::ifstream infile(config_file_);
  infile >> cfg_jv;
  config_.reset(new JsonConfigWrapper(cfg_jv));
  ParseConfig();
  vot_module_ = std::make_shared<VotModule>();
}

void SmartPlugin::ParseConfig() {
  xstream_workflow_cfg_file_ =
      config_->GetSTDStringValue("xstream_workflow_file");
  enable_profile_ = config_->GetBoolValue("enable_profile");
  profile_log_file_ = config_->GetSTDStringValue("profile_log_path");
  if (config_->HasMember("enable_result_to_json")) {
    result_to_json = config_->GetBoolValue("enable_result_to_json");
  }
  LOGI << "xstream_workflow_file:" << xstream_workflow_cfg_file_;
  LOGI << "enable_profile:" << enable_profile_
       << ", profile_log_path:" << profile_log_file_;
}

int SmartPlugin::Init() {
  // init for xstream sdk
  LOGD << "samrt plugin init";
  sdk_.reset(xstream::XStreamSDK::CreateSDK());
  sdk_->SetConfig("config_file", xstream_workflow_cfg_file_);
  if (sdk_->Init() != 0) {
    LOGE << "smart plugin init failed!!!";
    return kHorizonVisionInitFail;
  }
  if (enable_profile_) {
    sdk_->SetConfig("profiler", "on");
    sdk_->SetConfig("profiler_file", profile_log_file_);
  }
  sdk_->SetCallback(
      std::bind(&SmartPlugin::OnCallback, this, std::placeholders::_1));

  sdk2_.reset(xstream::XStreamSDK::CreateSDK());
  sdk2_->SetConfig("config_file", xstream_workflow_cfg_file_);
  if (sdk2_->Init() != 0) {
    LOGE << "smart plugin init failed!!!";
    return kHorizonVisionInitFail;
  }
  // if (enable_profile_) {
  //   sdk2->SetConfig("profiler", "on");
  //   sdk_2->SetConfig("profiler_file", profile_log_file_);
  // }
  sdk2_->SetCallback(
      std::bind(&SmartPlugin::OnCallback2, this, std::placeholders::_1));

  sdk3_.reset(xstream::XStreamSDK::CreateSDK());
  sdk3_->SetConfig("config_file", xstream_workflow_cfg_file_);
  if (sdk3_->Init() != 0) {
    LOGE << "smart plugin init failed!!!";
    return kHorizonVisionInitFail;
  }
  // if (enable_profile_) {
  //   sdk2->SetConfig("profiler", "on");
  //   sdk_2->SetConfig("profiler_file", profile_log_file_);
  // }
  sdk3_->SetCallback(
      std::bind(&SmartPlugin::OnCallback3, this, std::placeholders::_1));

  sdk4_.reset(xstream::XStreamSDK::CreateSDK());
  sdk4_->SetConfig("config_file", xstream_workflow_cfg_file_);
  if (sdk4_->Init() != 0) {
    LOGE << "smart plugin init failed!!!";
    return kHorizonVisionInitFail;
  }
  // if (enable_profile_) {
  //   sdk2->SetConfig("profiler", "on");
  //   sdk_2->SetConfig("profiler_file", profile_log_file_);
  // }
  sdk4_->SetCallback(
      std::bind(&SmartPlugin::OnCallback4, this, std::placeholders::_1));

  RegisterMsg(TYPE_IMAGE_MESSAGE,
              std::bind(&SmartPlugin::Feed, this, std::placeholders::_1));
  PipeModuleInfo module_info;
  vot_module_->Init(0, &module_info);
  buffer_[0] = (char *)malloc(960 * 540 * 3 / 2);
  return XPluginAsync::Init();
}

int SmartPlugin::Feed(XProtoMessagePtr msg) {
  // feed video frame to xstreamsdk.
  // 1. parse valid frame from msg
  LOGI << "smart plugin got one msg";
  auto valid_frame = std::static_pointer_cast<VioMessage>(msg);
  xstream::InputDataPtr input = Convertor::ConvertInput(valid_frame.get());
  SmartInput *input_wrapper = new SmartInput();
  input_wrapper->frame_info = valid_frame;
  input_wrapper->context = input_wrapper;
  monitor_->PushFrame(input_wrapper);
  monitor_->FrameStatistic(input_wrapper->frame_info->channel_);
  if (input_wrapper->frame_info->channel_ == 0) {
    if (sdk_->AsyncPredict(input) != 0) {
      return kHorizonVisionFailure;
    }
  } else if (input_wrapper->frame_info->channel_ == 1) {
    if (sdk2_->AsyncPredict(input) != 0) {
      return kHorizonVisionFailure;
    }
  } else if (input_wrapper->frame_info->channel_ == 2) {
    if (sdk3_->AsyncPredict(input) != 0) {
      return kHorizonVisionFailure;
    }
  } else {
    if (sdk4_->AsyncPredict(input) != 0) {
      return kHorizonVisionFailure;
    }
  }

  LOGI << "feed one task to xtream workflow";

  return 0;
}

int SmartPlugin::Start() {
  LOGW << "SmartPlugin Start";
  root.clear();
  return 0;
}

int SmartPlugin::Stop() {
  free(buffer_[0]);
  vot_module_->Stop();
  if (result_to_json) {
    remove("smart_data.json");
    Json::StreamWriterBuilder builder;
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    std::ofstream outputFileStream("smart_data.json");
    writer->write(root, &outputFileStream);
  }
  LOGW << "SmartPlugin Stop";
  return 0;
}

void SmartPlugin::OnCallback(xstream::OutputDataPtr xstream_out) {
  // On xstream async-predict returned,
  // transform xstream standard output to smart message.
  LOGI << "smart plugin got one smart result";
  HOBOT_CHECK(!xstream_out->datas_.empty()) << "Empty XStream Output";
  XStreamImageFramePtr *rgb_image = nullptr;

  for (const auto &output : xstream_out->datas_) {
    LOGD << output->name_ << ", type is " << output->type_;
    if (output->name_ == "rgb_image" || output->name_ == "image") {
      rgb_image = dynamic_cast<XStreamImageFramePtr *>(output.get());
    }
  }

  auto smart_msg = std::make_shared<CustomSmartMessage>(xstream_out);
  // Set origin input named "image" as output always.
  HOBOT_CHECK(rgb_image);
  VotData vot_data;
  vot_data.y_virtual_addr =
      (char *)((hobot::vision::PymImageFrame *)(rgb_image->value.get()))
          ->down_scale[4]
          .y_vaddr;
  vot_data.uv_virtual_addr =
      (char *)((hobot::vision::PymImageFrame *)(rgb_image->value.get()))
          ->down_scale[4]
          .c_vaddr;
  vot_data.channel = 0;
  for (const auto &output : xstream_out->datas_) {
    LOGD << output->name_ << ", type is " << output->type_;
    if (output->name_ == "face_final_box" ||
        output->name_ == "head_final_box" ||
        output->name_ == "body_final_box") {
      auto boxes = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGW << "box size: " << boxes->datas_.size();
      for (size_t i = 0; i < boxes->datas_.size(); ++i) {
        Json::Value target;
        auto box =
            std::static_pointer_cast<xstream::XStreamData<hobot::vision::BBox>>(
                boxes->datas_[i]);
        if (box->value.score < 0.9)
          continue;
        std::array<int, 4> box_temp;
        box_temp[0] = box->value.x1 / 2;
        box_temp[1] = box->value.y1 / 2;
        box_temp[2] = box->value.x2 / 2;
        box_temp[3] = box->value.y2 / 2;
        vot_data.boxes.push_back(box_temp);
      }
    }
    if (output->name_ == "kps") {
      auto lmks = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGW << "kps size: " << lmks->datas_.size();
      for (size_t i = 0; i < lmks->datas_.size(); ++i) {
        auto lmk = std::static_pointer_cast<
            xstream::XStreamData<hobot::vision::Landmarks>>(lmks->datas_[i]);
        for (size_t i = 0; i < lmk->value.values.size(); ++i) {
          const auto &point = lmk->value.values[i];
          if (point.score > 2) {
            std::array<int, 2> point_temp;
            point_temp[0] = point.x;
            point_temp[1] = point.y;
            vot_data.points.push_back(point_temp);
          } else {
          }
        }
      }
    }
  }
  vot_module_->Input(&vot_data);

  smart_msg->time_stamp = rgb_image->value->time_stamp;
  smart_msg->frame_id = rgb_image->value->frame_id;
  LOGI << "smart result frame_id = " << smart_msg->frame_id << std::endl;

  auto input = monitor_->PopFrame(smart_msg->frame_id);
  delete static_cast<SmartInput *>(input.context);
  PushMsg(smart_msg);
}

void SmartPlugin::OnCallback2(xstream::OutputDataPtr xstream_out) {
  // On xstream async-predict returned,
  // transform xstream standard output to smart message.
  LOGI << "smart plugin got one smart result callback2\n\n\n\n\n\n";
  HOBOT_CHECK(!xstream_out->datas_.empty()) << "Empty XStream Output";

  XStreamImageFramePtr *rgb_image = nullptr;

  for (const auto &output : xstream_out->datas_) {
    LOGD << output->name_ << ", type is " << output->type_;
    if (output->name_ == "rgb_image" || output->name_ == "image") {
      rgb_image = dynamic_cast<XStreamImageFramePtr *>(output.get());
    }
  }

  auto smart_msg = std::make_shared<CustomSmartMessage>(xstream_out);
  // Set origin input named "image" as output always.
  HOBOT_CHECK(rgb_image);
  VotData vot_data;
  vot_data.y_virtual_addr =
      (char *)((hobot::vision::PymImageFrame *)(rgb_image->value.get()))
          ->down_scale[4]
          .y_vaddr;
  vot_data.uv_virtual_addr =
      (char *)((hobot::vision::PymImageFrame *)(rgb_image->value.get()))
          ->down_scale[4]
          .c_vaddr;
  vot_data.channel = 1;
  for (const auto &output : xstream_out->datas_) {
    LOGD << output->name_ << ", type is " << output->type_;
    if (output->name_ == "face_final_box" ||
        output->name_ == "head_final_box" ||
        output->name_ == "body_final_box") {
      auto boxes = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGW << "box size: " << boxes->datas_.size();
      for (size_t i = 0; i < boxes->datas_.size(); ++i) {
        Json::Value target;
        auto box =
            std::static_pointer_cast<xstream::XStreamData<hobot::vision::BBox>>(
                boxes->datas_[i]);
        if (box->value.score < 0.9)
          continue;
        std::array<int, 4> box_temp;
        box_temp[0] = box->value.x1 / 2;
        box_temp[1] = box->value.y1 / 2;
        box_temp[2] = box->value.x2 / 2;
        box_temp[3] = box->value.y2 / 2;
        vot_data.boxes.push_back(box_temp);
      }
    }
    if (output->name_ == "kps") {
      auto lmks = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGW << "kps size: " << lmks->datas_.size();
      for (size_t i = 0; i < lmks->datas_.size(); ++i) {
        auto lmk = std::static_pointer_cast<
            xstream::XStreamData<hobot::vision::Landmarks>>(lmks->datas_[i]);
        for (size_t i = 0; i < lmk->value.values.size(); ++i) {
          const auto &point = lmk->value.values[i];
          if (point.score > 2) {
            std::array<int, 2> point_temp;
            point_temp[0] = point.x;
            point_temp[1] = point.y;
            vot_data.points.push_back(point_temp);
          } else {
          }
        }
      }
    }
  }
  vot_module_->Input(&vot_data);

  smart_msg->time_stamp = rgb_image->value->time_stamp;
  smart_msg->frame_id = rgb_image->value->frame_id;
  LOGI << "smart result frame_id = " << smart_msg->frame_id << std::endl;
  auto input = monitor_->PopFrame(smart_msg->frame_id);
  delete static_cast<SmartInput *>(input.context);
  PushMsg(smart_msg);
}

void SmartPlugin::OnCallback3(xstream::OutputDataPtr xstream_out) {
  // On xstream async-predict returned,
  // transform xstream standard output to smart message.
  LOGI << "smart plugin got one smart result callback3\n\n\n\n\n\n";
  HOBOT_CHECK(!xstream_out->datas_.empty()) << "Empty XStream Output";

  XStreamImageFramePtr *rgb_image = nullptr;

  for (const auto &output : xstream_out->datas_) {
    LOGD << output->name_ << ", type is " << output->type_;
    if (output->name_ == "rgb_image" || output->name_ == "image") {
      rgb_image = dynamic_cast<XStreamImageFramePtr *>(output.get());
    }
  }

  auto smart_msg = std::make_shared<CustomSmartMessage>(xstream_out);
  // Set origin input named "image" as output always.
  HOBOT_CHECK(rgb_image);
  VotData vot_data;
  vot_data.y_virtual_addr =
      (char *)((hobot::vision::PymImageFrame *)(rgb_image->value.get()))
          ->down_scale[4]
          .y_vaddr;
  vot_data.uv_virtual_addr =
      (char *)((hobot::vision::PymImageFrame *)(rgb_image->value.get()))
          ->down_scale[4]
          .c_vaddr;
  vot_data.channel = 2;
  for (const auto &output : xstream_out->datas_) {
    LOGD << output->name_ << ", type is " << output->type_;
    if (output->name_ == "face_final_box" ||
        output->name_ == "head_final_box" ||
        output->name_ == "body_final_box") {
      auto boxes = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGW << "box size: " << boxes->datas_.size();
      for (size_t i = 0; i < boxes->datas_.size(); ++i) {
        Json::Value target;
        auto box =
            std::static_pointer_cast<xstream::XStreamData<hobot::vision::BBox>>(
                boxes->datas_[i]);
        if (box->value.score < 0.9)
          continue;
        std::array<int, 4> box_temp;
        box_temp[0] = box->value.x1 / 2;
        box_temp[1] = box->value.y1 / 2;
        box_temp[2] = box->value.x2 / 2;
        box_temp[3] = box->value.y2 / 2;
        vot_data.boxes.push_back(box_temp);
      }
    }
    if (output->name_ == "kps") {
      auto lmks = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGW << "kps size: " << lmks->datas_.size();
      for (size_t i = 0; i < lmks->datas_.size(); ++i) {
        auto lmk = std::static_pointer_cast<
            xstream::XStreamData<hobot::vision::Landmarks>>(lmks->datas_[i]);
        for (size_t i = 0; i < lmk->value.values.size(); ++i) {
          const auto &point = lmk->value.values[i];
          if (point.score > 2) {
            std::array<int, 2> point_temp;
            point_temp[0] = point.x;
            point_temp[1] = point.y;
            vot_data.points.push_back(point_temp);
          } else {
          }
        }
      }
    }
  }
  vot_module_->Input(&vot_data);

  smart_msg->time_stamp = rgb_image->value->time_stamp;
  smart_msg->frame_id = rgb_image->value->frame_id;
  LOGI << "smart result frame_id = " << smart_msg->frame_id << std::endl;
  auto input = monitor_->PopFrame(smart_msg->frame_id);
  delete static_cast<SmartInput *>(input.context);
  PushMsg(smart_msg);
}

void SmartPlugin::OnCallback4(xstream::OutputDataPtr xstream_out) {
  // On xstream async-predict returned,
  // transform xstream standard output to smart message.
  LOGI << "smart plugin got one smart result callback4\n\n\n\n\n\n";
  HOBOT_CHECK(!xstream_out->datas_.empty()) << "Empty XStream Output";

  XStreamImageFramePtr *rgb_image = nullptr;

  for (const auto &output : xstream_out->datas_) {
    LOGD << output->name_ << ", type is " << output->type_;
    if (output->name_ == "rgb_image" || output->name_ == "image") {
      rgb_image = dynamic_cast<XStreamImageFramePtr *>(output.get());
    }
  }

  auto smart_msg = std::make_shared<CustomSmartMessage>(xstream_out);
  // Set origin input named "image" as output always.
  HOBOT_CHECK(rgb_image);
  VotData vot_data;
  vot_data.y_virtual_addr =
      (char *)((hobot::vision::PymImageFrame *)(rgb_image->value.get()))
          ->down_scale[4]
          .y_vaddr;
  vot_data.uv_virtual_addr =
      (char *)((hobot::vision::PymImageFrame *)(rgb_image->value.get()))
          ->down_scale[4]
          .c_vaddr;
  vot_data.channel = 3;
  for (const auto &output : xstream_out->datas_) {
    LOGD << output->name_ << ", type is " << output->type_;
    if (output->name_ == "face_final_box" ||
        output->name_ == "head_final_box" ||
        output->name_ == "body_final_box") {
      auto boxes = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGW << "box size: " << boxes->datas_.size();
      for (size_t i = 0; i < boxes->datas_.size(); ++i) {
        Json::Value target;
        auto box =
            std::static_pointer_cast<xstream::XStreamData<hobot::vision::BBox>>(
                boxes->datas_[i]);
        if (box->value.score < 0.9)
          continue;
        std::array<int, 4> box_temp;
        box_temp[0] = box->value.x1 / 2;
        box_temp[1] = box->value.y1 / 2;
        box_temp[2] = box->value.x2 / 2;
        box_temp[3] = box->value.y2 / 2;
        vot_data.boxes.push_back(box_temp);
      }
    }
    if (output->name_ == "kps") {
      auto lmks = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGW << "kps size: " << lmks->datas_.size();
      for (size_t i = 0; i < lmks->datas_.size(); ++i) {
        auto lmk = std::static_pointer_cast<
            xstream::XStreamData<hobot::vision::Landmarks>>(lmks->datas_[i]);
        for (size_t i = 0; i < lmk->value.values.size(); ++i) {
          const auto &point = lmk->value.values[i];
          if (point.score > 2) {
            std::array<int, 2> point_temp;
            point_temp[0] = point.x;
            point_temp[1] = point.y;
            vot_data.points.push_back(point_temp);
          } else {
          }
        }
      }
    }
  }
  vot_module_->Input(&vot_data);

  smart_msg->time_stamp = rgb_image->value->time_stamp;
  smart_msg->frame_id = rgb_image->value->frame_id;
  LOGI << "smart result frame_id = " << smart_msg->frame_id << std::endl;
  auto input = monitor_->PopFrame(smart_msg->frame_id);
  delete static_cast<SmartInput *>(input.context);
  PushMsg(smart_msg);
}
}  // namespace smartplugin_multiplebox
}  // namespace xproto
}  // namespace vision
}  // namespace horizon
