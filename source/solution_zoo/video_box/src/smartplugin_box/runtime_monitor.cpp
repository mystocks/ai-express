/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-04 03:07:26
 * @Version: v0.0.1
 * @Brief: runtime monitor implementation
 * @Note:  extracted from repo xperson's global_config.cpp
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-08-22 23:46:17
 */

#include "smartplugin_box/runtime_monitor.h"
#include "smartplugin/utils/time_helper.h"
#include "hobotlog/hobotlog.hpp"
#include "horizon/vision/util.h"
#include <memory>
#include <mutex>

namespace horizon {
namespace vision {
namespace xproto {
namespace smartplugin_multiplebox {
using ImageFramePtr = std::shared_ptr<hobot::vision::ImageFrame>;
using XStreamImageFramePtr = xstream::XStreamData<ImageFramePtr>;

void RuntimeMonitor::PushFrame(const SmartInput *input) {
  std::unique_lock<std::mutex> lock(map_mutex_);
  HOBOT_CHECK(input) << "Null HorizonVisionImageFrame";
  auto frame_info = input->frame_info;
  HOBOT_CHECK(frame_info->num_ > 0);
#ifdef X2
  auto image0 = frame_info->image_[0];
  uint64_t frame_id = image0->frame_id;
#endif
#ifdef X3
  std::shared_ptr<hobot::vision::PymImageFrame> image0 = frame_info->image_[0];
  uint64_t frame_id = image0->frame_id;
  LOGI << "PushFrame frame_id = " << frame_id << std::endl;
#endif
  input_frames_[frame_id].image_num = frame_info->num_;
  input_frames_[frame_id].img = frame_info->image_;
  input_frames_[frame_id].context = input->context;
}

RuntimeMonitor::InputFrameData
RuntimeMonitor::PopFrame(const int32_t &frame_id) {
  std::unique_lock<std::mutex> lock(map_mutex_);
  InputFrameData input;
  auto itr = input_frames_.find(frame_id);
  if (itr != input_frames_.end()) {
    input = itr->second;
    LOGI << "Pop frame " << frame_id;
    input_frames_.erase(itr);
  }
  return input;
}

RuntimeMonitor::RuntimeMonitor() { Reset(); }

void RuntimeMonitor::FrameStatistic(int channel) {
  static int frameCount[4] = { 0 };

  auto last_time = TP[channel];
  ++frameCount[channel];

  auto curTime = hobot::Timer::toc(*last_time);
  // 统计数据发送帧率
  if (curTime > 1000) {
    auto fps = frameCount[channel];
    frameCount[channel] = 0;
    *last_time = hobot::Timer::tic();
    LOGW << "Smart fps = " << fps << " channel: " << channel;
  }
}

bool RuntimeMonitor::Reset() {
  TP.clear();
  for (auto i = 0; i < channel_; ++i) {
    TP.emplace_back(std::shared_ptr<Time_Point>(
      new Time_Point(hobot::Timer::tic())));
  }
  return true;
}

}  // namespace smartplugin_multiplebox
}  // namespace xproto
}  //  namespace vision
}  //  namespace horizon
