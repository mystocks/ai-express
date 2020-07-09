//
// Copyright 2019 Horizon Robotics.
//

#include "ssd_method/ssd_predictor.h"

#include <sys/time.h>

#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>

#include "bpu_predict/bpu_internal.h"
#include "bpu_predict/bpu_predict.h"
#include "hobotlog/hobotlog.hpp"
#include "hobotxstream/profiler.h"
#include "horizon/vision_type/vision_type.hpp"
#include "ssd_method/ssd_post_process_module.h"
#ifdef X3
#include "./bpu_predict_x3.h"
#endif

using ImageFramePtr = std::shared_ptr<hobot::vision::ImageFrame>;
using hobot::vision::PymImageFrame;

namespace xstream {

const char *kPyramidImage = "PymImageFrame";
class ModelOutputBuffer {
 public:
  ModelOutputBuffer(const BPUModelInfo &output_info,
                    std::vector<BPU_Buffer_Handle> &output_buf)
      : output_buf_(output_buf) {
    // alloc output buffer.
    for (int i = 0; i < output_info.num; ++i) {
      BPU_Buffer_Handle out_handle = BPU_createEmptyBPUBuffer();
      output_buf_.push_back(out_handle);
    }
    LOGD << "create bpu buffer success.";
  }
  ~ModelOutputBuffer() {
    // release output buffer.
    for (auto &buf : output_buf_) {
      BPU_freeBPUBuffer(buf);
    }
    output_buf_.clear();
    LOGD << "release bpu buffer success.";
  }

 private:
  std::vector<BPU_Buffer_Handle> &output_buf_;
};

int SSD_Predictor::Init(const std::string &config_file) {
  ParseConfig(config_file);
  bpu_handle_ =
      BPUModelManager::Get().GetBpuHandle(model_file_path_, bpu_config_path_);
  BPUModelInfo input_info;
  int model_input_ret =
      BPU_getModelInputInfo(bpu_handle_, model_name_.data(), &input_info);
  if (model_input_ret != 0) {
    LOGF << "here get model: " << model_name_.data()
         << " input info failed: " << BPU_getLastError(bpu_handle_)
         << std::endl;
    return 1;
  }

  auto ret =
      BPU_getModelOutputInfo(bpu_handle_, model_name_.data(), &output_info_);

  if (ret != 0) {
    LOGE << "here get model: " << model_name_
         << " output info failed: " << BPU_getLastError(bpu_handle_);
    LOGF << "can't get model output info" << std::endl;
  }
  out_num_ = output_info_.num;
  GetModelInfo(model_name_);

  ssd_post_process_.reset(new xstream::SSDPostProcessModule(
      model_name_, bpu_handle_, pyramid_layer_, platform_, score_thld_,
      nms_thld_, config_file, output_layer_big_endian_, model_out_types_));
  int r = ssd_post_process_->Init();
  if (r != 0) {
    LOGF << "ssd_post_process failed" << std::endl;
    return 1;
  }
  return 0;
}

void SSD_Predictor::RunSingleFrame(const std::vector<BaseDataPtr> &frame_input,
                                   std::vector<BaseDataPtr> &frame_output) {
  // only one input slot -> PyramidImage
  HOBOT_CHECK(frame_input.size() == 1);
  const auto frame_img_ = frame_input[0];

  for (size_t out_index = 0; out_index < method_outs_.size(); ++out_index) {
    frame_output.push_back(std::make_shared<xstream::BaseDataVector>());
  }
  auto xstream_img =
      std::static_pointer_cast<XStreamData<ImageFramePtr>>(frame_img_);

  std::string img_type = xstream_img->value->type;

  BPUModelHandle model_handle;
  int ret = 0;
  int src_img_width = 0;
  int src_img_height = 0;

  ModelOutputBuffer output_buf(output_info_, out_buf_);
  {
    RUN_PROCESS_TIME_PROFILER("SSD RunModelFromPyramid");
    RUN_FPS_PROFILER("SSD RunModelFromPyramid");

    if (img_type == kPyramidImage) {
      auto pyramid_image =
          std::static_pointer_cast<PymImageFrame>(xstream_img->value);
#ifdef X2
      src_img_height = pyramid_image->img.src_img.height;
      src_img_width = pyramid_image->img.src_img.width;
      LOGD << "h and w " << src_img_height << ", " << src_img_width
           << std::endl;
      ret = BPU_runModelFromPyramid(bpu_handle_, model_name_.c_str(),
                                    static_cast<void *>(&(pyramid_image->img)),
                                    pyramid_layer_, out_buf_.data(),
                                    out_buf_.size(), &model_handle);
#endif
#ifdef X3
      src_img_height = pyramid_image->down_scale[0].height;
      src_img_width = pyramid_image->down_scale[0].width;

      bpu_predict_x3::PyramidResult bpu_predict_pyramid;
      Convert(*pyramid_image, bpu_predict_pyramid);

      ret = BPU_runModelFromPyramid(bpu_handle_, model_name_.c_str(),
                                    static_cast<void *>(&(bpu_predict_pyramid)),
                                    pyramid_layer_, out_buf_.data(),
                                    out_buf_.size(), &model_handle);
    #endif


    } else {
      HOBOT_CHECK(0) << "Not support this input type: " << img_type;
    }

    LOGD << "image height: " << src_img_height << "width: " << src_img_width;
    if (ret != 0) {
      LOGE << "Run model failed: " << BPU_getLastError(bpu_handle_);
      BPU_releaseModelHandle(bpu_handle_, model_handle);
      return;
    }
    ret = BPU_getModelOutput(bpu_handle_, model_handle);
    BPU_releaseModelHandle(bpu_handle_, model_handle);
    if (ret != 0) {
      LOGE << "Get model out put failed: " << BPU_getLastError(bpu_handle_);
      return;
    }
  }
  ssd_post_process_->GetFrameOutput(out_buf_, frame_output[0]);
  RUN_PROCESS_TIME_PROFILER("SSD PostProcess");
  RUN_FPS_PROFILER("SSD PostProcess");
}

void SSD_Predictor::ParseConfig(const std::string &config_file) {
  FR_Config cfg_jv;
  std::ifstream infile(config_file.c_str());
  infile >> cfg_jv;
  config_.reset(new Config(cfg_jv));

  auto net_info = config_->GetSubConfig("net_info");
  score_thld_ = config_->GetFloatValue("score_threshold", 0.0);
  nms_thld_ = config_->GetFloatValue("nms_threshold", 0.3);
  platform_ = config_->GetSTDStringValue("input_platform", "mxnet");

  model_name_ = net_info->GetSTDStringValue("model_name");
  model_version_ = net_info->GetSTDStringValue("model_version", "1.0.0");
  model_input_width_ = net_info->GetIntValue("model_input_width", 960);
  model_input_height_ = net_info->GetIntValue("model_input_height", 540);
  pyramid_layer_ = net_info->GetIntValue("pyramid_layer", 4);
  model_out_types_ = net_info->GetIntArray("model_out_type");

  method_outs_ = config_->GetSTDStringArray("method_outs");
  LOGD << "method out type:";
  for (const auto &method_out : method_outs_) {
    LOGD << method_out;
  }

  std::string parent_path = GetParentPath(config_file);
  bpu_config_path_ =
      parent_path + config_->GetSTDStringValue("bpu_config_path");
  model_file_path_ =
      parent_path + config_->GetSTDStringValue("model_file_path");
  LOGD << "config file parent path: " << parent_path
       << " bpu_config_path: " << bpu_config_path_
       << " model_file_path: " << model_file_path_;
}

void SSD_Predictor::GetModelInfo(const std::string &model_name) {
  uint32_t output_layer_num = output_info_.num;
  LOGD << "output_layer_num = " << output_layer_num
       << ", output_num = " << out_num_ << std::endl;
  output_layer_big_endian_.clear();
  output_layer_big_endian_.resize(output_layer_num);
  for (size_t i = 0; i < output_layer_num; ++i) {
    // get bit_endian
    output_layer_big_endian_.push_back(output_info_.is_big_endian[i]);
  }
}

std::string SSD_Predictor::GetParentPath(const std::string &path) {
  auto pos = path.rfind('/');
  if (std::string::npos != pos) {
    auto parent = path.substr(0, pos);
    return parent + "/";
  } else {
    return std::string("./");
  }
}
}  // namespace xstream

