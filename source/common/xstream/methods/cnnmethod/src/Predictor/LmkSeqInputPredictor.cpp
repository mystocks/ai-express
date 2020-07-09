/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: LmkSeqInputPredictor.cpp
 * @Brief: definition of the LmkSeqInputPredictor
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: 2020-05-25
 */

#include <algorithm>
#include <memory>
#include <vector>
#include "CNNMethod/Predictor/LmkSeqInputPredictor.h"
#include "hobotlog/hobotlog.hpp"
#include "hobotxstream/profiler.h"
#include "horizon/vision_type/vision_type.hpp"
#include "horizon/vision_type/vision_type_common.h"

using hobot::vision::BBox;
using hobot::vision::Landmarks;
using hobot::vision::Points;
using hobot::vision::ImageFrame;
using hobot::vision::PymImageFrame;
typedef std::shared_ptr<ImageFrame> ImageFramePtr;

namespace xstream {

void InputFloat2Int(int8_t* feat_buffer1, int input_quanti_factor,
                    float* BPU_input_data, int N, int H, int W, int C,
                    int frame_input_size) {
  int8_t tmp = 0;
  int index = 0;
  for (int n = 0; n < N; ++n) {
    for (int h = 0; h < H; ++h) {
      for (int w = 0; w < W; ++w) {
        for (int c = 0; c < C; ++c) {
          int offset = n * H * W * C + h * W * C + w * C + c;
          tmp = floor(BPU_input_data[offset] * input_quanti_factor);
          if (tmp > 127) {
            tmp = 127;
          } else if (tmp < -128) {
            tmp = -128;
          }
          feat_buffer1[index] = tmp;
          index++;
        }
      }
    }
  }
  HOBOT_CHECK(index == frame_input_size);
}

int32_t LmkSeqInputPredictor::Init(std::shared_ptr<CNNMethodConfig> config) {
  // TODO(shiyu.fu): do not need to create fake image handle
  Predictor::Init(config);

  seq_len_ = config->GetIntValue("seq_len");
  kps_len_ = config->GetIntValue("kps_len");
  buf_len_ = config->GetIntValue("buf_len");
  input_shift_ = config->GetIntValue("input_shift");
  stride_ = config->GetFloatValue("stride");
  max_gap_ = config->GetFloatValue("max_gap");
  kps_norm_scale_ = config->GetFloatValue("kps_norm_scale");
  norm_kps_conf_ = config->GetBoolValue("norm_kps_conf");

  data_processor_.Init(kps_len_, seq_len_, stride_, max_gap_,
                       kps_norm_scale_, norm_kps_conf_, buf_len_);
  return 0;
}

void LmkSeqInputPredictor::UpdateParam(
                           std::shared_ptr<CNNMethodConfig> config) {
  Predictor::UpdateParam(config);
  if (config->KeyExist("expand_scale")) {
    norm_params_.expand_scale = config->GetFloatValue("expand_scale");
  }
  if (config->KeyExist("seq_len")) {
    seq_len_ = config->GetIntValue("seq_len");
  }
  if (config->KeyExist("kps_len")) {
    kps_len_ = config->GetIntValue("kps_len");
  }
  if (config->KeyExist("stride")) {
    stride_ = config->GetFloatValue("stride");
  }
  if (config->KeyExist("max_gap")) {
    max_gap_ = config->GetFloatValue("max_gap");
  }
  if (config->KeyExist("buf_len")) {
    buf_len_ = config->GetIntValue("buf_len");
  }
  if (config->KeyExist("kps_norm_scale")) {
    kps_norm_scale_ = config->GetFloatValue("buf_len");
  }
}

void LmkSeqInputPredictor::Do(CNNMethodRunData *run_data) {
  static float timestamp_;

  int frame_size = run_data->input->size();
  run_data->mxnet_output.resize(frame_size);
  run_data->input_dim_size.resize(frame_size);
  run_data->real_nhwc = model_info_.real_nhwc_;
  run_data->elem_size = model_info_.elem_size_;

  for (int frame_idx = 0; frame_idx < frame_size; frame_idx++) {  // loop frame
    auto &input_data = (*(run_data->input))[frame_idx];
    // timestamp_ += delta_;
    timestamp_ += stride_;

    auto rois = std::static_pointer_cast<BaseDataVector>(input_data[0]);
    auto kpses = std::static_pointer_cast<BaseDataVector>(input_data[1]);
    auto disappeared_track_ids =
        std::static_pointer_cast<BaseDataVector>(input_data[2]);

    int box_num = rois->datas_.size();
    run_data->mxnet_output[frame_idx].resize(box_num);
    run_data->input_dim_size[frame_idx] = box_num;

    int input_n = model_info_.input_nhwc_[0];
    int input_h = model_info_.input_nhwc_[1];
    int input_w = model_info_.input_nhwc_[2];
    int input_c = model_info_.input_nhwc_[3];
    int input_size = input_n * input_h * input_w * input_c;

    int handle_num =
        max_handle_num_ < 0 ? box_num : std::min(max_handle_num_, box_num);
    for (int roi_idx = 0; roi_idx < box_num; ++roi_idx) {
      auto p_roi =
        std::static_pointer_cast<XStreamData<BBox>>(rois->datas_[roi_idx]);
      auto p_kps =
        std::static_pointer_cast<XStreamData<Landmarks>>(
            kpses->datas_[roi_idx]);
      if (p_roi->state_ != xstream::DataState::VALID ||
          p_kps->value.values.size() != static_cast<uint32_t>(kps_len_) ||
          roi_idx >= handle_num) {
        continue;
      } else {
        // pre-process
        int8_t* feature_buf_ = nullptr;
        {
          RUN_PROCESS_TIME_PROFILER(model_name_ + "_preprocess")
          RUN_FPS_PROFILER(model_name_ + "_preprocess")
          data_processor_.Update(p_roi, p_kps, timestamp_);
          auto cached_kpses = std::make_shared<BaseDataVector>();
          data_processor_.GetClipKps(cached_kpses, p_roi->value.id, timestamp_);
          // concatenate
          if (cached_kpses->datas_.size() < 32) {
            continue;
          } else {
            Tensor tensor(input_n, input_h, input_w, input_c);
            for (int nn = 0; nn < input_n; ++nn) {
              for (int hh = 0; hh < input_h; ++hh) {
                for (int ww = 0; ww < input_w; ++ww) {
                  auto cur_kps =
                      std::static_pointer_cast<XStreamData<Landmarks>>(
                          cached_kpses->datas_[ww]);
                  tensor.Set(nn, hh, ww, 0, cur_kps->value.values[hh].x);
                  tensor.Set(nn, hh, ww, 1, cur_kps->value.values[hh].y);
                  tensor.Set(nn, hh, ww, 2, cur_kps->value.values[hh].score);
                }
              }
            }
            float *BPU_input_data = tensor.data.data();
            feature_buf_ = static_cast<int8_t*>(malloc(input_size));
            int input_quanti_factor = 1 << input_shift_;
            InputFloat2Int(feature_buf_, input_quanti_factor, BPU_input_data,
                           input_n, input_h, input_w, input_c, input_size);
          }
        }
        // run model
        ModelOutputBuffer buf(model_info_, 1);
        {
          RUN_PROCESS_TIME_PROFILER(model_name_ + "_runmodel")
          RUN_FPS_PROFILER(model_name_ + "_runmodel")
          int ret = RunModelFromDDRWithConvertLayout(feature_buf_, input_size,
                                  buf.out_bufs_.data(), buf.out_bufs_.size());
          if (ret == -1) {
            LOGE << "Failed to RunModelFromDDR";
            free(feature_buf_);
            continue;
          }
        }
        LOGD << "RunModelFromDDR success";
        free(feature_buf_);
        // convert to mxnet out
        uint32_t layer_size = model_info_.output_layer_size_.size();
        auto &one_tgt_mxnet = run_data->mxnet_output[frame_idx][roi_idx];
        one_tgt_mxnet.resize(layer_size);
        {
          RUN_PROCESS_TIME_PROFILER(model_name_ + "_do_hbrt")
          RUN_FPS_PROFILER(model_name_ + "_do_hbrt")
          for (uint32_t j = 0; j < layer_size; j++) {
            uint32_t feature_size = model_info_.mxnet_output_layer_size_[j];
            one_tgt_mxnet[j].resize(feature_size);
            BPU_convertLayout(bpu_handle_, feature_bufs_[j].data(),
                              BPU_getRawBufferPtr(buf.out_bufs_[j]),
                              model_name_.c_str(), BPULayoutType::LAYOUT_NHWC,
                              j, -1, -1, -1, -1);
            ConvertOutputToMXNet(feature_bufs_[j].data(),
                                 one_tgt_mxnet[j].data(), j);
          }
        }
        LOGD << "do hbrt success";
      }
    }

    data_processor_.Clean(disappeared_track_ids);
  }
}

}  // namespace xstream
