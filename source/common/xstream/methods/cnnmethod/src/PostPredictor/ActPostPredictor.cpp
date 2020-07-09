/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: ActPostPredictor.cpp
 * @Brief: definition of the ActPostPredictor
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: 2020-05-25
 */

#include <memory>
#include <vector>
#include <string>
#include "CNNMethod/PostPredictor/ActPostPredictor.h"
#include "CNNMethod/CNNConst.h"
#include "CNNMethod/util/util.h"
#include "hobotlog/hobotlog.hpp"
#include "hobotxstream/profiler.h"

namespace xstream {

static void setVaule(std::vector<BaseDataPtr> &batch_output,
                     std::vector<BaseDataPtr> value) {
  if (batch_output.size() != value.size()) return;

  auto size = batch_output.size();

  for (std::size_t i = 0; i < size; ++i) {
    auto vPtr = value[i];
    auto data_vector =
        std::static_pointer_cast<BaseDataVector>(batch_output[i]);
    data_vector->datas_.push_back(vPtr);
  }
}

int32_t ActPostPredictor::Init(std::shared_ptr<CNNMethodConfig> config) {
  PostPredictor::Init(config);
  threshold_ = config->GetFloatValue("threshold");
  groups_str_ = config->GetSTDStringValue("merge_groups");
  target_group_ = config->GetIntValue("target_group_idx");
  size_t delimiter = std::string::npos;
  size_t sub_delimiter = std::string::npos;
  std::string group = "";
  std::vector<int> vec;
  if (groups_str_.length() > 0) {
    delimiter = groups_str_.find(";");
    while (delimiter != std::string::npos) {
      // one group
      group = groups_str_.substr(0, delimiter);
      // remove brackets
      group = group.substr(1, group.length() - 2);
      // remove current group from groups
      groups_str_ =
          groups_str_.substr(delimiter + 1, groups_str_.length() - delimiter);
      // string to int
      sub_delimiter = group.find(",");
      while (sub_delimiter != std::string::npos) {
        int index = std::stoi(group.substr(0, sub_delimiter));
        vec.push_back(index);
        group = group.substr(sub_delimiter + 1, group.length() - sub_delimiter);
        sub_delimiter = group.find(",");
      }
      int index = std::stoi(group);
      vec.push_back(index);
      merge_groups_.push_back(vec);
      delimiter = groups_str_.find(";");
      vec.clear();
    }
    if (groups_str_.length() != 0) {  // only one group
      group = groups_str_.substr(1, groups_str_.length() - 2);
      sub_delimiter = group.find(",");
      while (sub_delimiter != std::string::npos) {
        int index = std::stoi(group.substr(0, sub_delimiter));
        vec.push_back(index);
        group = group.substr(sub_delimiter + 1, group.length() - sub_delimiter);
        sub_delimiter = group.find(",");
      }
      int index = std::stoi(group);
      vec.push_back(index);
      merge_groups_.push_back(vec);
      vec.clear();
    }
  }
  HOBOT_CHECK(merge_groups_.size() != 0) << "Failed to parse merge groups";
  HOBOT_CHECK(merge_groups_[0].size() != 0) << "Empty group";
  return 0;
}

void ActPostPredictor::Do(CNNMethodRunData *run_data) {
  int batch_size = run_data->input_dim_size.size();
  run_data->output.resize(batch_size);

  for (int batch_idx = 0; batch_idx < batch_size; batch_idx++) {
    int dim_size = run_data->input_dim_size[batch_idx];
    auto &mxnet_output = run_data->mxnet_output[batch_idx];
    std::vector<BaseDataPtr> &batch_output = run_data->output[batch_idx];
    batch_output.resize(output_slot_size_);
    for (int i = 0; i < output_slot_size_; i++) {
      auto base_data_vector = std::make_shared<BaseDataVector>();
      batch_output[i] = std::static_pointer_cast<BaseData>(base_data_vector);
    }
    {
      RUN_PROCESS_TIME_PROFILER(model_name_ + "_post");
      RUN_FPS_PROFILER(model_name_ + "_post");

      for (int dim_idx = 0; dim_idx < dim_size; dim_idx++) {  // loop target
        auto &target_mxnet = mxnet_output[dim_idx];
        if (target_mxnet.size() == 0) {
          setVaule(batch_output, DefaultVaule(output_slot_size_));
        } else {
          setVaule(batch_output, TargetPro(target_mxnet));
        }
      }
    }
  }
}

std::vector<BaseDataPtr> ActPostPredictor::DefaultVaule(int size) {
  std::vector<BaseDataPtr> def;
  auto def_val = std::make_shared<XStreamData<hobot::vision::Attribute<int>>>();
  def_val->value.value = -1;
  def_val->value.score = 0;
  for (int i = 0; i < size; ++i) {
    def.push_back(std::static_pointer_cast<BaseData>(def_val));
  }
  return def;
}

std::vector<BaseDataPtr> ActPostPredictor::TargetPro(
    const std::vector<std::vector<int8_t>> &mxnet_outs) {
  std::vector<BaseDataPtr> vals;
  // softmax, merge by group, return target group
  for (size_t i = 0; i < mxnet_outs.size(); ++i) {
    auto mxnet_out = reinterpret_cast<const float *>(mxnet_outs[i].data());
    uint32_t model_output_size = mxnet_outs[i].size() / 4;
    std::vector<float> model_outs;
    model_outs.resize(model_output_size);
    float max_score = mxnet_out[0], sum_score = 0;
    for (size_t idx = 0; idx < model_output_size; ++idx) {
      model_outs[idx] = mxnet_out[idx];
      if (mxnet_out[idx] > max_score) {
        max_score = mxnet_out[idx];
      }
    }
    for (auto &item : model_outs) {
      item = std::exp(item - max_score);
      sum_score += item;
    }
    std::vector<float> fall_rets(merge_groups_.size(), 0);
    for (size_t g_idx = 0; g_idx < merge_groups_.size(); ++g_idx) {
      for (size_t idx = 0; idx < merge_groups_[g_idx].size(); ++idx) {
        fall_rets[g_idx] += model_outs[merge_groups_[g_idx][idx]] / sum_score;
      }
    }
    auto fall_ret =
        std::make_shared<XStreamData<hobot::vision::Attribute<int>>>();
    fall_ret->value.score = fall_rets[target_group_];
    if (fall_rets[target_group_] >= threshold_) {
      fall_ret->value.value = 1;
    } else if (fall_rets[target_group_] > 0 &&
               fall_rets[target_group_] < threshold_) {
      fall_ret->value.value = 0;
    } else {
      fall_ret->value.value = -1;
    }
    vals.push_back(std::static_pointer_cast<BaseData>(fall_ret));
    LOGD << "idx: " << i << " fall: " << fall_ret->value.score;
  }
  return vals;
}

}  // namespace xstream
