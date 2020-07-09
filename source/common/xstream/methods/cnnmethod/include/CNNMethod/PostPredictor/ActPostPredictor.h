/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: ActPostPredictor.h
 * @Brief: declaration of the ActPostPredictor
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: 2020-05-25
 */

#ifndef CNNMETHOD_POSTPREDICTOR_ACTPOSTPREDICTOR_H_
#define CNNMETHOD_POSTPREDICTOR_ACTPOSTPREDICTOR_H_

#include <memory>
#include <vector>
#include <string>
#include "CNNMethod/PostPredictor/PostPredictor.h"

namespace xstream {
class ActPostPredictor : public PostPredictor {
 public:
  virtual int32_t Init(std::shared_ptr<CNNMethodConfig> config);
  virtual void Do(CNNMethodRunData *run_data);

 private:
  std::vector<BaseDataPtr> TargetPro(
      const std::vector<std::vector<int8_t>> &mxnet_outs);

  std::vector<BaseDataPtr> DefaultVaule(int size);
  std::string groups_str_;
  int target_group_;
  std::vector<std::vector<int>> merge_groups_;
  float threshold_;
};
}  // namespace xstream

#endif  // CNNMETHOD_POSTPREDICTOR_ACTPOSTPREDICTOR_H_
