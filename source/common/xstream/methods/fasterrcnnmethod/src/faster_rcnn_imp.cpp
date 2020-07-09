//
// Created by yaoyao.sun on 2019-04-23.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#include <assert.h>
#include <stdint.h>

#include <algorithm>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "bpu_predict/bpu_internal.h"
#include "bpu_predict/bpu_io.h"
#include "bpu_predict/bpu_parse_utils.h"
#include "bpu_predict/bpu_predict.h"
#include "hobotlog/hobotlog.hpp"
#include "hobotxsdk/xstream_data.h"
#include "hobotxstream/profiler.h"
#include "horizon/vision_type/vision_type.hpp"
#include "json/json.h"
#include "opencv2/imgproc.hpp"
// #include "hobotxsdk/xstream_data.h"
// #include "hobotlog/hobotlog.hpp"
#include "FasterRCNNMethod/FasterRCNNMethod.h"
#include "FasterRCNNMethod/config.h"
#include "FasterRCNNMethod/faster_rcnn_imp.h"
#include "FasterRCNNMethod/result.h"
#include "FasterRCNNMethod/util.h"
#include "FasterRCNNMethod/yuv_utils.h"
#include "common/common.h"

#ifdef X3
#include "./bpu_predict_x3.h"
#endif

#define DMA_ALIGN_SIZE 64
#define BPU_CEIL_ALIGN(len) \
  ((((len) + DMA_ALIGN_SIZE - 1) / DMA_ALIGN_SIZE) * DMA_ALIGN_SIZE)

using hobot::vision::Attribute;
using hobot::vision::BBox;
using hobot::vision::CVImageFrame;
using hobot::vision::Landmarks;
using hobot::vision::Point;
using hobot::vision::Pose3D;
using hobot::vision::PymImageFrame;
using hobot::vision::Segmentation;

using xstream::BaseDataPtr;
using xstream::BaseDataVector;
using xstream::FasterRCNNParam;
using xstream::InputParamPtr;
using xstream::XStreamData;

using ImageFramePtr = std::shared_ptr<hobot::vision::ImageFrame>;

namespace faster_rcnn_method {
// used to decide output info for each layer of model. first we should use hbcc
// interface to get model's info,
// and get each layer's output info, then config it in config file.
static std::map<std::string, FasterRCNNBranchOutType> str2faster_rcnn_out_type =
    {{"bbox", FasterRCNNBranchOutType::BBOX},
     {"kps", FasterRCNNBranchOutType::KPS},
     {"mask", FasterRCNNBranchOutType::MASK},
     {"reid", FasterRCNNBranchOutType::REID},
     {"lmks2_label", FasterRCNNBranchOutType::LMKS2_LABEL},
     {"lmks2_offset", FasterRCNNBranchOutType::LMKS2_OFFSET},
     {"lmks1", FasterRCNNBranchOutType::LMKS1},
     {"3d_pose", FasterRCNNBranchOutType::POSE_3D},
     {"plate_color", FasterRCNNBranchOutType::PLATE_COLOR},
     {"plate_row", FasterRCNNBranchOutType::PLATE_ROW},
     {"kps_label", FasterRCNNBranchOutType::KPS_LABEL},
     {"kps_offset", FasterRCNNBranchOutType::KPS_OFFSET},
     {"lmks", FasterRCNNBranchOutType::LMKS}};

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
void FasterRCNNImp::ParseConfig(const std::string &config_file) {
  FR_Config cfg_jv;
  std::ifstream infile(config_file.c_str());
  infile >> cfg_jv;
  config_.reset(new Config(cfg_jv));

  auto net_info = config_->GetSubConfig("net_info");
  model_name_ = net_info->GetSTDStringValue("model_name");
  model_version_ = net_info->GetSTDStringValue("model_version", "1.0.0");
  std::vector<std::shared_ptr<Config>> model_out_sequence =
      net_info->GetSubConfigArray("model_out_sequence");

  LOGD << "rcnn branch out type:";
  for (size_t i = 0; i < model_out_sequence.size(); ++i) {
    std::string type_str = model_out_sequence[i]->GetSTDStringValue("type");
    HOBOT_CHECK(!type_str.empty());
    LOGD << type_str;
    if (str2faster_rcnn_out_type.find(type_str) ==
        str2faster_rcnn_out_type.end()) {
      out_level2rcnn_branch_info_[i].type = FasterRCNNBranchOutType::INVALID;
    } else {
      out_level2rcnn_branch_info_[i].type = str2faster_rcnn_out_type[type_str];
      out_level2rcnn_branch_info_[i].name =
          model_out_sequence[i]->GetSTDStringValue("name");
      out_level2rcnn_branch_info_[i].box_name =
          model_out_sequence[i]->GetSTDStringValue("box_name");
      out_level2rcnn_branch_info_[i].labels =
          model_out_sequence[i]->GetLabelsMap("labels");
    }
  }

  model_input_width_ = net_info->GetIntValue("model_input_width", 960);
  model_input_height_ = net_info->GetIntValue("model_input_height", 540);
  pyramid_layer_ = net_info->GetIntValue("pyramid_layer", 4);

  kps_pos_distance_ = net_info->GetFloatValue("kps_pos_distance", 0.1);
  kps_feat_width_ = net_info->GetIntValue("kps_feat_width", 16);
  kps_feat_height_ = net_info->GetIntValue("kps_feat_height", 16);
  kps_points_number_ = net_info->GetIntValue("kps_points_number", 17);
  kps_feat_stride_ = net_info->GetIntValue("kps_feat_stride", 16);
  kps_anchor_param_ = net_info->GetFloatValue("kps_anchor_param", 0.0);

  lmk_feat_height_ = net_info->GetIntValue("lmk_feat_height", 8);
  lmk_feat_width_ = net_info->GetIntValue("lmk_feat_width", 8);
  lmk_feat_stride_ = net_info->GetIntValue("lmk_feat_stride", 16);
  lmk_points_number_ = net_info->GetIntValue("lmk_points_number", 5);
  lmk_pos_distance_ = net_info->GetFloatValue("lmk_pos_distance", 12);
  lmk_anchor_param_ = net_info->GetFloatValue("lmk_anchor_param", 0.0);
  face_pose_number_ = net_info->GetIntValue("3d_pose_number", 3);

  plate_color_num_ = net_info->GetIntValue("plate_color_num", 6);
  plate_row_num_ = net_info->GetIntValue("plate_row_num", 2);

  face_pv_thr_ = net_info->GetFloatValue("face_pv_thr", 0.0);

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

void FasterRCNNImp::GetModelInfo(const std::string &model_name) {
  uint32_t output_layer_num = output_info_.num;
  for (size_t i = 0; i < output_layer_num; ++i) {
    const auto &branch_info = out_level2rcnn_branch_info_[i];
    auto out_type = branch_info.type;
    // get shifts
    const uint8_t *shift_value = output_info_.shift_value[i];
    // dim of per shape = 4
    int *aligned_dim = output_info_.aligned_shape_array + i * 4;
    int element_type = output_info_.dtype_array[i];
    switch (out_type) {
      case FasterRCNNBranchOutType::INVALID:
        break;
      case FasterRCNNBranchOutType::KPS:
        // TODO(yaoyao.sun) pack into a function
        kps_shift_ = shift_value[0];
        aligned_kps_dim = aligned_dim;
        kps_element_type = element_type;
        break;
      case FasterRCNNBranchOutType::KPS_LABEL:
        // TODO(yaoyao.sun) pack into a function
        kps_label_shift_ = shift_value[0];
        aligned_kps_label_dim = aligned_dim;
        kps_label_element_type = element_type;
        break;
      case FasterRCNNBranchOutType::KPS_OFFSET:
        // TODO(yaoyao.sun) pack into a function
        kps_offset_shift_ = shift_value[0];
        aligned_kps_offset_dim = aligned_dim;
        kps_offset_element_type = element_type;
        break;
      case FasterRCNNBranchOutType::MASK:
        mask_shift_ = shift_value[0];
        aligned_mask_dim = aligned_dim;
        mask_element_type = element_type;
        break;
      case FasterRCNNBranchOutType::REID:
        reid_shift_ = shift_value[0];
        aligned_reid_dim = aligned_dim;
        reid_element_type = element_type;
        break;
      case FasterRCNNBranchOutType::LMKS:
        lmks_shift_ = shift_value[0];
        aligned_lmks_dim = aligned_dim;
        lmk_element_type = element_type;
        break;
      case FasterRCNNBranchOutType::LMKS2_LABEL:
        lmks2_label_shift_ = shift_value[0];
        aligned_lmks2_label_dim = aligned_dim;
        lmk2_label_element_type = element_type;
        break;
      case FasterRCNNBranchOutType::LMKS2_OFFSET:
        lmks2_offset_shift_ = shift_value[0];
        aligned_lmks2_offset_dim = aligned_dim;
        lmk2_offet_element_type = element_type;
        break;
      case FasterRCNNBranchOutType::LMKS1:
        lmks1_shift_ = shift_value[0];
        aligned_lmks1_dim = aligned_dim;
        lmk1_element_type = element_type;
        break;
      case FasterRCNNBranchOutType::POSE_3D:
        face_pose_shift_ = shift_value[0];
        aligned_face_pose_dim = aligned_dim;
        face_pose_element_type = element_type;
        break;
      case FasterRCNNBranchOutType::PLATE_COLOR:
        plate_color_shift_ = shift_value[0];
        aligned_plate_color_dim = aligned_dim;
        plate_color_element_type = element_type;
        HOBOT_CHECK(plate_color_num_ <= aligned_plate_color_dim[3]);
        break;
      case FasterRCNNBranchOutType::PLATE_ROW:
        plate_row_shift_ = shift_value[0];
        aligned_plate_row_dim = aligned_dim;
        plate_row_element_type = element_type;
        HOBOT_CHECK(plate_row_num_ <= aligned_plate_row_dim[3]);
        break;
      default:
        break;
    }
  }
}

int FasterRCNNImp::Init(const std::string &config_file) {
  faster_rcnn_param_ = nullptr;
  // parse config file.
  ParseConfig(config_file);

  bpu_handle_ =
      BPUModelManager::Get().GetBpuHandle(model_file_path_, bpu_config_path_);
  int ret =
      BPU_getModelOutputInfo(bpu_handle_, model_name_.c_str(), &output_info_);
  HOBOT_CHECK(ret == 0) << "Get model " << model_name_
                        << " output info failed: "
                        << BPU_getLastError(bpu_handle_);
  LOGD << "BPU_getModelOutputInfo success.";
  GetModelInfo(model_name_);
  return 0;
}

void FasterRCNNImp::RunSingleFrame(const std::vector<BaseDataPtr> &frame_input,
                                   std::vector<BaseDataPtr> &frame_output) {
  // only one input slot -> PyramidImage or CVImage
  HOBOT_CHECK(frame_input.size() == 1);
  const auto frame_img_ = frame_input[0];

  for (size_t out_index = 0; out_index < method_outs_.size(); ++out_index) {
    frame_output.push_back(std::make_shared<xstream::BaseDataVector>());
  }

  auto xstream_img =
      std::static_pointer_cast<XStreamData<ImageFramePtr>>(frame_img_);

  std::string img_type = xstream_img->value->type;
  LOGD << "image type: " << img_type << std::endl;

  BPUModelHandle model_handle;
  int ret = 0;

  int src_img_width = 0;
  int src_img_height = 0;

  ModelOutputBuffer output_buf(output_info_, out_buf_);
  {
    RUN_PROCESS_TIME_PROFILER("FasterRCNN RunModelFromPyramid");
    RUN_FPS_PROFILER("FasterRCNN RunModelFromPyramid");

    if (img_type == kPyramidImage) {
      auto pyramid_image =
          std::static_pointer_cast<PymImageFrame>(xstream_img->value);
  #ifdef X2
      src_img_height = pyramid_image->img.src_img.height;
      src_img_width = pyramid_image->img.src_img.width;
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
      LOGD << "Begin call BPU_runModelFromPyramid";
      ret = BPU_runModelFromPyramid(bpu_handle_, model_name_.c_str(),
                                    static_cast<void *>(&(bpu_predict_pyramid)),
                                    pyramid_layer_, out_buf_.data(),
                                    out_buf_.size(), &model_handle);
  #endif

      LOGD << "image height: " << src_img_height << "width: " << src_img_width;
      if (ret != 0) {
        LOGE << "Run model failed: " << BPU_getLastError(bpu_handle_);
        BPU_releaseModelHandle(bpu_handle_, model_handle);
        return;
      }
      LOGD << "begin call BPU_getModelOutput";
      ret = BPU_getModelOutput(bpu_handle_, model_handle);
      BPU_releaseModelHandle(bpu_handle_, model_handle);
      if (ret != 0) {
        LOGE << "Get model out put failed: " << BPU_getLastError(bpu_handle_);
        return;
      }

    } else if (img_type == kCVImageFrame) {
      auto cv_image =
          std::static_pointer_cast<CVImageFrame>(xstream_img->value);

      HOBOT_CHECK(cv_image->pixel_format ==
                  HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawBGR);

      src_img_height = cv_image->Height();
      src_img_width = cv_image->Width();

      auto img_mat = cv_image->img;
      cv::Mat resized_mat = img_mat;
      if (src_img_height != model_input_height_ &&
          src_img_width != model_input_width_) {
        LOGD << "need resize.";
        cv::resize(img_mat, resized_mat,
                   cv::Size(model_input_width_, model_input_height_));
      }

      cv::Mat img_nv12;
      uint8_t *bgr_data = resized_mat.ptr<uint8_t>();
      bgr_to_nv12(bgr_data, model_input_height_, model_input_width_, img_nv12);
      uint8_t *nv12_data = img_nv12.ptr<uint8_t>();

      // construct BPUFakeImage
      BPUFakeImageHandle fake_img_handle;

      int img_len = model_input_width_ * model_input_height_ * 3 / 2;
      LOGD << "nv12 img_len: " << img_len;

      ret = BPU_createFakeImageHandle(model_input_height_, model_input_width_,
                                      &fake_img_handle);
      HOBOT_CHECK(ret == 0) << "create fake image handle failed";

      BPUFakeImage *fake_img_ptr = nullptr;
      fake_img_ptr = BPU_getFakeImage(fake_img_handle, nv12_data, img_len);
      HOBOT_CHECK(fake_img_ptr != nullptr) << "get fake image failed";
      ret = BPU_runModelFromImage(bpu_handle_, model_name_.c_str(),
                                  fake_img_ptr, out_buf_.data(),
                                  out_buf_.size(), &model_handle);

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
      BPU_releaseFakeImage(fake_img_handle, fake_img_ptr);
      BPU_releaseFakeImageHandle(fake_img_handle);
    } else {
      HOBOT_CHECK(0) << "Not support this input type: " << img_type;
    }
  }

  RUN_PROCESS_TIME_PROFILER("FasterRCNN PostProcess");
  RUN_FPS_PROFILER("FasterRCNN PostProcess");

  // Post process
  GetFrameOutput(src_img_width, src_img_height, frame_output);
}

void FasterRCNNImp::GetFrameOutput(int src_img_width, int src_img_height,
                                   std::vector<BaseDataPtr> &frame_output) {
  FasterRCNNOutMsg det_result;
  PostProcess(det_result);
  CoordinateTransform(det_result, src_img_width, src_img_height,
                      model_input_width_, model_input_height_);
  for (auto &boxes : det_result.boxes) {
    LOGD << boxes.first << ", num: " << boxes.second.size();
    for (auto &box : boxes.second) {
      LOGD << box;
    }
  }

  // TODO(yaoyao.sun) Packaged into a function, GetResultMsg()
  // convert FasterRCNNOutMsg to xstream data structure
  std::map<std::string, std::shared_ptr<BaseDataVector>> xstream_det_result;

  // get landmark by landmark2 and landmark1.
  if (det_result.landmarks.find("landmark2") != det_result.landmarks.end() &&
      det_result.landmarks.find("landmark1") != det_result.landmarks.end()) {
    std::vector<Landmarks> vec_landmarks;
    auto &landmarks2 = det_result.landmarks["landmark2"];
    auto &landmarks1 = det_result.landmarks["landmark1"];
    HOBOT_CHECK(landmarks2.size() == landmarks1.size())
        << "landmarks2's size not equal to landmarks1's size.";
    for (size_t lmk_index = 0; lmk_index < landmarks2.size(); ++lmk_index) {
      Landmarks landmarks;
      auto &landmark2 = landmarks2[lmk_index];
      auto &landmark1 = landmarks1[lmk_index];
      HOBOT_CHECK(landmark2.values.size() == landmark1.values.size())
          << "landmark2's size not equal to landmark1's size.";
      for (size_t point_index = 0; point_index < landmark2.values.size();
           ++point_index) {
        auto &landmark2_point = landmark2.values[point_index];
        auto &landmark1_point = landmark1.values[point_index];
        if (landmark2_point.score > 0.5) {
          landmarks.values.push_back(std::move(landmark2_point));
        } else {
          landmarks.values.push_back(std::move(landmark1_point));
        }
      }
      vec_landmarks.push_back(landmarks);
    }
    det_result.landmarks["landmark"] = vec_landmarks;
    det_result.landmarks.erase("landmark2");
    det_result.landmarks.erase("landmark1");
  }

  std::set<int> invalid_idx;
  // box
  for (const auto &boxes : det_result.boxes) {
    xstream_det_result[boxes.first] =
        std::make_shared<xstream::BaseDataVector>();
    xstream_det_result[boxes.first]->name_ = "rcnn_" + boxes.first;
    for (uint i = 0; i < boxes.second.size(); ++i) {
      const auto &box = boxes.second[i];
      if (boxes.first == "face_box") {
        if (box.score < face_pv_thr_) {
          invalid_idx.emplace(i);
          continue;
        }
      }
      auto xstream_box = std::make_shared<XStreamData<BBox>>();
      xstream_box->value = std::move(box);
      xstream_det_result[boxes.first]->datas_.push_back(xstream_box);
    }
  }

  // landmark
  for (const auto &landmarks_vec : det_result.landmarks) {
    xstream_det_result[landmarks_vec.first] =
        std::make_shared<xstream::BaseDataVector>();
    xstream_det_result[landmarks_vec.first]->name_ =
        "rcnn_" + landmarks_vec.first;
    for (uint i = 0; i < landmarks_vec.second.size(); ++i) {
      if (invalid_idx.find(i) != invalid_idx.end()) {
        continue;
      }
      const auto &landmarks = landmarks_vec.second[i];
      LOGD << "lmk point: [";
      for (const auto &point : landmarks.values) {
        LOGD << point.x << "," << point.y << "," << point.score;
      }
      LOGD << "]";
      auto xstream_landmark = std::make_shared<XStreamData<Landmarks>>();
      xstream_landmark->value = std::move(landmarks);
      xstream_det_result[landmarks_vec.first]->datas_.push_back(
          xstream_landmark);
    }
  }

  // feature
  for (const auto &feature_vec : det_result.features) {
    xstream_det_result[feature_vec.first] =
        std::make_shared<xstream::BaseDataVector>();
    xstream_det_result[feature_vec.first]->name_ = "rcnn_" + feature_vec.first;
    for (auto &feature : feature_vec.second) {
      auto xstream_feature = std::make_shared<XStreamData<Feature>>();
      xstream_feature->value = std::move(feature);
      xstream_det_result[feature_vec.first]->datas_.push_back(xstream_feature);
    }
  }

  // segmentations
  for (const auto &segmentation_vec : det_result.segmentations) {
    xstream_det_result[segmentation_vec.first] =
        std::make_shared<xstream::BaseDataVector>();
    xstream_det_result[segmentation_vec.first]->name_ =
        "rcnn_" + segmentation_vec.first;
    for (auto &segmentation : segmentation_vec.second) {
      auto xstream_segmentation = std::make_shared<XStreamData<Segmentation>>();
      xstream_segmentation->value = std::move(segmentation);
      xstream_det_result[segmentation_vec.first]->datas_.push_back(
          xstream_segmentation);
    }
  }

  // poses
  for (const auto &pose_vec : det_result.poses) {
    xstream_det_result[pose_vec.first] =
        std::make_shared<xstream::BaseDataVector>();
    xstream_det_result[pose_vec.first]->name_ = "rcnn_" + pose_vec.first;
    for (uint i = 0; i < pose_vec.second.size(); ++i) {
      if (invalid_idx.find(i) != invalid_idx.end()) {
        continue;
      }
      const auto &pose = pose_vec.second[i];
      auto xstream_pose = std::make_shared<XStreamData<Pose3D>>();
      xstream_pose->value = std::move(pose);
      xstream_det_result[pose_vec.first]->datas_.push_back(xstream_pose);
    }
  }

  // attributes
  for (const auto &attribute_vec : det_result.attributes) {
    xstream_det_result[attribute_vec.first] =
        std::make_shared<xstream::BaseDataVector>();
    xstream_det_result[attribute_vec.first]->name_ =
        "rcnn_" + attribute_vec.first;
    for (auto &attribute : attribute_vec.second) {
      auto xstream_attribute = std::make_shared<XStreamData<Attribute<int>>>();
      xstream_attribute->value = std::move(attribute);
      xstream_det_result[attribute_vec.first]->datas_.push_back(
          xstream_attribute);
    }
  }

  for (size_t out_index = 0; out_index < method_outs_.size(); ++out_index) {
    if (xstream_det_result[method_outs_[out_index]]) {
      frame_output[out_index] = xstream_det_result[method_outs_[out_index]];
    }
  }
}

void FasterRCNNImp::PostProcess(FasterRCNNOutMsg &det_result) {
  BPU_Buffer_Handle lmk2_label_out_put = nullptr;
  BPU_Buffer_Handle lmk2_offset_out_put = nullptr;
  BPU_Buffer_Handle kps_label_out_put = nullptr;
  BPU_Buffer_Handle kps_offset_out_put = nullptr;

  for (size_t out_level = 0; out_level < out_buf_.size(); ++out_level) {
    const auto &branch_info = out_level2rcnn_branch_info_[out_level];
    auto out_type = branch_info.type;
    switch (out_type) {
      case FasterRCNNBranchOutType::INVALID:
        break;
      case FasterRCNNBranchOutType::BBOX:
        GetRppRects(det_result.boxes, out_level,
                    branch_info.name, branch_info.labels);
        break;
      case FasterRCNNBranchOutType::LMKS2_LABEL:
        lmk2_label_out_put = out_buf_[out_level];
        break;
      case FasterRCNNBranchOutType::LMKS2_OFFSET:
        lmk2_offset_out_put = out_buf_[out_level];
        break;
      case FasterRCNNBranchOutType::KPS_LABEL:
        kps_label_out_put = out_buf_[out_level];
        break;
      case FasterRCNNBranchOutType::KPS_OFFSET:
        kps_offset_out_put = out_buf_[out_level];
        break;
      default:
        break;
    }
  }

  for (size_t out_level = 0; out_level < out_buf_.size(); ++out_level) {
    const auto &branch_info = out_level2rcnn_branch_info_[out_level];
    auto out_type = branch_info.type;
    switch (out_type) {
      case FasterRCNNBranchOutType::INVALID:
        break;
      case FasterRCNNBranchOutType::KPS:
        LOGD << "begin GetKps";
        GetKps(det_result.landmarks[branch_info.name], out_buf_[out_level],
               det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::KPS_LABEL:
        LOGD << "begin GetKps2";
        GetKps2(det_result.landmarks["kps"], kps_label_out_put,
                kps_offset_out_put, det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::MASK:
        LOGD << "begin GetMask";
        GetMask(det_result.segmentations[branch_info.name], out_buf_[out_level],
                det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::REID:
        LOGD << "begin GetReid";
        GetReid(det_result.features[branch_info.name], out_buf_[out_level],
                det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::LMKS:
        LOGD << "begin GetLMKS";
        GetLMKS(det_result.landmarks["landmark"], out_buf_[out_level],
                det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::LMKS2_LABEL:
        LOGD << "begin GetLMKS2";
        GetLMKS2(det_result.landmarks["landmark2"], lmk2_label_out_put,
                 lmk2_offset_out_put, det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::LMKS1:
        LOGD << "begin GetLMKS1";
        GetLMKS1(det_result.landmarks["landmark1"], out_buf_[out_level],
                 det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::POSE_3D:
        LOGD << "begin GetPose";
        GetPose(det_result.poses[branch_info.name], out_buf_[out_level],
                det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::PLATE_COLOR:
        LOGD << "begin GetPlateColor";
        GetPlateColor(&(det_result.attributes[branch_info.name]),
                      out_buf_[out_level],
                      det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::PLATE_ROW:
        LOGD << "begin GetPlateRow";
        GetPlateRow(&(det_result.attributes[branch_info.name]),
                    out_buf_[out_level],
                    det_result.boxes[branch_info.box_name]);
        break;
      default:
        break;
    }
  }
}

void FasterRCNNImp::GetRects
    (std::map<std::string, std::vector<BBox>> &boxes,
     BPU_Buffer_Handle output, const std::string &name,
     const std::unordered_map<int, std::string> &labels) {
  std::vector<float> rects;
  void *feature_map = BPU_getRawBufferPtr(output);
  // TODO(yaoyao.sun) use feature_info's element_size to replace unit8_t
  uint8_t *box_infos = reinterpret_cast<uint8_t *>(feature_map);
  uint32_t int_out_sz = *(reinterpret_cast<uint32_t *>(box_infos));
  LOGD << "int_out_sz: " << int_out_sz;
  box_infos += BPU_CEIL_ALIGN(int_out_sz + 16);
  uint32_t fp_out_sz = *(reinterpret_cast<uint32_t *>(box_infos));
  LOGD << "fp_out_sz: " << fp_out_sz;
  box_infos += 16;
  int box_num = fp_out_sz / (sizeof(float) * 6);

  rects.resize(box_num * 6);
  memcpy(rects.data(), reinterpret_cast<float *>(box_infos),
         rects.size() * sizeof(float));

  // add boxes
  for (size_t j = 0; j < rects.size(); j += 6) {
    hobot::vision::BBox box;
    box.x1 = rects[j];
    box.y1 = rects[j + 1];
    box.x2 = rects[j + 2];
    box.y2 = rects[j + 3];
    box.score = rects[j + 4];
    if (j + 5 < rects.size()) {
      int type = rects[j + 5];
      if (labels.find(type) != labels.end()) {
        box.category_name = labels.at(type);
      } else {
        box.category_name = name;
      }
    }
    boxes[box.category_name].push_back(std::move(box));
  }
}

void FasterRCNNImp::GetRppRects
    (std::map<std::string, std::vector<BBox>> &boxes,
    int index, const std::string &name,
     const std::unordered_map<int, std::string> &labels) {
  BPURppBBox rpp_bbox;
  int ret = BPU_parseRPPResult(
      bpu_handle_, model_name_.c_str(), out_buf_.data(), index, &rpp_bbox);
  if (ret != 0) {
    LOGE << "here parse model output failed.";
    return;
  }
  int nBox = rpp_bbox.bbox_num;
  HOBOT_CHECK(rpp_bbox.result_type == BPURppBBox::bbox_type_f32);

  auto box_ptr = rpp_bbox.bbox_ptr_f32;
  LOGD << "rpp box num: " << nBox;

  for (int i = 0; i < nBox; ++i) {
    hobot::vision::BBox box;
    box.x1 = box_ptr[i].left;
    box.y1 = box_ptr[i].top;
    box.x2 = box_ptr[i].right;
    box.y2 = box_ptr[i].bottom;
    box.score = box_ptr[i].score;
    int type = box_ptr[i].class_label;
    if (labels.find(type) != labels.end()) {
      box.category_name = labels.at(type);
    } else {
      box.category_name = name;
    }
    boxes[box.category_name].push_back(std::move(box));
  }
}

void FasterRCNNImp::GetKps(std::vector<Landmarks> &kpss,
                           BPU_Buffer_Handle output,
                           const std::vector<BBox> &body_boxes) {
  int32_t *kps_feature =
      reinterpret_cast<int32_t *>(BPU_getRawBufferPtr(output));

  float pos_distance = kps_pos_distance_ * kps_feat_width_;
  size_t body_box_num = body_boxes.size();
  int feature_size = aligned_kps_dim[1] *
                     aligned_kps_dim[2] *
                     aligned_kps_dim[3];
  int h_stride = aligned_kps_dim[2] * aligned_kps_dim[3];
  int w_stride = aligned_kps_dim[3];

  for (size_t box_id = 0; box_id < body_box_num; ++box_id) {
    const auto &body_box = body_boxes[box_id];
    float x1 = body_box.x1;
    float y1 = body_box.y1;
    float x2 = body_box.x2;
    float y2 = body_box.y2;
    float w = x2 - x1 + 1;
    float h = y2 - y1 + 1;

    float scale_x = kps_feat_width_ / w;
    float scale_y = kps_feat_height_ / h;

    Landmarks skeleton;
    skeleton.values.resize(kps_points_number_);
    HOBOT_CHECK(kps_element_type == BPU_DTYPE_FLOAT32);

    auto *mxnet_out_for_one_point_begin = kps_feature + feature_size * box_id;
    for (int kps_id = 0; kps_id < kps_points_number_; ++kps_id) {
      // find the best position
      int max_w = 0;
      int max_h = 0;
      int max_score_before_shift = mxnet_out_for_one_point_begin[kps_id];
      int32_t *mxnet_out_for_one_point = nullptr;
      for (int hh = 0; hh < kps_feat_height_; ++hh) {
        for (int ww = 0; ww < kps_feat_width_; ++ww) {
          mxnet_out_for_one_point =
              mxnet_out_for_one_point_begin + hh * h_stride + ww * w_stride;
          if (mxnet_out_for_one_point[kps_id] > max_score_before_shift) {
            max_w = ww;
            max_h = hh;
            max_score_before_shift = mxnet_out_for_one_point[kps_id];
          }
        }
      }

      float max_score =
          GetFloatByInt(max_score_before_shift, kps_shift_);

      // get delta
      mxnet_out_for_one_point =
          mxnet_out_for_one_point_begin + max_h * h_stride + max_w * w_stride;
      const auto x_delta =
          mxnet_out_for_one_point[2 * kps_id + kps_points_number_];
      float fp_delta_x = GetFloatByInt(x_delta, kps_shift_) * pos_distance;

      const auto y_delta =
          mxnet_out_for_one_point[2 * kps_id + kps_points_number_ + 1];
      float fp_delta_y = GetFloatByInt(y_delta, kps_shift_) * pos_distance;

      Point point;
      point.x =
          (max_w + fp_delta_x + 0.46875 + kps_anchor_param_) / scale_x + x1;
      point.y =
          (max_h + fp_delta_y + 0.46875 + kps_anchor_param_) / scale_y + y1;
      point.score = SigMoid(max_score);
      skeleton.values[kps_id] = point;
    }
    kpss.push_back(std::move(skeleton));
  }
}

void FasterRCNNImp::GetKps2(
    std::vector<hobot::vision::Landmarks> &kpss,
    BPU_Buffer_Handle kps_label_output, BPU_Buffer_Handle kps_offset_output,
    const std::vector<hobot::vision::BBox> &body_boxes) {
  int32_t *label_feature =
      reinterpret_cast<int32_t *>(BPU_getRawBufferPtr(kps_label_output));
  int32_t *offset_feature =
      reinterpret_cast<int32_t *>(BPU_getRawBufferPtr(kps_offset_output));

  int input_height = kps_feat_height_ * kps_feat_stride_;
  int input_width = kps_feat_width_ * kps_feat_stride_;
  float base_center = (kps_feat_stride_ - 1) / 2.0;

  int label_feature_size = aligned_kps_label_dim[1] * aligned_kps_label_dim[2] *
                           aligned_kps_label_dim[3];
  int label_h_stride = aligned_kps_label_dim[2] * aligned_kps_label_dim[3];
  int label_w_stride = aligned_kps_label_dim[3];

  int offset_feature_size = aligned_kps_offset_dim[1] *
                            aligned_kps_offset_dim[2] *
                            aligned_kps_offset_dim[3];
  int offset_h_stride = aligned_kps_offset_dim[2] * aligned_kps_offset_dim[3];
  int offset_w_stride = aligned_kps_offset_dim[3];

  size_t body_box_num = body_boxes.size();

  HOBOT_CHECK(kps_label_element_type == BPU_DTYPE_FLOAT32);
  HOBOT_CHECK(kps_offset_element_type == BPU_DTYPE_FLOAT32);

  for (size_t box_id = 0; box_id < body_box_num; ++box_id) {
    const auto &body_box = body_boxes[box_id];
    float x1 = body_box.x1;
    float y1 = body_box.y1;
    float x2 = body_box.x2;
    float y2 = body_box.y2;
    float w = x2 - x1 + 1;
    float h = y2 - y1 + 1;

    float scale_x = w / input_width;
    float scale_y = h / input_height;
    float scale_pos_x = kps_pos_distance_ * scale_x;
    float scale_pos_y = kps_pos_distance_ * scale_y;

    Landmarks skeleton;
    skeleton.values.resize(kps_points_number_);
    auto *label_feature_begin = label_feature + label_feature_size * box_id;

    for (int kps_id = 0; kps_id < kps_points_number_; ++kps_id) {
      // find the best position
      int max_w = 0;
      int max_h = 0;
      int max_score_before_shift = label_feature_begin[kps_id];
      int32_t *mxnet_out_for_one_point = nullptr;
      for (int hh = 0; hh < kps_feat_height_; ++hh) {
        for (int ww = 0; ww < kps_feat_width_; ++ww) {
          mxnet_out_for_one_point =
              label_feature_begin + hh * label_h_stride + ww * label_w_stride;
          if (mxnet_out_for_one_point[kps_id] > max_score_before_shift) {
            max_w = ww;
            max_h = hh;
            max_score_before_shift = mxnet_out_for_one_point[kps_id];
          }
        }
      }
      float max_score = GetFloatByInt(max_score_before_shift, kps_label_shift_);

      float base_x = (max_w * kps_feat_stride_ + base_center) * scale_x + x1;
      float base_y = (max_h * kps_feat_stride_ + base_center) * scale_y + y1;

      // get delta
      int32_t *offset_feature_begin =
          offset_feature + offset_feature_size * box_id;
      mxnet_out_for_one_point = offset_feature_begin + max_h * offset_h_stride +
                                max_w * offset_w_stride;

      const auto x_delta = mxnet_out_for_one_point[2 * kps_id];
      float fp_delta_x = GetFloatByInt(x_delta, kps_offset_shift_);

      const auto y_delta = mxnet_out_for_one_point[2 * kps_id + 1];
      float fp_delta_y = GetFloatByInt(y_delta, kps_offset_shift_);

      Point point;
      point.x = base_x + fp_delta_x * scale_pos_x;
      point.y = base_y + fp_delta_y * scale_pos_y;
      point.score = SigMoid(max_score);
      skeleton.values[kps_id] = point;
    }
    kpss.push_back(std::move(skeleton));
  }
}

void FasterRCNNImp::GetMask(std::vector<Segmentation> &masks,
                            BPU_Buffer_Handle output,
                            const std::vector<BBox> &body_boxes) {
  size_t body_box_num = body_boxes.size();
  int32_t *mask_feature =
      reinterpret_cast<int32_t *>(BPU_getRawBufferPtr(output));

  int feature_size =
      aligned_mask_dim[1] * aligned_mask_dim[2] * aligned_mask_dim[3];

  int h_stride = aligned_mask_dim[2] * aligned_mask_dim[3];
  int w_stride = aligned_mask_dim[3];

  HOBOT_CHECK(mask_element_type == BPU_DTYPE_FLOAT32);

  for (size_t box_id = 0; box_id < body_box_num; ++box_id) {
    int32_t *mask_feature_begin = mask_feature + feature_size * box_id;
    int32_t *mxnet_out_for_one_box = nullptr;

    float fp_for_this_mask;
    Segmentation mask;
    for (int hh = 0; hh < aligned_mask_dim[1]; ++hh) {
      for (int ww = 0; ww < aligned_mask_dim[2]; ++ww) {
        mxnet_out_for_one_box =
            mask_feature_begin + hh * h_stride + ww * w_stride;
        fp_for_this_mask = GetFloatByInt(mxnet_out_for_one_box[0], mask_shift_);
        mask.values.push_back(fp_for_this_mask);
      }
    }
    mask.height = aligned_mask_dim[1];
    mask.width = aligned_mask_dim[2];
    masks.push_back(std::move(mask));
  }
}

void FasterRCNNImp::GetReid(std::vector<Feature> &reids,
                            BPU_Buffer_Handle output,
                            const std::vector<BBox> &body_boxes) {
  size_t body_box_num = body_boxes.size();
  int32_t *reid_feature =
      reinterpret_cast<int32_t *>(BPU_getRawBufferPtr(output));

  int feature_size =
      aligned_reid_dim[1] * aligned_reid_dim[2] * aligned_reid_dim[3];
  HOBOT_CHECK(reid_element_type == BPU_DTYPE_FLOAT32);

  for (size_t box_id = 0; box_id < body_box_num; ++box_id) {
    int32_t *mxnet_out_for_one_box = reid_feature + feature_size * box_id;
    float fp_for_this_reid;
    Feature reid;
    for (int32_t reid_i = 0; reid_i < aligned_reid_dim[3]; ++reid_i) {
      fp_for_this_reid =
          GetFloatByInt(mxnet_out_for_one_box[reid_i], reid_shift_);
      reid.values.push_back(fp_for_this_reid);
    }
    // l2norm
    l2_norm(reid);
    reids.push_back(std::move(reid));
  }
}

void FasterRCNNImp::GetLMKS(
    std::vector<hobot::vision::Landmarks> &landmarks, BPU_Buffer_Handle output,
    const std::vector<hobot::vision::BBox> &face_boxes) {
  int32_t *lmk_feature =
      reinterpret_cast<int32_t *>(BPU_getRawBufferPtr(output));

  float pos_distance = lmk_pos_distance_ * lmk_feat_width_;
  size_t face_box_num = face_boxes.size();
  int feature_size =
      aligned_lmks_dim[1] * aligned_lmks_dim[2] * aligned_lmks_dim[3];
  int h_stride = aligned_lmks_dim[2] * aligned_lmks_dim[3];
  int w_stride = aligned_lmks_dim[3];

  HOBOT_CHECK(lmk_element_type == BPU_DTYPE_FLOAT32);

  for (size_t box_id = 0; box_id < face_box_num; ++box_id) {
    const auto &face_box = face_boxes[box_id];
    float x1 = face_box.x1;
    float y1 = face_box.y1;
    float x2 = face_box.x2;
    float y2 = face_box.y2;
    float w = x2 - x1 + 1;
    float h = y2 - y1 + 1;

    float scale_x = lmk_feat_width_ / w;
    float scale_y = lmk_feat_height_ / h;

    Landmarks landmark;
    landmark.values.resize(lmk_points_number_);

    for (int lmk_id = 0; lmk_id < lmk_points_number_; ++lmk_id) {
      int32_t *lmk_feature_begin = lmk_feature + feature_size * box_id;
      // find the best position
      int max_w = 0;
      int max_h = 0;
      int max_score_before_shift = lmk_feature_begin[lmk_id];
      int32_t *mxnet_out_for_one_point = nullptr;
      for (int hh = 0; hh < lmk_feat_height_; ++hh) {
        for (int ww = 0; ww < lmk_feat_width_; ++ww) {
          mxnet_out_for_one_point =
              lmk_feature_begin + hh * h_stride + ww * w_stride;
          if (mxnet_out_for_one_point[lmk_id] > max_score_before_shift) {
            max_w = ww;
            max_h = hh;
            max_score_before_shift = mxnet_out_for_one_point[lmk_id];
          }
        }
      }
      float max_score = GetFloatByInt(max_score_before_shift, lmks_shift_);

      // get delta
      mxnet_out_for_one_point =
          lmk_feature_begin + max_h * h_stride + max_w * w_stride;
      const auto x_delta =
          mxnet_out_for_one_point[2 * lmk_id + lmk_points_number_];
      float fp_delta_x = GetFloatByInt(x_delta, kps_shift_) * pos_distance;
      const auto y_delta =
          mxnet_out_for_one_point[2 * lmk_id + lmk_points_number_ + 1];
      float fp_delta_y = GetFloatByInt(y_delta, kps_shift_) * pos_distance;

      Point point;
      point.x =
          (max_w + fp_delta_x + 0.46875 + lmk_anchor_param_) / scale_x + x1;
      point.y =
          (max_h + fp_delta_y + 0.46875 + lmk_anchor_param_) / scale_y + y1;
      point.score = max_score;
      landmark.values[lmk_id] = point;
    }
    landmarks.push_back(std::move(landmark));
  }
}

void FasterRCNNImp::GetLMKS2(std::vector<Landmarks> &landmarks,
                             BPU_Buffer_Handle lmks2_label_output,
                             BPU_Buffer_Handle lmks2_offset_output,
                             const std::vector<BBox> &face_boxes) {
  int32_t *label_feature =
      reinterpret_cast<int32_t *>(BPU_getRawBufferPtr(lmks2_label_output));
  int32_t *offset_feature =
      reinterpret_cast<int32_t *>(BPU_getRawBufferPtr(lmks2_offset_output));

  HOBOT_CHECK(lmk2_label_element_type == BPU_DTYPE_FLOAT32);
  HOBOT_CHECK(lmk2_offet_element_type == BPU_DTYPE_FLOAT32);

  int input_height = lmk_feat_height_ * lmk_feat_stride_;
  int input_width = lmk_feat_width_ * lmk_feat_stride_;
  float base_center = (lmk_feat_stride_ - 1) / 2.0;
  size_t face_box_num = face_boxes.size();

  int label_feature_size = aligned_lmks2_label_dim[1] *
                           aligned_lmks2_label_dim[2] *
                           aligned_lmks2_label_dim[3];
  int label_h_stride = aligned_lmks2_label_dim[2] * aligned_lmks2_label_dim[3];
  int label_w_stride = aligned_lmks2_label_dim[3];
  int offset_feature_size = aligned_lmks2_offset_dim[1] *
                            aligned_lmks2_offset_dim[2] *
                            aligned_lmks2_offset_dim[3];
  int offset_h_stride =
      aligned_lmks2_offset_dim[2] * aligned_lmks2_offset_dim[3];
  int offset_w_stride = aligned_lmks2_offset_dim[3];

  for (size_t box_id = 0; box_id < face_box_num; ++box_id) {
    const auto &face_box = face_boxes[box_id];
    float x1 = face_box.x1;
    float y1 = face_box.y1;
    float x2 = face_box.x2;
    float y2 = face_box.y2;
    float w = x2 - x1 + 1;
    float h = y2 - y1 + 1;

    assert(input_width != 0 && input_height != 0);
    float scale_x = w / input_width;
    float scale_y = h / input_height;
    float scale_pos_x = lmk_pos_distance_ * scale_x;
    float scale_pos_y = lmk_pos_distance_ * scale_y;

    Landmarks landmark;
    landmark.values.resize(lmk_points_number_);
    int32_t *label_feature_begin = label_feature + label_feature_size * box_id;
    int32_t *offset_feature_begin =
        offset_feature + offset_feature_size * box_id;
    for (int kps_id = 0; kps_id < lmk_points_number_; ++kps_id) {
      // find the best position
      int max_w = 0;
      int max_h = 0;
      int max_score_before_shift = label_feature_begin[kps_id];
      int32_t *mxnet_out_for_one_point = nullptr;
      for (int hh = 0; hh < lmk_feat_height_; ++hh) {
        for (int ww = 0; ww < lmk_feat_width_; ++ww) {
          mxnet_out_for_one_point =
              label_feature_begin + hh * label_h_stride + ww * label_w_stride;
          if (mxnet_out_for_one_point[kps_id] > max_score_before_shift) {
            max_w = ww;
            max_h = hh;
            max_score_before_shift = mxnet_out_for_one_point[kps_id];
          }
        }
      }
      float max_score = GetFloatByInt(max_score_before_shift,
                                      lmks2_label_shift_);
      float base_x = (max_w * lmk_feat_stride_ + base_center) * scale_x + x1;
      float base_y = (max_h * lmk_feat_stride_ + base_center) * scale_y + y1;

      // get delta
      mxnet_out_for_one_point = offset_feature_begin + max_h * offset_h_stride +
                                max_w * offset_w_stride;
      auto x_delta = mxnet_out_for_one_point[2 * kps_id];
      float fp_delta_x = GetFloatByInt(x_delta, lmks2_offset_shift_);
      auto y_delta = mxnet_out_for_one_point[2 * kps_id + 1];
      float fp_delta_y = GetFloatByInt(y_delta, lmks2_offset_shift_);
      Point point;
      point.x = base_x + fp_delta_x * scale_pos_x;
      point.y = base_y + fp_delta_y * scale_pos_y;
      point.score = SigMoid(max_score);
      landmark.values[kps_id] = point;
    }
    landmarks.push_back(std::move(landmark));
  }
}

void FasterRCNNImp::GetLMKS1(std::vector<Landmarks> &landmarks,
                             BPU_Buffer_Handle output,
                             const std::vector<BBox> &face_boxes) {
  size_t face_box_num = face_boxes.size();
  int32_t *lmks1_feature =
      reinterpret_cast<int32_t *>(BPU_getRawBufferPtr(output));

  int feature_size =
      aligned_lmks1_dim[1] * aligned_lmks1_dim[2] * aligned_lmks1_dim[3];

  HOBOT_CHECK(lmk2_offet_element_type == BPU_DTYPE_FLOAT32);

  for (size_t box_id = 0; box_id < face_box_num; ++box_id) {
    const auto &face_box = face_boxes[box_id];
    float x1 = face_box.x1;
    float y1 = face_box.y1;
    float x2 = face_box.x2;
    float y2 = face_box.y2;
    float w = x2 - x1 + 1;
    float h = y2 - y1 + 1;

    int32_t *mxnet_out_for_this_box = lmks1_feature + feature_size * box_id;

    hobot::vision::Landmarks landmark;
    for (int i = 0; i < lmk_points_number_; ++i) {
      float x =
          GetFloatByInt(mxnet_out_for_this_box[2 * i], lmks1_shift_) * w + x1;
      float y =
          GetFloatByInt(mxnet_out_for_this_box[2 * i + 1], lmks1_shift_) * h +
          y1;
      landmark.values.push_back(Point(x, y));
    }
    landmarks.push_back(std::move(landmark));
  }
}

void FasterRCNNImp::GetPose(std::vector<Pose3D> &face_pose,
                            BPU_Buffer_Handle output,
                            const std::vector<BBox> &face_boxes) {
  size_t face_box_num = face_boxes.size();
  int32_t *pose_feature =
      reinterpret_cast<int32_t *>(BPU_getRawBufferPtr(output));

  HOBOT_CHECK(face_pose_element_type == BPU_DTYPE_FLOAT32);

  int feature_size = aligned_face_pose_dim[1] * aligned_face_pose_dim[2] *
                     aligned_face_pose_dim[3];

  for (size_t box_id = 0; box_id < face_box_num; ++box_id) {
    int32_t *mxnet_out_for_one_box = pose_feature + feature_size * box_id;
    hobot::vision::Pose3D pose;
    pose.yaw = GetFloatByInt(mxnet_out_for_one_box[0], face_pose_shift_) * 90.0;
    pose.pitch =
        GetFloatByInt(mxnet_out_for_one_box[1], face_pose_shift_) * 90.0;
    pose.roll =
        GetFloatByInt(mxnet_out_for_one_box[2], face_pose_shift_) * 90.0;
    face_pose.push_back(pose);
  }
}

void FasterRCNNImp::GetPlateColor(
    std::vector<hobot::vision::Attribute<int>> *plates_color,
    BPU_Buffer_Handle output,
    const std::vector<hobot::vision::BBox> &plate_boxes) {
  size_t plate_box_num = plate_boxes.size();
  int32_t *plate_color_feature =
      reinterpret_cast<int32_t *>(BPU_getRawBufferPtr(output));

  HOBOT_CHECK(plate_color_element_type == BPU_DTYPE_FLOAT32);

  int feature_size = aligned_plate_color_dim[1] * aligned_plate_color_dim[2] *
                     aligned_plate_color_dim[3];

  LOGD << "plate color: ";
  for (size_t box_id = 0; box_id < plate_box_num; ++box_id) {
    int32_t *mxnet_out_for_one_box =
        plate_color_feature + feature_size * box_id;

    hobot::vision::Attribute<int> one_plate_color;
    int max_index = 0;
    float max_score = -1000;
    for (int32_t color_index = 0; color_index < plate_color_num_;
         ++color_index) {
      float color_score =
          GetFloatByInt(mxnet_out_for_one_box[color_index], plate_color_shift_);
      if (color_score > max_score) {
        max_score = color_score;
        max_index = color_index;
      }
    }
    one_plate_color.value = max_index;
    one_plate_color.score = max_score;
    LOGD << "value: " << one_plate_color.value
         << ", score: " << one_plate_color.score << "\n";
    plates_color->push_back(one_plate_color);
  }
}

void FasterRCNNImp::GetPlateRow(
    std::vector<hobot::vision::Attribute<int>> *plates_row,
    BPU_Buffer_Handle output,
    const std::vector<hobot::vision::BBox> &plate_boxes) {
  size_t plate_box_num = plate_boxes.size();
  int32_t *plate_row_feature =
      reinterpret_cast<int32_t *>(BPU_getRawBufferPtr(output));

  int feature_size = aligned_plate_row_dim[1] * aligned_plate_row_dim[2] *
                     aligned_plate_row_dim[3];

  HOBOT_CHECK(plate_row_element_type == BPU_DTYPE_FLOAT32);

  LOGD << "plate row: ";
  for (size_t box_id = 0; box_id < plate_box_num; ++box_id) {
    int32_t *mxnet_out_for_one_box = plate_row_feature + feature_size * box_id;
    hobot::vision::Attribute<int> one_plate_row;

    int max_index = 0;
    float max_score = -1000;
    for (int32_t row_index = 0; row_index < plate_row_num_; ++row_index) {
      float row_score =
          GetFloatByInt(mxnet_out_for_one_box[row_index], plate_row_shift_);
      if (row_score > max_score) {
        max_score = row_score;
        max_index = row_index;
      }
    }
    one_plate_row.value = max_index;
    one_plate_row.score = max_score;
    LOGD << "value: " << one_plate_row.value
         << ", score: " << one_plate_row.score << "\n";
    plates_row->push_back(one_plate_row);
  }
}

void FasterRCNNImp::Finalize() {
  if (bpu_handle_) {
    BPUModelManager::Get().ReleaseBpuHandle(model_file_path_);
    LOGD << "release " << model_file_path_ << "\n";
  }
}

}  // namespace faster_rcnn_method
