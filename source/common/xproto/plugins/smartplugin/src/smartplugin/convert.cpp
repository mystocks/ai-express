/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-02 04:05:52
 * @Version: v0.0.1
 * @Brief: implemenation of converter.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-08-02 06:26:01
 */
#include <turbojpeg.h>
#include "hobotxsdk/xstream_data.h"
#include "horizon/vision/util.h"

#include "hobotlog/hobotlog.hpp"
#include "smartplugin/convert.h"
#include "xproto_msgtype/vioplugin_data.h"
using ImageFramePtr = std::shared_ptr<hobot::vision::ImageFrame>;
namespace horizon {
namespace iot {

using horizon::vision::util::ImageFrameConversion;
int Convertor::image_compress_quality = 50;

xstream::InputDataPtr Convertor::ConvertInput(const VioMessage *input) {
  xstream::InputDataPtr inputdata(new xstream::InputData());
  HOBOT_CHECK(input != nullptr && input->num_ > 0 && input->is_valid_uri_);

  // \todo need a better way to identify mono or semi cameras
  for (uint32_t image_index = 0; image_index < 1; ++image_index) {
    xstream::BaseDataPtr xstream_input_data;
    if (input->num_ > image_index) {
#ifdef X2
      auto xstream_img = ImageFrameConversion(input->image_[image_index]);
      xstream_input_data = xstream::BaseDataPtr(xstream_img);
      LOGI << "Input Frame ID = " << xstream_img->value->frame_id
           << ", Timestamp = " << xstream_img->value->time_stamp;
#endif

#ifdef X3
      std::shared_ptr<hobot::vision::PymImageFrame> pym_img =
          input->image_[image_index];
      LOGI << "vio message, frame_id = " << pym_img->frame_id;
      for (uint32_t i = 0; i < DOWN_SCALE_MAX; ++i) {
        LOGD << "vio message, pym_level_" << i
        << ", width=" << pym_img->down_scale[i].width
        << ", height=" << pym_img->down_scale[i].height
        << ", stride=" << pym_img->down_scale[i].stride;
      }
      auto xstream_img =
          std::make_shared<xstream::XStreamData<ImageFramePtr>>();
      xstream_img->type_ = "ImageFrame";
      xstream_img->value =
          std::static_pointer_cast<hobot::vision::ImageFrame>(pym_img);
      LOGI << "Input Frame ID = " << xstream_img->value->frame_id
           << ", Timestamp = " << xstream_img->value->time_stamp;
      xstream_input_data = xstream::BaseDataPtr(xstream_img);
#endif
    } else {
      xstream_input_data = std::make_shared<xstream::BaseData>();
      xstream_input_data->state_ = xstream::DataState::INVALID;
    }

    if (image_index == uint32_t{0}) {
      if (input->num_ == 1) {
        xstream_input_data->name_ = "image";
      } else {
        xstream_input_data->name_ = "rgb_image";
      }
    } else {
      xstream_input_data->name_ = "nir_image";
    }
    LOGI << "input name:" << xstream_input_data->name_;
    inputdata->datas_.emplace_back(xstream_input_data);
  }

  return inputdata;
}

void Convertor::ConvertVehicleInfoToProto(
    const vision::xproto::smartplugin::VehicleInfo &vehicle_info,
    vehicle::VehicleInfo *vehicle_info_pb) {
  vehicle_info_pb->set_track_id(vehicle_info.track_id);
  vehicle_info_pb->set_type(vehicle_info.type);
  vehicle_info_pb->set_color(vehicle_info.color);
  auto vehicle_points_pb = vehicle_info_pb->mutable_points();
  vehicle_points_pb->set_type("vehicle_key_points");
  for (const auto &vehicle_point : vehicle_info.points) {
    auto vehicle_point_pb = vehicle_points_pb->add_points();
    vehicle_point_pb->set_x(vehicle_point.x);
    vehicle_point_pb->set_y(vehicle_point.y);
    vehicle_point_pb->set_score(vehicle_point.score);
  }
  auto vehicle_box = vehicle_info_pb->mutable_box();
  vehicle_box->set_type("vehicle");
  vehicle_box->mutable_top_left()->set_x(vehicle_info.box.x1);
  vehicle_box->mutable_top_left()->set_y(vehicle_info.box.y1);
  vehicle_box->mutable_bottom_right()->set_x(vehicle_info.box.x2);
  vehicle_box->mutable_bottom_right()->set_y(vehicle_info.box.y2);
  vehicle_box->set_score(vehicle_info.box.score);
  auto vehicle_plate = vehicle_info_pb->mutable_plate_info();
  auto plate_box = vehicle_plate->mutable_box();
  plate_box->set_type("plate");
  plate_box->mutable_top_left()->set_x(vehicle_info.plate_info.box.x1);
  plate_box->mutable_top_left()->set_y(vehicle_info.plate_info.box.y1);
  plate_box->mutable_bottom_right()->set_x(vehicle_info.plate_info.box.x2);
  plate_box->mutable_bottom_right()->set_y(vehicle_info.plate_info.box.y2);
  vehicle_plate->set_is_double_plate(vehicle_info.plate_info.is_double_plate);
  for (const auto &plate_point : vehicle_info.points) {
    auto plate_point_pb = vehicle_plate->mutable_points()->add_points();
    plate_point_pb->set_x(plate_point.x);
    plate_point_pb->set_y(plate_point.y);
    plate_point_pb->set_score(plate_point.score);
  }
  vehicle_plate->set_plate_num(vehicle_info.plate_info.plate_num);
  vehicle_plate->set_color(vehicle_info.plate_info.color);
  vehicle_plate->set_type(vehicle_info.plate_info.type);
  auto vehicle_location = vehicle_info_pb->mutable_location();
  vehicle_location->set_x(vehicle_info.location.x);
  vehicle_location->set_y(vehicle_info.location.y);
  vehicle_location->set_score(vehicle_info.location.score);
  vehicle_info_pb->set_lane_id(vehicle_info.land_id);
  vehicle_info_pb->set_speed(vehicle_info.speed);
  if (vehicle_info.gis_info != nullptr) {
    auto gis_info_pb = vehicle_info_pb->mutable_gis_info();
    gis_info_pb->set_longitude(vehicle_info.gis_info->longitude);
    gis_info_pb->set_latitude(vehicle_info.gis_info->latitude);
    gis_info_pb->set_height(vehicle_info.gis_info->height);
    gis_info_pb->set_width(vehicle_info.gis_info->width);
    gis_info_pb->set_orientation(vehicle_info.gis_info->orientation);
  }
}

void Convertor::ConvertPersonInfoToProto(
    const vision::xproto::smartplugin::PersonInfo &person_info,
    vehicle::Person *person_info_pb) {
  person_info_pb->set_track_id(person_info.track_id);

  auto person_box = person_info_pb->mutable_box();
  person_box->set_type("person");
  person_box->mutable_top_left()->set_x(person_info.box.x1);
  person_box->mutable_top_left()->set_y(person_info.box.y1);
  person_box->mutable_bottom_right()->set_x(person_info.box.x2);
  person_box->mutable_bottom_right()->set_y(person_info.box.y2);
  person_box->set_score(person_info.box.score);
  if (person_info.gis_info != nullptr) {
    auto gis_info_pb = person_info_pb->mutable_gis_info();
    gis_info_pb->set_longitude(person_info.gis_info->longitude);
    gis_info_pb->set_latitude(person_info.gis_info->latitude);
  }
}

void Convertor::ConvertNomotorInfoToProto(
    const vision::xproto::smartplugin::NoMotorVehicleInfo &nomotor_info,
    vehicle::Nonmotor *nomotor_info_pb) {
  nomotor_info_pb->set_track_id(nomotor_info.track_id);

  auto nomotor_box = nomotor_info_pb->mutable_box();
  nomotor_box->set_type("no-motor");
  nomotor_box->mutable_top_left()->set_x(nomotor_info.box.x1);
  nomotor_box->mutable_top_left()->set_y(nomotor_info.box.y1);
  nomotor_box->mutable_bottom_right()->set_x(nomotor_info.box.x2);
  nomotor_box->mutable_bottom_right()->set_y(nomotor_info.box.y2);
  nomotor_box->set_score(nomotor_info.box.score);

  if (nomotor_info.gis_info != nullptr) {
    auto gis_info_pb = nomotor_info_pb->mutable_gis_info();
    gis_info_pb->set_longitude(nomotor_info.gis_info->longitude);
    gis_info_pb->set_latitude(nomotor_info.gis_info->latitude);
  }
}

void Convertor::ConvertVehicleCaptureInfoToProto(
    const vision::xproto::smartplugin::VehicleCapture &vehicle_capture,
    vehicle::VehicleCapture *vehicle_capture_pb) {
  vehicle_capture_pb->set_timestamp(vehicle_capture.image->time_stamp);
  ConvertVehicleInfoToProto(vehicle_capture.vehicle_info,
                            vehicle_capture_pb->mutable_vehicle());

  auto img_pb = vehicle_capture_pb->mutable_image();
  auto cv_img = std::dynamic_pointer_cast<hobot::vision::CVImageFrame>(
      vehicle_capture.image);
  img_pb->set_type("jpg");
  tjhandle handle_tj = tjInitCompress();
  unsigned char *jpeg_buf = nullptr;
  uint64_t jpeg_size = 0;
  tjCompress2(handle_tj, cv_img->img.data, cv_img->img.cols, 0,
              cv_img->img.rows, TJPF_BGR, &jpeg_buf, &jpeg_size, TJSAMP_420,
              image_compress_quality, 0);
  tjDestroy(handle_tj);
  img_pb->set_buf(jpeg_buf, jpeg_size);
  img_pb->set_width(cv_img->Width());
  img_pb->set_height(cv_img->Height());
  tjFree(jpeg_buf);
}

}  // namespace iot
}  // namespace horizon
