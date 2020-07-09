/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-02 03:42:16
 * @Version: v0.0.1
 * @Brief:  convert to xstream inputdata from input VioMessage
 * @Note:  extracted from xperson repo.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-09-29 02:57:24
 */
#ifndef INCLUDE_SMARTPLUGIN_CONVERT_H_
#define INCLUDE_SMARTPLUGIN_CONVERT_H_

#include "hobotxsdk/xstream_data.h"
#include "xproto_msgtype/vioplugin_data.h"
#include "smartplugin/traffic_info.h"
#include "xproto_msgtype/protobuf/vehicle.pb.h"

namespace horizon {
namespace iot {
using horizon::vision::xproto::basic_msgtype::VioMessage;

class Convertor {
 public:
  /**
   * @brief convert input VioMessage to xstream inputdata.
   * @param input input VioMessage
   * @return xstream::InputDataPtr xstream input
   */
  static xstream::InputDataPtr ConvertInput(const VioMessage *input);
 /**
   * @brief 将车辆结构化信息转换成pb格式
   *
   * @param vehicle_info
   * @param vehicle_info_pb
   */
  static void ConvertVehicleInfoToProto(
      const vision::xproto::smartplugin::VehicleInfo &vehicle_info,
      vehicle::VehicleInfo *vehicle_info_pb);

 /**
   *@brief 将人体结构化信息转换成pb格式
   *
   *@param person_info
   *@param person_info_pb
   *
   */
  static void ConvertPersonInfoToProto(
      const vision::xproto::smartplugin::PersonInfo &person_info,
      vehicle::Person *person_info_pb);

  /**
   *@brief 将非机动车结构化信息转换成pb格式
   *
   *@param nomotor_info
   *@param nomotor_info_pb
   *
   */
  static void ConvertNomotorInfoToProto(
      const vision::xproto::smartplugin::NoMotorVehicleInfo &nomotor_info,
      vehicle::Nonmotor *nomotor_info_pb);

   /**
   * @brief 将车辆capture信息转换成pb格式
   *
   * @param vehicle_cap_info
   * @param vehicle_cap_pb
   */
  static void ConvertVehicleCaptureInfoToProto(
      const vision::xproto::smartplugin::VehicleCapture &vehicle_capture,
      vehicle::VehicleCapture *vehicle_capture_pb);

 public:
  static int image_compress_quality;
};

}  // namespace iot
}  // namespace horizon

#endif  //  INCLUDE_SMARTPLUGIN_CONVERT_H_
