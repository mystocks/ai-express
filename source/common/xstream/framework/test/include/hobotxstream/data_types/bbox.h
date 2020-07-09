/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     bbox base data type
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.23
 */

#ifndef TEST_INCLUDE_HOBOTXSTREAM_DATA_TYPES_BBOX_H_
#define TEST_INCLUDE_HOBOTXSTREAM_DATA_TYPES_BBOX_H_

// #include "./array.h"
#include "hobotxsdk/xstream_data.h"
#include "horizon/vision_type/vision_type.hpp"

namespace xstream {

typedef XStreamData<hobot::vision::BBox> BBox;
}  // namespace xstream

#endif  // TEST_INCLUDE_HOBOTXSTREAM_DATA_TYPES_BBOX_H_
