/*!
 * -------------------------------------------
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     profile.h
 * \Author   hangjun.yang
 * \Mail     hangjun.yang@horizon.ai
 * \Version  1.0.0.0
 * \Date     2020.08.03
 * \Brief    implement profile of message
 * -------------------------------------------
 */

#ifndef XPROTO_UTILS_PROFILE_H_
#define XPROTO_UTILS_PROFILE_H_

#include <memory>
#include <string>
#include <map>
#include <vector>
#include <mutex>
#include <chrono>
#include "hobotlog/hobotlog.hpp"
#include "xproto/message/pluginflow/flowmsg.h"

namespace horizon {
namespace vision {
namespace xproto {

class MsgScopeProfile {
 public:
  explicit MsgScopeProfile(std::string msg_name) {
    message_name_ = msg_name;
    start_time_ = std::chrono::system_clock::now();
  }

  ~MsgScopeProfile() {
    auto end_time = std::chrono::system_clock::now();
    auto duration_time = std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time - start_time_);
    LOGD << "[profile] " << message_name_ << " last " << duration_time.count()
         << " ms" << std::endl;
  }
 private:
  std::string message_name_;
  std::chrono::system_clock::time_point start_time_;
};

}  // namespace xproto
}  // namespace vision
}  // namespace horizon

#endif  // XPROTO_UTILS_PROFILE_H_
