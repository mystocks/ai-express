/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-04 02:41:22
 * @Version: v0.0.1
 * @Brief: smartplugin declaration
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-09-30 00:45:01
 */

#ifndef INCLUDE_SMARTPLUGIN_SMARTPLUGIN_H_
#define INCLUDE_SMARTPLUGIN_SMARTPLUGIN_H_

#include <memory>
#include <string>
#include <vector>

#include "xproto/message/pluginflow/flowmsg.h"
#include "xproto/plugin/xpluginasync.h"

#include "hobotxsdk/xstream_sdk.h"
#include "smartplugin_box/runtime_monitor.h"
#include "smartplugin_box/smart_config.h"
#include "smartplugin_box/traffic_info.h"
#include "votmodule.h"
#include "xproto_msgtype/smartplugin_data.h"

namespace horizon {
namespace vision {
namespace xproto {
namespace smartplugin_multiplebox {

using horizon::vision::xproto::XPluginAsync;
using horizon::vision::xproto::XProtoMessage;
using horizon::vision::xproto::XProtoMessagePtr;

using horizon::vision::xproto::basic_msgtype::SmartMessage;
using xstream::InputDataPtr;
using xstream::OutputDataPtr;
using xstream::XStreamSDK;

struct CustomSmartMessage : SmartMessage {
  explicit CustomSmartMessage(xstream::OutputDataPtr out) : smart_result(out) {
    type_ = TYPE_SMART_MESSAGE;
  }
  std::string Serialize() { return ""; };
  void Serialize_Print(Json::Value &root){};

private:
  xstream::OutputDataPtr smart_result;
};

class SmartPlugin : public XPluginAsync {
public:
  SmartPlugin() = default;
  explicit SmartPlugin(const std::string &config_file);

  void SetConfig(const std::string &config_file) { config_file_ = config_file; }

  ~SmartPlugin() = default;
  int Init() override;
  int Start() override;
  int Stop() override;

private:
  int Feed(XProtoMessagePtr msg);
  void OnCallback(xstream::OutputDataPtr out);
  void OnCallback2(xstream::OutputDataPtr out);
  void OnCallback3(xstream::OutputDataPtr out);
  void OnCallback4(xstream::OutputDataPtr out);
  void ParseConfig();

  std::shared_ptr<XStreamSDK> sdk_;
  std::shared_ptr<XStreamSDK> sdk2_;
  std::shared_ptr<XStreamSDK> sdk3_;
  std::shared_ptr<XStreamSDK> sdk4_;
  std::string config_file_;
  std::shared_ptr<RuntimeMonitor> monitor_;
  std::shared_ptr<JsonConfigWrapper> config_;
  std::string xstream_workflow_cfg_file_;
  bool enable_profile_{false};
  std::string profile_log_file_;
  bool result_to_json{false};
  Json::Value root;
  std::shared_ptr<VotModule> vot_module_;
  char *buffer_[4];
};

}  // namespace smartplugin_multiplebox
}  // namespace xproto
}  // namespace vision
}  // namespace horizon
#endif  // INCLUDE_SMARTPLUGIN_SMARTPLUGIN_H_
