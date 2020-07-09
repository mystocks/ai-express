/*!
 * -------------------------------------------
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     msg_manager.h
 * \Author   Yingmin Li
 * \Mail     yingmin.li@horizon.ai
 * \Version  1.0.0.0
 * \Date     2019-04-22
 * \Brief    implement of msg_manager.h
 * \DO NOT MODIFY THIS COMMENT, \
 * \WHICH IS AUTO GENERATED BY EDITOR
 * -------------------------------------------
 */

#ifndef XPROTO_INCLUDE_XPROTO_MANAGER_MSG_MANAGER_H_
#define XPROTO_INCLUDE_XPROTO_MANAGER_MSG_MANAGER_H_
#include <map>
#include <memory>
#include <mutex>
#include <vector>
#include <string>
#include <algorithm>
#include "xproto/message/pluginflow/flowmsg.h"
#include "xproto/plugin/xplugin.h"
#include "xproto/utils/singleton.h"
#include "xproto/threads/threadpool.h"
#include "xproto/message/pluginflow/msg_registry.h"
#include "hobotlog/hobotlog.hpp"

namespace horizon {
namespace vision {
namespace xproto {
class XMsgQueue : public hobot::CSingleton<XMsgQueue> {
 public:
  XMsgQueue() {
    msg_handle_.CreatThread(1);
  }
  ~XMsgQueue() = default;

 public:
  void RegisterPlugin(const XPluginPtr &plugin, const std::string& msg_type) {
    std::lock_guard<std::mutex> lck(mutex_);
    auto type_handle = XPluginMsgRegistry::Instance().Get(msg_type);
    if (type_handle == XPLUGIN_INVALID_MSG_TYPE) {
      LOGW << "try to register invalid msg type:" << msg_type
           << ", for plugin " << plugin->desc();
      return;
    }
    table_[type_handle].push_back(plugin);
  }
  void UnRegisterPlugin(const XPluginPtr &plugin, const std::string& msg_type) {
    std::lock_guard<std::mutex> lck(mutex_);
    auto type_handle = XPluginMsgRegistry::Instance().Get(msg_type);
    if (type_handle == XPLUGIN_INVALID_MSG_TYPE ||
        table_.find(type_handle) == table_.end()) {
      LOGW << "try to unregister invalid msg type:" << msg_type
           << ", for plugin " << plugin->desc();
      return;
    }
    auto iter = std::find(table_[type_handle].begin(),
                          table_[type_handle].end(), plugin);
    if (iter != table_[type_handle].end()) {
      LOGW << "to erase plugin: " << plugin->desc();
      table_[type_handle].erase(iter);
    }
  }

  void PushMsg(XProtoMessagePtr msg) {
    msg_handle_.PostTask(std::bind(&XMsgQueue::Dispatch, this, msg));
  }

 private:
  void Dispatch(XProtoMessagePtr msg) {
    std::lock_guard<std::mutex> lck(mutex_);
    auto type_handle = XPluginMsgRegistry::Instance().Get(msg->type());
    if (type_handle == XPLUGIN_INVALID_MSG_TYPE) {
      LOGW << "push no consumer message，type:" << msg->type();
      return;
    }
    auto &plugins = table_[type_handle];
    for (auto &plugin : plugins) {
      plugin->OnMsg(msg);
    }
  }

 private:
  std::map<XPluginMsgTypeHandle, std::vector<XPluginPtr>> table_;
  hobot::CThreadPool msg_handle_;

  std::mutex mutex_;
};

}  // namespace xproto
}  // namespace vision
}  // namespace horizon
#endif  // XPROTO_INCLUDE_XPROTO_MANAGER_MSG_MANAGER_H_
