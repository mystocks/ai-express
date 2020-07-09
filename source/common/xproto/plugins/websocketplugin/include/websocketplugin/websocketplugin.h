/*!
 * Copyright (c) 2016-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     websocketplugin.h
 * \Author   ronghui.zhang
 * \Version  1.0.0.0
 * \Date     2020/5/12
 * \Brief    implement of api header
 */
#ifndef INCLUDE_WEBSOCKETPLUGIN_WEBSOCKETPLUGIN_H_
#define INCLUDE_WEBSOCKETPLUGIN_WEBSOCKETPLUGIN_H_
#include <memory>
#include <string>
#include <vector>

#include "websocketplugin/websocketconfig.h"
#include "websocketplugin/server/uws_server.h"
#include "xproto/message/pluginflow/flowmsg.h"
#include "xproto/plugin/xpluginasync.h"
#include "xproto_msgtype/protobuf/x3.pb.h"
#ifdef X3_MEDIA_CODEC
#include "media_codec/media_codec_manager.h"
#endif

namespace horizon {
namespace vision {
namespace xproto {
namespace websocketplugin {
using horizon::vision::xproto::server::UwsServer;
using horizon::vision::xproto::XPluginAsync;
using horizon::vision::xproto::XProtoMessagePtr;

class WebsocketPlugin : public xproto::XPluginAsync {
 public:
  WebsocketPlugin() = delete;
  explicit WebsocketPlugin(std::string config_path);
  ~WebsocketPlugin() override;
  int Init() override;
  int Start() override;
  int Stop() override;

 private:
  int FeedVideo(XProtoMessagePtr msg);
  int FeedSmart(XProtoMessagePtr msg);
  void ParseConfig();
  int Reset();
 private:
  std::shared_ptr<UwsServer> uws_server_;
  std::string config_file_;
  std::shared_ptr<WebsocketConfig> config_;
  std::shared_ptr<std::thread> worker_;
  std::mutex map_mutex_;
  const uint8_t cache_size_ = 25;  // max input cache size
  std::vector<x3::FrameMessage> x3_frames_;
  int origin_image_width_ = 1920;  // update by FeedVideo
  int origin_image_height_ = 1080;
  int dst_image_width_ = 1920;  // update by FeedVideo
  int dst_image_height_ = 1080;
  std::mutex smart_mutex_;
  bool smart_stop_flag_;
  std::mutex video_mutex_;
  bool video_stop_flag_;
#ifdef X3_MEDIA_CODEC
  int chn_;
#endif
};

}  // namespace websocketplugin
}  // namespace xproto
}  // namespace vision
}  // namespace horizon

#endif  // INCLUDE_WEBSOCKETPLUGIN_WEBSOCKETPLUGIN_H_
