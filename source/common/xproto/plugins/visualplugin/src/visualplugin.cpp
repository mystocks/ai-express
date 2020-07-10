/*
 * @Copyright 2020 Horizon Robotics, Inc.
 */
#include <fstream>
#include <iostream>

#include "visualplugin/visualplugin.h"
#include "hobotlog/hobotlog.hpp"
#include "xproto/message/pluginflow/msg_registry.h"
#include "xproto_msgtype/protobuf/x2.pb.h"
#include "xproto_msgtype/protobuf/vehicle.pb.h"
#ifdef X3_MEDIA_CODEC
#include "media_codec/media_codec_manager.h"
#endif


namespace horizon {
namespace vision {
namespace xproto {
namespace visualplugin {

VisualPlugin::VisualPlugin(const std::string &config_file)
  : config_file_(config_file) {
  stop_flag_ = false;
  Reset();
}

VisualPlugin::~VisualPlugin() {
  config_ = nullptr;
}

int VisualPlugin::Init() {
  LOGI << __FUNCTION__;
  // load config
  config_ = std::make_shared<VisualConfig>(config_file_);
  if(!config_ || !config_->LoadConfig()) {
    LOGE << "failed to load config file";
    return -1;
  }
  LOGI << "load config " << config_file_;
  // live555 init
  /*
  send_buf_ = (char*)malloc(send_buf_size_);
  if (nullptr == send_buf_) {
    LOGE << "failed to init send_buf_";
    return -1;
  }
  */

  if (config_->display_mode_ != VisualConfig::QT_MODE) {
    LOGE << "not support web display";
    return 0;
  }

  memset(&server_param_, 0, sizeof(SERVER_PARAM_S));
  if(config_->auth_mode_ > 0) {
    server_param_.hasConnAuth = true;
    snprintf(server_param_.username, 64, "%s", config_->user_.c_str());
    snprintf(server_param_.password, 64, "%s", config_->password_.c_str());
  } else {
    server_param_.hasConnAuth = false;
  }
  server_param_.data_buf_size = config_->data_buf_size_;
  server_param_.packet_size = config_->packet_size_;

  RegisterMsg(TYPE_IMAGE_MESSAGE,
    std::bind(&VisualPlugin::FeedVideo, this, std::placeholders::_1));
  RegisterMsg(TYPE_SMART_MESSAGE,
    std::bind(&VisualPlugin::FeedSmart, this, std::placeholders::_1));
  // 调用父类初始化成员函数注册信息
  XPluginAsync::Init();

  LOGI << __FUNCTION__;
  return 0;
}

int VisualPlugin::Start() {
  LOGI << __FUNCTION__;
  if (config_->display_mode_ != VisualConfig::QT_MODE) {
    LOGE << "not support web display";
    return 0;
  }

  server_run(&server_param_);
  if(nullptr == worker_) {
    stop_flag_ = false;
    worker_ = std::make_shared<std::thread>(&VisualPlugin::EncodeThread, this);
  }
  return 0;
}

int VisualPlugin::Stop() {
  LOGI << __FUNCTION__;
  if (config_->display_mode_ != VisualConfig::QT_MODE) {
    LOGE << "not support web display";
    return 0;
  }

  if(worker_ && worker_->joinable()) {
    stop_flag_ = true;
    worker_->join();
    worker_ = nullptr;
    LOGI << "VisualPlugin stop worker";
  }
  server_stop();
  return 0;
}

int VisualPlugin::Reset() {
  LOGI << __FUNCTION__ << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
  std::unique_lock<std::mutex> lock(map_mutex_);
  input_frames_.clear();
  input_frames_.reserve(cache_size_);
  // to do: reset the work thread
  return 0;
}

void VisualPlugin::PushFrame(InputType type, XProtoMessagePtr frame_msg) {
  LOGI << __FUNCTION__;
  if(nullptr == frame_msg) {
    LOGW << "VisualPlugin::PushFrame frame_msg is null";
    return;
  }
  std::unique_lock<std::mutex> lock(map_mutex_);
  VisualInput *input = new VisualInput();
  input->type = type;
  input->frame_msg = frame_msg;
  input_frames_.push_back(input);
  if(input_frames_.size() > cache_size_)
    LOGW << "the cache is full, maybe the encode thread is slowly";
  return;
}

int VisualPlugin::EncodeJPG(tjhandle handle_tj,
    const unsigned char *yuv_buf, int width, int height,
    unsigned char **jpeg_buf, unsigned long *jpeg_size, int quality) {
  int padding = 1;
  int subsample = TJSAMP_420;
  int flags = 0;
  return tjCompressFromYUV(handle_tj, yuv_buf, width, padding,
    height, subsample, jpeg_buf, jpeg_size, quality, flags);
}

int VisualPlugin::EncodeThread() {
  LOGI << __FUNCTION__;
  int rv;
  bool bret;
  std::size_t i = 0;
  std::string send_data;
  cv::Mat yuv_img;
  std::vector<uchar> img_buf;
  ioAV::Perception perception;
  std::vector<VisualInput*> frames;
  
  send_data.reserve(1024);
  frames.reserve(cache_size_);
  img_buf.clear();
  img_buf.reserve(500*1024);

#ifdef X3_MEDIA_CODEC
  /* 1. media codec init */
  iot_venc_src_buf_t *frame_buf = nullptr;
  iot_venc_stream_buf_t *stream_buf = nullptr;
  /* 1.1 get media codec manager and module init */
  MediaCodecManager &manager = MediaCodecManager::Get();
  rv = manager.ModuleInit();
  HOBOT_CHECK(rv == 0);
  /* 1.2 get media codec venc chn */
  int chn = manager.GetEncodeChn();
  /* 1.3 media codec venc chn init */
  int pic_width = config_->image_width_;
  int pic_height = config_->image_height_;
  int frame_buf_depth = config_->frame_buf_depth_;
  rv = manager.EncodeChnInit(chn, PT_JPEG, pic_width, pic_height,
          frame_buf_depth, HB_PIXEL_FORMAT_NV12);
  HOBOT_CHECK(rv == 0);
  /* 1.4 set media codec venc jpg chn qfactor params */
  rv = manager.SetUserQfactorParams(chn, config_->jpeg_quality_);
  HOBOT_CHECK(rv == 0);
  /* 1.5 set media codec venc jpg chn qfactor params */
  rv = manager.EncodeChnStart(chn);
  HOBOT_CHECK(rv == 0);
  /* 1.6 alloc media codec vb buffer init */
  int vb_num = frame_buf_depth;
  int pic_stride = config_->image_width_;
  int pic_size = pic_stride * pic_height * 3 / 2;  // nv12 format
  rv = manager.VbBufInit(chn, pic_width, pic_height, pic_stride,
          pic_size, vb_num);
  HOBOT_CHECK(rv == 0);
#endif

  while (!stop_flag_) {
    frames.clear();
    std::unique_lock<std::mutex> lock(map_mutex_);
    frames.assign(input_frames_.begin(), input_frames_.end());
    input_frames_.clear();
    lock.unlock();
    for (i=0; i<frames.size(); ++i) {
      perception.Clear();
      perception.set_checkcode("");
      perception.set_sync_flag(0);
      VisualInput *frame = frames[i];
      if (frame->type==VT_VIDEO) {
        // get yuv with target layer
        auto vframe = std::dynamic_pointer_cast<VioMessage>(frame->frame_msg);
#ifndef X3_MEDIA_CODEC
        rv = Convertor::GetYUV(yuv_img, vframe.get(), config_->layer_);
#else
        /* 2. start encode yuv to jpeg */
        /* 2.1 get media codec vb buf for store src yuv data */
        rv = manager.GetVbBuf(chn, &frame_buf);
        HOBOT_CHECK(rv == 0);
        /* 2.2 get src yuv data */
        rv = Convertor::GetYUV(frame_buf, vframe.get(), config_->layer_);
        HOBOT_CHECK(rv == 0);
        frame_buf->frame_info.pts = vframe->time_stamp_;
#endif
        if (0 == rv) {
          // jpeg encode
#ifndef X3_MEDIA_CODEC
          bret = Convertor::YUV2JPG(img_buf, yuv_img, config_->jpeg_quality_);
#else
          /* 2.3. encode yuv data to jpg */
          rv = manager.EncodeYuvToJpg(chn, frame_buf, &stream_buf);
          if (rv == 0) {
              bret = true;
              auto data_ptr = stream_buf->stream_info.pstPack.vir_ptr;
              auto data_size = stream_buf->stream_info.pstPack.size;
              img_buf.assign(data_ptr, data_ptr + data_size);
              /* dump jpg picture */
              if (config_->dump_jpg_num_-- > 0) {
                  static int frame_id = 0;
                  std::string file_name = "out_stream_" +
                      std::to_string(frame_id++) + ".jpg";
                  std::fstream fout(file_name,
                          std::ios::out | std::ios::binary);
                  fout.write((const char *)data_ptr, data_size);
                  fout.close();
              }
          } else {
              bret = false;
              LOGE << "X3 media codec jpeg encode failed!";
          }
#endif
#ifdef X3_MEDIA_CODEC
          /* 2.4 free jpg stream buf */
          rv = manager.FreeStream(chn, stream_buf);
          HOBOT_CHECK(rv == 0);
          /* 2.5 free media codec vb buf */
          rv = manager.FreeVbBuf(chn, frame_buf);
          HOBOT_CHECK(rv == 0);
#endif
          if(bret) {
            perception.set_type(frame->type);
            perception.set_body(img_buf.data(), img_buf.size());
            perception.set_timestamp(vframe->time_stamp_);
          }
        }
      } else {
        std::string body;
        auto sframe = std::dynamic_pointer_cast<SmartMessage>(frame->frame_msg);
        rv = Convertor::PackSmartMsg(body, sframe.get(), config_->smart_type_);
        if (0 == rv) {
          perception.set_type(frame->type);
          perception.set_body(body);
          perception.set_timestamp(sframe->time_stamp);
        }
      }
      bret = perception.SerializeToString(&send_data);
      if (bret){
        server_send((unsigned char*)send_data.c_str(),
            send_data.size(), MsgSmart);
      }
      LOGI << "visualplugin begin delete one frame";
      delete frame;
    }
    frames.clear();
  }
#ifdef X3_MEDIA_CODEC
  /* 3. media codec deinit */
  /* 3.1 media codec chn stop */
  rv = manager.EncodeChnStop(chn);
  /* 3.2 media codec chn deinit */
  rv = manager.EncodeChnDeInit(chn);
  /* 3.3 media codec vb buf deinit */
  rv = manager.VbBufDeInit(chn);
  /* 3.4 media codec module deinit */
  rv = manager.ModuleDeInit();
#endif

  return 0;
}

int VisualPlugin::FeedVideo(XProtoMessagePtr msg) {
  LOGI << __FUNCTION__;
  PushFrame(VT_VIDEO, msg);
  return 0;
}

int VisualPlugin::FeedSmart(XProtoMessagePtr msg) {
  LOGI << __FUNCTION__;
  PushFrame(VT_SMART, msg);
  return 0;
}


}  // namespace visualplugin
}  // namespace xproto
}  // namespace vision
}  // namespace horizon
