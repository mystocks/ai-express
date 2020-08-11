#include "mediapipemanager/meidapipeline.h"

#include "hb_comm_vdec.h"
#include "hb_vps_api.h"
#include "hobotlog/hobotlog.hpp"

namespace horizon {
namespace vision {
MediaPipeLine::MediaPipeLine(uint32_t gp_id0, uint32_t gp_id1)
    : vdec_group_id_(gp_id0), vps_group_id_(gp_id1) {
  vps_module_ = std::make_shared<VpsModule>();
  vdec_module_ = std::make_shared<VdecModule>();
  // vot_module_ = std::make_shared<VotModule>();
}

int MediaPipeLine::Init() {
  int ret = 0;
  PipeModuleInfo module_info;
  module_info.input_height = 1088;
  module_info.input_width = 1920;
  module_info.output_height = 1080;
  module_info.output_width = 1920;
  module_info.frame_depth = 4;
  ret = vdec_module_->Init(vdec_group_id_, &module_info);
  module_info.frame_depth = 6;
  ret = vps_module_->Init(vps_group_id_, &module_info);
  // ret = vot_module_->Init(vps_group_id_, &module_info);
  return ret;
}

int MediaPipeLine::Start() {
  vdec_module_->Start();
  vps_module_->Start();
  return 0;
}

int MediaPipeLine::Input(void *data) {
  vdec_module_->Input(data);
  return 0;
}

int MediaPipeLine::Output(void **data) {
  int ret = 0;
  hb_vio_buffer_t hb_vio_buf;
  void *data_temp = nullptr;
  ret = vdec_module_->Output(&data_temp);
  if (ret != 0) {
    *data = nullptr;
    return -1;
  }
  if (data_temp == nullptr) {
    LOGE << "data_temp is nullptr";
    return -2;
  }

  VIDEO_FRAME_S *video_frame = (VIDEO_FRAME_S *)(data_temp);
  memset(&hb_vio_buf, 0, sizeof(hb_vio_buffer_t));
  hb_vio_buf.img_addr.addr[0] = video_frame->stVFrame.vir_ptr[0];
  hb_vio_buf.img_addr.paddr[0] = video_frame->stVFrame.phy_ptr[0];
  hb_vio_buf.img_addr.addr[1] = video_frame->stVFrame.vir_ptr[1];
  hb_vio_buf.img_addr.paddr[1] = video_frame->stVFrame.phy_ptr[1];
  hb_vio_buf.img_addr.width = video_frame->stVFrame.width;
  hb_vio_buf.img_addr.height = video_frame->stVFrame.height;
  hb_vio_buf.img_addr.stride_size = video_frame->stVFrame.width;
  hb_vio_buf.img_info.planeCount = 2;
  hb_vio_buf.img_info.img_format = 8;
  hb_vio_buf.img_info.fd[0] = video_frame->stVFrame.fd[0];
  hb_vio_buf.img_info.fd[1] = video_frame->stVFrame.fd[1];
  ret = vps_module_->Input(&hb_vio_buf);
  if (ret != 0) {
    LOGW << "vps input timeout.";
    vdec_module_->OutputBufferFree(data_temp);
    *data = nullptr;
    return -2;
  }
  ret = vps_module_->Output(data);
  vdec_module_->OutputBufferFree(data_temp);
  if (ret != 0) {
    LOGW << "vps output timeout.";
    *data = nullptr;
    return -3;
  }
  // vot_module_->Input(*data);
  return 0;
}

int MediaPipeLine::OutputBufferFree(void *data) {
  vps_module_->OutputBufferFree(data);
  return 0;
}

int MediaPipeLine::Stop() {
  vdec_module_->Stop();
  vps_module_->Stop();
  return 0;
}

int MediaPipeLine::DeInit() {
  vdec_module_->DeInit();
  vps_module_->DeInit();
  return 0;
}

}  // namespace vision
}  // namespace horizon