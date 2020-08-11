#ifndef INCLUDE_MEDIAPIPELINE_H_
#define INCLUDE_MEDIAPIPELINE_H_

#include <memory>

#include "mediapipemanager/vdecmodule.h"
#include "mediapipemanager/vpsmodule.h"
// #include "mediapipemanager/votmodule.h"

namespace horizon {
namespace vision {
class MediaPipeLine {
public:
  MediaPipeLine(uint32_t gp_id0, uint32_t gp_id1);
  virtual int Init();
  virtual int Start();
  virtual int Stop();
  virtual int Input(void *data);
  virtual int Output(void **data);
  virtual int OutputBufferFree(void *data);
  virtual int DeInit();

protected:
private:
  MediaPipeLine() = delete;
  MediaPipeLine(const MediaPipeLine &) = delete;
  MediaPipeLine &operator=(const MediaPipeLine &) = delete;

  uint32_t vdec_group_id_;
  uint32_t vps_group_id_;

  std::shared_ptr<VdecModule> vdec_module_;
  std::shared_ptr<VpsModule> vps_module_;
  // std::shared_ptr<VotModule> vot_module_;
};

}  // namespace vision
}  // namespace horizon

#endif  // INCLUDE_MEDIAPIPELINE_H_