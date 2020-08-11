#ifndef INCLUDE_VOTMODULE_H_
#define INCLUDE_VOTMODULE_H_

#include <array>
#include <vector>

#include "hb_vio_interface.h"
#include "mediapipemanager/basicmediamoudle.h"

namespace horizon {
namespace vision {

struct VotData {
  uint32_t channel;
  std::vector<std::array<int, 4>> boxes;
  std::vector<std::array<int, 2>> points;
  char *y_virtual_addr;
  char *uv_virtual_addr;
};

// class VotModule : public BasicMediaModule
// {
// public:
//   VotModule(/* args */);
//   ~VotModule();
//   virtual int Init(uint32_t group_id,
//                    const PipeModuleInfo *module_info) override;
//   virtual int Start() override;
//   virtual int Input(void *data) override;
//   virtual int Output(void **data) override;
//   virtual int OutputBufferFree(void *data) override;
//   virtual int Stop() override;
//   virtual int DeInit() override;

// protected:

// private:
//   uint32_t group_id_;
//   uint32_t timeout_;
//   uint32_t image_width_;
//   uint32_t image_height_;
//   char *buffer_[4];
//   // uint32_t frameDepth_;
//   // uint32_t buffer_index_;
//   // std::vector<pym_buffer_t> buffers_;
// };

class VotModule {
public:
  VotModule(/* args */);
  ~VotModule();
  int Init(uint32_t group_id, const PipeModuleInfo *module_info);
  int Start();
  int Input(void *data);
  int Output(void **data);
  int OutputBufferFree(void *data);
  int Stop();
  int DeInit();

protected:
private:
  uint32_t group_id_;
  uint32_t timeout_;
  uint32_t image_width_;
  uint32_t image_height_;
  char *buffer_;
  // uint32_t frameDepth_;
  // uint32_t buffer_index_;
  // std::vector<pym_buffer_t> buffers_;
};

}  // namespace vision
}  // namespace horizon
#endif  // INCLUDE_VOTMODULE_H_