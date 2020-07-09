/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2019 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef INCLUDE_X2_AE_H_
#define INCLUDE_X2_AE_H_
#ifdef __cplusplus
extern "C" {
#endif

struct rect_t {
  int32_t start_x;
  int32_t start_y;
  int32_t width;
  int32_t height;
};

struct face_track_info {
  // face track id
  int32_t track_id;
  struct rect_t face_rio;
};
struct face_info_t {
  int face_rio_size;                   // face rio size, none set to 0
  struct face_track_info *face_track;  // face track info
};

/******************************************************************************
 * int hb_cam_set_ae_rio(struct rect_t *win_rect,	struct face_info_t  *
 *face_info);
 *
 * input :
 *
 *  win_rect							: the preview windows
 *rect
 *  face_info 						: the face detect information
 *include track id, face rio
 *
 * output :
 *   None
 ******************************************************************************/

extern int hb_cam_set_ae_rio(struct rect_t *win_rect,
                             struct face_info_t *face_info);

#ifdef __cplusplus
}
#endif

#endif  // INCLUDE_X2_AE_H_
