#include "wire/pose_estimate.hpp"
#include "wire/codec.hpp"
#include <cstring>

namespace wire {

static inline void put_f32(uint8_t*& p, float v){
  static_assert(sizeof(float) == 4);
  std::memcpy(p, &v, 4);
  p += 4;
}

static inline float get_f32(const uint8_t*& p){
  float v;
  std::memcpy(&v, p, 4);
  p += 4;
  return v;
}

size_t encode_pose_estimate(const msg::PoseEstimate& in, uint8_t* out, size_t cap)
{
  if (!out || cap < POSE_EST_BYTES) return 0;

  uint8_t* p = out;

  // Timestamp (RSOM timebase, exposure end)
  wire::put_u64(p, in.t_exp_end_us);

  // Translation (meters)
  put_f32(p, in.t_CbyP[0]);
  put_f32(p, in.t_CbyP[1]);
  put_f32(p, in.t_CbyP[2]);

  // Quaternion (w,x,y,z)
  put_f32(p, in.q_C_P[0]);
  put_f32(p, in.q_C_P[1]);
  put_f32(p, in.q_C_P[2]);
  put_f32(p, in.q_C_P[3]);

  // Quality
  put_f32(p, in.reproj_rms_px);

  return (size_t)(p - out); // should be POSE_EST_BYTES
}

bool decode_pose_estimate(const uint8_t* data, size_t len, msg::PoseEstimate& out)
{
  if (!data || len != POSE_EST_BYTES) return false;

  const uint8_t* p = data;
  out = {}; // clear everything

  out.t_exp_end_us = wire::get_u64(p);

  out.t_CbyP[0] = get_f32(p);
  out.t_CbyP[1] = get_f32(p);
  out.t_CbyP[2] = get_f32(p);

  out.q_C_P[0] = get_f32(p);
  out.q_C_P[1] = get_f32(p);
  out.q_C_P[2] = get_f32(p);
  out.q_C_P[3] = get_f32(p);

  out.reproj_rms_px = get_f32(p);

  // minimal semantics
  out.valid = 1;
  return true;
}

} // namespace wire