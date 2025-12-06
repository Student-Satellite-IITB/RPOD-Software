#pragma once
#include <cstdint>
#include "msg/FeatureFrame.hpp"   // for msg::TrackState and msg::PatternId

namespace msg {

// Pose of docking target LED Pattern P expressed in the camera frame C.
// Keep SOM strictly in camera frame; MCU owns extrinsics & frame transforms.

struct PoseEstimate {
  uint64_t t_exp_end_us;   // Exposure-end timestamp (µs) of the source image.

  // Pose of docking target LED Pattern P expressed in the camera frame C (H_P/C).
  float q_PbyC[4];   // Unit quaternion (w,x,y,z). Orientation of P relative to C.
                     // Convention: rotates vectors from P into C (v_C = R(q)*v_P).
  float t_PbyC[3];   // Translation of the origin of P expressed in C: [x,y,z] (m).

  float az;     // Line-of-sight azimuth angle to pattern center (radians).
  float el;     // Line-of-sight elevation angle to pattern center (radians).
  float roll;   // Roll angle of pattern P in radians.
  float pitch;  // Pitch angle of pattern P in radians.
  float yaw;    // Yaw angle of pattern P in radians.
  float range_m; // Range from camera C to pattern P along camera Z axis (m
  
  // --- Traceability---
  PatternId pattern_id;  // PatternId enum of the observed pattern (INNER/OUTER).
  uint32_t frame_id;       // Image frame counter (for correlation & drop detection).
  msg::TrackState state;   // INIT / TRACK / LOST (pass-through from detector/tracker).
  uint8_t  used_led_mask;  // bit i=1 if slot i contributed (INNER: 0..4); 0 if unknown/unused
  uint8_t valid;           // 1 if this pose passed estimator gates; 0 if unusable this frame.

  // --- Quality Metrics TODO (add later, when needed) ---
  // float cov_rpy_upper[6];  // 3x3 orientation covariance (upper-tri, rad^2).
  // float cov_t_upper[6];    // 3x3 translation covariance (upper-tri, m^2).
  float reproj_rms_px;     // RMS reprojection error [px].
  // uint16_t inlier_count;   // Inliers used in PnP.
  // float plane_nC[3];       // Unit normal of docking plane in C (for ToF gating).
  // float plane_dC;          // Plane offset so n·x + d = 0 (m).
};

} // namespace msg
