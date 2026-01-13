#pragma once
#include <cstdint>

namespace msg {

struct RNAVState {
  uint64_t t_meas_us; // Time at which measurement was taken passed on from ImageCapture
  uint64_t t_pub_us;  // publication time (OBC-disciplined)
  float    r_nav[3]; // target in NAV frame (m)
  float    v_nav[3]; // m/s
  float    q_rel[4]; // target orientation (choose BCH or NAV, document once)
  float    w_rel[3]; // rad/s (0 if not observable)
  float    P[12*12]; // State Covariance Matrix (12x12) Row major P[r*12 + c]
  uint8_t  mode;     // RnavMode

  uint64_t exec_us; // Optional
};

} // namespace msg
