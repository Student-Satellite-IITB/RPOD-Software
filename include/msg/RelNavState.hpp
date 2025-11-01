#pragma once
#include <cstdint>

namespace msg {

struct RelNavState {
  uint64_t t_ns;     // publication time (OBC-disciplined)
  float    r_nav[3]; // target in NAV frame (m)
  float    v_nav[3]; // m/s
  float    q_rel[4]; // target orientation (choose BCH or NAV, document once)
  float    w_rel[3]; // rad/s (0 if not observable)
  float    P_diag[12]; // conservative diagonal of covariance
  uint8_t  mode;     // RnavMode
};

} // namespace msg
