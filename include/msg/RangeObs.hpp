#pragma once
#include <cstdint>

namespace msg {

struct RangeObs {
  uint64_t t_ns;      // measurement time in MCU/OBC epoch
  float    range_m;   // scalar range
  float    sigma_m;   // 1-sigma uncertainty
  uint8_t  valid;
};

} // namespace msg
