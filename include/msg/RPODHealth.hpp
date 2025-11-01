// #pragma once
// #include <cstdint>

// namespace msg {

// struct RpodHealth {
//   uint64_t t_ns;
//   uint8_t  som_alive;    // heartbeat OK
//   uint8_t  tof_alive;    // sensor OK
//   uint8_t  ekf_healthy;  // not diverged / finite
//   uint8_t  abort;        // KOZ risk asserted by MCU
//   uint8_t  degrade;      // running TOF_ONLY or VISION_ONLY
//   uint8_t  _pad[3];
//   float    last_innov_norm; // recent Mahalanobis mean
//   float    temp_c;          // board temp
//   float    cpu_load;        // 0..1
// };
// static_assert(sizeof(RpodHealth)==(8+4+4*3), "POD");

// } // namespace msg
