#pragma once
#include <cstdint>

namespace idl::ports {
    static constexpr uint8_t SYNC_V1      = 1;   // optional
    static constexpr uint8_t CMD_V1       = 2;   // optional
    static constexpr uint8_t POSE_EST_V1  = 10;  // RSOM -> RCU
    static constexpr uint8_t RNAV_STATE_V1 = 11; // RCU  -> RSOM
    static constexpr uint8_t SYNC_V1      = 1;   // optional
    static constexpr uint8_t CMD_V1       = 2;   // optional
}