#pragma once
#include <vector>
#include <cstdint>
#include <array>

// struct ImageFrame {
//     uint8_t* data;
//     int width;
//     int height;
// };

// struct Keypoint {
//     int x, y;
//     float response;
// };

// struct FeatureFrame {
//     std::vector<Keypoint> keypoints;
// };

// struct PoseEstimate {
//     float x = 0.0f;
//     float y = 0.0f;
//     float z = 0.0f;
//     float roll = 0.0f;   // Roll in radians
//     float pitch = 0.0f;  // Pitch in radians
//     float yaw = 0.0f;    // Yaw in radians
//     // std::array<float, 4> attitude_quat = {1.0f, 0.0f, 0.0f, 0.0f};
//     // uint64_t timestamp_us = 0;
//     bool valid = true;
// };

struct CommandPacket {
    uint8_t commandId;
    std::array<uint8_t, 64> payload;  // Fixed size for simplicity
};

struct Event{
    uint8_t EvenId;
};

enum class RPODState {
    IDLE,
    SEARCH,
    FAR_APPROACH,
    CLOSE_APPROACH,
    SOFT_DOCKING,
    HARD_DOCKING,
    DOCKED
};

enum class RPODEvent {
    CMD_START_APPROACH,
    VBN_LOCK_FAILED,
    VBN_LOCK_ACQUIRED,
    OUTER_PATTERN_DETECTED,
    INNER_PATTERN_DETECTED,
    HANDOVER_COMPLETE,
    SOFT_DOCKING_DETECTED,
    HARD_DOCKING_CONFIRMED,
    CMD_RESET
};
