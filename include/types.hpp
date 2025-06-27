#pragma once
#include <vector>
#include <cstdint>
#include <array>

struct ImageFrame {
    uint8_t* data;
    int width;
    int height;
};

struct Keypoint {
    int x, y;
    float response;
};

struct FeatureFrame {
    std::vector<Keypoint> keypoints;
};

struct CommandPacket {
    uint8_t commandId;
    std::array<uint8_t, 64> payload;  // Fixed size for simplicity
};