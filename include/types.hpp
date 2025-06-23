#pragma once
#include <vector>
#include <cstdint>

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