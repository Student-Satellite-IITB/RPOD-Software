#pragma once
#include <vector>
#include <cstdint>

struct ImageFrame {
    uint32_t width;
    uint32_t height;
    std::vector<uint8_t> data; // grayscale or RGB
};
