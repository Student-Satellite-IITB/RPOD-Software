#pragma once
#include <cstdint>

namespace msg {

enum class PixelFormat : uint8_t { GRAY8 = 0 };

struct ImageFrame {
    const uint8_t* data;          // Non-owning pointer to contiguous bytes, const to prevent modification
    uint32_t width;         // pixels
    uint32_t height;        // pixels
    uint32_t stride;        // bytes per row
    PixelFormat format;     // GRAY8 for now
    uint64_t t_exp_end_us;  // exposure-end timestamp (SOM monotonic, µs)
    uint32_t frame_id;      // increasing counter

  constexpr uint32_t byteSize() const { return stride * height; }
};

} // namespace msg