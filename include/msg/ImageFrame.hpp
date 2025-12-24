#pragma once
#include <cstdint>

namespace msg {

struct ImageFrame {
    // Non-owning pointer to the first byte of a contiguous image buffer 
    // const to prevent modification
    const uint8_t* data;     
    
    // Image dimensions in pixels
    uint32_t width;         // pixels
    uint32_t height;        // pixels

    // Stride = number of BYTES between the start of row v and the start of row v+1.
    // For tightly packed images: stride == width * bytes_per_px.
    // For aligned/padded images: stride may be larger
    uint32_t stride;        // bytes per row

    // Bytes per Pixel: Storage size per pixel in bytes (container width).
    // Examples: GRAY8 -> 1, GRAY16/RAW10-in-16 -> 2.
    uint8_t bytes_per_px;

    // Bit Depth: Number of meaningful bits in each pixel sample.
    // Examples: RAW8 -> 8, RAW10 -> 10, RAW12 -> 12, RAW16 -> 16.
    // Note: RAW10 may be stored in 16-bit containers (bytes_per_px=2, bit_depth=10).
    uint8_t bit_depth;

    uint64_t t_exp_end_us;  // exposure-end timestamp (SOM monotonic, µs)
    uint32_t frame_id;      // increasing counter

  constexpr uint32_t byteSize() const { return stride * height; }
};

} // namespace msg