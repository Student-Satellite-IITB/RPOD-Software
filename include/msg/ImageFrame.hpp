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

    // Right-shift to convert stored container DN -> native DN.
    //
    // Examples:
    // - RAW10 LSB-aligned in uint16: bits are in [9..0]      => bit_shift = 0
    // - RAW10 MSB-aligned in uint16: bits are in [15..6]     => bit_shift = 6
    // - RAW12 MSB-aligned in uint16: bits are in [15..4]     => bit_shift = 4
    // - True GRAY16 / RAW16: full 16 meaningful bits         => bit_shift = 0
    //
    // After read_dn(), the returned DN should lie in [0, 2^bit_depth - 1].
    uint8_t bit_shift;

    uint64_t t_exp_end_us;  // exposure-end timestamp (SOM monotonic, µs)
    uint32_t frame_id;      // increasing counter

  constexpr uint32_t byteSize() const { return stride * height; }
};

} // namespace msg