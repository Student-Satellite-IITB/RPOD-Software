#pragma once
#include <vector>
#include <cstdint>

/// \brief A 2D feature point (e.g., marker, LED blob, corner).
struct FeaturePoint2D {
    float x;  ///< Pixel x-coordinate
    float y;  ///< Pixel y-coordinate
};

/// \brief Represents a set of detected 2D features in an image.
struct FeatureFrame {
    std::vector<FeaturePoint2D> points; ///< List of 2D points
    uint64_t timestamp_us = 0;          ///< Timestamp in microseconds
    uint32_t frame_id = 0;              ///< Optional sequential frame ID
};
