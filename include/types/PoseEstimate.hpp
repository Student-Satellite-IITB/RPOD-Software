#pragma once
#include <array>
#include <cstdint>

/// @brief 6-DOF pose estimate output by the RPOD vision system.
struct PoseEstimate {
    // Position in meters (in target or inertial frame depending on convention)
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    // Orientation as a quaternion (w, x, y, z)
    std::array<float, 4> attitude_quat = {1.0f, 0.0f, 0.0f, 0.0f};

    // Optional: Pose covariance (diagonal only for embedded constraints)
    std::array<float, 6> covariance_diag = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // Time at which this estimate is valid (microseconds)
    uint64_t timestamp_us = 0;

    // Validity flag (e.g., set false if tracking was lost)
    bool valid = true;
};
