#pragma once
#include <cstdint>
#include <array>
#include "msg/FeatureFrame.hpp"
#include "msg/PoseEstimate.hpp"

namespace vbn{

struct CameraIntrinsics{
    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    float s = 0.0f; // skew

    float k1 = 0.0f; // radial distortion
    float k2 = 0.0f;
    float k3 = 0.0f;

    float p1 = 0.0f; // tangential distortion
    float p2 = 0.0f;
};

struct PatternGeometry{
    float PATTERN_RADIUS = 0.050f;
    float PATTERN_OFFSET = 0.020f;

    float D = PATTERN_RADIUS;
    float H = PATTERN_OFFSET;

    std::array<float, 15> P_PINV ={
        0.0f, -1/(2*D), 0.0f, 1/(2*D), 0.0f,
        -1/(2*D), 0.0f, 1/(2*D), 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, -1/H
    };
};

enum class AlgoType: uint8_t{
    ANALYTICAL_INNER = 0,
    ANALYTICAL_GENERIC = 1,
    // Add more algorithms as needed
};

// Forward declaration
// Defined in .cpp
struct Pose;

struct StaticPoseEstimatorConfig{

    CameraIntrinsics CAM_INTRINSICS{};
    PatternGeometry PATTERN_GEOMETRY{};

    AlgoType ALGO = AlgoType::ANALYTICAL_INNER;

    float MAX_REPROJ_ERROR_PX = 100.0f; // Maximum acceptable reprojection error in pixels
};

class StaticPoseEstimator{
public:
    explicit StaticPoseEstimator(const StaticPoseEstimatorConfig& cfg = {});
    void setConfig(const StaticPoseEstimatorConfig& cfg);

    // inline implemneted get configuration
    // to retrieve current configuration of the estimator maybe for TLM
    const StaticPoseEstimatorConfig& getConfig() const { return m_cfg; }

    // Core API: consume one FeatureFrame, produce one PoseEstimate.
    bool estimate(const msg::FeatureFrame& feature_frame, msg::PoseEstimate& out);

private:

    StaticPoseEstimatorConfig m_cfg{};

    // LED PACKING
    using InnerPatternLeds = std::array<msg::Led2D, 5>;

    // OUTER: for now just collect them; no geometry enforced yet
    static constexpr std::size_t MAX_OUTER_LEDS = msg::MAX_LEDS;
    using OuterPatternLeds = std::array<msg::Led2D, MAX_OUTER_LEDS>;

    struct PackedLeds {
        InnerPatternLeds inner{};
        std::size_t inner_count = 0;  // unique inner slots filled

        OuterPatternLeds outer{};
        std::size_t outer_count = 0;  // number of OUTER LEDs stored
    };

    bool packetLeds(const msg::FeatureFrame& feature_frame, PackedLeds& packed);

    // FRAME TRANSFORMATION AND UNDISTORTION
    void transformLedstoPPF(PackedLeds& packed) const;
    void undistortLedsPPF(PackedLeds& packed) const; // currently a no-op

    // LOS ANGLE COMPUTATION
    bool computeLosAngles(const PackedLeds& packed,
                         float& az,
                         float& el) const;

    // POSE ESTIMATION ALGORITHMS

    // Pose estimation dispatcher
    bool estimatePose(const PackedLeds& packed, Pose& pose,
                      float az, float el,
                      float& roll, float& pitch, float& yaw,
                      float& range_m) const;

    // Analytical pose estimation algorithm for INNER pattern
    bool estimatePoseAnalyticalInner(const PackedLeds& packed,
                                     float az, float el,
                                     float& roll, float& pitch, float& yaw,
                                     float& range_m) const;
    // Generic Analytical Pose Estimation algorithm
    bool genericAnalyticalPose(const PackedLeds& packed, float az, float el, Pose& pose) const;

    // MEASUREMENT CONFIDENCE EVALUATION
    float evaluateReprojectionError(const PackedLeds& packed,
                                   float roll, float pitch, float yaw,
                                   float range_m);


};

}