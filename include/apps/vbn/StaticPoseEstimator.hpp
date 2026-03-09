// StaticPoseEstimator.hpp

#pragma once
#include <array>
#include <cstdint>

#include "msg/FeatureFrame.hpp"
#include "msg/PoseEstimate.hpp"

namespace vbn {

struct CameraIntrinsics {
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

struct PatternGeometry {
    float PATTERN_RADIUS = 0.050f;
    float PATTERN_OFFSET = 0.020f;
    
    std::array<float, 15> P_PINV = {};
};

enum class AlgoType : uint8_t {
    ANALYTICAL_INNER = 0,
    ANALYTICAL_GENERIC = 1,
};

// Forward declaration
// Defined in .cpp
struct Pose;

struct StaticPoseEstimatorConfig {
    CameraIntrinsics CAM_INTRINSICS{};
    PatternGeometry PATTERN_GEOMETRY{};

    AlgoType ALGO = AlgoType::ANALYTICAL_INNER;

    float MAX_REPROJ_ERROR_PX = 100.0f; // Maximum acceptable reprojection error in pixels
    // Intentionally permissive for initial characterisation
};

class StaticPoseEstimator {
public:
    explicit StaticPoseEstimator(const StaticPoseEstimatorConfig& cfg = {});
    void setConfig(const StaticPoseEstimatorConfig& cfg);

    // to retrieve current configuration of the estimator maybe for TLM
    const StaticPoseEstimatorConfig& getConfig() const { return m_cfg; }

    enum class Status : uint8_t {
        OK = 0,
        INVALID_FEATURE_FRAME,
        PACK_LED_FAIL,
        LOS_FAIL,
        POSE_FAIL,
        REPROJECTION_FAIL,
    };

    Status lastStatus() const { return m_status; }
    static const char* StatusStr(Status s);

    // Core API: consume one FeatureFrame, produce one PoseEstimate.
    bool estimate(const msg::FeatureFrame& feature_frame, msg::PoseEstimate& out);

private:
    StaticPoseEstimatorConfig m_cfg{};

    using InnerPatternLeds = std::array<msg::Feature, 5>;

    static constexpr std::size_t MAX_OUTER_LEDS = msg::MAX_FEATS;
    using OuterPatternLeds = std::array<msg::Feature, MAX_OUTER_LEDS>;

    struct PackedLeds {
        InnerPatternLeds inner{};
        std::size_t inner_count = 0;

        OuterPatternLeds outer{};
        std::size_t outer_count = 0;
    };

    bool packetLeds(const msg::FeatureFrame& feature_frame, PackedLeds& packed);

    void transformLedstoPPF(PackedLeds& packed) const;
    void undistortLedsPPF(PackedLeds& packed) const;

    bool computeLosAngles(const PackedLeds& packed, float& az, float& el) const;

    bool estimatePose(const PackedLeds& packed, float az, float el, Pose& pose) const;

    bool estimatePoseAnalyticalInner(const PackedLeds& packed, float az, float el, Pose& pose) const;

    bool genericAnalyticalPose(const PackedLeds& packed, Pose& pose) const;

    float evaluateReprojectionError(const PackedLeds& packed, const Pose& pose);

private:
    Status m_status = Status::OK;
};

} // namespace vbn
