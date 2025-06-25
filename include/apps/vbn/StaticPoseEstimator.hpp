#pragma once
#include "types/FeatureFrame.hpp"
#include "types/PoseEstimate.hpp"

class StaticPoseEstimator {
public:
    virtual bool estimate(const FeatureFrame& features, PoseEstimate& pose) = 0;
    virtual ~StaticPoseEstimator() = default;
};