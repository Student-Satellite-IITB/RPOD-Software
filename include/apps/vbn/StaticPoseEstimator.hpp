#pragma once
#include "types.hpp"

class StaticPoseEstimator {
public:
    StaticPoseEstimator();
    bool estimate(const msg::FeatureFrame& features, PoseEstimate& pose);
    static void Run(void* arg);  // Static method to run the pose estimation task

private:

    
};