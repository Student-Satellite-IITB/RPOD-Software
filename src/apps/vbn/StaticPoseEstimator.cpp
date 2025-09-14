#include "apps/vbn/StaticPoseEstimator.hpp"
#include "apps/queues.hpp"
#include <iostream>

StaticPoseEstimator::StaticPoseEstimator(){
    // Constructor can initialize any required resources
}

bool StaticPoseEstimator::estimate(const msg::FeatureFrame& features, PoseEstimate& pose) {
    // Placeholder for actual pose estimation logic
    // For now, we just set a dummy pose
    pose.x = 0.0f;
    pose.y = 0.0f;
    pose.z = 0.0f;
    pose.roll = 0.0f;   // Roll in radians
    pose.pitch = 0.0f;  // Pitch in radians
    pose.yaw = 0.0f;    // Yaw in radians
    pose.valid = true; // Assume valid for now

    return true; // Indicate success
}

void StaticPoseEstimator::Run(void* arg) {
    StaticPoseEstimator estimator;
    msg::FeatureFrame features;
    PoseEstimate pose;

    while (true) {
        // Wait for a feature frame to be available
        featureFrameQueue.receive(features); // Blocking behaviour\
        // Note to self: Implementation changes if recieve function has timeout enabled
        pose = PoseEstimate(); // Reset pose
        if(estimator.estimate(features, pose)){
            if(poseEstimateQueue.try_send(pose)) {
                std::cout << "[SPE] POSE ESTIMATE SENT TO QUEUE\n";
                if(poseEstimateQueue.wasLastSendOverwritten()) {
                    std::cerr << "[SPE] POSE ESTIMATE QUEUE OVERWRITTEN!\n";
                }
            } else {
                std::cerr << "[SPE] FAILED TO SEND POSE: QUEUE FULL!\n";
            }
        }
        else {
            std::cerr << "[SPE] POSE ESTIMATION FAILED!\n";
        }

        Rtos::SleepMs(200);  // Run task at 5 Hz
    }
}