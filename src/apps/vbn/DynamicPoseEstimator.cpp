// #include "apps/vbn/DynamicPoseEstimator.hpp"
// #include "apps/queues.hpp"
// #include <iostream>

// DynamicPoseEstimator::DynamicPoseEstimator(){
//     // Constructor can initialize any required resources
// }

// bool DynamicPoseEstimator::estimate(const PoseEstimate* measurement, PoseEstimate& pose){

//     // For now, we just set a dummy pose
//     pose.x = 0.0f;
//     pose.y = 0.0f;
//     pose.z = 0.0f;
//     pose.roll = 0.0f;   // Roll in radians
//     pose.pitch = 0.0f;  // Pitch in radians
//     pose.yaw = 0.0f;    // Yaw in radians
//     pose.valid = true; // Assume valid for now

//     return true;
// }

// void DynamicPoseEstimator::Run(void* arg){
//     DynamicPoseEstimator estimator;
//     PoseEstimate pose;
//     PoseEstimate filtered_pose;
//     bool poseAvailable = false;
    
//     //Initialising dynamic pose estimator filter
//     poseEstimateQueue.receive(pose);
//     estimator.estimate(&pose, filtered_pose);
//     std::cout << "[DPE] DPE INITIALISED\n";

//     while(true){
//         if(poseEstimateQueue.try_receive(pose)){
//             poseAvailable = true;
//         }
//         filtered_pose = PoseEstimate(); //Reset variable
//         if(poseAvailable) {
//             estimator.estimate(&pose, filtered_pose);
//         } else {
//             estimator.estimate(nullptr, filtered_pose);
//         }
//         std::cout<<"[DPE] Filtered Pose Estimate\n";

//         Rtos::SleepMs(100);
//     }
// }