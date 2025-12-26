#include "apps/vbn/FeatureDetector.hpp"
// #include "apps/vbn/StaticPoseEstimator.hpp"
// #include "apps/vbn/DynamicPoseEstimator.hpp"
#include "apps/core/CommandDataHandler.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

#include "os/rtos.hpp"
#include "apps/queues.hpp"

cv::Mat img;
msg::ImageFrame input;
msg::FeatureFrame output;

// Define the RTOS task objects
// Core tasks

// VBN tasks
Rtos::Task ImageCaptureTask;
Rtos::Task FeatureDetectionTask;
Rtos::Task StaticPoseEstimationTask;
// SOM Communication tasks


int main() {
    // Create tasks
    // ImageCaptureTask.Create("ImageCapture", ImageCapture, nullptr);
    // FeatureDetectionTask.Create("FeatureDetection", vbn::FeatureDetector::Run, nullptr);

    Rtos::SleepMs(1000);  // Allow tasks to initialize
    //commandQueue.send({0x01, {}});  // Send a dummy command to start processing
    //std::cout << "[MAIN] SENT COMMAND!\n";

    // Keep main alive (could add heartbeat log here)
    while (true) {
        std::cout << "[MAIN] HEARTBEAT\n";
        Rtos::SleepMs(1000);  // 1 Hz heartbeat
    }

    return 0;
}
