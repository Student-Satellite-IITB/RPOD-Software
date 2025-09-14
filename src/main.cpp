// #include "apps/vbn/FeatureDetector.hpp"
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
Rtos::Task StateManagerTask;
Rtos::Task CommandHandlerTask;
Rtos::Task TelemetryHandlerTask;
Rtos::Task HeartbeatTask;
// VBN tasks
Rtos::Task ImageCaptureTask;
Rtos::Task FeatureDetectionTask;
Rtos::Task StaticPoseEstimationTask;
Rtos::Task DynamicPoseEstimationTask;
// Control tasks
Rtos::Task CaptureControllerTask;

static inline uint64_t now_us() {
    using clock = std::chrono::steady_clock;
    return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(
        clock::now().time_since_epoch()).count();
}
static uint32_t g_frame_id = 0;

// === Image Capture Task Loop ===
// to be replaced by camera driver interface
void ImageCapture(void* arg) {
    // Load the image
    img = cv::imread("../tools/simulated-image.png", cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::cerr << "Could not load image\n";
        return;
    }
    if (img.type() != CV_8UC1) {
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        img = gray; // keep a GRAY8 copy alive for the loop
    }

    msg::ImageFrame frame{};
    frame.data         = img.data;
    frame.width        = (uint32_t)img.cols;
    frame.height       = (uint32_t)img.rows;
    frame.stride       = (uint32_t)img.step[0];      // bytes per row
    frame.format       = msg::PixelFormat::GRAY8;    // new field

    while (true) {
        // Simulate image capture by using the static image
        
        // Update dynamic fields each loop
        frame.t_exp_end_us = now_us();               // mock exposure-end time
        frame.frame_id     = g_frame_id++;

        // try_send to the queue to ensure non-blocking behavior
        // image frames will be dropped if the queue is full
        // to avoid processing stale data
        if (imageFrameQueue.try_send(frame)) {
            std::cout << "[CAMERA] IMAGE SENT id=" << frame.frame_id << "\n";
        } else {
            std::cerr << "[CAMERA] QUEUE FULL, DROP id=" << frame.frame_id << "\n";
        }
        Rtos::SleepMs(200);  // 5 Hz

    }      
}


int main() {
    // Create tasks
    ImageCaptureTask.Create("ImageCapture", ImageCapture, nullptr);
    // FeatureDetectionTask.Create("FeatureDetection", FeatureDetector::Run, nullptr);
    // StaticPoseEstimationTask.Create("StaticPoseEstimation", StaticPoseEstimator::Run, nullptr);
    // DynamicPoseEstimationTask.Create("DynamicPoseEstimation", DynamicPoseEstimator::Run, nullptr);
    //CommandHandlerTask.Create("CommandDataHandler", CommandDataHandler::CommandHandlerRun, nullptr);
    //TelemetryHandlerTask.Create("TelemetryHandler", CommandDataHandler::TelemetryHandlerRun, nullptr);

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
