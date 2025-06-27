#include "apps/vbn/FeatureDetector.hpp"
#include "apps/vbn/StaticPoseEstimator.hpp"
#include "apps/core/CommandDataHandler.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "os/rtos.hpp"
#include "apps/queues.hpp"

cv::Mat img;
ImageFrame input;
FeatureFrame output;

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


// === Image Capture Task Loop ===
// to be replaced by camera driver interface
void ImageCapture(void* arg) {
    // Load the image
    img = cv::imread("../tools/simulated-image.png", cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::cerr << "Could not load image\n";
        return;
    }

    ImageFrame frame = {img.data, img.cols, img.rows};

    while (true) {
        // Simulate image capture by using the static image

        // try_send to the queue to ensure non-blocking behavior
        // image frames will be dropped if the queue is full
        // to avoid processing stale data
        if(imageFrameQueue.try_send(frame)) {
            std::cout << "[CAMERA] IMAGE SENT TO QUEUE\n";
        } else {
            std::cerr << "[CAMERA] FAILED: QUEUE FULL!\n";
        }
        Rtos::SleepMs(200);  // 5 Hz

    }      
}


int main() {
    // Create tasks
    ImageCaptureTask.Create("ImageCapture", ImageCapture, nullptr);
    FeatureDetectionTask.Create("FeatureDetection", FeatureDetector::Run, nullptr);
    StaticPoseEstimationTask.Create("StaticPoseEstimation", StaticPoseEstimator::Run, nullptr);
    CommandHandlerTask.Create("CommandDataHandler", CommandDataHandler::CommandHandlerRun, nullptr);
    TelemetryHandlerTask.Create("TelemetryHandler", CommandDataHandler::TelemetryHandlerRun, nullptr);

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
