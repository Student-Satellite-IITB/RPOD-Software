#include "vbn/FeatureDetector.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "os/rtos.hpp"

cv::Mat img;
ImageFrame input;
FeatureFrame output;

// Define the RTOS task object
Rtos::Task FeatureDetectionTask;

// === Feature Detection Task Loop ===
void FeatureDetection(void* arg) {
    FeatureDetector detector;

    while (true) {
        output.keypoints.clear();
        bool success = detector.detect(input, output);

        if (success) {
            std::cout << "[FeatureDetection] Success: "
                      << output.keypoints.size() << " keypoints\n";

        } else {
            std::cout << "[FeatureDetection] No features detected\n";
        }

        Rtos::SleepMs(200);  // Run task at 5 Hz
    }
}

int main() {
    // Load the image
    img = cv::imread("../tools/simulated-image.png", cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::cerr << "Could not load image\n";
        return -1;
    }

    input = {img.data, img.cols, img.rows};

    // Create feature detection task
    FeatureDetectionTask.Create("FeatureDetection", FeatureDetection, nullptr);

    // Keep main alive (could add heartbeat log here)
    while (true) {
        Rtos::SleepMs(10000);
    }

    return 0;
}
