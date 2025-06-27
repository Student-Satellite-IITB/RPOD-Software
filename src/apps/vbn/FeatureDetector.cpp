#include "apps/vbn/FeatureDetector.hpp"
#include "apps/queues.hpp"
#include <iostream>

FeatureDetector::FeatureDetector() : extractor_() {}

bool FeatureDetector::detect(const ImageFrame& img, FeatureFrame& features) {
    // Platform-agnostic processing could go here, e.g.:
    // Filtering keypoints, sorting, packing metadata
    return extractor_.extract(img, features);
}

// ==============================================
// =========== Feature Detection Task ===========
// ==============================================

// Static method to run the feature detection task
// This method will be called by the RTOS task system
void FeatureDetector::Run(void* arg) {
    FeatureDetector detector;
    FeatureFrame output;

    while (true) {
        ImageFrame input;
        // Wait for an image frame to be available
        imageFrameQueue.receive(input); // Blocking behaviour

        output.keypoints.clear();
        bool success = detector.detect(input, output);

        if (success) {

            std::cout << "[FEAT_DET] SUCCESS: "
                      << output.keypoints.size() << " keypoints\n";

            // Try to send the feature frame to the queue
            // If the queue is full, the frame will be dropped
            if(featureFrameQueue.try_send(output)) {
                std::cout << "[FEAT_DET] FEATURE SENT TO QUEUE\n";
            } else {
                std::cerr << "[FEAT_DET] FAILED TO SEND: QUEUE FULL!\n";
            }

        } else {
            std::cout << "[FEAT_DET] NO FEATURES DETECTED!\n";
        }

        Rtos::SleepMs(200);  // Run task at 5 Hz
    }
}