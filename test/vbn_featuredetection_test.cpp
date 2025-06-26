#include "apps/vbn/FeatureDetector.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    cv::Mat img = cv::imread("../tools/simulated-image.png", cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::cerr << "Could not load image\n";
        return -1;
    }

    ImageFrame input = {img.data, img.cols, img.rows};
    FeatureFrame output;

    FeatureDetector detector;
    if (detector.detect(input, output)) {
        std::cout << "Detected " << output.keypoints.size() << " features\n";
    } else {
        std::cout << "No features detected\n";
    }

    // Draw darker circles to mark LEDs in grayscale
    for (const auto& kp : output.keypoints) {
        cv::circle(
            img,
            cv::Point(kp.x, kp.y),
            50,           // bigger radius to stand out
            cv::Scalar(30), // dark gray for visibility
            2
        );
    }

    cv::imwrite("../tools/annotated.jpg", img);
    std::cout << "Saved grayscale annotated image.\n";

    return 0;
}
