#include "apps/vbn/FeatureDetector.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

int main() {

    cv::Mat img = cv::imread("../tools/range0_exp200_1.png", cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::cerr << "ERROR: Could not load image\n";
        return -1;
    }
    if (!img.isContinuous()) img = img.clone();

    msg::ImageFrame input;
    input.data   = img.data;
    input.width  = img.cols;
    input.height = img.rows;
    input.stride = img.cols;

    msg::FeatureFrame out;
    vbn::FeatureDetector detector;

    auto t0 = std::chrono::high_resolution_clock::now();
    bool ok = detector.detect(input, out);
    auto t1 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::micro> dt = t1 - t0;

    std::cout << "detect() returned = " << (ok ? "TRUE" : "FALSE") << "\n";
    std::cout << "LEDs detected = " << static_cast<int>(out.led_count) << "\n";
    std::cout << "Track state = "
              << (out.state == msg::TrackState::TRACK ? "TRACK" : "LOST")
              << "\n";
    std::cout << "detect() time = " << dt.count() / 1000.0 << " ms\n";

    // Convert to color for annotation
    cv::Mat annotated;
    cv::cvtColor(img, annotated, cv::COLOR_GRAY2BGR);

    // Draw circles + index labels
    for (int i = 0; i < static_cast<int>(out.led_count); ++i) {
        const auto& L = out.leds[i];

        cv::Point pt(static_cast<int>(L.u_px),
                     static_cast<int>(L.v_px));

        // red circle
        cv::circle(
            annotated,
            pt,
            20,
            cv::Scalar(0, 0, 255),  // BGR
            2
        );

        cv::Scalar color(0, 255, 0); 
        cv::drawMarker(annotated, pt, color, cv::MARKER_CROSS, 50, 2);

        // green text: LED index or slot_id
        std::string label = std::to_string(i); // or std::to_string(L.slot_id)
        cv::putText(
            annotated,
            label,
            pt + cv::Point(5, -5),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(0, 255, 0),
            1
        );
    }

    cv::imwrite("../tools/range0_exp200_1_annotated.jpg", annotated);
    std::cout << "Saved annotated image: ../tools/annotated.jpg\n";

    return 0;
}
