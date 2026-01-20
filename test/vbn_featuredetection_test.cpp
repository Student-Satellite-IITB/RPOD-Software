#include "apps/vbn/FeatureDetector.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

int main() {

    // Choose a base name once
    std::string base = "../tools/data/cases/sim/tmp_case/image";

    // Build input and output filenames from it
    std::string input_path  = base + ".png";
    std::string output_path = base + "_annotated_fd.jpg";

    cv::Mat img = cv::imread(input_path, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::cerr << "ERROR: Could not load image\n";
        return -1;
    }
    if (!img.isContinuous()) img = img.clone();

    msg::ImageFrame input{};
    input.data         = img.data; // pointer to raw bytes (can represent 8 or 16-bit containers)
    input.width        = static_cast<uint32_t>(img.cols);
    input.height       = static_cast<uint32_t>(img.rows);
    input.stride       = static_cast<uint32_t>(img.step);        // bytes per row
    input.bytes_per_px = static_cast<uint8_t>(img.elemSize1());  // 1 for 8U, 2 for 16U
    input.bit_depth    = static_cast<uint8_t>(8 * img.elemSize1()); // container bit-depth (8 or 16)
    input.bit_shift    = 0; // LSB aligned containers

    vbn::FeatureDetectorConfig det_cfg;
    det_cfg.BIN_THRESH = 250; // Example: adjust threshold if needed
    det_cfg.MIN_BLOB_AREA = 10;
    det_cfg.MAX_BLOB_AREA = 20000;
    det_cfg.PATTERN_MAX_SCORE = 150.0f;
    det_cfg.MAX_OFFSET_SCORE = 10.0f;
    det_cfg.ROI_RADIUS_MARGIN = 2.0f;
    det_cfg.ROI_BORDER_PX = 8;

    msg::FeatureFrame out;
    vbn::FeatureDetector detector(det_cfg);

    auto t0 = std::chrono::high_resolution_clock::now();
    bool ok = detector.detect(input, out);
    auto t1 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::micro> dt = t1 - t0;

    std::cout << "detect() returned = " << (ok ? "TRUE" : "FALSE") << "\n";
    std::cout << "LEDs detected = " << static_cast<int>(out.feat_count) << "\n";
    std::cout << "Track state = "
              << (out.state == msg::TrackState::TRACK ? "TRACK" : "LOST")
              << "\n";
    std::cout << "detect() time = " << dt.count() / 1000.0 << " ms\n";

    
    // Convert to color for annotation
    cv::Mat annotated;
    cv::cvtColor(img, annotated, cv::COLOR_GRAY2BGR);

    // Draw circles + index labels
    for (int i = 0; i < static_cast<int>(out.feat_count); ++i) {
        const auto& L = out.feats[i];

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

    cv::imwrite(output_path, annotated);
    std::cout << "Saved annotated image: " << output_path << "\n";

    return 0;
}
