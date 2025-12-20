#include "apps/vbn/FeatureDetector.hpp"
#include "apps/vbn/StaticPoseEstimator.hpp"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <cmath>
#include <iomanip> // for std::setprecision

constexpr double RAD2DEG = 180.0 / M_PI;

int main() {

    // Choose a base name once
    std::string base = "../tools/frame_actual";

    // Build input and output filenames from it
    std::string input_path  = base + ".png";
    std::string output_path = base + "_annotated_spe.jpg";

    // 1) Load test image
    cv::Mat img = cv::imread(input_path, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::cerr << "ERROR: Could not load image\n";
        return -1;
    }
    if (!img.isContinuous()) img = img.clone();

    // 2) Wrap into ImageFrame
    msg::ImageFrame input;
    input.data   = img.data;
    input.width  = img.cols;
    input.height = img.rows;
    input.stride = img.cols;

    vbn::FeatureDetectorConfig det_cfg;
    det_cfg.BIN_THRESH = 100; // Example: adjust threshold if needed
    det_cfg.MIN_BLOB_AREA = 50;
    det_cfg.MAX_BLOB_AREA = 20000;
    det_cfg.PATTERN_MAX_SCORE = 150.0f;
    det_cfg.MAX_OFFSET_SCORE = 0.6f;
    det_cfg.ROI_RADIUS_MARGIN = 2.5f;
    det_cfg.ROI_BORDER_PX = 10;

    msg::FeatureFrame features;
    vbn::FeatureDetector detector(det_cfg);

    auto t0 = std::chrono::high_resolution_clock::now();
    bool fd_ok = detector.detect(input, features);
    auto t1 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::micro> dt = t1 - t0;

    std::cout << "[FD] detect() returned = " << (fd_ok ? "TRUE" : "FALSE") << "\n";
    std::cout << "[FD] LEDs detected = " << static_cast<int>(features.led_count) << "\n";
    std::cout << "[FD] Track state = "
              << (features.state == msg::TrackState::TRACK ? "TRACK" : "LOST")
              << "\n";
    std::cout << "[FD] detect() time = " << dt.count() / 1000.0 << " ms\n";

    if (!fd_ok || features.led_count == 0) {
        std::cerr << "[MAIN] Feature detection failed or no LEDs found. Exiting.\n";
        return 0;
    }

    // 4) Draw FD annotations
    cv::Mat annotated;
    cv::cvtColor(img, annotated, cv::COLOR_GRAY2BGR);

    for (int i = 0; i < static_cast<int>(features.led_count); ++i) {
        const auto& L = features.leds[i];

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

        // index label (or slot_id if you prefer)
        std::string label = std::to_string(i);
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

    // 5) Configure StaticPoseEstimator (SPE)
    vbn::StaticPoseEstimatorConfig spe_cfg;

    // Camera intrinsics (TODO: plug in real calibration)
    spe_cfg.CAM_INTRINSICS.fx = 908.62425565f;                 // [px] placeholder
    spe_cfg.CAM_INTRINSICS.fy = 908.92570486f;                 // [px] placeholder
    spe_cfg.CAM_INTRINSICS.cx = img.cols * 0.5f;        // assume principal point at image centre
    spe_cfg.CAM_INTRINSICS.cy = img.rows * 0.5f;
    // spe_cfg.CAM_INTRINSICS.cx = 643.93436085f;       
    // spe_cfg.CAM_INTRINSICS.cy = 393.45889572f;
    spe_cfg.CAM_INTRINSICS.k1 = 0.0f;
    spe_cfg.CAM_INTRINSICS.k2 = 0.0f;
    spe_cfg.CAM_INTRINSICS.k3 = 0.0f;
    spe_cfg.CAM_INTRINSICS.p1 = 0.0f;
    spe_cfg.CAM_INTRINSICS.p2 = 0.0f;

    // Pattern geometry (TODO: plug in your real D,H in meters)
    spe_cfg.PATTERN_GEOMETRY.PATTERN_RADIUS = 0.010f;   // 1 cm
    spe_cfg.PATTERN_GEOMETRY.PATTERN_OFFSET = 0.010f;   // H = D for analytic v1

    // Algorithm selection + reprojection threshold
    spe_cfg.ALGO = vbn::AlgoType::ANALYTICAL_INNER;
    spe_cfg.MAX_REPROJ_ERROR_PX = 600.0f;                 // from your config

    vbn::StaticPoseEstimator spe(spe_cfg);

    // 6) Run StaticPoseEstimator
    msg::PoseEstimate pose{};

    auto t2 = std::chrono::high_resolution_clock::now();
    bool spe_ok = spe.estimate(features, pose);
    auto t3 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::micro> dt1 = t3 - t2;

    if (!spe_ok || pose.valid == 0) {
        std::cerr << "[SPE] Pose estimation FAILED or marked invalid.\n";
    } else {
        double roll_deg  = pose.roll  * RAD2DEG;
        double pitch_deg = pose.pitch * RAD2DEG;
        double yaw_deg   = pose.yaw   * RAD2DEG;
        double range_cm  = pose.range_m * 100.0; // m → cm

        std::cout << std::fixed << std::setprecision(3); // Set precision for floating-point output

        std::cout << "[SPE] Pose estimation SUCCESS.\n";
        std::cout << "      Azimuth  = " << pose.az  * RAD2DEG << " deg\n";
        std::cout << "      Elevation = " << pose.el  * RAD2DEG << " deg\n";
        std::cout << "      Roll  = " << roll_deg  << " deg\n";
        std::cout << "      Pitch = " << pitch_deg << " deg\n";
        std::cout << "      Yaw   = " << yaw_deg   << " deg\n";
        std::cout << "      Range = " << range_cm  << " cm\n";
        std::cout << "      Reproj RMS = " << pose.reproj_rms_px << " px\n";
    }

    std::cout << "[SPE] estimate() time = " << dt1.count() / 1000.0 << " ms\n";

    // Optional: write pose text on image
    if (spe_ok && pose.valid) {
        double roll_deg  = pose.roll  * RAD2DEG;
        double pitch_deg = pose.pitch * RAD2DEG;
        double yaw_deg   = pose.yaw   * RAD2DEG;
        double range_cm  = pose.range_m * 100.0;

        std::string text1 = "r,p,y[deg]=(" +
            std::to_string(roll_deg) + "," +
            std::to_string(pitch_deg) + "," +
            std::to_string(yaw_deg) + ")";
        std::string text2 = "range=" + std::to_string(range_cm) +
                            " cm, RMS=" + std::to_string(pose.reproj_rms_px) + " px";

        cv::putText(
            annotated,
            text1,
            cv::Point(20, 30),
            cv::FONT_HERSHEY_SIMPLEX,
            1.0,
            cv::Scalar(0, 255, 0),
            1
        );

        cv::putText(
            annotated,
            text2,
            cv::Point(20, 70),
            cv::FONT_HERSHEY_SIMPLEX,
            1.0,
            cv::Scalar(0, 255, 0),
            1
        );

        // Draw crosshairs at image center
        cv::line(annotated, cv::Point(0, annotated.rows/2), cv::Point(annotated.cols - 1, annotated.rows/2), cv::Scalar(255, 255, 255), 1);
        cv::line(annotated, cv::Point(annotated.cols/2, 0), cv::Point(annotated.cols/2, annotated.rows - 1), cv::Scalar(255, 255, 255), 1); 
    }

    // 7) Save annotated image
    cv::imwrite(output_path, annotated);
    std::cout << "Saved annotated image: " << output_path << "\n";

    return 0;
}
