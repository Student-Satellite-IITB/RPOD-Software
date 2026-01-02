#include "apps/vbn/FeatureDetector.hpp"
#include "apps/vbn/StaticPoseEstimator.hpp"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <cmath>
#include <iomanip> // for std::setprecision
#include <fstream>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;

constexpr double RAD2DEG = 180.0 / M_PI;

int main(int argc, char** argv) {
    // -----------------------------
    // Usage:
    //   ./vbn_staticposeestimation_test <case_dir>
    //
    // Example:
    //   ./vbn_staticposeestimation_test ../tools/data/cases/sim/tmp_case
    // -----------------------------

    fs::path case_dir = (argc >= 2) ? fs::path(argv[1]) : fs::path("../tools/data/cases/sim/tmp_case");

    fs::path input_path   = case_dir / "image.png";
    fs::path output_path  = case_dir / "image_annotated_spe.jpg";
    fs::path results_path = case_dir / "results.txt";

    std::string case_id = case_dir.filename().string();
    if (case_id.empty()) case_id = "CASE_UNKNOWN";

    // -----------------------------
    // 1) Load test image (keep container: 8-bit or 16-bit)
    // -----------------------------
    
    //cv::Mat img = cv::imread(input_path, cv::IMREAD_GRAYSCALE);
    // cv::IMREAD_UNCHANGED keeps image size unchanged
    cv::Mat img = cv::imread(input_path, cv::IMREAD_UNCHANGED);

    // Hard-coded for now based on setting in simulator
    int BIT_DEPTH = 10;
    
    if (img.empty()) {
        std::cerr << "ERROR: Could not load image: " << input_path.string() << "\n";
        return -1;
    }
    if (!img.isContinuous()) img = img.clone();

    if (img.channels() != 1) {
        std::cerr << "ERROR: Expected 1-channel grayscale image. Got channels=" << img.channels() << "\n";
        return -1;
    }

    // -----------------------------
    // 2) Wrap into ImageFrame
    // -----------------------------

    msg::ImageFrame input;
    input.data   = img.data;
    input.width  = static_cast<uint32_t>(img.cols);
    input.height = static_cast<uint32_t>(img.rows);
    input.stride = static_cast<uint32_t>(img.step);  // bytes per row
    input.bytes_per_px = static_cast<uint8_t>(img.elemSize1()); // 1 or 2
    input.bit_depth    = static_cast<uint8_t>(BIT_DEPTH); // Number of meaningful bits in each pixel sample.
    // Examples: RAW8 -> 8, RAW10 -> 10, RAW12 -> 12, RAW16 -> 16.
    // Note: RAW10 may be stored in 16-bit containers (bytes_per_px=2, bit_depth=10).
    // input.bit_shift = 0; // LSB aligned containers
    input.bit_shift = 6; // RAW10 MSB-aligned in uint16 container (bits [15..6])

    // -----------------------------
    // Open results file early (so failures still log metadata)
    // -----------------------------
    std::ofstream rf(results_path.string());
    if (!rf.is_open()) {
        std::cerr << "ERROR: Could not open results file: " << results_path.string() << "\n";
        return -1;
    }

    // Basic metadata
    rf << "case_id = " << case_id << "\n";
    rf << "case_dir = " << case_dir.string() << "\n";
    rf << "input_image = " << input_path.string() << "\n";
    rf << "opencv_type = " << img.type() << "\n";
    rf << "channels = " << img.channels() << "\n";
    rf << "width = " << img.cols << "\n";
    rf << "height = " << img.rows << "\n";
    rf << "stride_bytes = " << img.step << "\n";
    rf << "bytes_per_px = " << static_cast<int>(input.bytes_per_px) << "\n";
    rf << "container_bit_depth = " << static_cast<int>(input.bit_depth) << "\n";

    // -----------------------------
    // 3) Feature Detector config + run
    // -----------------------------

    vbn::FeatureDetectorConfig det_cfg;
    // det_cfg.BIN_THRESH = 20000; // For 16-Bit Image
    // det_cfg.BIN_THRESH = 100; // For 8-Bit Image
    det_cfg.BIN_THRESH = 250; // For 10-Bit Image
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

        // Log FD results
    rf << "\n[FD]\n";
    rf << "fd_ok = " << (fd_ok ? 1 : 0) << "\n";
    rf << "fd_track_state = " << (features.state == msg::TrackState::TRACK ? "TRACK" : "LOST") << "\n";
    rf << "fd_led_count = " << static_cast<int>(features.led_count) << "\n";
    rf << "fd_time_ms = " << dt.count() / 1000.0 << "\n";
    rf << "fd_bin_thresh = " << det_cfg.BIN_THRESH << "\n";
    rf << "fd_min_blob_area = " << det_cfg.MIN_BLOB_AREA << "\n";
    rf << "fd_max_blob_area = " << det_cfg.MAX_BLOB_AREA << "\n";

    for (int i = 0; i < static_cast<int>(features.led_count); ++i) {
        const auto& L = features.leds[i];
        rf << "fd_led" << i << ".id = " << static_cast<int>(L.slot_id) << "\n";
        rf << "fd_led" << i << ".u_px = " << L.u_px << "\n";
        rf << "fd_led" << i << ".v_px = " << L.v_px << "\n";
        rf << "fd_led" << i << ".area = " << L.area << "\n";
    }

    // -----------------------------
    // 4) Draw FD annotations (KEEP your style)
    // IMPORTANT: cvtColor(CV_16UC1 -> BGR) can be finicky, so convert to 8U only for annotation.
    // This does NOT affect the FD/SPE input path.
    // -----------------------------
    cv::Mat annotated8;

    if (img.type() == CV_16UC1) {
        // Convert 16-bit container -> native DN (LSB-aligned) into dn16
        cv::Mat dn16(img.size(), CV_16UC1);

        const int shift = static_cast<int>(input.bit_shift);  // RAW10 MSB-aligned-in-16 => 6, LSB-aligned => 0

        // Mask to keep only "bit_depth" meaningful bits after shifting.
        const uint16_t mask =
            (input.bit_depth > 0 && input.bit_depth < 16)
                ? static_cast<uint16_t>((1u << input.bit_depth) - 1u)
                : static_cast<uint16_t>(0xFFFF);

        for (int y = 0; y < img.rows; ++y) {
            const uint16_t* src = img.ptr<uint16_t>(y);
            uint16_t* dst       = dn16.ptr<uint16_t>(y);
            for (int x = 0; x < img.cols; ++x) {
                dst[x] = static_cast<uint16_t>((src[x] >> shift) & mask);
            }
        }

        // Now dn16 is native DN in [0 .. (2^bit_depth - 1)].
        // Scale DN -> 8-bit for visualization.
        const double denom = double((input.bit_depth >= 1 && input.bit_depth <= 16)
                                    ? ((1u << input.bit_depth) - 1u)
                                    : 65535u);
        const double scale = 255.0 / denom;

        dn16.convertTo(annotated8, CV_8U, scale);

    } else if (img.type() == CV_8UC1) {
        annotated8 = img; // already 8-bit grayscale
    } else {
        // Optional: handle unexpected types robustly
        cv::Mat tmp;
        img.convertTo(tmp, CV_8U);
        annotated8 = tmp;
    }

    cv::Mat annotated;
    cv::cvtColor(annotated8, annotated, cv::COLOR_GRAY2BGR);


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

    if (!fd_ok || features.led_count == 0) {
        std::cerr << "[MAIN] Feature detection failed or no LEDs found. Exiting.\n";

        // Still save annotated + results for debugging
        cv::imwrite(output_path.string(), annotated);
        std::cout << "Saved annotated image: " << output_path.string() << "\n";
        std::cout << "Saved results: " << results_path.string() << "\n";

        rf << "\n[SPE]\n";
        rf << "spe_ok = 0\n";
        rf << "spe_valid = 0\n";
        return 0;
    }

    // -----------------------------
    // 5) Configure StaticPoseEstimator (SPE)
    // -----------------------------

    vbn::StaticPoseEstimatorConfig spe_cfg;

    // Camera intrinsics (TODO: plug in real calibration)
    spe_cfg.CAM_INTRINSICS.fx = 908.62425565f;          // [px] placeholder
    spe_cfg.CAM_INTRINSICS.fy = 908.92570486f;          // [px] placeholder
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
    //spe_cfg.ALGO = vbn::AlgoType::ANALYTICAL_INNER;
    spe_cfg.ALGO = vbn::AlgoType::ANALYTICAL_GENERIC;
    spe_cfg.MAX_REPROJ_ERROR_PX = 600.0f;                 // from your config

    vbn::StaticPoseEstimator spe(spe_cfg);

    // Run StaticPoseEstimator
    msg::PoseEstimate pose{};

    auto t2 = std::chrono::high_resolution_clock::now();
    bool spe_ok = spe.estimate(features, pose);
    auto t3 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::micro> dt1 = t3 - t2;

    std::cout << "[SPE] estimate() time = " << dt1.count() / 1000.0 << " ms\n";

    // Log SPE results
    rf << "\n[SPE]\n";
    rf << "spe_ok = " << (spe_ok ? 1 : 0) << "\n";
    rf << "spe_valid = " << static_cast<int>(pose.valid) << "\n";
    rf << "spe_time_ms = " << dt1.count() / 1000.0 << "\n";
    rf << "spe_reproj_rms_px = " << pose.reproj_rms_px << "\n";

    if (!spe_ok || pose.valid == 0) {
        std::cerr << "[SPE] Pose estimation FAILED or marked invalid.\n";
    } else {
        auto q = pose.q_C_P;
        auto t = pose.t_CbyP;
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
        std::cout << "      Quaternion = [ "<< q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] <<"]\n";
        std::cout << "      Reproj RMS = " << pose.reproj_rms_px << " px\n";

        // Save core outputs to results.txt
        rf << std::fixed << std::setprecision(9);
        rf << "az_deg = " << (pose.az * RAD2DEG) << "\n";
        rf << "el_deg = " << (pose.el * RAD2DEG) << "\n";
        rf << "roll_deg = " << roll_deg << "\n";
        rf << "pitch_deg = " << pitch_deg << "\n";
        rf << "yaw_deg = " << yaw_deg << "\n";
        rf << "range_m = " << pose.range_m << "\n";
        rf << "R_C_P_rowmajor = ";
        for (int i = 0; i < 9; ++i) {
            rf << pose.R_C_P[i] << (i == 8 ? '\n' : ' ');
        }
        rf << "q_C_P = " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << "\n";
        rf << "t_Cbyp = " << t[0] << " " << t[1] << " " << t[2] << "\n"; 
    }

    // Optional: write pose text on image
    if (spe_ok && pose.valid) {
        double roll_deg  = pose.roll  * RAD2DEG;
        double pitch_deg = pose.pitch * RAD2DEG;
        double yaw_deg   = pose.yaw   * RAD2DEG;
        double range_cm  = pose.range_m * 100.0;

        std::string text1 = "R, P, Y [deg]=(" +
            std::to_string(roll_deg) + "," +
            std::to_string(pitch_deg) + "," +
            std::to_string(yaw_deg) + ")";
        std::string text2 = "range = " + std::to_string(range_cm) +
                            " cm, RMS = " + std::to_string(pose.reproj_rms_px) + " px";

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

    // -----------------------------
    // 6) Save annotated image + done
    // -----------------------------
    cv::imwrite(output_path.string(), annotated);
    std::cout << "Saved annotated image: " << output_path.string() << "\n";
    std::cout << "Saved results: " << results_path.string() << "\n";

    return 0;
}
