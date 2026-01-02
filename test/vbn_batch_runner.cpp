#include "apps/vbn/FeatureDetector.hpp"
#include "apps/vbn/StaticPoseEstimator.hpp"

#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <string>

namespace fs = std::filesystem;
constexpr double RAD2DEG = 180.0 / M_PI;

struct Args {
    fs::path cases_root;
    std::string image_name = "image.png";
    bool write_annotated = false;
    uint16_t bin_thresh = 250; // native DN units (e.g. 250 for 10-bit data)
};

static void print_usage(const char* exe) {
    std::cerr
        << "Usage:\n"
        << "  " << exe << " --cases_root <DIR> [--image_name image.png] [--bin_thresh N] [--write_annotated 0|1]\n"
        << "\nExample:\n"
        << "  " << exe << " --cases_root ../tools/data/cases/sim/sweep_range_20251225_0500 --bin_thresh 250 --write_annotated 0\n";
}

static bool parse_args(int argc, char** argv, Args& out) {
    if (argc < 3) return false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];

        auto need_value = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << "\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (a == "--cases_root") {
            const char* v = need_value("--cases_root");
            if (!v) return false;
            out.cases_root = fs::path(v);
        } else if (a == "--image_name") {
            const char* v = need_value("--image_name");
            if (!v) return false;
            out.image_name = v;
        } else if (a == "--bin_thresh") {
            const char* v = need_value("--bin_thresh");
            if (!v) return false;
            out.bin_thresh = static_cast<uint16_t>(std::stoi(v));
        } else if (a == "--write_annotated") {
            const char* v = need_value("--write_annotated");
            if (!v) return false;
            out.write_annotated = (std::stoi(v) != 0);
        } else {
            std::cerr << "Unknown arg: " << a << "\n";
            return false;
        }
    }

    if (out.cases_root.empty()) return false;
    return true;
}

static cv::Mat make_vis_gray_u8(const cv::Mat& img) {
    // For annotation/debug image only (FD uses original img data).
    if (img.depth() == CV_8U) return img;
    if (img.depth() == CV_16U) {
        cv::Mat vis;
        // Simple MSB mapping: 16-bit -> 8-bit by /256. Good enough for debugging.
        img.convertTo(vis, CV_8U, 1.0 / 256.0);
        return vis;
    }
    // Fallback: convert anything else to 8U (rare)
    cv::Mat vis;
    img.convertTo(vis, CV_8U);
    return vis;
}

static bool run_one_case(const fs::path& case_dir, const Args& args) {
    const fs::path input_path  = case_dir / args.image_name;
    const fs::path results_path = case_dir / "results.txt";
    const fs::path annotated_path = case_dir / "image_annotated_spe.jpg";

    if (!fs::exists(input_path)) {
        // Not a case folder (or missing image) -> skip silently
        return false;
    }

    cv::Mat img = cv::imread(input_path.string(), cv::IMREAD_UNCHANGED);
    if (img.empty()) {
        std::cerr << "[CASE] " << case_dir.string() << " : ERROR loading image\n";
        return true; // "processed", but failed
    }
    if (!img.isContinuous()) img = img.clone();

    // Expect grayscale (1 channel). If not, still try to proceed by converting.
    if (img.channels() != 1) {
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        img = gray;
        if (!img.isContinuous()) img = img.clone();
    }

    // Wrap into ImageFrame (stride in BYTES per row)
    msg::ImageFrame input{};
    input.data         = img.data; // pointer to raw bytes (can represent 8 or 16-bit containers)
    input.width        = static_cast<uint32_t>(img.cols);
    input.height       = static_cast<uint32_t>(img.rows);
    input.stride       = static_cast<uint32_t>(img.step);        // bytes per row
    input.bytes_per_px = static_cast<uint8_t>(img.elemSize1());  // 1 for 8U, 2 for 16U
    input.bit_depth    = static_cast<uint8_t>(8 * img.elemSize1()); // container bit-depth (8 or 16)
    input.bit_shift    = 0; // LSB aligned containers

    // Configure FD
    vbn::FeatureDetectorConfig det_cfg;
    det_cfg.BIN_THRESH = args.bin_thresh;
    det_cfg.MIN_BLOB_AREA = 50;
    det_cfg.MAX_BLOB_AREA = 20000;
    det_cfg.PATTERN_MAX_SCORE = 150.0f;
    det_cfg.MAX_OFFSET_SCORE = 0.6f;
    det_cfg.ROI_RADIUS_MARGIN = 2.5f;
    det_cfg.ROI_BORDER_PX = 10;

    vbn::FeatureDetector detector(det_cfg);
    msg::FeatureFrame features{};

    auto t0 = std::chrono::high_resolution_clock::now();
    const bool fd_ok = detector.detect(input, features);
    auto t1 = std::chrono::high_resolution_clock::now();
    const double fd_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Configure SPE
    vbn::StaticPoseEstimatorConfig spe_cfg;
    spe_cfg.CAM_INTRINSICS.fx = 908.62425565f;
    spe_cfg.CAM_INTRINSICS.fy = 908.92570486f;
    spe_cfg.CAM_INTRINSICS.cx = img.cols * 0.5f;
    spe_cfg.CAM_INTRINSICS.cy = img.rows * 0.5f;
    spe_cfg.CAM_INTRINSICS.k1 = 0.0f;
    spe_cfg.CAM_INTRINSICS.k2 = 0.0f;
    spe_cfg.CAM_INTRINSICS.k3 = 0.0f;
    spe_cfg.CAM_INTRINSICS.p1 = 0.0f;
    spe_cfg.CAM_INTRINSICS.p2 = 0.0f;

    spe_cfg.PATTERN_GEOMETRY.PATTERN_RADIUS = 0.010f;
    spe_cfg.PATTERN_GEOMETRY.PATTERN_OFFSET = 0.010f;

    spe_cfg.ALGO = vbn::AlgoType::ANALYTICAL_GENERIC;
    spe_cfg.MAX_REPROJ_ERROR_PX = 600.0f;

    vbn::StaticPoseEstimator spe(spe_cfg);
    msg::PoseEstimate pose{};

    auto t2 = std::chrono::high_resolution_clock::now();
    const bool spe_ok = (fd_ok && features.led_count > 0) ? spe.estimate(features, pose) : false;
    auto t3 = std::chrono::high_resolution_clock::now();
    const double spe_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();

    // Write results.txt (RAII closes file automatically when it goes out of scope)
    {
        std::ofstream rf(results_path);
        rf << "case_id = " << case_dir.filename().string() << "\n";
        rf << "case_dir = " << case_dir.string() << "\n";
        rf << "input_image = " << input_path.string() << "\n";
        rf << "opencv_type = " << img.type() << "\n";
        rf << "channels = " << img.channels() << "\n";
        rf << "width = " << img.cols << "\n";
        rf << "height = " << img.rows << "\n";
        rf << "stride_bytes = " << img.step << "\n";
        rf << "bytes_per_px = " << static_cast<int>(input.bytes_per_px) << "\n";
        rf << "container_bit_depth = " << static_cast<int>(input.bit_depth) << "\n\n";

        rf << "[FD]\n";
        rf << "fd_ok = " << (fd_ok ? 1 : 0) << "\n";
        rf << "fd_track_state = " << (features.state == msg::TrackState::TRACK ? "TRACK" : "LOST") << "\n";
        rf << "fd_led_count = " << static_cast<int>(features.led_count) << "\n";
        rf << "fd_time_ms = " << fd_ms << "\n";
        rf << "fd_bin_thresh = " << det_cfg.BIN_THRESH << "\n";
        rf << "fd_min_blob_area = " << det_cfg.MIN_BLOB_AREA << "\n";
        rf << "fd_max_blob_area = " << det_cfg.MAX_BLOB_AREA << "\n";

        for (int i = 0; i < static_cast<int>(features.led_count); ++i) {
            const auto& L = features.leds[i];
            rf << "fd_led" << i << ".id = " << i << "\n";
            rf << "fd_led" << i << ".u_px = " << std::fixed << std::setprecision(6) << L.u_px << "\n";
            rf << "fd_led" << i << ".v_px = " << std::fixed << std::setprecision(6) << L.v_px << "\n";
            rf << "fd_led" << i << ".area = " << L.area << "\n";
        }

        rf << "\n[SPE]\n";
        rf << "spe_ok = " << (spe_ok ? 1 : 0) << "\n";
        rf << "spe_valid = " << (pose.valid ? 1 : 0) << "\n";
        rf << "spe_time_ms = " << spe_ms << "\n";
        rf << "spe_reproj_rms_px = " << pose.reproj_rms_px << "\n";

        if (spe_ok && pose.valid) {
            rf << "az_deg = "    << pose.az    * RAD2DEG << "\n";
            rf << "el_deg = "    << pose.el    * RAD2DEG << "\n";
            rf << "roll_deg = "  << pose.roll  * RAD2DEG << "\n";
            rf << "pitch_deg = " << pose.pitch * RAD2DEG << "\n";
            rf << "yaw_deg = "   << pose.yaw   * RAD2DEG << "\n";
            rf << "range_m = "   << pose.range_m << "\n";

            // If you have pose.R_C_P available as row-major float[9], print it similarly.
            // rf << "R_C_P_rowmajor = ...\n";

            rf << "q_C_P = " << pose.q_C_P[0] << " " << pose.q_C_P[1] << " " << pose.q_C_P[2] << " " << pose.q_C_P[3] << "\n";
        }
    }

    // Optional annotated image (for debugging)
    if (args.write_annotated) {
        cv::Mat gray_vis = make_vis_gray_u8(img);
        cv::Mat annotated;
        cv::cvtColor(gray_vis, annotated, cv::COLOR_GRAY2BGR);

        for (int i = 0; i < static_cast<int>(features.led_count); ++i) {
            const auto& L = features.leds[i];
            cv::Point pt(static_cast<int>(L.u_px), static_cast<int>(L.v_px));

            cv::circle(annotated, pt, 20, cv::Scalar(0, 0, 255), 2);
            cv::drawMarker(annotated, pt, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 50, 2);

            cv::putText(annotated, std::to_string(i), pt + cv::Point(5, -5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }

        if (spe_ok && pose.valid) {
            const double roll_deg  = pose.roll  * RAD2DEG;
            const double pitch_deg = pose.pitch * RAD2DEG;
            const double yaw_deg   = pose.yaw   * RAD2DEG;
            const double range_cm  = pose.range_m * 100.0;

            std::string text1 = "r,p,y[deg]=(" +
                std::to_string(roll_deg) + "," +
                std::to_string(pitch_deg) + "," +
                std::to_string(yaw_deg) + ")";
            std::string text2 = "range=" + std::to_string(range_cm) +
                                " cm, RMS=" + std::to_string(pose.reproj_rms_px) + " px";

            cv::putText(annotated, text1, cv::Point(20, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 1);
            cv::putText(annotated, text2, cv::Point(20, 70),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 1);

            cv::line(annotated, cv::Point(0, annotated.rows/2), cv::Point(annotated.cols - 1, annotated.rows/2),
                     cv::Scalar(255, 255, 255), 1);
            cv::line(annotated, cv::Point(annotated.cols/2, 0), cv::Point(annotated.cols/2, annotated.rows - 1),
                     cv::Scalar(255, 255, 255), 1);
        }

        cv::imwrite(annotated_path.string(), annotated);
    }

    return true; // processed (even if failed)
}

int main(int argc, char** argv) {
    Args args;
    if (!parse_args(argc, argv, args)) {
        print_usage(argv[0]);
        return 2;
    }

    if (!fs::exists(args.cases_root) || !fs::is_directory(args.cases_root)) {
        std::cerr << "ERROR: --cases_root is not a directory: " << args.cases_root.string() << "\n";
        return 2;
    }

    int total = 0;
    int processed = 0;

    for (const auto& e : fs::directory_iterator(args.cases_root)) {
        if (!e.is_directory()) continue;
        ++total;
        const bool did = run_one_case(e.path(), args);
        if (did) ++processed;
    }

    std::cout << "[BATCH] cases_root = " << args.cases_root.string() << "\n";
    std::cout << "[BATCH] subdirs_seen = " << total << "\n";
    std::cout << "[BATCH] cases_processed (had image) = " << processed << "\n";
    return 0;
}
