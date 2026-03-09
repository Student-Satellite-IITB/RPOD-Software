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
#include <unordered_map>
#include <algorithm>
#include <cctype>

namespace fs = std::filesystem;
constexpr double RAD2DEG = 180.0 / M_PI;

enum class RunMode : uint8_t { FULL = 0, SPE_ONLY = 1 };

struct Args {
    fs::path cases_root;
    std::string image_name = "image.png";
    bool write_annotated = false;
    uint16_t bin_thresh = 250; // native DN units (e.g. 250 for 10-bit data)
    RunMode mode = RunMode::FULL;
};

static void print_usage(const char* exe) {
    std::cerr
        << "Usage:\n"
        << "  " << exe << " --cases_root <DIR> [--mode full|spe_only] [--image_name image.png] [--bin_thresh N] [--write_annotated 0|1]\n"
        << "\nExample:\n"
        << "  " << exe << " --cases_root ../tools/data/cases/sim/sweep_range_20251225_0500 --mode full --bin_thresh 250 --write_annotated 0\n";
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
        } else if (a == "--mode") {
            const char* v = need_value("--mode");
            if (!v) return false;
            const std::string mode = v;
            if (mode == "full") {
                out.mode = RunMode::FULL;
            } else if (mode == "spe_only") {
                out.mode = RunMode::SPE_ONLY;
            } else {
                std::cerr << "Invalid --mode value: " << mode << " (expected full or spe_only)\n";
                return false;
            }
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

static std::string trim_copy(const std::string& in) {
    const auto first = std::find_if_not(in.begin(), in.end(),
                                        [](unsigned char ch){ return std::isspace(ch) != 0; });
    if (first == in.end()) return {};
    const auto last = std::find_if_not(in.rbegin(), in.rend(),
                                       [](unsigned char ch){ return std::isspace(ch) != 0; }).base();
    return std::string(first, last);
}

static bool parse_keyval_file(const fs::path& path, std::unordered_map<std::string, std::string>& out) {
    out.clear();
    std::ifstream f(path);
    if (!f.is_open()) return false;

    std::string line;
    while (std::getline(f, line)) {
        const std::string s = trim_copy(line);
        if (s.empty()) continue;
        if (s.front() == '[' && s.back() == ']') continue;

        const std::size_t eq = s.find('=');
        if (eq == std::string::npos) continue;

        const std::string k = trim_copy(s.substr(0, eq));
        const std::string v = trim_copy(s.substr(eq + 1));
        if (!k.empty()) out[k] = v;
    }

    return true;
}

static bool kv_get_float(const std::unordered_map<std::string, std::string>& kv,
                         const std::string& key, float& out) {
    const auto it = kv.find(key);
    if (it == kv.end()) return false;
    try {
        out = std::stof(it->second);
    } catch (...) {
        return false;
    }
    return true;
}

static bool kv_get_int(const std::unordered_map<std::string, std::string>& kv,
                       const std::string& key, int& out) {
    const auto it = kv.find(key);
    if (it == kv.end()) return false;
    try {
        out = std::stoi(it->second);
    } catch (...) {
        return false;
    }
    return true;
}

static bool load_truth_features(const fs::path& truth_path,
                                msg::FeatureFrame& out_features,
                                int& out_width,
                                int& out_height) {
    std::unordered_map<std::string, std::string> truth{};
    if (!parse_keyval_file(truth_path, truth)) return false;

    int width = 0;
    int height = 0;
    if (!kv_get_int(truth, "width", width) || !kv_get_int(truth, "height", height)) {
        return false;
    }

    out_features = {};
    out_features.valid = true;
    out_features.state = msg::TrackState::TRACK;
    out_features.visible_mask = msg::VISIBLE_INNER;
    out_features.pattern_id = msg::PatternId::INNER;
    out_features.feat_count = 5;

    // SPE expects INNER slots in fixed order: 0=T,1=L,2=B,3=R,4=C.
    for (int i = 0; i < 5; ++i) {
        float u = 0.0f;
        float v = 0.0f;
        if (!kv_get_float(truth, "led" + std::to_string(i) + ".u_px_true", u) ||
            !kv_get_float(truth, "led" + std::to_string(i) + ".v_px_true", v)) {
            return false;
        }

        auto& feat = out_features.feats[static_cast<std::size_t>(i)];
        feat.u_px = u;
        feat.v_px = v;
        feat.area = 1;
        feat.intensity = 0.0f;
        feat.pattern_id = msg::PatternId::INNER;
        feat.slot_id = static_cast<uint8_t>(i);
        feat.valid = 1;

        int area_px_true = 0;
        if (kv_get_int(truth, "led" + std::to_string(i) + ".area_px_true", area_px_true)) {
            feat.area = area_px_true;
        }
    }

    out_width = width;
    out_height = height;
    return true;
}

// For annotation/debug image only (FD uses original img data).
// Assumes img is single-channel grayscale: CV_8UC1 or CV_16UC1.
//
// bit_depth: meaningful bits (e.g. 10 for RAW10, 16 for GRAY16)
// bit_shift: how many bits the DN is shifted up inside the 16-bit container
//            RAW10 MSB-aligned-in-16 => bit_shift=6
//            RAW10 LSB-aligned-in-16 => bit_shift=0
static cv::Mat make_vis_gray_u8(const cv::Mat& img, uint8_t bit_depth, uint8_t bit_shift)
{
    CV_Assert(img.channels() == 1);

    if (img.type() == CV_8UC1) {
        return img; // already 8-bit view
    }

    if (img.type() == CV_16UC1) {
        CV_Assert(bit_depth >= 1 && bit_depth <= 16);
        CV_Assert(bit_shift <= 15);

        // Step 1: container16 -> dn16 (LSB-aligned DN)
        cv::Mat dn16(img.rows, img.cols, CV_16UC1);

        const uint16_t mask = (bit_depth == 16) ? 0xFFFFu
                                                : static_cast<uint16_t>((1u << bit_depth) - 1u);

        for (int r = 0; r < img.rows; ++r) {
            const uint16_t* src = img.ptr<uint16_t>(r);
            uint16_t* dst       = dn16.ptr<uint16_t>(r);
            for (int c = 0; c < img.cols; ++c) {
                uint16_t s = src[c];
                s = static_cast<uint16_t>(s >> bit_shift); // undo MSB packing if any
                s = static_cast<uint16_t>(s & mask);       // keep only meaningful bits
                dst[c] = s;
            }
        }

        // Step 2: DN -> 8-bit for visualization
        // Map [0 .. (2^bit_depth-1)] -> [0 .. 255]
        const double denom = double((1u << bit_depth) - 1u);
        const double scale = (denom > 0.0) ? (255.0 / denom) : 1.0;

        cv::Mat vis8;
        dn16.convertTo(vis8, CV_8U, scale);
        return vis8;
    }

    // Fallback: best-effort
    cv::Mat vis8;
    img.convertTo(vis8, CV_8U);
    return vis8;
}

static bool run_one_case(const fs::path& case_dir, const Args& args) {
    const fs::path input_path  = case_dir / args.image_name;
    const fs::path truth_path = case_dir / "truth.txt";
    const fs::path results_path = case_dir / "results.txt";
    const fs::path annotated_path = case_dir / "image_annotated_spe.jpg";

    cv::Mat img;
    msg::ImageFrame input{};
    msg::FeatureFrame features{};
    bool fd_ok = false;
    double fd_ms = 0.0;

    // Keep these independent from image presence so SPE-only can run without image files.
    int width_px = 0;
    int height_px = 0;

    // Configure FD.
    vbn::FeatureDetectorConfig det_cfg;
    det_cfg.BIN_THRESH = args.bin_thresh;
    det_cfg.MIN_BLOB_AREA = 50;
    det_cfg.MAX_BLOB_AREA = 20000;
    det_cfg.PATTERN_MAX_SCORE = 150.0f;
    det_cfg.MAX_OFFSET_SCORE = 0.6f;
    det_cfg.ROI_RADIUS_MARGIN = 2.5f;
    det_cfg.ROI_BORDER_PX = 10;

    // Visualization defaults: keep identical to RAW10-in-16 assumptions used in current runner.
    uint8_t vis_bit_depth = 10;
    uint8_t vis_bit_shift = 6;

    if (args.mode == RunMode::FULL) {
        if (!fs::exists(input_path)) {
            // Not a case folder (or missing image) -> skip silently
            return false;
        }

        img = cv::imread(input_path.string(), cv::IMREAD_UNCHANGED);
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
        input.data         = img.data; // pointer to raw bytes (can represent 8 or 16-bit containers)
        input.width        = static_cast<uint32_t>(img.cols);
        input.height       = static_cast<uint32_t>(img.rows);
        input.stride       = static_cast<uint32_t>(img.step);        // bytes per row
        input.bytes_per_px = static_cast<uint8_t>(img.elemSize1());  // 1 for 8U, 2 for 16U
        input.bit_depth    = 10; // RAW10 in 16-bit containers
        input.bit_shift    = 6;  // MSB aligned containers

        width_px = img.cols;
        height_px = img.rows;
        vis_bit_depth = input.bit_depth;
        vis_bit_shift = input.bit_shift;

        vbn::FeatureDetector detector(det_cfg);

        auto t0 = std::chrono::high_resolution_clock::now();
        fd_ok = detector.detect(input, features);
        auto t1 = std::chrono::high_resolution_clock::now();
        fd_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    } else {
        // SPE-only mode: skip image/FD and use deterministic centroids from truth.txt.
        if (!fs::exists(truth_path)) {
            return false; // not a SIM case directory
        }

        if (!load_truth_features(truth_path, features, width_px, height_px)) {
            std::cerr << "[CASE] " << case_dir.string() << " : ERROR parsing truth.txt for SPE-only mode\n";
            return true;
        }

        // Optional image load only for annotation output, never for SPE input in this mode.
        if (args.write_annotated && fs::exists(input_path)) {
            img = cv::imread(input_path.string(), cv::IMREAD_UNCHANGED);
            if (!img.empty() && img.channels() != 1) {
                cv::Mat gray;
                cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
                img = gray;
            }
            if (!img.empty() && !img.isContinuous()) img = img.clone();
        }

        // Emulate successful FD handoff with idealized features.
        fd_ok = true;
        fd_ms = 0.0;
    }

    // Configure SPE
    vbn::StaticPoseEstimatorConfig spe_cfg;
    spe_cfg.CAM_INTRINSICS.fx = 908.62425565f;
    spe_cfg.CAM_INTRINSICS.fy = 908.92570486f;
    spe_cfg.CAM_INTRINSICS.cx = width_px * 0.5f;
    spe_cfg.CAM_INTRINSICS.cy = height_px * 0.5f;
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
    const bool spe_ok = (fd_ok && features.feat_count > 0) ? spe.estimate(features, pose) : false;
    auto t3 = std::chrono::high_resolution_clock::now();
    const double spe_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();

    // Write results.txt (RAII closes file automatically when it goes out of scope)
    {
        std::ofstream rf(results_path);
        rf << "case_id = " << case_dir.filename().string() << "\n";
        rf << "run_mode = " << (args.mode == RunMode::FULL ? "full" : "spe_only") << "\n";
        rf << "case_dir = " << case_dir.string() << "\n";
        rf << "input_image = " << input_path.string() << "\n";

        if (args.mode == RunMode::FULL) {
            rf << "opencv_type = " << img.type() << "\n";
            rf << "channels = " << img.channels() << "\n";
            rf << "width = " << img.cols << "\n";
            rf << "height = " << img.rows << "\n";
            rf << "stride_bytes = " << img.step << "\n";
            rf << "bytes_per_px = " << static_cast<int>(input.bytes_per_px) << "\n";
            rf << "container_bit_depth = " << static_cast<int>(input.bit_depth) << "\n\n";
        } else {
            rf << "width = " << width_px << "\n";
            rf << "height = " << height_px << "\n";
            rf << "truth_file = " << truth_path.string() << "\n\n";
        }

        rf << "[FD]\n";
        rf << "fd_ok = " << (fd_ok ? 1 : 0) << "\n";
        rf << "fd_track_state = " << (features.state == msg::TrackState::TRACK ? "TRACK" : "LOST") << "\n";
        rf << "fd_led_count = " << static_cast<int>(features.feat_count) << "\n";
        rf << "fd_time_ms = " << fd_ms << "\n";
        rf << "fd_bin_thresh = " << det_cfg.BIN_THRESH << "\n";
        rf << "fd_min_blob_area = " << det_cfg.MIN_BLOB_AREA << "\n";
        rf << "fd_max_blob_area = " << det_cfg.MAX_BLOB_AREA << "\n";

        for (int i = 0; i < static_cast<int>(features.feat_count); ++i) {
            const auto& L = features.feats[i];
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
    if (args.write_annotated && !img.empty()) {
        cv::Mat gray_vis = make_vis_gray_u8(img, vis_bit_depth, vis_bit_shift);
        cv::Mat annotated;
        cv::cvtColor(gray_vis, annotated, cv::COLOR_GRAY2BGR);

        for (int i = 0; i < static_cast<int>(features.feat_count); ++i) {
            const auto& L = features.feats[i];
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
    std::cout << "[BATCH] mode = " << (args.mode == RunMode::FULL ? "full" : "spe_only") << "\n";
    std::cout << "[BATCH] subdirs_seen = " << total << "\n";
    std::cout << "[BATCH] cases_processed = " << processed << "\n";
    return 0;
}
