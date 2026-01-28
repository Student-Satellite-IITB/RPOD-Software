// test/vbn_pipeline_test.cpp

#include <iostream>
#include <atomic>

#include "os/rtos.hpp"

// Your modules
#include "apps/vbn/ImageCapture.hpp"
#include "apps/vbn/VBNTask.hpp"

// Message types
#include "msg/ImageFrame.hpp"
#include "msg/FeatureFrame.hpp"
#include "msg/PoseEstimate.hpp"

#include <linux/videodev2.h> // only for V4L2_PIX_FMT_* in this test

#include "tools/groundmonitor/GroundMonitor.hpp"

struct Args {
    uint16_t bin_thresh = 250; // native DN units (e.g. 250 for 10-bit data)
    int min_area = 0;
    
};

static void print_usage(const char* exe) {
    std::cerr
        << "Usage:\n"
        << "  " << exe << " [--bin_thresh N] [--min_area N]\n"
        << "\nExample:\n"
        << "  " << exe << " --bin_thresh 250 --min_area 50\n";
}

static bool parse_args(int argc, char** argv, Args& out) {
    // Defaults already set in Args. No args is OK.
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];

        auto need_value = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << "\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (a == "--bin_thresh") {
            const char* v = need_value("--bin_thresh");
            if (!v) return false;
            int x = std::stoi(v);
            if (x < 0 || x > 65535) {
                std::cerr << "Invalid --bin_thresh (0..65535): " << x << "\n";
                return false;
            }
            out.bin_thresh = static_cast<uint16_t>(x);

        } else if (a == "--min_area"){
            const char* v = need_value("--min_area");
            if (!v) return false;
            int x = std::stoi(v);
            if (x < 0) {
                std::cerr << "Invalid --min_area (>=0): " << x << "\n";
                return false;
            }
            out.min_area = x;

        } else if (a == "--help" || a == "-h") {
            return false; // triggers usage print in main

        } else {
            std::cerr << "Unknown arg: " << a << "\n";
            return false;
        }
    }
    return true;
}

int main(int argc, char** argv) {
    Args args;
    if (!parse_args(argc, argv, args)) {
        print_usage(argv[0]);
        return 2;
    }

    std::cout << "=== VBN LED CHARACTERISATION TEST ===\n";

    // ---- QUEUES ----
    vbn::LiveFrameQueue liveFrameQueue(/*overwrite=*/true);
    vbn::ReleaseFrameQueue releaseFrameQueue(/*overwrite=*/false);

    vbn::FeatureFrameQueue featureFrameQueue(/*overwrite=*/true);
    vbn::PoseEstimateQueue poseEstimateQueue(/*overwrite=*/true);

    // ---- CONFIGURATION----
    vbn::ImageCaptureConfig cap_cfg{};
    cap_cfg.dev = "/dev/video0";
    cap_cfg.width = 1280;
    cap_cfg.height = 800;
    cap_cfg.v4l2_pixfmt = V4L2_PIX_FMT_Y16;
    cap_cfg.buffer_count = 6;
    cap_cfg.bit_depth = 10;
    cap_cfg.bit_shift = 6;

    vbn::FeatureDetectorConfig fd_cfg{};
    fd_cfg.BIN_THRESH = args.bin_thresh; // For 10-Bit Image
    fd_cfg.MIN_BLOB_AREA = args.min_area;
    fd_cfg.MAX_BLOB_AREA = 20000;
    fd_cfg.PATTERN_MAX_SCORE = 150.0f;
    fd_cfg.MAX_OFFSET_SCORE = 0.6f;
    fd_cfg.ROI_RADIUS_MARGIN = 2.5f;
    fd_cfg.ROI_BORDER_PX = 300;
    fd_cfg.OUTPUT_MODE = vbn::FDOutputMode::FILTERED_BLOBS;

    vbn::StaticPoseEstimatorConfig spe_cfg{};
    // SPE Not Needed

    // ---- MODULES ----
    vbn::ImageCapture cap(cap_cfg);
    vbn::VBNTask vbn(fd_cfg, spe_cfg);

    // ---- TASK CONTEXTS  ----
    //Task contexts owned by main; lifetime OK because we Join
    vbn::ImageCapture::TaskCtx cap_ctx{};
    cap_ctx.self = &cap;
    cap_ctx.live_out = &liveFrameQueue;
    cap_ctx.release_in = &releaseFrameQueue;

    vbn::VBNTask::TaskCtx vbn_ctx{};
    vbn_ctx.self = &vbn;
    vbn_ctx.live_in = &liveFrameQueue;
    vbn_ctx.release_out = &releaseFrameQueue;
    vbn_ctx.feat_out = &featureFrameQueue;
    vbn_ctx.pose_out = &poseEstimateQueue;

    ground::GroundMonitorCtx mon_ctx{};
    mon_ctx.feat_in = &featureFrameQueue;
    mon_ctx.pose_in = &poseEstimateQueue;
    mon_ctx.vbn = &vbn;
    // Configure monitor
    mon_ctx.cfg.enable_server    = true;   // MJPEG HTTP
    mon_ctx.cfg.enable_snapshots = true;   // copy+annotate+JPEG
    mon_ctx.cfg.enable_csv       = true;  
    mon_ctx.cfg.testcase         = ground::Test::CENTROID_LOG;
    mon_ctx.cfg.log_n = 1000;
    mon_ctx.cfg.log_every = 5;
    mon_ctx.cfg.out_dir = "../tools/data/tmp/vbn_monitor";
    mon_ctx.cfg.port = 8080;
    mon_ctx.cfg.snapshot_period_ms = 200;
    mon_ctx.cfg.stream_fps = 10;

    // ---- TASKS ----
    Rtos::Task ImageCaptureTask;
    Rtos::Task VBNTask;

    Rtos::Task MonitorTask;
    
    // Create Tasks
    ImageCaptureTask.Create("ImageCapture", &vbn::ImageCapture::TaskEntry, &cap_ctx);
    VBNTask.Create("VBN", &vbn::VBNTask::TaskEntry, &vbn_ctx);
    
    MonitorTask.Create("Monitor", &ground::TaskEntry, &mon_ctx);

    // Join Task (Waits indefinitely)
    ImageCaptureTask.Join();
    VBNTask.Join();
    MonitorTask.Join();

    return 0;
}
