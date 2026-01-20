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

int main() {
    std::cout << "=== VBN PIPELINE TEST ===\n";

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
    fd_cfg.BIN_THRESH = 250; // For 10-Bit Image
    fd_cfg.MIN_BLOB_AREA = 50;
    fd_cfg.MAX_BLOB_AREA = 20000;
    fd_cfg.PATTERN_MAX_SCORE = 150.0f;
    fd_cfg.MAX_OFFSET_SCORE = 0.6f;
    fd_cfg.ROI_RADIUS_MARGIN = 2.5f;
    fd_cfg.ROI_BORDER_PX = 10;
    fd_cfg.OUTPUT_MODE = vbn::FDOutputMode::RAW_BLOBS;

    vbn::StaticPoseEstimatorConfig spe_cfg{};
    // Camera intrinsics (TODO: plug in real calibration)
    spe_cfg.CAM_INTRINSICS.fx = 908.62425565f;          // [px] placeholder
    spe_cfg.CAM_INTRINSICS.fy = 908.92570486f;          // [px] placeholder
    spe_cfg.CAM_INTRINSICS.cx = cap_cfg.width * 0.5f;        // assume principal point at image centre
    spe_cfg.CAM_INTRINSICS.cy = cap_cfg.height * 0.5f;
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
    mon_ctx.cfg.log_n = 100;
    mon_ctx.cfg.log_every = 1;
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
