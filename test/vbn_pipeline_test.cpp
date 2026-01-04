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
#include <cmath> // For M_PI

constexpr double RAD2DEG = 180.0 / M_PI;

// -------------------- Monitor task --------------------
// Low-priority "observer" task:
// - Non-blocking reads from pose/feature queues (freshest-wins).
// - Prints at a slow cadence so it doesn't perturb IC/VBN timing.
// - Computes a rough pose update rate (Hz) from timestamps.
namespace {

// mono_us: monotonic clock time helper using clock_gettime
static uint64_t mono_us_test() {
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec) * 1000000ull + uint64_t(ts.tv_nsec) / 1000ull;
}

struct MonitorTaskCtx {
    vbn::FeatureFrameQueue* feat_in = nullptr;   // optional
    vbn::PoseEstimateQueue* pose_in = nullptr;   // optional
};

void MonitorTaskEntry(void* arg) {
    auto* ctx = static_cast<MonitorTaskCtx*>(arg);

    // Defensive: allow running with only pose or only features
    if (!ctx || (!ctx->feat_in && !ctx->pose_in)) {
        return;
    }

    std::cout << "[Monitor] started\n";

    uint64_t last_print_us = mono_us_test();
    constexpr uint64_t PRINT_PERIOD_US = 1'000'000;

    uint32_t pose_count = 0;
    uint32_t feat_count = 0;

    msg::PoseEstimate last_pose{};
    msg::FeatureFrame last_feat{};
    bool have_pose = false;
    bool have_feat = false;

    while (true) {

        // Drain newest features (non-blocking)
        if (ctx->feat_in) {
            msg::FeatureFrame features{};
            while (ctx->feat_in->try_receive(features)) {
                last_feat = features;
                have_feat = true;
                feat_count++;
            }
        }

        // Drain newest pose (non-blocking)
        if (ctx->pose_in) {
            msg::PoseEstimate pose{};
            while (ctx->pose_in->try_receive(pose)) {
                last_pose = pose;
                have_pose = true;
                pose_count++;
            }
        }

        const uint64_t now_us = mono_us_test();
        if (now_us - last_print_us >= PRINT_PERIOD_US) {
            const double dt_s = double(now_us - last_print_us) * 1e-6;
            const double pose_hz = (dt_s > 0.0) ? (double(pose_count) / dt_s) : 0.0;
            const double feat_hz = (dt_s > 0.0) ? (double(feat_count) / dt_s) : 0.0;

            std::cout << "[Monitor] pose_hz=" << pose_hz
                      << " feat_hz=" << feat_hz;

            if (have_feat) {
                std::cout << "[FD] OK ";
                std::cout << "[FD] LEDs detected = " << static_cast<int>(last_feat.led_count) << "\n";
                std::cout << "[FD] Track state = " << (last_feat.state == msg::TrackState::TRACK ? "TRACK" : "LOST")<< "\n";
            } else {
                std::cout << "[FD]  ";
            }

            if (have_pose) {
                // std::cout << " range_m=" << last_pose.range_m;
                // std::cout << " yaw_deg=" << last_pose.yaw_deg;

                auto q = last_pose.q_C_P;
                auto t = last_pose.t_CbyP;
                double az_deg = last_pose.az * RAD2DEG;
                double el_deg = last_pose.el * RAD2DEG;
                double roll_deg  = last_pose.roll  * RAD2DEG;
                double pitch_deg = last_pose.pitch * RAD2DEG;
                double yaw_deg   = last_pose.yaw   * RAD2DEG;
                double range_cm  = last_pose.range_m * 100.0; // m → cm

                std::cout << "[SPE] OK ";
                std::cout << "[SPE] Pose estimation SUCCESS.\n";
                // std::cout << "      Azimuth  = " << az_deg<< " deg\n";
                // std::cout << "      Elevation = " << el_deg << " deg\n";
                // std::cout << "      Roll  = " << roll_deg  << " deg\n";
                // std::cout << "      Pitch = " << pitch_deg << " deg\n";
                // std::cout << "      Yaw   = " << yaw_deg   << " deg\n";
                std::cout << "      Range = " << range_cm  << " cm\n";
                // std::cout << "      Quaternion = [ "<< q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] <<"]\n";
                std::cout << "      Reproj RMS = " << last_pose.reproj_rms_px << " px\n";
            } else {
                std::cout << "[SPE] NONE ";
            }

            std::cout << "\n";

            last_print_us = now_us;
            pose_count = 0;
            feat_count = 0;
        }

        // Yield CPU, don't spin.
        Rtos::SleepMs(1000);
    }
}

} // namespace



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

    MonitorTaskCtx mon_ctx{};
    mon_ctx.feat_in = &featureFrameQueue;
    mon_ctx.pose_in = &poseEstimateQueue;

    // ---- TASKS ----
    Rtos::Task ImageCaptureTask;
    Rtos::Task VBNTask;

    Rtos::Task MonitorTask;
    
    // Create Tasks
    ImageCaptureTask.Create("ImageCapture", &vbn::ImageCapture::TaskEntry, &cap_ctx);
    VBNTask.Create("VBN", &vbn::VBNTask::TaskEntry, &vbn_ctx);

    MonitorTask.Create("Monitor", MonitorTaskEntry, &mon_ctx);

    // Join Task (Waits indefinitely)
    ImageCaptureTask.Join();
    VBNTask.Join();
    MonitorTask.Join();

    return 0;
}
