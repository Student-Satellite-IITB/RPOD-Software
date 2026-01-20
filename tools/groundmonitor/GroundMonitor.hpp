#pragma once
#include <cstdint>

#include "apps/vbn/ImageCapture.hpp"
#include "apps/vbn/VBNTask.hpp"
#include "apps/vbn/RNAVFilter.hpp"

#include "msg/FeatureFrame.hpp"
#include "msg/PoseEstimate.hpp"
#include "msg/RNAVState.hpp"

#include <string>

namespace ground {

enum class Test : uint8_t {
    NONE = 0,
    RANGE_LOG,
    CENTROID_LOG,
    //ADD MORE
};

// -------------------- Monitor task --------------------
// Low-priority "observer" task:
// - Non-blocking reads from pose/feature queues (freshest-wins).
// - Prints at a slow cadence so it doesn't perturb IC/VBN timing.
// - Computes a rough pose update rate (Hz) from timestamps.

struct GroundMonitorConfig {
    // Logging
    bool enable_csv = false;
    std::string  out_dir = "tools/data/tmp/vbn_monitor";
    uint32_t log_n = 1000;
    uint32_t log_every = 1;
    Test testcase = Test::NONE;

    // Snapshots (copy from VBNTask)
    bool enable_snapshots = false;
    bool enable_server = false;
    uint32_t snapshot_period_ms = 200; // request copies at this rate

    // MJPEG server
    int port = 8080;
    int stream_fps = 10;              // how often to push to client (cap)
    int jpeg_quality = 80;
};

struct GroundMonitorCtx {
    vbn::FeatureFrameQueue* feat_in = nullptr;
    vbn::PoseEstimateQueue* pose_in = nullptr;
    rnav::StateEstimateQueue* state_in = nullptr;
    vbn::VBNTask* vbn = nullptr;              // needed for RequestCopy/TryReceiveCopied/ReleaseCopied
    GroundMonitorConfig cfg;
};

// OSAL task entry
void TaskEntry(void* arg);

} // namespace ground
