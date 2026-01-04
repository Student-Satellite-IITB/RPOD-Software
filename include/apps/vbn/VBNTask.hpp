#pragma once
#include <atomic>
#include <cstdint>

#include "os/rtos.hpp"

#include "apps/vbn/ImageCapture.hpp"        // for LiveFrameQueue / ReleaseFrameQueue typedefs
#include "apps/vbn/FeatureDetector.hpp"
#include "apps/vbn/StaticPoseEstimator.hpp"

#include "msg/ImageFrame.hpp"
#include "msg/FeatureFrame.hpp"
#include "msg/PoseEstimate.hpp"

namespace vbn {

// Optional (for later): command types to modify runtime behavior safely at frame boundaries.
enum class VBNCmdType : uint8_t {
    Stop,
    // SetFDConfig,
    // SetSPEConfig,
    // RequestCopy,
};

struct VBNCmd {
    VBNCmdType type;
    // FeatureDetectorConfig fd_cfg;
    // StaticPoseEstimatorConfig spe_cfg;
};

struct CopiedImage {
    uint8_t*  data = nullptr;      // points into internal pool
    uint32_t  size_bytes = 0;      // actual copied bytes (height * stride)

    uint32_t  width = 0;
    uint32_t  height = 0;
    uint32_t  stride = 0;
    uint8_t   bytes_per_px = 0;
    uint8_t   bit_depth = 0;
    uint8_t   bit_shift = 0;

    uint64_t  t_us = 0;
    uint32_t  frame_id = 0;

    uint8_t   pool_index = 0;      // must be returned
};

// ---- Copy-out interface (VBN-owned buffers) ----
static constexpr std::size_t COPY_MAX_W = 1280;
static constexpr std::size_t COPY_MAX_H = 800;
static constexpr std::size_t COPY_MAX_BPP = 2; // Y16 container
static constexpr std::size_t COPY_MAX_BYTES = COPY_MAX_W * COPY_MAX_H * COPY_MAX_BPP;

static constexpr std::size_t COPY_BUF_COUNT = 2;      // keep small; 2 is usually enough
static constexpr uint32_t    COPY_MAX_PENDING = 4;    // cap outstanding requests


// ------------------------------
// Queue types (keep them explicit and boring)
// ------------------------------
// Optional Debug/Telemetry Tap Queues
using FeatureFrameQueue = Rtos::Queue<msg::FeatureFrame, 1>;
using PoseEstimateQueue   = Rtos::Queue<msg::PoseEstimate, 1>;

// Command Queue 
using CmdQueue = Rtos::Queue<VBNCmd, 8>;

// Copy Frame Queues
using CopyFreeQueue = Rtos::Queue<CopiedImage, COPY_BUF_COUNT>;
using CopyFullQueue = Rtos::Queue<CopiedImage, COPY_BUF_COUNT>;

// ---------------------------------------------------------------------------
//  VBNTask
// ---------------------------------------------------------------------------
class VBNTask {
public:
    // Task entry wiring for OSAL (void* arg).
    // NOTE: The TaskCtx object must outlive the task (static or main-scope that never exits).
    struct TaskCtx {
        //Pointer to VBNTask object
        VBNTask* self = nullptr;
        
        // Pointers to Queues
        LiveFrameQueue*    live_in     = nullptr;   // IC -> VBN (freshest-wins)
        ReleaseFrameQueue* release_out = nullptr;   // VBN -> IC (must not drop)

        // Optional debug/telemetry taps
        FeatureFrameQueue* feat_out = nullptr;
        PoseEstimateQueue* pose_out = nullptr;

        // Optional future control channel (TO BE IMPLEMENTED)
        CmdQueue* command_in = nullptr;
    };

public:
    explicit VBNTask(const FeatureDetectorConfig& fd_cfg,
                     const StaticPoseEstimatorConfig& spe_cfg);

    // OSAL-compatible entry point
    static void TaskEntry(void* arg);

    // Request graceful stop (thread-safe).
    void RequestStop() { m_stop_requested.store(true); }
    bool StopRequested() const { return m_stop_requested.load(); }

    enum class Status : uint8_t {
        OK = 0,
        FD_FAILED,
        SPE_FAILED,
    };

    // FDIR FUNCTIONS
    // static const char* StatusStr(Status s);

    // Status lastStatus() const { return m_status; }
    // int    lastErrno()  const { return m_errno; }

    // Public control: request copying of next N frames (capped).
    // Thread-safe and atomic
    void RequestCopy(uint32_t n = 1);
    // This can be directly called by any thread at anytime
    // Alternative would be to add a command to Command Queue which the Run function
    // polls every loop and calls a suitable private handler function.
    // in that case we can shift RequestCopy to a private function
    // and make the m_copy_requests variable non-atomic

    // Consumer API:
    bool ReceiveCopied(CopiedImage& out, uint32_t timeout_ticks) {
        return m_copy_full_q.receive(out, timeout_ticks);
    }

    bool TryReceiveCopied(CopiedImage& out) {
        return m_copy_full_q.try_receive(out);
    }

    bool ReleaseCopied(const CopiedImage& img) {
        // constexpr uint32_t T = 10
        // and try_send(img,T)
        if (!m_copy_free_q.try_send(img)) {
            // BUG: free pool queue is full => likely double release or queue misuse.
            return false;
            // If ReleaseCopied() returns false, the caller must keep img and retry later (or escalate fault handling).
            // We cannot lose image buffers as the pool will shrink and deplete
        }
        return true;
    }

private:

    // Main run loop. Intended to be called only by the VBN thread.
    void Run(LiveFrameQueue& live_in, ReleaseFrameQueue& release_out,
             FeatureFrameQueue* feat_out, PoseEstimateQueue* pose_out);
    // Run takes pointers to feat_out and pose_out as they are optional
    
    // Called once by TaskEntry to populate copy free queue.
    void initCopyPool();

    // Called at frame boundary (never blocks VBN).
    void tryCopyFrame(const msg::ImageFrame& frame);

    // (stub) Later: drain control queue and apply changes at a safe point (frame boundary).
    void drainControlQueue_Stub(CmdQueue* command_in);

private:
    FeatureDetector       m_fd;
    StaticPoseEstimator   m_spe;

    std::atomic<bool> m_stop_requested{false};
    std::atomic<uint32_t> m_copy_requests{0};

    bool m_copy_pool_inited = false;
    // Copy buffers (IMPORTANT: large; keep VBNTask off the stack if you keep full frames)
    alignas(64) uint8_t m_copy_storage[COPY_BUF_COUNT][COPY_MAX_BYTES]{};

    CopyFreeQueue m_copy_free_q{/*overwrite=*/false};
    CopyFullQueue m_copy_full_q{/*overwrite=*/false};

    // FDIR
    Status m_status = Status::OK;
    // Fail
    //bool fail(Status s);
};

} // namespace vbn
