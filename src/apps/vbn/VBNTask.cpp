// VBNTask.cpp
#include "apps/vbn/VBNTask.hpp"
#include <cstddef> 
#include <cstring> // memcpy

namespace vbn{

VBNTask::VBNTask(const FeatureDetectorConfig& fd_cfg,
                 const StaticPoseEstimatorConfig& spe_cfg)
: m_fd(fd_cfg)
, m_spe(spe_cfg) {
    // Reseting any previous errors
    m_status = Status::OK;
}

void VBNTask::TaskEntry(void* arg) {
    auto* ctx = static_cast<TaskCtx*>(arg);

    // Minimal defensive checks. In flight SW: raise a fault/event.
    if (!ctx || !ctx->self || !ctx->live_in || !ctx->release_out) {
        return;
    }

    // Initialise Copy Pool
    ctx->self->initCopyPool();

    // VBN just consumes frames + releases them deterministically.
    ctx->self->Run(*ctx->live_in, *ctx->release_out, 
                    ctx->feat_out, ctx->pose_out);
}


void VBNTask::Run(LiveFrameQueue& live_in, ReleaseFrameQueue& release_out,
                  FeatureFrameQueue* feat_out, PoseEstimateQueue* pose_out) {

    // Minimal: no command queue yet;
    // The only requirements:
    //  - block for newest frame from ImageCapture
    //  - run FD -> SPE
    //  - always release the frame back to ImageCapture (blocking send)
    while (!StopRequested()) {

        msg::ImageFrame frame{};

        if (!live_in.receive(frame, Rtos::MAX_TIMEOUT)) {
            continue;
        }

        // ---- Feature Detection ----
        msg::FeatureFrame feat{};
        const bool ok_fd = m_fd.detect(frame, feat);
        if(!ok_fd){
            m_status = Status::FD_FAILED;
        }

        // ---- Pose Estimation ----
        msg::PoseEstimate pose{};
        const bool ok_pose = m_spe.estimate(feat, pose);
        if(ok_fd && !ok_pose){
            m_status = Status::SPE_FAILED;
        }

        // Publish feat/pose to optional tap queues via ctx
        // Future correction: Maybe make this optional
        if (feat_out) (void)feat_out->try_send(feat);
        if (pose_out) (void)pose_out->try_send(pose);


        // Copy request happens at frame boundary
        tryCopyFrame(frame);

        // ---- Release buffer back to ImageCapture ----
        // Critical: do this even if FD/SPE failed.
        (void)release_out.send(frame, Rtos::MAX_TIMEOUT);
    }

    // If StopRequested() is used later, we exit cleanly.
    // Any currently-held frame was released each iteration.
}

void VBNTask::drainControlQueue_Stub(CmdQueue* command_in) {
    (void)command_in;
    // Stub: later we will drain VBNCmd items and apply at safe points (frame boundaries).
    // Example policy:
    //  - Stop -> RequestStop()
    //  - SetFDConfig/SetSPEConfig -> replace configs only BETWEEN frames
    //  - RequestCopy -> increment atomic counter etc.
}

// -------------------- private helpers --------------------

void VBNTask::initCopyPool() {
    if (m_copy_pool_inited) return; // To avoid multi-allocations
    for (uint8_t i = 0; i < COPY_BUF_COUNT; ++i) {
        CopiedImage slot{};
        slot.data = m_copy_storage[i];
        slot.pool_index = i;
        slot.size_bytes = 0;
        (void)m_copy_free_q.send(slot, Rtos::MAX_TIMEOUT); 
        // Since this is init-time it is OK to block
    m_copy_pool_inited = true;
    }
}

void VBNTask::RequestCopy(uint32_t n) {
    uint32_t cur = m_copy_requests.load(std::memory_order_relaxed);
    while (true) {
        uint32_t next = cur + n;
        if (next > COPY_MAX_PENDING) next = COPY_MAX_PENDING;
        if (m_copy_requests.compare_exchange_weak(
                cur, next, std::memory_order_release, std::memory_order_relaxed)) {
            return;
        }
    }
}

void VBNTask::tryCopyFrame(const msg::ImageFrame& frame) {
    if (m_copy_requests.load(std::memory_order_acquire) == 0) return;

    CopiedImage slot{};
    if (!m_copy_free_q.try_receive(slot)) return; // no buffer free

    const uint32_t bytes = frame.height * frame.stride;
    if (bytes > COPY_MAX_BYTES || !frame.data) {
        (void)m_copy_free_q.try_send(slot);
        return;
    }

    std::memcpy(slot.data, frame.data, bytes);

    slot.size_bytes   = bytes;
    slot.width        = frame.width;
    slot.height       = frame.height;
    slot.stride       = frame.stride;
    slot.bytes_per_px = frame.bytes_per_px;
    slot.bit_depth    = frame.bit_depth;
    slot.bit_shift    = frame.bit_shift;
    slot.t_us         = frame.t_exp_end_us;
    slot.frame_id     = frame.frame_id;

    if (!m_copy_full_q.try_send(slot)) {
        // Shouldn't happen if consumers behave, but handle gracefully
        (void)m_copy_free_q.try_send(slot);
        return;
    }

    m_copy_requests.fetch_sub(1, std::memory_order_release);
}


} // namespace vbn