#pragma once
#include <cstdint>
#include <cstddef>

#include "os/rtos.hpp"
#include "msg/ImageFrame.hpp"

namespace vbn {
static constexpr uint32_t IMAGECAP_MAX_BUFS = 16;
}

namespace vbn {

// ------------------------------
// Queue types (keep them explicit and boring)
// ------------------------------
using LiveFrameQueue = Rtos::Queue<msg::ImageFrame, 1>;   // freshest-wins
static constexpr std::size_t RELEASE_Q_CAP = 16;          // >= V4L2 buffer_count (+ margin)
using ReleaseFrameQueue   = Rtos::Queue<msg::ImageFrame, RELEASE_Q_CAP>; // Never drop

// ------------------------------
// Config
// ------------------------------
struct ImageCaptureConfig {

    // V4L2 Device Node
    const char* dev = "/dev/video0";

    uint32_t width  = 1280;
    uint32_t height = 800;

    // V4L2 pixelformat fourcc (e.g., V4L2_PIX_FMT_Y16). Stored as uint32_t to avoid linux headers here.
    uint32_t v4l2_pixfmt = 0;

    // How many MMAP buffers to request via VIDIOC_REQBUFS.
    uint32_t buffer_count = 6;

    // DN interpretation metadata for downstream FeatureDetector.
    uint8_t bit_depth = 10;
    uint8_t bit_shift = 6; // MSB-Aligned

    // Timestamp policy: if true, try to use V4L2-provided timestamps; else stamp at DQBUF with monotonic time.
    bool prefer_v4l2_timestamps = false;
};

// ------------------------------
// ImageCapture: owns V4L2 streaming + mmapped buffers.
// Only the ImageCapture thread may call V4L2 ioctls.
// ------------------------------
class ImageCapture {
public:
    // Task entry wiring for OSAL (void* arg).
    // NOTE: The TaskCtx object must outlive the task (static or main-scope that never exits).
    struct TaskCtx {
        // Pointer to ImageCapture object
        ImageCapture*      self    = nullptr;

        // Pointers to Queues
        LiveFrameQueue*    live_out  = nullptr;
        ReleaseFrameQueue* release_in = nullptr;
    };

public:
    explicit ImageCapture(const ImageCaptureConfig& cfg);
    ~ImageCapture();

    // Open device, set+verify format, allocate+map buffers, queue buffers, stream on.
    // Returns false on any mismatch or ioctl failure.
    bool Start();

    // Stream off and release all resources. Safe to call even if Start() partially failed.
    void Stop();

    // One blocking dequeue: fills 'out' as a non-owning view into an mmapped buffer.
    // The dequeued buffer remains owned by userspace until Requeue(out.v4l2_buff_index).
    bool Dequeue(msg::ImageFrame& out);

    // Return a previously dequeued buffer to the driver (VIDIOC_QBUF by index).
    // Must only be called by the ImageCapture thread.
    bool Requeue(uint16_t v4l2_buff_index);

    // Thread loop helper:
    // - drains release_in (QBUF indices)
    // - DQBUF one frame
    // - publishes newest to live_out (overwrite=true)
    // This is the only place queues matter.
    void Run(LiveFrameQueue& live_out, ReleaseFrameQueue& release_in);

    // OSAL-compatible entry point
    static void TaskEntry(void* arg);

    // Optional introspection for logging/debug
    uint32_t negotiatedWidth()  const { return m_width; }
    uint32_t negotiatedHeight() const { return m_height; }
    uint32_t negotiatedStride() const { return m_stride; }
    uint8_t  negotiatedBytesPerPx() const { return m_bytes_per_px; }

    enum class Status : uint8_t {
        OK = 0,
        // SYSCALL FAILS
        OPEN_FAIL,
        QUERYCAP_FAIL,
        SETFMT_FAIL,
        REQBUFS_FAIL,
        QUERYBUF_FAIL,
        MMAP_FAIL,
        QBUF_FAIL,
        STREAMON_FAIL,
        DQBUF_FAIL,

        // LOGIC FAILS
        NOT_RUNNING,
        BAD_BUFF_INDEX,
        UNSUPPORTED_CAPS,
    };

    static const char* StatusStr(Status s);

    Status lastStatus() const { return m_status; }
    int    lastErrno()  const { return m_errno; }

private:
    struct MmapBuf {
        uint8_t* ptr = nullptr;
        uint32_t len = 0;
    };

    ImageCaptureConfig m_cfg{};
    int m_fd = -1; // File descriptor

    // Negotiated format (read back from VIDIOC_G_FMT and stored once)
    uint32_t m_width = 0;
    uint32_t m_height = 0;
    uint32_t m_stride = 0;        // bytes per row (bytesperline)
    uint8_t  m_bytes_per_px = 0;

    // MMAP buffers
    MmapBuf  m_bufs[vbn::IMAGECAP_MAX_BUFS]{};
    uint32_t m_buf_count = 0;

    uint32_t m_frame_id = 0;
    bool m_running = false;
    bool m_live_valid = false;
    uint16_t m_live_idx = 0;

    // FDIR
    Status m_status = Status::OK;
    int    m_errno  = 0;

private:
    // Setup helpers
    bool openDevice();
    bool queryCaps();
    bool setAndVerifyFormat();
    bool requestAndMapBuffers();
    bool queueAllBuffers();
    bool streamOn();
    void streamOff();
    void unmapBuffers();
    void closeDevice();

    // Run-loop helper
    void drainReleases(ReleaseFrameQueue& release_in);

    // Fail
    bool fail(Status s);
};

} // namespace vbn
