// ImageCapture.cpp
#include "apps/vbn/ImageCapture.hpp"

#include <linux/videodev2.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <string.h>
#include <time.h>

namespace {

// Retry ioctl if interrupted by a signal.
// Without it we can get rare failures under load/interrupts.
static int xioctl(int fd, unsigned long req, void* arg) {
    int r;
    do { r = ::ioctl(fd, req, arg); }
    while (r == -1 && errno == EINTR);
    return r;
}

// Monotonic timestamp in microseconds.
static uint64_t mono_us() {
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec) * 1000000ull + uint64_t(ts.tv_nsec) / 1000ull;
}

// A tiny helper for common formats; expand as needed.
static uint8_t bytes_per_px_from_pixfmt(uint32_t pixfmt) {
    switch (pixfmt) {
        case V4L2_PIX_FMT_GREY: return 1; // 8-bit mono
        case V4L2_PIX_FMT_Y16:  return 2; // 16-bit container mono
        default:                return 0; // unknown; handled by fallback
    }
}

} // anonymous namespace

namespace vbn {

static inline ImageCaptureConfig sanitise(const ImageCaptureConfig& in) {
    ImageCaptureConfig cfg = in;

    if (!cfg.dev) cfg.dev = "/dev/video0";

    if (cfg.width == 0)  cfg.width  = 1280;
    if (cfg.height == 0) cfg.height = 800;

    // Must be set by caller; if not, you can keep it 0 and let Start() fail loudly.
    // If you want "fail fast in sanitise", you can assert/log here instead.
    // if (cfg.v4l2_pixfmt == 0) { ... }

    if (cfg.buffer_count < 2) cfg.buffer_count = 2;
    if (cfg.buffer_count > vbn::IMAGECAP_MAX_BUFS) cfg.buffer_count = vbn::IMAGECAP_MAX_BUFS;

    if (cfg.bit_depth < 1) cfg.bit_depth = 1;
    if (cfg.bit_depth > 16) cfg.bit_depth = 16;

    // For 16-bit container: bit_shift must leave bit_depth bits in range.
    const uint8_t max_shift = static_cast<uint8_t>(16 - cfg.bit_depth);
    if (cfg.bit_shift > max_shift) cfg.bit_shift = max_shift;

    return cfg;
}

} // namespace vbn


namespace vbn {

ImageCapture::ImageCapture(const ImageCaptureConfig& cfg)
: m_cfg(sanitise(cfg)) {
    // Reseting any previous errors
    m_status = Status::OK;
    m_errno  = 0;
}

ImageCapture::~ImageCapture() {
    Stop();
}

bool ImageCapture::Start() {
    if (m_running) return true;

    // Reseting any previous errors
    m_status = Status::OK;
    m_errno  = 0;

    if (!openDevice()) return false; // OK because openDevice() sets status
    if (!queryCaps()) { Stop(); return false; }
    if (!setAndVerifyFormat()) { Stop(); return false; }
    if (!requestAndMapBuffers()) { Stop(); return false; }
    if (!queueAllBuffers()) { Stop(); return false; }
    if (!streamOn()) { Stop(); return false; }

    m_frame_id = 0;
    m_live_valid = false;
    return true;
}

void ImageCapture::Stop() {
    if (m_running) {
        streamOff();
        m_running = false;
    }
    unmapBuffers();
    closeDevice();
}

bool ImageCapture::Dequeue(msg::ImageFrame& out) {
    if (!m_running) return fail(Status::NOT_RUNNING);

    v4l2_buffer buf{};
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (xioctl(m_fd, VIDIOC_DQBUF, &buf) == -1) {
        return fail(Status::DQBUF_FAIL);
    }

    const uint32_t idx = buf.index;
    if (idx >= m_buf_count) {
        // Should never happen if driver is sane.
        return fail(Status::BAD_BUFF_INDEX);
    }

    out.data         = m_bufs[idx].ptr;
    out.width        = m_width;
    out.height       = m_height;
    out.stride       = m_stride;
    out.bytes_per_px = m_bytes_per_px;

    out.bit_depth    = m_cfg.bit_depth;
    out.bit_shift    = m_cfg.bit_shift;

    // v1: stamp at DQBUF (monotonic). You can later switch to v4l2 timestamps if needed.
    out.t_exp_end_us = mono_us();

    out.frame_id        = m_frame_id++;
    out.v4l2_buff_index = static_cast<uint16_t>(idx);

    return true;
}

bool ImageCapture::Requeue(uint16_t v4l2_buff_index) {
    if (!m_running) return fail(Status::NOT_RUNNING);

    const uint32_t idx = v4l2_buff_index;
    if (idx >= m_buf_count) return fail(Status::BAD_BUFF_INDEX);

    v4l2_buffer buf{};
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index  = idx;

    if (xioctl(m_fd, VIDIOC_QBUF, &buf) != -1){
        return fail(Status::QBUF_FAIL);
    }

    return true;
}

void ImageCapture::Run(LiveFrameQueue& live_out, ReleaseFrameQueue& release_in) {
    // Minimal run loop: drain releases, DQBUF, publish newest (overwrite),
    // and if overwrite happened, immediately QBUF the overwritten unconsumed buffer.
    while (true) {

        // 1) Drain releases (buffers VBN finished processing)
        msg::ImageFrame rel{};
        while (release_in.try_receive(rel)) {
            (void)Requeue(rel.v4l2_buff_index);
        }

        // 2) Dequeue one fresh frame (blocking)
        msg::ImageFrame f{};
        if (!Dequeue(f)) {
            // In flight SW you’d signal fault / attempt recovery.
            // For minimal version: just continue.
            continue;
        }

        // 3) Publish newest frame to VBN (freshest-wins queue of depth 1)
        // We must QBUF the overwritten buffer (if any), because the queue doesn't give it back.
        const uint16_t old_idx = m_live_idx;

        // live_out is constructed with overwrite=true in main.
        (void)live_out.send(f, Rtos::MAX_TIMEOUT);

        const bool overwritten = live_out.wasLastSendOverwritten();
        if (overwritten && m_live_valid) {
            // Overwrite means queue was full => old frame was NOT consumed by VBN yet,
            // therefore it's safe to QBUF immediately.
            (void)Requeue(old_idx);
        }

        m_live_idx   = f.v4l2_buff_index;
        m_live_valid = true;
    }
}

// Static task entry function compatible with OSAL:
//
// OSAL wants:   void (*fn)(void*)
// C++ methods:  need an object (this) to run on
//
// So we pass a small context struct (TaskCtx) through the void*.
// TaskCtx contains pointers to the ImageCapture instance and its queues.

void ImageCapture::TaskEntry(void* arg) {

    auto* ctx = static_cast<TaskCtx*>(arg);

    // Defensive checks: if someone accidentally passed nullptr or an
    // incomplete context, we exit the task cleanly.
    // In flight we will raise an event/fault here instead of returning.
    if (!ctx || !ctx->imgcap || !ctx->live_out || !ctx->release_in) {
        return; // dev behavior: fail silently; later you can raise an event
    }

    // Start the V4L2 stream and mmap buffers.
    if (!ctx->imgcap->Start()) {
        return; // optional: log ctx->ic->lastStatus()/lastErrno()
    }

    // Main capture loop:
    // - drains ReleaseFrameQueue (QBUF finished buffers)
    // - DQBUF one fresh frame
    // - publishes to LiveFrameQueue (freshest wins)
    //
    // Minimal version: Run() is an infinite loop and does not return.
    ctx->imgcap->Run(*ctx->live_out, *ctx->release_in);
}

// -------------------- private helpers --------------------

bool ImageCapture::openDevice() {
    m_fd = ::open(m_cfg.dev, O_RDWR | O_CLOEXEC);
    if (m_fd < 0) return fail(Status::OPEN_FAIL);
    return true;
}

bool ImageCapture::queryCaps() {
    v4l2_capability cap{};
    if (xioctl(m_fd, VIDIOC_QUERYCAP, &cap) == -1) return fail(Status::QUERYCAP_FAIL);

    const bool streaming = (cap.capabilities & V4L2_CAP_STREAMING) != 0;
    const bool capture   = (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) != 0;

    if (!streaming || !capture){
        return fail(Status::UNSUPPORTED_CAPS);
    }
    return true;
}

bool ImageCapture::setAndVerifyFormat() {
    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    fmt.fmt.pix.width       = m_cfg.width;
    fmt.fmt.pix.height      = m_cfg.height;
    fmt.fmt.pix.pixelformat = m_cfg.v4l2_pixfmt;
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;

    if (xioctl(m_fd, VIDIOC_S_FMT, &fmt) == -1) return fail(Status::SETFMT_FAIL);
    if (xioctl(m_fd, VIDIOC_G_FMT, &fmt) == -1) return fail(Status::SETFMT_FAIL);

    // Strict verification for determinism
    if (fmt.fmt.pix.width  != m_cfg.width)  return fail(Status::SETFMT_FAIL);
    if (fmt.fmt.pix.height != m_cfg.height) return fail(Status::SETFMT_FAIL);
    if (fmt.fmt.pix.pixelformat != m_cfg.v4l2_pixfmt) return fail(Status::SETFMT_FAIL);

    m_width  = fmt.fmt.pix.width;
    m_height = fmt.fmt.pix.height;
    m_stride = fmt.fmt.pix.bytesperline;

    // Determine container bytes/pixel
    uint8_t bpp = bytes_per_px_from_pixfmt(fmt.fmt.pix.pixelformat);
    if (bpp == 0) {
        // Fallback: estimate from sizeimage if stride info is weird
        const uint32_t sizeimage = fmt.fmt.pix.sizeimage;
        if (m_width == 0 || m_height == 0) return fail(Status::SETFMT_FAIL);
        const uint32_t approx = sizeimage / (m_width * m_height);
        if (approx == 0 || approx > 8) return fail(Status::SETFMT_FAIL);
        bpp = static_cast<uint8_t>(approx);
    }
    m_bytes_per_px = bpp;

    // Basic sanity: stride must fit at least one row
    if (m_stride < (m_width * m_bytes_per_px)) return fail(Status::SETFMT_FAIL);

    return true;
}

bool ImageCapture::requestAndMapBuffers() {

    v4l2_requestbuffers req{};
    req.count  = m_cfg.buffer_count;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(m_fd, VIDIOC_REQBUFS, &req) == -1) return fail(Status::REQBUFS_FAIL);
    if (req.count < 2) return fail(Status::REQBUFS_FAIL);

    m_buf_count = req.count;

    for (uint32_t i = 0; i < m_buf_count; ++i) {
        v4l2_buffer buf{};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;

        if (xioctl(m_fd, VIDIOC_QUERYBUF, &buf) == -1) return fail(Status::QUERYBUF_FAIL);

        void* p = ::mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, buf.m.offset);
        if (p == MAP_FAILED) return fail(Status::MMAP_FAIL);

        m_bufs[i].ptr = static_cast<uint8_t*>(p);
        m_bufs[i].len = buf.length;
    }

    return true;
}

bool ImageCapture::queueAllBuffers() {
    for (uint32_t i = 0; i < m_buf_count; ++i) {
        v4l2_buffer buf{};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;

        if (xioctl(m_fd, VIDIOC_QBUF, &buf) == -1) return fail(Status::QBUF_FAIL);
    }
    return true;
}

bool ImageCapture::streamOn() {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(m_fd, VIDIOC_STREAMON, &type) == -1) return fail(Status::STREAMON_FAIL);
    m_running = true;
    return true;
}

void ImageCapture::streamOff() {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    (void)xioctl(m_fd, VIDIOC_STREAMOFF, &type);
}

void ImageCapture::unmapBuffers() {
    for (uint32_t i = 0; i < m_buf_count; ++i) {
        if (m_bufs[i].ptr && m_bufs[i].len) {
            ::munmap(m_bufs[i].ptr, m_bufs[i].len);
        }
        m_bufs[i].ptr = nullptr;
        m_bufs[i].len = 0;
    }
    m_buf_count = 0;
}

void ImageCapture::closeDevice() {
    if (m_fd >= 0) {
        ::close(m_fd);
        m_fd = -1;
    }
}

// FDIR

bool ImageCapture::fail(Status s) {
    m_status = s;

    switch (s) {
        //ioctl fails have an errno associated
        case Status::OPEN_FAIL:
        case Status::QUERYCAP_FAIL:
        case Status::SETFMT_FAIL:
        case Status::REQBUFS_FAIL:
        case Status::QUERYBUF_FAIL:
        case Status::MMAP_FAIL:
        case Status::QBUF_FAIL:
        case Status::STREAMON_FAIL:
        case Status::DQBUF_FAIL:
            m_errno = errno;
            break;

        default:
            m_errno = 0;   // logic failure
            break;
    }
    return false;
}

const char* ImageCapture::StatusStr(ImageCapture::Status s) {
    switch (s) {
        case ImageCapture::Status::OK:            return "OK";
        case ImageCapture::Status::OPEN_FAIL:     return "OPEN_FAIL";
        case ImageCapture::Status::QUERYCAP_FAIL: return "QUERYCAP_FAIL";
        case ImageCapture::Status::SETFMT_FAIL:   return "SETFMT_FAIL";
        case ImageCapture::Status::REQBUFS_FAIL:  return "REQBUFS_FAIL";
        case ImageCapture::Status::QUERYBUF_FAIL: return "QUERYBUF_FAIL";
        case ImageCapture::Status::MMAP_FAIL:     return "MMAP_FAIL";
        case ImageCapture::Status::QBUF_FAIL:     return "QBUF_FAIL";
        case ImageCapture::Status::STREAMON_FAIL: return "STREAMON_FAIL";
        case ImageCapture::Status::DQBUF_FAIL:    return "DQBUF_FAIL";
        default:                                  return "UNKNOWN";
    }
}

} // namespace vbn
