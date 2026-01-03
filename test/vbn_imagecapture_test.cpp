#include <iostream>
#include <cstdint>

#include "apps/vbn/ImageCapture.hpp"

// Only the test needs videodev2.h to get V4L2_PIX_FMT_* constants.
// Our core module can keep pixfmt as uint32_t.
#include <linux/videodev2.h>

#include <fstream>
#include <filesystem>

namespace fs = std::filesystem;

static void printStatus(const vbn::ImageCapture& cap, const char* where) {
    std::cout << "  [" << where << "] status="
              << vbn::ImageCapture::StatusStr(cap.lastStatus())
              << " errno=" << cap.lastErrno()
              << "\n";
}

static bool checkFrameBasic(const msg::ImageFrame& f) {
    if (!f.data) return false;
    if (f.width == 0 || f.height == 0) return false;
    if (f.stride == 0) return false;
    if (f.bytes_per_px == 0) return false;
    if (f.byteSize() == 0) return false;
    if (f.stride < f.width * f.bytes_per_px) return false;
    return true;
}

static bool writeRawFrame(const msg::ImageFrame& f, const std::string& path) {
    fs::path p(path);
    fs::create_directories(p.parent_path());

    std::ofstream ofs(path, std::ios::binary | std::ios::trunc);
    if (!ofs) return false;

    // Write the full MMAP-visible image region row-by-row (includes stride padding).
    const std::size_t nbytes = static_cast<std::size_t>(f.byteSize());
    ofs.write(reinterpret_cast<const char*>(f.data), static_cast<std::streamsize>(nbytes));
    return ofs.good();
}


int main(int argc, char** argv) {
    using namespace vbn;

    const char* dev = "/dev/video0";
    if (argc >= 2) dev = argv[1];

    const std::string out_path = "../tools/data/temp/frame.raw";

    std::cout << "=== vbn_imagecapture_test (RasPi) ===\n";
    std::cout << "Device: " << dev << "\n";

    ImageCaptureConfig cfg{};
    cfg.dev = dev;

    // Must match your provisioned media graph.
    cfg.width  = 1280;
    cfg.height = 800;
    cfg.v4l2_pixfmt = V4L2_PIX_FMT_Y16;

    cfg.buffer_count = 6;
    cfg.bit_depth = 10;
    cfg.bit_shift = 6;

    ImageCapture cap(cfg);

    std::cout << "\n[Test 0] Start()\n";
    if (!cap.Start()) {
        std::cout << "Start(): FAIL\n";
        printStatus(cap, "Start()");
        return 1;
    }
    std::cout << "Start(): OK\n";

    std::cout << "\n[Test 1] Dequeue/Requeue loop + save one frame\n";
    constexpr int N = 20;

    bool saved = false;
    uint64_t last_t = 0;
    bool first = true;

    uint64_t sum_dt_us = 0;
    int dt_count = 0;

    for (int k = 0; k < N; ++k) {
        msg::ImageFrame f{};

        if (!cap.Dequeue(f)) {
            std::cout << "Dequeue(): FAIL at k=" << k << "\n";
            printStatus(cap, "Dequeue()");
            cap.Stop();
            return 1;
        }

        if (!checkFrameBasic(f)) {
            std::cout << "Frame sanity: FAIL at k=" << k << "\n";
            (void)cap.Requeue(f.v4l2_buff_index);
            cap.Stop();
            return 1;
        }

        if (!saved) {
            const bool ok = writeRawFrame(f, out_path);
            if (!ok) {
                std::cout << "Failed to write raw frame to: " << out_path << "\n";
                // Requeue before exiting to avoid starving driver buffers.
                (void)cap.Requeue(f.v4l2_buff_index);
                cap.Stop();
                return 1;
            }
            std::cout << "Saved raw frame to: " << out_path
                      << " (" << f.byteSize() << " bytes)\n";
            saved = true;
        }

        if (!first && f.t_exp_end_us < last_t) {
            std::cout << "  WARN: timestamp went backwards (prev=" << last_t
                      << " now=" << f.t_exp_end_us << ")\n";
        }

        std::cout << "  k=" << k
                  << " idx=" << f.v4l2_buff_index
                  << " frame_id=" << f.frame_id
                  << " t_us=" << f.t_exp_end_us
                  << " stride=" << f.stride
                  << " bpp=" << int(f.bytes_per_px);

        if (!first) {
            const uint64_t dt_us = f.t_exp_end_us - last_t;
            const double fps_inst = (dt_us > 0) ? (1e6 / double(dt_us)) : 0.0;
            sum_dt_us += dt_us;
            dt_count++;
            std::cout << " dt_us=" << dt_us << " fps=" << fps_inst;
        }

        std::cout << std::endl;

        if (!cap.Requeue(f.v4l2_buff_index)) {
            std::cout << "Requeue(): FAIL at k=" << k << "\n";
            printStatus(cap, "Requeue()");
            cap.Stop();
            return 1;
        }

        last_t = f.t_exp_end_us;
        first = false;
    }

    if (dt_count > 0) {
        const double avg_dt_us = double(sum_dt_us) / double(dt_count);
        const double fps_avg = (avg_dt_us > 0) ? (1e6 / avg_dt_us) : 0.0;

        std::cout << "\nAverage over " << dt_count << " intervals: "
                << "avg_dt_us=" << avg_dt_us
                << " fps_avg=" << fps_avg << "\n";
    }

    std::cout << "\n[Test 2] Stop() idempotence\n";
    cap.Stop();
    cap.Stop();
    std::cout << "Stop() twice: OK\n";

    std::cout << "\nvbn_imagecapture_test: PASS\n";
    return 0;
}