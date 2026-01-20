#include "GroundMonitor.hpp"

#include <iostream>
#include <fstream>
#include <filesystem>
#include <mutex>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <cerrno>
#include <cstring>

// POSIX sockets (Linux)
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include <cstdio>
#include <string>

namespace ground {

static constexpr double RAD2DEG = 180.0 / M_PI;

// ---------------- time ----------------
static uint64_t mono_us() {
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec) * 1000000ull + uint64_t(ts.tv_nsec) / 1000ull;
}

// Best-effort to get local IP (Linux only, uses hostname -I)
static std::string get_ip_hint() {
    std::string out;
    FILE* fp = ::popen("hostname -I 2>/dev/null | awk '{print $1}'", "r");
    if (!fp) return out;
    char buf[128];
    if (::fgets(buf, sizeof(buf), fp)) out = buf;
    ::pclose(fp);
    // trim
    while (!out.empty() && (out.back() == '\n' || out.back() == ' ' || out.back() == '\t')) out.pop_back();
    return out;
}

// ---------------- MJPEG shared state ----------------
namespace mjpeg {
static std::mutex g_mtx;
static std::vector<uint8_t> g_latest_jpg;
static std::atomic<uint64_t> g_latest_t_us{0};
static std::atomic<uint32_t> g_latest_frame_id{0};

static std::atomic<bool> g_started{false};

static void update_latest(const std::vector<uint8_t>& jpg, uint64_t t_us, uint32_t frame_id) {
    std::lock_guard<std::mutex> lk(g_mtx);
    g_latest_jpg = jpg;
    g_latest_t_us.store(t_us, std::memory_order_relaxed);
    g_latest_frame_id.store(frame_id, std::memory_order_relaxed);
}

static bool get_latest(std::vector<uint8_t>& out, uint64_t& t_us, uint32_t& frame_id) {
    std::lock_guard<std::mutex> lk(g_mtx);
    if (g_latest_jpg.empty()) return false;
    out = g_latest_jpg;
    t_us = g_latest_t_us.load(std::memory_order_relaxed);
    frame_id = g_latest_frame_id.load(std::memory_order_relaxed);
    return true;
}

static bool send_all(int fd, const uint8_t* data, size_t n) {
    while (n > 0) {
        ssize_t w = ::send(fd, data, n, MSG_NOSIGNAL);
        if (w <= 0) return false;
        data += (size_t)w;
        n -= (size_t)w;
    }
    return true;
}

static bool send_str(int fd, const std::string& s) {
    return send_all(fd, (const uint8_t*)s.data(), s.size());
}

static std::string index_html() {
    return "<!doctype html><html><head><meta charset='utf-8'>"
           "<title>VBN Monitor</title></head><body>"
           "<h3>VBN Monitor (MJPEG)</h3>"
           "<p><img src='/stream' style='max-width:100%; height:auto;'/></p>"
           "</body></html>";
}

static void handle_client(int cfd, int port, int stream_fps) {
    char buf[2048];
    int r = ::recv(cfd, buf, sizeof(buf) - 1, 0);
    if (r <= 0) return;
    buf[r] = '\0';
    std::string req(buf);

    std::string path = "/";
    {
        size_t p0 = req.find("GET ");
        if (p0 != std::string::npos) {
            size_t p1 = req.find(' ', p0 + 4);
            if (p1 != std::string::npos) path = req.substr(p0 + 4, p1 - (p0 + 4));
        }
    }

    if (path == "/" || path == "/index.html") {
        std::string body = index_html();
        std::string hdr =
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html; charset=utf-8\r\n"
            "Cache-Control: no-cache\r\n"
            "Connection: close\r\n"
            "Content-Length: " + std::to_string(body.size()) + "\r\n\r\n";
        send_str(cfd, hdr);
        send_str(cfd, body);
        return;
    }

    if (path != "/stream") {
        std::string body = "Not Found\n";
        std::string hdr =
            "HTTP/1.1 404 Not Found\r\n"
            "Content-Type: text/plain\r\n"
            "Connection: close\r\n"
            "Content-Length: " + std::to_string(body.size()) + "\r\n\r\n";
        send_str(cfd, hdr);
        send_str(cfd, body);
        return;
    }

    const std::string boundary = "frame";
    std::string hdr =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=" + boundary + "\r\n"
        "Cache-Control: no-cache\r\n"
        "Pragma: no-cache\r\n"
        "Connection: close\r\n\r\n";
    if (!send_str(cfd, hdr)) return;

    const int sleep_ms = (stream_fps > 0) ? (1000 / stream_fps) : 100;
    uint64_t last_sent = 0;

    while (true) {
        std::vector<uint8_t> jpg;
        uint64_t t_us = 0;
        uint32_t fid = 0;

        if (!get_latest(jpg, t_us, fid)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        if (t_us == last_sent) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
            continue;
        }
        last_sent = t_us;

        std::string part;
        part += "--" + boundary + "\r\n";
        part += "Content-Type: image/jpeg\r\n";
        part += "Content-Length: " + std::to_string(jpg.size()) + "\r\n\r\n";

        if (!send_str(cfd, part)) return;
        if (!send_all(cfd, jpg.data(), jpg.size())) return;
        if (!send_str(cfd, "\r\n")) return;

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
}

static void server_main(int port, int stream_fps) {
    int sfd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sfd < 0) {
        std::cout << "[MJPEG] socket() failed errno=" << errno << " " << std::strerror(errno) << "\n";
        return;
    }
    int yes = 1;
    ::setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);

    if (::bind(sfd, (sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cout << "[MJPEG] bind() failed port=" << port << " errno=" << errno << " " << std::strerror(errno) << "\n";
        ::close(sfd);
        return;
    }
    if (::listen(sfd, 4) < 0) {
        std::cout << "[MJPEG] listen() failed errno=" << errno << " " << std::strerror(errno) << "\n";
        ::close(sfd);
        return;
    }

    std::cout << "[MJPEG] Server started on port " << port << "\n";

    while (true) {
        sockaddr_in caddr{};
        socklen_t clen = sizeof(caddr);
        int cfd = ::accept(sfd, (sockaddr*)&caddr, &clen);
        if (cfd < 0) continue;
        handle_client(cfd, port, stream_fps);
        ::close(cfd);
    }
}

static void start_once(int port, int stream_fps) {
    bool expected = false;
    if (!g_started.compare_exchange_strong(expected, true)) return;
    std::thread([=]() { server_main(port, stream_fps); }).detach();
}

} // namespace mjpeg

// ---------------- image helpers ----------------
static cv::Mat y16_to_8u_preview(const vbn::CopiedImage& img) {
    cv::Mat m16((int)img.height, (int)img.width, CV_16UC1, (void*)img.data, img.stride);

    cv::Mat dn16(m16.size(), CV_16UC1);
    const int shift = (int)img.bit_shift;
    const uint16_t mask = (img.bit_depth > 0 && img.bit_depth < 16)
        ? (uint16_t)((1u << img.bit_depth) - 1u)
        : (uint16_t)0xFFFF;

    for (int y = 0; y < m16.rows; ++y) {
        const uint16_t* src = m16.ptr<uint16_t>(y);
        uint16_t* dst = dn16.ptr<uint16_t>(y);
        for (int x = 0; x < m16.cols; ++x) {
            dst[x] = (uint16_t)((src[x] >> shift) & mask);
        }
    }

    const double denom = (img.bit_depth >= 1 && img.bit_depth <= 16)
        ? double((1u << img.bit_depth) - 1u)
        : 65535.0;

    cv::Mat m8;
    dn16.convertTo(m8, CV_8U, 255.0 / denom);
    return m8;
}

static void annotate(cv::Mat& bgr, const msg::FeatureFrame& feat, const msg::PoseEstimate& pose) {

    // Crosshairs
    cv::line(bgr, {0, bgr.rows/2}, {bgr.cols-1, bgr.rows/2}, {255,255,255}, 1);
    cv::line(bgr, {bgr.cols/2, 0}, {bgr.cols/2, bgr.rows-1}, {255,255,255}, 1);

    // LEDs annotatiion
    for (int i = 0; i < (int)feat.feat_count; ++i) {
        const auto& L = feat.feats[i];
        cv::Point pt((int)std::lround(L.u_px), (int)std::lround(L.v_px));
        cv::circle(bgr, pt, 20, cv::Scalar(0,0,255), 2);
        cv::drawMarker(bgr, pt, cv::Scalar(0,255,0), cv::MARKER_CROSS, 50, 2);
        cv::putText(bgr, std::to_string(i), pt + cv::Point(5, -5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 1);
    }

    int y = 28;
    auto put = [&](const std::string& s) {
        cv::putText(bgr, s, cv::Point(10, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
        y += 28;
    };

    put("FD: " + std::string(feat.state == msg::TrackState::TRACK ? "TRACK" : "LOST")
        + " leds=" + std::to_string((int)feat.feat_count));

    if (pose.valid) {
        const double az    = pose.az    * RAD2DEG;
        const double el    = pose.el    * RAD2DEG;
        const double roll  = pose.roll  * RAD2DEG;
        const double pitch = pose.pitch * RAD2DEG;
        const double yaw   = pose.yaw   * RAD2DEG;
        const double range_cm = pose.range_m * 100.0;

        put("SPE: OK");
        {
            char s[128];
            std::snprintf(s, sizeof(s), "Az=%.1fdeg  El=%.1fdeg", az, el);
            put(s);
        }
        {
            char s[128];
            std::snprintf(s, sizeof(s), "RPY(deg)=[%.2f, %.2f, %.2f]", roll, pitch, yaw);
            put(s);
        }
        {
            char s[128];
            std::snprintf(s, sizeof(s), "range=%.2fcm  rms=%.2fpx", range_cm, pose.reproj_rms_px);
            put(s);
        }
    } else {
        put("SPE: NONE");
    }
}

// ---------------- CSV logging ----------------
static std::ofstream open_csv(
    const GroundMonitorCtx& ctx,
    const std::string& filename,      // e.g. "range_log.csv"
    const std::string& header_line    // e.g. "k,t_us,range_cm,reproj_rms_px\n"
) {
    std::filesystem::create_directories(ctx.cfg.out_dir);
    const std::string path = ctx.cfg.out_dir + "/" + filename;

    std::ofstream f(path, std::ios::out | std::ios::trunc);
    if (!f) {
        std::cout << "[MONITOR] ERROR: could not open CSV: " << path
                  << " errno=" << errno << " (" << std::strerror(errno) << ")\n";
        return f;
    }

    std::cout << "[MONITOR] CSV open: " << path << "\n";
    if (!header_line.empty()) {
        f << header_line;
    }
    return f;
}

// ---------------- task entry ----------------
void TaskEntry(void* arg) {
    auto* ctx = static_cast<GroundMonitorCtx*>(arg);
    if (!ctx || (!ctx->feat_in && !ctx->pose_in)) return;

    // Force snapshots if server enabled 
    ctx->cfg.enable_snapshots = ctx->cfg.enable_server;

    std::cout << "[MONITOR] started\n";

    if (ctx->cfg.enable_server) {
        mjpeg::start_once(ctx->cfg.port, ctx->cfg.stream_fps);
    }

    if (ctx->cfg.enable_server) {
        const int port = ctx->cfg.port;
        const std::string ip = get_ip_hint();

        std::cout << "[MONITOR] MJPEG UI:\n";
        std::cout << "  - http://vbn.local:" << port << "/\n";
        if (!ip.empty()) {
            std::cout << "  - http://" << ip << ":" << port << "/\n";
        }
        std::cout << "  (Open in your PC browser)\n";
    }

    std::ofstream csv;
    if (ctx->cfg.enable_csv) {
        if(ctx->cfg.testcase == Test::RANGE_LOG){
            csv = open_csv(
                *ctx,
                "range_log.csv",
                "k,t_us,range_cm,reproj_rms_px\n"
            );
        }
        if(ctx->cfg.testcase == Test::CENTROID_LOG){
            csv = open_csv(
                *ctx,
                "centroid_log.csv",
                "k,t_us,blob_i,u_px,v_px,area,intensity\n"
            );
        }
    }

    constexpr uint64_t PRINT_PERIOD_US = 1'000'000; // 1Hz Printing
    uint64_t last_print_us = mono_us();
    uint64_t last_snap_us = mono_us();

    uint32_t pose_count = 0;
    uint32_t feat_count = 0;
    uint32_t state_count = 0;

    msg::PoseEstimate last_pose{};
    msg::FeatureFrame last_feat{};
    msg::RNAVState last_state{};

    uint32_t k = 0;
    uint32_t logged = 0;

    while (true) {
        bool got_new_feat = false;
        bool got_new_pose = false;
        bool got_new_state = false;

        if (ctx->feat_in) {
            msg::FeatureFrame f{};
            while (ctx->feat_in->try_receive(f)) {
                last_feat = f;
                got_new_feat = true;
                feat_count++;
            }
        }

        if (ctx->pose_in) {
            msg::PoseEstimate p{};
            while (ctx->pose_in->try_receive(p)) {
                last_pose = p;
                got_new_pose = true;
                pose_count++;
            }
        }

        if (ctx->state_in){
            msg::RNAVState s{};
            while (ctx->state_in->try_receive(s)) {
                last_state = s;
                got_new_state = true;
                state_count++;
            }
        }

        if (ctx->cfg.enable_csv && csv && logged < ctx->cfg.log_n) {

            // -------- RANGE LOG (unchanged behavior) --------
            if (ctx->cfg.testcase == Test::RANGE_LOG) {
                if (got_new_pose && (k % ctx->cfg.log_every == 0)) {
                    csv << logged << "," << last_pose.t_exp_end_us << ","
                        << (last_pose.range_m * 100.0) << ","
                        << last_pose.reproj_rms_px << "\n";
                    logged++;
                    if (logged == ctx->cfg.log_n) {
                        csv.flush();
                        csv.close();
                        std::cout << "[MONITOR] Wrote range_log.csv (" << ctx->cfg.log_n << " rows)\n";
                    }
                }
                k++;
            }

            // -------- CENTROID LOG (blob centroids only) --------
            else if (ctx->cfg.testcase == Test::CENTROID_LOG) {
                if (got_new_feat && (k % ctx->cfg.log_every == 0) && last_feat.valid) {

                    // Log ALL blobs/features as-is (no validity/pattern assumptions)
                    const uint8_t n = last_feat.feat_count;
                    for (uint8_t i = 0; i < n; ++i) {
                        const msg::Feature& b = last_feat.feats[i];
                        csv << logged << ","
                            << last_feat.t_exp_end_us << ","
                            << int(i) << ","
                            << b.u_px << ","
                            << b.v_px << ","
                            << b.area << ","
                            << b.intensity << "\n";
                    }

                    // Here: logged counts "frames logged", not "rows"
                    logged++;
                    std::cout <<"LOGGED : "<< logged << "\n";
                    if (logged == ctx->cfg.log_n) {
                        csv.flush();
                        csv.close();
                        std::cout << "[MONITOR] Wrote centroid_log.csv (" << ctx->cfg.log_n << " frames)\n";
                    }
                }
                k++;
            }
        }

        const uint64_t now = mono_us();

        // request copy at slow cadence
        if (ctx->cfg.enable_snapshots && ctx->vbn) {
            const uint64_t period_us = uint64_t(ctx->cfg.snapshot_period_ms) * 1000ull;
            if (now - last_snap_us >= period_us) {
                ctx->vbn->RequestCopy(1);
                last_snap_us = now;
            }
        }

        // drain copied frames: keep newest, release rest
        if (ctx->cfg.enable_snapshots && ctx->vbn) {
            vbn::CopiedImage ci{};
            vbn::CopiedImage newest{};
            bool have = false;

            while (ctx->vbn->TryReceiveCopied(ci)) {
                if (have) ctx->vbn->ReleaseCopied(newest);
                newest = ci;
                have = true;
            }

            if (have) {
                cv::Mat gray8 = y16_to_8u_preview(newest);
                cv::Mat bgr;
                cv::cvtColor(gray8, bgr, cv::COLOR_GRAY2BGR);
                annotate(bgr, last_feat, last_pose);

                std::vector<uint8_t> jpg;
                std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, ctx->cfg.jpeg_quality};
                cv::imencode(".jpg", bgr, jpg, params);

                mjpeg::update_latest(jpg, newest.t_us, newest.frame_id);
                ctx->vbn->ReleaseCopied(newest);
            }
        }

        // PRINT ON CONSOLE
        if (now - last_print_us >= PRINT_PERIOD_US) {
            const double dt_s = double(now - last_print_us) * 1e-6;
            const double pose_hz = (dt_s > 0.0) ? (double(pose_count) / dt_s) : 0.0;
            const double feat_hz = (dt_s > 0.0) ? (double(feat_count) / dt_s) : 0.0;

            std::cout << "[Monitor] feat_hz=" << feat_hz
                    << " pose_hz=" << pose_hz << "\n";

            const bool new_feat = (feat_count > 0);
            const bool new_pose = (pose_count > 0);
            const bool new_state = (state_count > 0);

            if (new_feat) {
                std::cout << "[FD] State = " << (last_feat.state == msg::TrackState::TRACK ? "TRACK" : "LOST") << "\n";
                std::cout << "[FD] LEDs detected = " << int(last_feat.feat_count) << "\n";
            } else {
                std::cout << "[FD] NONE\n";
            }

            if (new_pose && last_pose.valid) {
                double az_deg = last_pose.az * RAD2DEG;
                double el_deg = last_pose.el * RAD2DEG;
                double roll_deg  = last_pose.roll  * RAD2DEG;
                double pitch_deg = last_pose.pitch * RAD2DEG;
                double yaw_deg   = last_pose.yaw   * RAD2DEG;
                double range_cm  = last_pose.range_m * 100.0;

                std::cout << "[SPE] OK\n";
                std::cout << "      Azimuth  = " << az_deg << " deg\n";
                std::cout << "      Elevation = " << el_deg << " deg\n";
                std::cout << "      Roll  = " << roll_deg  << " deg\n";
                std::cout << "      Pitch = " << pitch_deg << " deg\n";
                std::cout << "      Yaw   = " << yaw_deg   << " deg\n";
                std::cout << "      Range = " << range_cm  << " cm\n";
                std::cout << "      Reproj RMS = " << last_pose.reproj_rms_px << " px\n";
            } else {
                std::cout << "[SPE] NONE\n";
            }

            
            if (new_state) {
                std::cout << "[RNAV] OK\n";

                std::cout << std::fixed << std::setprecision(3);

                std::cout << " Relative Position r_nav [m]   = ["
                        << last_state.r_nav[0] << ", "
                        << last_state.r_nav[1] << ", "
                        << last_state.r_nav[2] << "]\n";

                std::cout << " Relative Velocity v_nav [m/s] = ["
                        << last_state.v_nav[0] << ", "
                        << last_state.v_nav[1] << ", "
                        << last_state.v_nav[2] << "]\n";

                std::cout << " Relative Orientation q_rel    = ["
                        << last_state.q_rel[0] << ", "
                        << last_state.q_rel[1] << ", "
                        << last_state.q_rel[2] << ", "
                        << last_state.q_rel[3] << "]\n";

                std::cout << " Relative Rate w_rel [rad/s]   = ["
                        << last_state.w_rel[0] << ", "
                        << last_state.w_rel[1] << ", "
                        << last_state.w_rel[2] << "]\n";

                // Covariance diagonals (assumed 12-state ordering):
                // [pos(0..2), vel(3..5), angle_error(6..8), rate(9..11)]
                auto Pdiag = [&](int i) -> float { return last_state.P[i * 12 + i]; };

                std::cout << " Covariance diag (P):\n";
                std::cout << "   pos  [m^2]        = ["
                        << Pdiag(0) << ", " << Pdiag(1) << ", " << Pdiag(2) << "]\n";
                std::cout << "   vel  [(m/s)^2]    = ["
                        << Pdiag(3) << ", " << Pdiag(4) << ", " << Pdiag(5) << "]\n";
                std::cout << "   ang  [rad^2]      = ["
                        << Pdiag(6) << ", " << Pdiag(7) << ", " << Pdiag(8) << "]\n";
                std::cout << "   rate [(rad/s)^2]  = ["
                        << Pdiag(9) << ", " << Pdiag(10) << ", " << Pdiag(11) << "]\n";
            }

            std::cout << "\n";

            last_print_us = now;
            pose_count = 0;
            feat_count = 0;
        }


        // Optional: Yield CPU, don't spin.
        // Commented for Full Speeed
        Rtos::SleepMs(12.5);
    }
}

} // namespace ground
