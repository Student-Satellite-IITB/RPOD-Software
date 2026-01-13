// tests/rnav_rnavfilter_test.cpp
//
// RNAV Filter unit test (synthetic) with CSV logging.
// Assumes msg::RNAVState has:
//   uint64_t t_meas_us;   // last fused measurement time (camera truth time propagated through pipeline)
//   uint64_t t_pub_us;    // publish time (local clock of the RNAV task)
//   uint64_t exec_us;     // measured CPU time inside filter loop (prop+optional update)
//   float    r_nav[3], v_nav[3], q_rel[4], w_rel[3];
//   float    P[12*12];
//   uint8_t  mode;
//
// Assumes msg::PoseEstimate has:
//   uint64_t t_exp_end_us; // measurement time (camera time from ImageCapture)
//   float    t_CbyP[3];
//   float    q_C_P[4];
//
// Writes CSV to: ../tools/data/tmp/rnav_filter_log.csv  (relative to build/)
// Creates directories if missing.
//
// NOTE: This is a test script; it does not need to follow flight-software principles.

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <random>
#include <filesystem>
#include <cerrno>
#include <cstring>
#include <cstdint>

#include "os/rtos.hpp"
#include "apps/vbn/RNAVFilter.hpp"
#include "msg/PoseEstimate.hpp"
#include "msg/RNAVState.hpp"

// ============================================================
// ======================= TEST SETTINGS =======================
// ============================================================

static const char*  CSV_PATH         = "../tools/data/tmp/rnav_filter_log.csv";

static constexpr float   T_SEC       = 10.0f;     // test duration (seconds)
static constexpr float   MEAS_HZ     = 100.0f;    // measurement producer rate
static constexpr uint64_t WAIT_US    = 200000ULL; // wait for fused state per measurement (us)
static constexpr int     STARTUP_MS  = 50;        // let RNAV task start

// Truth initial conditions
static constexpr float R0_X = 1.0f;
static constexpr float R0_Y = 0.5f;
static constexpr float R0_Z = -0.2f;

static constexpr float V0_X = 0.05f;
static constexpr float V0_Y = -0.02f;
static constexpr float V0_Z = 0.00f;

static constexpr float W0_X = 0.00f;
static constexpr float W0_Y = 0.00f;
static constexpr float W0_Z = 0.01f;

// Measurement noise (also copied into filter cfg here)
static constexpr float SIGMA_R     = 0.02f;   // meters
static constexpr float SIGMA_THETA = 0.005f;  // radians

// Filter settings
static constexpr uint32_t PROPAGATE_HZ = 100;
static constexpr uint64_t MEAS_TIMEOUT_US = 200000ULL;

// ============================================================
// ======================= TIME HELPERS ========================
// ============================================================

static uint64_t now_us() {
    return Rtos::NowUs();
}

static float randn(std::mt19937& rng, float sigma){
    std::normal_distribution<float> nd(0.0f, sigma);
    return nd(rng);
}

// ============================================================
// ==================== QUATERNION HELPERS =====================
// ============================================================

// Minimal quaternion helpers (scalar-first w,x,y,z)
static void quat_normalize(float q[4]) {
    const float n2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (n2 < 1e-12f) { q[0]=1; q[1]=q[2]=q[3]=0; return; }
    const float inv = 1.0f / std::sqrt(n2);
    q[0]*=inv; q[1]*=inv; q[2]*=inv; q[3]*=inv;
    if (q[0] < 0.0f) { q[0]=-q[0]; q[1]=-q[1]; q[2]=-q[2]; q[3]=-q[3]; }
}

static void quat_conj(const float q[4], float qc[4]) {
    qc[0] = q[0]; qc[1] = -q[1]; qc[2] = -q[2]; qc[3] = -q[3];
}

static void quat_mul(const float a[4], const float b[4], float out[4]) {
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
    quat_normalize(out);
}

// Exp map: theta (axis-angle vector, rad) -> quaternion
static void quat_exp(const float theta[3], float q[4]) {
    const float ax = theta[0], ay = theta[1], az = theta[2];
    const float a  = std::sqrt(ax*ax + ay*ay + az*az);
    const float half = 0.5f * a;

    if (a < 1e-6f) {
        q[0] = 1.0f;
        q[1] = 0.5f * ax;
        q[2] = 0.5f * ay;
        q[3] = 0.5f * az;
        quat_normalize(q);
        return;
    }

    const float s = std::sin(half);
    const float c = std::cos(half);
    const float k = s / a;

    q[0] = c;
    q[1] = k * ax;
    q[2] = k * ay;
    q[3] = k * az;
    quat_normalize(q);
}

// ============================================================
// ======================= CSV HELPERS =========================
// ============================================================

static std::ofstream open_csv(const std::string& path) {
    namespace fs = std::filesystem;
    fs::path p(path);

    if (p.has_parent_path()) {
        std::error_code ec;
        fs::create_directories(p.parent_path(), ec);
        if (ec) {
            std::cerr << "[TEST] ERROR: create_directories failed for "
                      << p.parent_path().string() << " : " << ec.message() << "\n";
        }
    }

    std::ofstream f(path, std::ios::out | std::ios::trunc);
    if (!f) {
        std::cerr << "[TEST] ERROR: could not open CSV: " << path
                  << " errno=" << errno << " (" << std::strerror(errno) << ")\n";
    }
    return f;
}

static void write_csv_header(std::ofstream& f) {
    // ONE LINE header (pandas-friendly)
    f << "t_sim,"
      << "t_meas_us,t_pub_us,rx_us,"
      << "pub_to_rx_us,meas_to_pub_us,meas_to_rx_us,"
      << "exec_us,state_period_us,"
      << "truth_r_x,truth_r_y,truth_r_z,"
      << "truth_v_x,truth_v_y,truth_v_z,"
      << "truth_q_w,truth_q_x,truth_q_y,truth_q_z,"
      << "truth_w_x,truth_w_y,truth_w_z,"
      << "meas_r_x,meas_r_y,meas_r_z,"
      << "meas_q_w,meas_q_x,meas_q_y,meas_q_z,"
      << "est_r_x,est_r_y,est_r_z,"
      << "est_v_x,est_v_y,est_v_z,"
      << "est_q_w,est_q_x,est_q_y,est_q_z,"
      << "est_w_x,est_w_y,est_w_z";

    for (int r=0; r<12; ++r) {
        for (int c=0; c<12; ++c) {
            f << ",P_" << r << "_" << c;
        }
    }
    f << "\n";
}

static void append_row(std::string& buf,
                       float t_sim,
                       uint64_t t_meas_us, uint64_t t_pub_us, uint64_t rx_us,
                       uint64_t pub_to_rx_us, uint64_t meas_to_pub_us, uint64_t meas_to_rx_us,
                       uint64_t exec_us, uint64_t state_period_us,
                       const float truth_r[3], const float truth_v[3], const float truth_q[4], const float truth_w[3],
                       const float meas_r[3],  const float meas_q[4],
                       const msg::RNAVState& s) {
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss << std::setprecision(9);

    ss << t_sim << ","
       << t_meas_us << "," << t_pub_us << "," << rx_us << ","
       << pub_to_rx_us << "," << meas_to_pub_us << "," << meas_to_rx_us << ","
       << exec_us << "," << state_period_us << ","
       << truth_r[0] << "," << truth_r[1] << "," << truth_r[2] << ","
       << truth_v[0] << "," << truth_v[1] << "," << truth_v[2] << ","
       << truth_q[0] << "," << truth_q[1] << "," << truth_q[2] << "," << truth_q[3] << ","
       << truth_w[0] << "," << truth_w[1] << "," << truth_w[2] << ","
       << meas_r[0]  << "," << meas_r[1]  << "," << meas_r[2]  << ","
       << meas_q[0]  << "," << meas_q[1]  << "," << meas_q[2]  << "," << meas_q[3]  << ","
       << s.r_nav[0] << "," << s.r_nav[1] << "," << s.r_nav[2] << ","
       << s.v_nav[0] << "," << s.v_nav[1] << "," << s.v_nav[2] << ","
       << s.q_rel[0] << "," << s.q_rel[1] << "," << s.q_rel[2] << "," << s.q_rel[3] << ","
       << s.w_rel[0] << "," << s.w_rel[1] << "," << s.w_rel[2];

    for (int i=0; i<12*12; ++i) ss << "," << s.P[i];
    ss << "\n";

    buf += ss.str();
}

// Wait until we see a state that corresponds to the measurement timestamp we just sent.
// Returns true if found; false on timeout.
static bool wait_for_fused_state(rnav::StateEstimateQueue& state_q,
                                uint64_t want_t_meas_us,
                                msg::RNAVState& out_state,
                                uint64_t timeout_us,
                                uint64_t* rx_us_out = nullptr) {
    const uint64_t t_start = now_us();
    msg::RNAVState last_seen{};
    uint64_t last_rx = 0;

    while (true) {
        const uint64_t elapsed = now_us() - t_start;
        if (elapsed >= timeout_us) {
            std::cerr << "[TEST] wait timed out. want_t_meas_us=" << want_t_meas_us
                      << " last_seen.t_meas_us=" << last_seen.t_meas_us
                      << " last_seen.t_pub_us="  << last_seen.t_pub_us
                      << " last_rx_us="          << last_rx
                      << "\n";
            return false;
        }

        const uint64_t rem_us = timeout_us - elapsed;
        int timeout_ms = (int)((rem_us + 999) / 1000);
        if (timeout_ms < 1) timeout_ms = 1;

        msg::RNAVState s{};
        if (!state_q.receive(s, timeout_ms)) continue;

        last_seen = s;
        last_rx = now_us();

        // Accept the first state that has fused THIS measurement or anything newer.
        if (s.t_meas_us != 0 && s.t_meas_us >= want_t_meas_us) {
            out_state = s;
            if (rx_us_out) *rx_us_out = last_rx;
            return true;
        }
    }
}

int main() {
    std::cout << "=== RNAV FILTER UNIT TEST (synthetic, CSV) ===\n";

    // Queues (overwrite newest)
    rnav::PoseMeasQueue meas_q(/*overwrite=*/true);
    rnav::StateEstimateQueue state_q(/*overwrite=*/true);

    // Filter config: identity extrinsics => FrameTransformation pass-through
    rnav::RNAVFilterConfig cfg{};
    cfg.PropagateHz = PROPAGATE_HZ;
    cfg.sigma_r = SIGMA_R;
    cfg.sigma_theta = SIGMA_THETA;
    cfg.enable_gating = false;

    cfg.H_CAMbyBc.t = rnav::Vec3::Zero();
    cfg.H_CAMbyBc.q = rnav::Quat::Identity();
    cfg.H_PATbyBt.t = rnav::Vec3::Zero();
    cfg.H_PATbyBt.q = rnav::Quat::Identity();

    cfg.meas_timeout_us = MEAS_TIMEOUT_US;

    rnav::RNAVFilter filter(cfg);

    // Task ctx
    rnav::RNAVFilter::TaskCtx ctx{};
    ctx.self = &filter;
    ctx.meas_in = &meas_q;
    ctx.state_out = &state_q;
    ctx.command_in = nullptr;

    // Start filter task
    Rtos::Task RNAVTask;
    RNAVTask.Create("RNAV", &rnav::RNAVFilter::TaskEntry, &ctx);

    // CSV
    const std::string csv_path = CSV_PATH;
    std::ofstream csv = open_csv(csv_path);
    if (!csv) {
        std::cerr << "FAIL: could not open CSV output file\n";
        return 1;
    }
    write_csv_header(csv);
    std::cout << "[TEST] CSV path: " << std::filesystem::absolute(csv_path).string() << "\n";

    std::string buf;
    buf.reserve(1 << 20);

    // ------------------------------
    // Synthetic truth (simulation-time)
    // ------------------------------
    float r_true[3] = {R0_X, R0_Y, R0_Z};
    float v_true[3] = {V0_X, V0_Y, V0_Z};

    float q_true[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float w_true[3] = {W0_X, W0_Y, W0_Z};

    std::mt19937 rng(123);

    const float meas_hz = MEAS_HZ;
    const float meas_dt = 1.0f / meas_hz;

    const int N = (int)std::ceil(T_SEC * meas_hz);
    float t_sim = 0.0f;

    // Let the filter task start and initialize cleanly
    Rtos::SleepMs(STARTUP_MS);

    // Timing aggregates (for quick prints)
    uint64_t last_pub_us = 0;

    uint64_t sum_pub_to_rx_us = 0;
    uint64_t sum_meas_to_pub_us = 0;
    uint64_t sum_meas_to_rx_us = 0;
    uint64_t sum_exec_us = 0;
    uint64_t sum_state_period_us = 0;
    uint64_t n_rows = 0;

    // Real-time pacing of measurement producer (prevents bursting)
    uint64_t next_send_us = now_us();

    for (int i = 0; i < N; ++i) {
        // advance simulation truth by meas_dt
        r_true[0] += v_true[0] * meas_dt;
        r_true[1] += v_true[1] * meas_dt;
        r_true[2] += v_true[2] * meas_dt;

        float dth_truth[3] = { w_true[0]*meas_dt, w_true[1]*meas_dt, w_true[2]*meas_dt };
        float dq_truth[4], qnew[4];
        quat_exp(dth_truth, dq_truth);
        quat_mul(dq_truth, q_true, qnew);
        q_true[0]=qnew[0]; q_true[1]=qnew[1]; q_true[2]=qnew[2]; q_true[3]=qnew[3];

        t_sim += meas_dt;

        // send measurement
        msg::PoseEstimate m{};
        const uint64_t t_meas_us = now_us();
        m.t_exp_end_us = t_meas_us;

        float meas_r[3];
        meas_r[0] = r_true[0] + randn(rng, cfg.sigma_r);
        meas_r[1] = r_true[1] + randn(rng, cfg.sigma_r);
        meas_r[2] = r_true[2] + randn(rng, cfg.sigma_r);

        m.t_CbyP[0] = meas_r[0];
        m.t_CbyP[1] = meas_r[1];
        m.t_CbyP[2] = meas_r[2];

        // attitude measurement: q_meas = Exp(noise) ⊗ q_true
        float ntheta[3] = {
            randn(rng, cfg.sigma_theta),
            randn(rng, cfg.sigma_theta),
            randn(rng, cfg.sigma_theta)
        };
        float qn[4], q_meas_arr[4];
        quat_exp(ntheta, qn);
        quat_mul(qn, q_true, q_meas_arr);

        m.q_C_P[0] = q_meas_arr[0];
        m.q_C_P[1] = q_meas_arr[1];
        m.q_C_P[2] = q_meas_arr[2];
        m.q_C_P[3] = q_meas_arr[3];

        meas_q.send(m, Rtos::MAX_TIMEOUT);

        // wait for the state that fused THIS measurement
        msg::RNAVState s{};
        uint64_t rx_us = 0;
        const bool ok = wait_for_fused_state(state_q, t_meas_us, s, WAIT_US, &rx_us);
        if (!ok) {
            std::cerr << "[TEST] FAIL: timed out waiting for fused state for t_meas_us=" << t_meas_us << "\n";
            return 1;
        }

        // timing metrics
        uint64_t pub_to_rx_us = (rx_us >= s.t_pub_us) ? (rx_us - s.t_pub_us) : 0;
        uint64_t meas_to_pub_us = (s.t_pub_us >= s.t_meas_us) ? (s.t_pub_us - s.t_meas_us) : 0;
        uint64_t meas_to_rx_us = (rx_us >= s.t_meas_us) ? (rx_us - s.t_meas_us) : 0;

        uint64_t state_period_us = 0;
        if (last_pub_us != 0 && s.t_pub_us > last_pub_us) state_period_us = s.t_pub_us - last_pub_us;
        last_pub_us = s.t_pub_us;

        // sanity: check NaNs in estimate/cov
        bool finite_ok = true;
        for (int kP=0;kP<12*12;kP++){
            if (!std::isfinite(s.P[kP])) { finite_ok = false; break; }
        }
        for (int j=0;j<3;j++){
            if (!std::isfinite(s.r_nav[j]) || !std::isfinite(s.v_nav[j]) || !std::isfinite(s.w_rel[j])) finite_ok = false;
        }
        for (int j=0;j<4;j++){
            if (!std::isfinite(s.q_rel[j])) finite_ok = false;
        }
        if (!finite_ok) {
            std::cerr << "FAIL: non-finite state/covariance\n";
            return 1;
        }

        append_row(buf,
                   t_sim,
                   t_meas_us, s.t_pub_us, rx_us,
                   pub_to_rx_us, meas_to_pub_us, meas_to_rx_us,
                   s.exec_us, state_period_us,
                   r_true, v_true, q_true, w_true,
                   meas_r, q_meas_arr,
                   s);

        // aggregates
        sum_pub_to_rx_us += pub_to_rx_us;
        sum_meas_to_pub_us += meas_to_pub_us;
        sum_meas_to_rx_us += meas_to_rx_us;
        sum_exec_us += s.exec_us;
        sum_state_period_us += state_period_us;
        n_rows++;

        if (buf.size() > (1 << 20)) {
            csv << buf;
            buf.clear();
        }

        // pacing
        next_send_us += (uint64_t)(1000000.0f / meas_hz);
        Rtos::SleepUntilUs(next_send_us);
    }

    csv << buf;
    csv.close();

    const double avg_pub_to_rx = (n_rows > 0) ? (double)sum_pub_to_rx_us / (double)n_rows : 0.0;
    const double avg_meas_to_pub = (n_rows > 0) ? (double)sum_meas_to_pub_us / (double)n_rows : 0.0;
    const double avg_meas_to_rx = (n_rows > 0) ? (double)sum_meas_to_rx_us / (double)n_rows : 0.0;
    const double avg_exec = (n_rows > 0) ? (double)sum_exec_us / (double)n_rows : 0.0;
    const double avg_state_period = (n_rows > 1) ? (double)sum_state_period_us / (double)(n_rows - 1) : 0.0;

    std::cout << "Wrote: " << csv_path << "\n";
    std::cout << "Rows: " << n_rows << "\n";
    std::cout << "Avg pub->rx (ms): " << avg_pub_to_rx/1000.0 << "\n";
    std::cout << "Avg meas->pub (ms): " << avg_meas_to_pub/1000.0 << "\n";
    std::cout << "Avg meas->rx (ms): " << avg_meas_to_rx/1000.0 << "\n";
    std::cout << "Avg exec (ms): " << avg_exec/1000.0 << "\n";
    std::cout << "Avg state period (ms): " << avg_state_period/1000.0 << "\n";
    std::cout << "=== DONE (no NaNs, CSV logged) ===\n";

    return 0;
}
