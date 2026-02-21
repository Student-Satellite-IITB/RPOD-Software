#include "system/rcu/rcu.hpp"

// Pull in CubeMX-generated pin defines like LD2_GPIO_Port / LD2_Pin
extern "C" {
#include "main.h"
}

#include <cmath>
#include <cstring>
#include <cstdio>

#include "apps/vbn/RNAVFilter.hpp"
#include "msg/PoseEstimate.hpp"
#include "msg/RNAVState.hpp"
#include "os/rtos.hpp"

namespace {

// ---------- Quaternion helpers (from your test) ----------
static void quat_normalize(float q[4]) {
    const float n2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (n2 < 1e-12f) { q[0]=1; q[1]=q[2]=q[3]=0; return; }
    const float inv = 1.0f / std::sqrt(n2);
    q[0]*=inv; q[1]*=inv; q[2]*=inv; q[3]*=inv;
    if (q[0] < 0.0f) { q[0]=-q[0]; q[1]=-q[1]; q[2]=-q[2]; q[3]=-q[3]; }
}

static void quat_mul(const float a[4], const float b[4], float out[4]) {
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
    quat_normalize(out);
}

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

} // namespace


namespace rpod::system::rcu {

static Rtos::Queue<msg::PoseEstimate, 1> pose_q(/*overwrite=*/true);
static rnav::RNAVFilter* g_filter = nullptr;
static rnav::RNAVFilterConfig g_cfg{};

void init(void){
    // INIT 
    // uart_print("RCU init...\r\n");
    printf("\r\n[RCU] hello over COM1\r\n");
    // Config copied from your synthetic test defaults
    g_cfg.PropagateHz = 100;
    g_cfg.q_a = 1e-3f;
    g_cfg.q_alpha = 1e-2f;
    g_cfg.sigma_r = 0.02f;
    g_cfg.sigma_theta = 0.005f;
    g_cfg.enable_gating = false;
    g_cfg.meas_timeout_us = 200000ULL;

    // identity extrinsics (test assumes pass-through)
    g_cfg.H_CAMbyBc.t = rnav::Vec3::Zero();
    g_cfg.H_CAMbyBc.q = rnav::Quat::Identity();
    g_cfg.H_PATbyBt.t = rnav::Vec3::Zero();
    g_cfg.H_PATbyBt.q = rnav::Quat::Identity();

    static rnav::RNAVFilter filter(g_cfg);
    g_filter = &filter;

    // uart_write("[RCU] RNAVFilter constructed\r\n");
    printf("\r\n[RCU] RNAVFilter constructed\r\n");
}

static void synth_publish_pose_20hz()
{
    static float r_true[3] = {-1.0f, 0.05f, 0.03f};
    static float v_true[3] = { 0.02f, 0.00f,-0.001f};
    static float q_true[4] = { 1.0f, 0.0f, 0.0f, 0.0f};
    static float w_true[3] = { 0.0f, 0.0f, 0.0f};

    constexpr float dt = 0.05f; // 20 Hz
    r_true[0] += v_true[0] * dt;
    r_true[1] += v_true[1] * dt;
    r_true[2] += v_true[2] * dt;

    float dth[3] = { w_true[0]*dt, w_true[1]*dt, w_true[2]*dt };
    float dq[4], qnew[4];
    quat_exp(dth, dq);
    quat_mul(dq, q_true, qnew);
    q_true[0]=qnew[0]; q_true[1]=qnew[1]; q_true[2]=qnew[2]; q_true[3]=qnew[3];

    msg::PoseEstimate m{};
    m.t_exp_end_us = Rtos::NowUs()- 1000; // 1 ms earlier than 'now'
    m.t_CbyP[0] = r_true[0];
    m.t_CbyP[1] = r_true[1];
    m.t_CbyP[2] = r_true[2];

    m.q_C_P[0] = q_true[0];
    m.q_C_P[1] = q_true[1];
    m.q_C_P[2] = q_true[2];
    m.q_C_P[3] = q_true[3];

    m.reproj_rms_px = 0.5f;
    m.valid = 1;

    // overwrite mailbox: always succeeds
    pose_q.try_send(m);
}

static inline void pulse(GPIO_TypeDef* port, uint16_t pin, uint32_t ms){
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    HAL_Delay(ms);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}


void loop(void){
    // // LED Bouncing Up and Down LED1, LED2, LED3
    // pulse(LED1_GPIO_PORT, LED1_PIN, 80);
    // pulse(LED2_GPIO_PORT, LED2_PIN, 80);
    // pulse(LED3_GPIO_PORT, LED3_PIN, 80);
    // pulse(LED2_GPIO_PORT, LED2_PIN, 80);

    // Producer side (later this becomes "NCP RX handler publishes pose")
    synth_publish_pose_20hz();

    // Consumer side
    msg::PoseEstimate meas{};

    if (pose_q.try_receive(meas) && g_filter) {
        msg::RNAVState out{};
        const bool ok = g_filter->processMeasurement(meas, out);

        // Print at 1 Hz only (don’t spam UART)
        static int k = 0;
        if ((k++ % 20) == 0) {

            // Split u64 print like you already did (safe on embedded printf)
            const uint32_t lo = (uint32_t)(out.t_meas_us & 0xFFFFFFFFu);
            const uint32_t hi = (uint32_t)(out.t_meas_us >> 32);

            const int rx_mm = (int)(out.r_nav[0] * 1000.0f);
            const int ry_mm = (int)(out.r_nav[1] * 1000.0f);
            const int rz_mm = (int)(out.r_nav[2] * 1000.0f);

            printf("[RNAV] ok=%d t_meas_hi=%lu t_meas_lo=%lu r_mm=(%d %d %d)\r\n",
                   ok ? 1 : 0,
                   (unsigned long)hi,
                   (unsigned long)lo,
                   rx_mm, ry_mm, rz_mm);
        }
    }

    HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
    HAL_Delay(50); // roughly matches dt=0.05s
}

[[noreturn]] void start()
{
    // If you aren't using RTOS yet, just behave like a superloop.
    for (;;)
        loop();
}

} // namespace rpod::system::rcu