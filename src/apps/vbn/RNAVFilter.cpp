// RNAVFilter.cpp
#include "apps/vbn/RNAVFilter.hpp"

#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace{

using EVec3 = Eigen::Vector3f;
using EQuat = Eigen::Vector4f; // (w,x,y,z)
using EMat12 = Eigen::Matrix<float,12,12>;
using EMat6  = Eigen::Matrix<float, 6, 6>;
using EMat12x6 = Eigen::Matrix<float,12,6>;
using EMat6x12 = Eigen::Matrix<float, 6,12>;
using EVec12 = Eigen::Matrix<float,12,1>;
using EVec6  = Eigen::Matrix<float, 6,1>;

static inline float clampf(float x, float lo, float hi){
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

// struct EPose{
//     EVec3 t; // translation vector
//     EQuat q; // orientation quaternion (w,x,y,z)
// };

// Eigen Utils
static inline EVec3 toE(const rnav::Vec3& v){ return {v.x, v.y, v.z}; }
static inline rnav::Vec3 fromE(const EVec3& v){ return {v.x(), v.y(), v.z()}; }

static inline EQuat toE(const rnav::Quat& q){ return {q.w, q.x, q.y, q.z}; }
static inline rnav::Quat fromE(const EQuat& q){ return {q[0], q[1], q[2], q[3]}; }

static inline EMat12 toE(const rnav::Mat12& M){
    EMat12 EM;
    for(int r=0;r<12;r++)
        for(int c=0;c<12;c++)
            EM(r,c) = M.data[r*12 + c];
    return EM;
}
static inline rnav::Mat12 fromE(const EMat12& EM){
    rnav::Mat12 M;
    for(int r=0;r<12;r++)
        for(int c=0;c<12;c++)
            M.data[r*12 + c] = EM(r,c);
    return M;
} 

// ------------------------------
// ------  Utils ------
// ------------------------------

// ------------------------------
// Quaternion ops on POD (w,x,y,z)
// ------------------------------
static inline rnav::Quat quat_normalize(const rnav::Quat& q_in) {

    const float n2 = q_in.w*q_in.w + q_in.x*q_in.x + q_in.y*q_in.y + q_in.z*q_in.z;
    if (n2 < 1e-12f) return {1,0,0,0};

    const float invn = 1.0f / std::sqrt(n2);
    rnav::Quat q { q_in.w*invn, q_in.x*invn, q_in.y*invn, q_in.z*invn };

    // enforce w >= 0 to reduce sign flips
    if (q.w < 0.0f) { q.w=-q.w; q.x=-q.x; q.y=-q.y; q.z=-q.z; }

    return q;
}

static inline rnav::Quat quat_conj(const rnav::Quat& q_in) {
    // conj([w,x,y,z]) = [w,-x,-y,-z]
    return { q_in.w, -q_in.x, -q_in.y, -q_in.z };
}

static inline rnav::Quat quat_mul(const rnav::Quat& a_in, const rnav::Quat& b_in) {
    // Hamilton product (scalar-first w,x,y,z)
    const rnav::Quat a = quat_normalize(a_in);
    const rnav::Quat b = quat_normalize(b_in);

    rnav::Quat out;
    out.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    out.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    out.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    out.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;

    return quat_normalize(out);
}

static inline rnav::Vec3 rotate(const rnav::Quat& q_in, const rnav::Vec3& v_in) {
    // v' = v + qw*(2 qv×v) + qv×(2 qv×v)
    const rnav::Quat qN = quat_normalize(q_in);

    const EVec3 v = toE(v_in);
    const EVec3 qv(qN.x, qN.y, qN.z);
    const float qw = qN.w;

    const EVec3 t = 2.0f * qv.cross(v);
    const EVec3 vprime = v + qw * t + qv.cross(t);
    return fromE(vprime);
}

// ------------------------------
// Pose ops on POD
// Pose (A/B): q rotates B->A, t is B-origin->A-origin expressed in A.
// ------------------------------
static inline rnav::Pose pose_inverse(const rnav::Pose& H_AbyB) {
    rnav::Pose H_BbyA;
    const rnav::Quat qA_B = quat_normalize(H_AbyB.q);

    H_BbyA.q = quat_normalize(quat_conj(qA_B));           // q_B_A
    H_BbyA.t = {0,0,0};

    // t_B_A = -R(q_B_A) * t_A_B
    const rnav::Vec3 tA_B = H_AbyB.t;
    const rnav::Vec3 Rt = rotate(H_BbyA.q, tA_B);
    H_BbyA.t = {-Rt.x, -Rt.y, -Rt.z};

    return H_BbyA;
}

static inline rnav::Pose pose_compose(const rnav::Pose& H_AbyB,
                                      const rnav::Pose& H_BbyC) {
    rnav::Pose H_AbyC;

    const rnav::Quat qA_B = quat_normalize(H_AbyB.q);
    const rnav::Quat qB_C = quat_normalize(H_BbyC.q);

    H_AbyC.q = quat_mul(qA_B, qB_C);

    // t_A_C = t_A_B + R(qA_B) * t_B_C
    const rnav::Vec3 Rt = rotate(qA_B, H_BbyC.t);
    H_AbyC.t = { H_AbyB.t.x + Rt.x,
                 H_AbyB.t.y + Rt.y,
                 H_AbyB.t.z + Rt.z };

    return H_AbyC;
}

// Continuous-time white acceleration noise discretized over dt (3D).
// Model: rdot = v, vdot = w_a(t),  E[w_a(t) w_a(τ)^T] = q_a * δ(t-τ) * I
// Discrete Q_rv = q_a * [[dt^3/3 I, dt^2/2 I],
//                       [dt^2/2 I, dt     I]]
// NOTE: q_a is a noise intensity (PSD) with units m^2/s^3 (NOT m/s^2).
static inline void add_Q_rv(EMat12& Q, float dt, float q_a){
    if (dt <= 0.0f || q_a <= 0.0f) return;

    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;

    const float q_rr = q_a * (dt3 / 3.0f);
    const float q_rv = q_a * (dt2 / 2.0f);
    const float q_vv = q_a * (dt);

    // indices: dr(0..2), dv(3..5)
    for(int i=0;i<3;i++){
        Q(i,i)       += q_rr;
        Q(i,3+i)     += q_rv;
        Q(3+i,i)     += q_rv;
        Q(3+i,3+i)   += q_vv;
    }
}

// Continuous-time white angular-acceleration noise discretized over dt (3D).
// Model: dtheta_dot = domega, domega_dot = w_alpha(t),
//        E[w_alpha(t) w_alpha(τ)^T] = q_alpha * δ(t-τ) * I
// Discrete Q_thw = q_alpha * [[dt^3/3 I, dt^2/2 I],
//                            [dt^2/2 I, dt     I]]
// NOTE: q_alpha is a noise intensity (PSD) with units rad^2/s^3 (NOT rad/s^2).
static inline void add_Q_thw(EMat12& Q, float dt, float q_alpha){
    if (dt <= 0.0f || q_alpha <= 0.0f) return;

    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;

    const float q_tt = q_alpha * (dt3 / 3.0f);
    const float q_tw = q_alpha * (dt2 / 2.0f);
    const float q_ww = q_alpha * (dt);

    // indices: dtheta(6..8), dw(9..11)
    for(int i=0;i<3;i++){
        Q(6+i,6+i)     += q_tt;
        Q(6+i,9+i)     += q_tw;
        Q(9+i,6+i)     += q_tw;
        Q(9+i,9+i)     += q_ww;
    }
}


static inline rnav::Quat ExpQ(const EVec3& theta) {
    const float a = theta.norm();         // angle (rad)
    const float half = 0.5f * a;

    rnav::Quat q;

    if (a < 1e-6f) {
        // sin(half)/a ≈ 0.5 for small a
        q.w = 1.0f;
        q.x = 0.5f * theta.x();
        q.y = 0.5f * theta.y();
        q.z = 0.5f * theta.z();
        return quat_normalize(q);
    }

    const float s = std::sin(half);
    const float c = std::cos(half);
    const float k = s / a;

    q.w = c;
    q.x = k * theta.x();
    q.y = k * theta.y();
    q.z = k * theta.z();
    return quat_normalize(q);
}

static inline EVec3 LogQ(const rnav::Quat& q_in) {
    rnav::Quat q = quat_normalize(q_in);

    // shortest rotation
    if (q.w < 0.0f) { q.w=-q.w; q.x=-q.x; q.y=-q.y; q.z=-q.z; }

    const float vnorm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z);

    if (vnorm < 1e-8f) {
        // For tiny angles, theta ≈ 2*v (since q ≈ [1, 0.5*theta])
        return EVec3(2.0f*q.x, 2.0f*q.y, 2.0f*q.z);
    }

    // alpha = 2*atan2(|v|, w) in [0, pi]
    const float alpha = 2.0f * std::atan2(vnorm, q.w);

    // axis = v / |v|
    const float k = alpha / vnorm;
    return EVec3(k*q.x, k*q.y, k*q.z);
}


} // namespace


namespace rnav {

// Internal sanitisation of config parameters
// not public API
static inline RNAVFilterConfig sanitise(const RNAVFilterConfig& in) {
    RNAVFilterConfig cfg = in;
    // Add sanitisation of config parameters as needed

    if (cfg.PropagateHz == 0) {
        cfg.PropagateHz = 1; // minimum 1 Hz
    }

    cfg.H_CAMbyBc.q = quat_normalize(cfg.H_CAMbyBc.q);
    cfg.H_PATbyBt.q = quat_normalize(cfg.H_PATbyBt.q);

    if (cfg.meas_timeout_us == 0) cfg.meas_timeout_us = 200000; // default 200ms

    return cfg;
}

RNAVFilter::RNAVFilter(const RNAVFilterConfig& cfg)
    : m_cfg(sanitise(cfg)){
}

void RNAVFilter::setConfig(const RNAVFilterConfig& cfg) {
    m_cfg = sanitise(cfg);
}

void RNAVFilter::TaskEntry(void* arg) {
    auto* ctx = static_cast<TaskCtx*>(arg);

    if (!ctx || !ctx->self || !ctx->meas_in || !ctx->state_out) {
        return; // invalid context
    }

    // Initialise
    ctx->self->Init(*ctx->meas_in);

    // Run loop
    ctx->self->Run(*ctx->meas_in, *ctx->state_out, 
                    ctx->command_in);
}

void RNAVFilter::Run(PoseMeasQueue& meas_in, StateEstimateQueue& state_out, 
                     CmdQueue* command_in) {
    // Main run loop
    // No command queue handling yet

    msg::PoseEstimate meas{};
    RNAVMeasurement z{};
    msg::RNAVState state{};

    const uint64_t period_us = (m_cfg.PropagateHz > 0)
        ? (1000000ULL / (uint64_t)m_cfg.PropagateHz)
        : 1000000ULL;

    uint64_t last_tick_us = Rtos::NowUs();
    uint64_t next_tick_us = last_tick_us + period_us;

    while(true){
        
        // Initialise if state lost during Run
        if(!m_has_initialized){
            if(!Init(meas_in)){
                continue; // try again
            }
            m_has_initialized = true;

            last_tick_us = Rtos::NowUs();
            next_tick_us = last_tick_us + period_us;
        }

        // --- tick start time ---
        const uint64_t t0_us = Rtos::NowUs();

        // dt from actual time
        uint64_t dt_us = (t0_us > last_tick_us) ? (t0_us - last_tick_us) : 0;
        last_tick_us = t0_us;

        // Clamp dt to avoid blow-ups if the task is paused/stalled
        // (pick bounds that make sense for your mission)
        const float dt_s = std::min(std::max((float)dt_us * 1e-6f, 0.0f), 0.1f);

        // PROPAGATE
        // const float dt_s = 1.0f / static_cast<float>(m_cfg.PropagateHz);
        Propagate(dt_s);

        // UPDATE
        // Check for new measurement
        bool fused = false;
        if(meas_in.try_receive(meas)){

            const uint64_t now = Rtos::NowUs();
            if(AcceptMeasurement(meas,now)){
                m_meas_available = true;
                m_last_meas_us = meas.t_exp_end_us;

                FrameTransformation(meas, z);
                fused = Update(z);
                if (fused) {
                    m_last_fused_meas_us = meas.t_exp_end_us;
                }
            } else{
                m_meas_available = false;
            }
        }
        else{
            m_meas_available = false;
        }

        const uint64_t t1_us = Rtos::NowUs();
        const uint64_t exec_us = (t1_us >= t0_us) ? (t1_us - t0_us) : 0;

        // Publish state
        state.t_meas_us = m_last_fused_meas_us;   // 0 if never fused
        state.t_pub_us  = t1_us;
        state.exec_us   = exec_us;

        state.r_nav[0] = m_x.r_BcbyBt.x;
        state.r_nav[1] = m_x.r_BcbyBt.y;
        state.r_nav[2] = m_x.r_BcbyBt.z;

        state.v_nav[0] = m_x.v_BcbyBt.x;
        state.v_nav[1] = m_x.v_BcbyBt.y;
        state.v_nav[2] = m_x.v_BcbyBt.z;

        state.q_rel[0] = m_x.q_BcbyBt.w;
        state.q_rel[1] = m_x.q_BcbyBt.x;
        state.q_rel[2] = m_x.q_BcbyBt.y;
        state.q_rel[3] = m_x.q_BcbyBt.z;

        state.w_rel[0] = m_x.w_BcbyBt.x;
        state.w_rel[1] = m_x.w_BcbyBt.y;
        state.w_rel[2] = m_x.w_BcbyBt.z;

        // Covariance diagonal
        // Copy full covariance (row-major)
        for (int i = 0; i < 12*12; ++i) {
            state.P[i] = m_P.data[i];
        }

        // I have heard contradicting opinions
        // and results about std::memcpy being faster than a for loop
        // std::memcpy(state.P, m_P.data, sizeof(state.P));
        // Lets optimize only if we need!

        state_out.send(state, Rtos::MAX_TIMEOUT);

        // --- schedule next tick (absolute) ---
        // If we're behind by more than one period, resync to "now + period" to avoid runaway catch-up.
        uint64_t now2 = Rtos::NowUs();
        if (now2 > next_tick_us + period_us) {
            next_tick_us = now2 + period_us;
        } else {
            next_tick_us += period_us;
        }
        Rtos::SleepUntilUs(next_tick_us);
    }
}

bool RNAVFilter::Init(PoseMeasQueue& meas_in) {
    // Initialisation logic here
    msg::PoseEstimate init_meas{};
    RNAVMeasurement z{};
    RNAVInternalState xhat0{};

    // Block until first measurement is available
    if(!meas_in.receive(init_meas, Rtos::MAX_TIMEOUT)){
        // Can add fallback logic here
        return false; // no measurement available
    }
    
    const uint64_t now = Rtos::NowUs();
    if(!AcceptMeasurement(init_meas,now)){
        return false;
    }

    m_last_meas_us = init_meas.t_exp_end_us;
    m_last_fused_meas_us = init_meas.t_exp_end_us;  // so t_meas_us starts meaningful

    // Measurement Frame Transformation
    FrameTransformation(init_meas, z);

    // Initialise state estimate
    xhat0.r_BcbyBt = z.r_BcbyBt;
    xhat0.v_BcbyBt = Vec3::Zero();
    xhat0.q_BcbyBt = z.q_BcbyBt;
    xhat0.w_BcbyBt = Vec3::Zero();

    m_x = xhat0;

    // initial covariance identity
    m_P = Mat12::Identity(); 

    m_has_initialized = true;
    return true;
}

void RNAVFilter::Propagate(float dt_s){
    // Implement propagation logic here

    // Constant velocity model
    m_x.r_BcbyBt.x += m_x.v_BcbyBt.x * dt_s;
    m_x.r_BcbyBt.y += m_x.v_BcbyBt.y * dt_s;
    m_x.r_BcbyBt.z += m_x.v_BcbyBt.z * dt_s;

    m_x.v_BcbyBt = m_x.v_BcbyBt; // constant

    // Constant angular velocity model
    EVec3 w = toE(m_x.w_BcbyBt);
    EVec3 dtheta = w * dt_s;
    
    // Update orientation
    // delta_q = ExpQ(dtheta)
    // m_x.q_BcbyBt = quatmul(delta_q, m_x.q_BcbyBt)
    // m_x.w_BcbyBt = m_x.w_BcbyBt; // constant

    // dq = ExpQ(dtheta)
    rnav::Quat dq = ExpQ(dtheta);
    // left-multiply: applies rotation in current body frame convention
    m_x.q_BcbyBt = quat_mul(dq, m_x.q_BcbyBt);
    m_x.q_BcbyBt = quat_normalize(m_x.q_BcbyBt);

    m_x.w_BcbyBt = m_x.w_BcbyBt; // constant

    // Covariance propagation for error-state:
    // dx = [dr dv dtheta dw]
    // F ≈ [[I dtI 0  0],
    //      [0  I  0  0],
    //      [0  0  I dtI],
    //      [0  0  0  I ]]
    const float dt = dt_s;
    EMat12 F = EMat12::Identity();
    for(int i=0;i<3;i++){
        F(i,3+i)     = dt;  // dr depends on dv
        F(6+i,9+i)   = dt;  // dtheta depends on dw
    }

    EMat12 Q = EMat12::Zero();
    add_Q_rv(Q, dt, m_cfg.q_a);
    add_Q_thw(Q, dt, m_cfg.q_alpha);

    EMat12 P = toE(m_P); // convert to Eigen
    P = F * P * F.transpose() + Q; 

    m_P = fromE(P); // convert back to POD
}

bool RNAVFilter::Update(const RNAVMeasurement& z) {
    // Implement update logic here

    // Build innovation y (6x1)
    EVec6 y = EVec6::Zero();

    // position residual
    y(0) = z.r_BcbyBt.x - m_x.r_BcbyBt.x;
    y(1) = z.r_BcbyBt.y - m_x.r_BcbyBt.y;
    y(2) = z.r_BcbyBt.z - m_x.r_BcbyBt.z;

    // attitude residual: q_err = q_meas ⊗ conj(q_nom)
    const rnav::Quat q_meas = quat_normalize(z.q_BcbyBt);
    const rnav::Quat q_nom  = quat_normalize(m_x.q_BcbyBt);
    const rnav::Quat q_err  = quat_mul(q_meas, quat_conj(q_nom));

    const EVec3 dtheta_err = LogQ(q_err); // robust, not small-angle only
    y(3) = dtheta_err.x();
    y(4) = dtheta_err.y();
    y(5) = dtheta_err.z();

    // H: measurement matrix (6x12)
    // maps [dr dv dtheta dw] -> [pos_res, att_res]
    EMat6x12 H = EMat6x12::Zero();
    for(int i=0;i<3;i++){
        H(i, i)       = 1.0f;   // pos residual depends on dr
        H(3+i, 6+i)   = 1.0f;   // att residual depends on dtheta
    }

    // R: measurement noise covariance (6x6)
    EMat6 R = EMat6::Zero();
    const float sr2 = m_cfg.sigma_r * m_cfg.sigma_r;
    const float st2 = m_cfg.sigma_theta * m_cfg.sigma_theta;
    for(int i=0;i<3;i++){
        R(i,i)       = sr2;
        R(3+i,3+i)   = st2;
    }

    // P: Process covariance (12x12)
    EMat12 P = toE(m_P);

    // Innovation covariance S = HPH' + R (6x6)
    EMat6 S = H * P * H.transpose() + R;

    // Explicit symmetry of S
    S = 0.5f * (S + S.transpose());

    Eigen::LLT<EMat6> llt(S);
    if (llt.info() != Eigen::Success) {
        // S not SPD; treat as rejected measurement or fall back to LDLT
        return false;
    }

    // Optional gating
    if (m_cfg.enable_gating) {
        // NIS: yᵀ S⁻¹ y
        const EVec6 Sinv_y = llt.solve(y);
        const float nis = y.dot(Sinv_y);
        if (nis > m_cfg.nis_gate){
            // Rejected Update
            return false;
        }
    }

    // Kalman Gain K = PH'S^{-1} (12x6)
    // We want K = P Hᵀ S⁻¹  =>  Kᵀ = S⁻¹ H Pᵀ
    const EMat6x12 Sinv_HP = llt.solve(H * P);   // solves S * X = (H*P)
    const EMat12x6 K = Sinv_HP.transpose();      // (S⁻¹ H P)ᵀ = P Hᵀ S⁻¹

    // dx = K y
    EVec12 dx = K * y;

    // Inject into nominal state
    m_x.r_BcbyBt.x += dx(0);
    m_x.r_BcbyBt.y += dx(1);
    m_x.r_BcbyBt.z += dx(2);

    m_x.v_BcbyBt.x += dx(3);
    m_x.v_BcbyBt.y += dx(4);
    m_x.v_BcbyBt.z += dx(5);

    const EVec3 dtheta_update(dx(6), dx(7), dx(8));
    const rnav::Quat dq = ExpQ(dtheta_update);
    m_x.q_BcbyBt = quat_mul(dq, m_x.q_BcbyBt);
    m_x.q_BcbyBt = quat_normalize(m_x.q_BcbyBt);

    m_x.w_BcbyBt.x += dx(9);
    m_x.w_BcbyBt.y += dx(10);
    m_x.w_BcbyBt.z += dx(11);

    // Joseph covariance update
    const EMat12 I = EMat12::Identity();
    const EMat12 KH = K * H;
    const EMat12 A = (I - KH);
    P = A * P * A.transpose() + K * R * K.transpose();
    P = 0.5*(P + P.transpose());

    m_P = fromE(P); // convert back to POD
    return true;
}

void RNAVFilter::FrameTransformation(const msg::PoseEstimate& meas, RNAVMeasurement& z) {
    Pose H_CAMbyPAT{};

    H_CAMbyPAT.t = {meas.t_CbyP[0], meas.t_CbyP[1], meas.t_CbyP[2]};
    H_CAMbyPAT.q = {meas.q_C_P[0], meas.q_C_P[1], meas.q_C_P[2], meas.q_C_P[3]};
    H_CAMbyPAT.q = quat_normalize(H_CAMbyPAT.q);

    Pose H_CAMbyBc = m_cfg.H_CAMbyBc;
    Pose H_PATbyBt = m_cfg.H_PATbyBt;

    // Compute Chaser Body wrt Target Body
    // H_BcbyBt = H_Bc/CAM * H_CAM/PAT * H_PAT/Bt
    Pose H_BcbyBt = pose_compose(
                        pose_inverse(H_CAMbyBc),
                        pose_compose(H_CAMbyPAT,H_PATbyBt)
                    );

    // Populate RNAVMeasurement
    z.r_BcbyBt = H_BcbyBt.t;
    z.q_BcbyBt = H_BcbyBt.q; 
}

bool RNAVFilter::AcceptMeasurement(const msg::PoseEstimate& meas, uint64_t now_us) {
    // 1) ordering check
    if (m_last_meas_us != 0 && meas.t_exp_end_us <= m_last_meas_us) {
        return false;
    }

    // 2) age check
    if (now_us > meas.t_exp_end_us) {
        const uint64_t age = now_us - meas.t_exp_end_us;
        if (age > m_cfg.meas_timeout_us) {
            return false;
        }
    } else {
        // Measurement timestamp in the future -> clock mismatch; reject conservatively
        return false;
    }

    return true;
}

} // namespace rnav