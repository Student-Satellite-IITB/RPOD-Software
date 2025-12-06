#include "apps/vbn/StaticPoseEstimator.hpp"
#include <cmath>
#include <iostream>

namespace vbn {
// Internal sanitisation of config parameters
// not public API
static inline StaticPoseEstimatorConfig sanitise(const StaticPoseEstimatorConfig& in) {
    StaticPoseEstimatorConfig cfg = in;

    // If invalid parameters are set, replace with safe defaults
    if (cfg.MAX_REPROJ_ERROR_PX <= 0.0f) cfg.MAX_REPROJ_ERROR_PX = 1.0f;
    if (cfg.CAM_INTRINSICS.fx <= 0.0f) cfg.CAM_INTRINSICS.fx = 1.0f;
    if (cfg.CAM_INTRINSICS.fy <= 0.0f) cfg.CAM_INTRINSICS.fy = 1.0f;

    return cfg;
}
} // namespace vbn

// IMPLEMENTATION

vbn::StaticPoseEstimator::StaticPoseEstimator(const StaticPoseEstimatorConfig& cfg)
    : m_cfg(sanitise(cfg)) {
}

void vbn::StaticPoseEstimator::setConfig(const StaticPoseEstimatorConfig& cfg) {
    m_cfg = sanitise(cfg);
}

bool vbn::StaticPoseEstimator::packetLeds(const msg::FeatureFrame& feature_frame, PackedLeds& packed){

    packed.inner_count = 0;
    packed.outer_count = 0;

    // Iterate through the LEDs in the feature frame
    // and pack them into inner and outer arrays
    // based on their pattern_id and slot_id.

    for (std::size_t i = 0; i < feature_frame.led_count; ++i) {
        const msg::Led2D& led = feature_frame.leds[i];

        if(led.valid == 0) {
            continue; // skip invalid LEDs
        }

        if (led.pattern_id == msg::PatternId::INNER) {
            const std::size_t idx = static_cast<std::size_t>(led.slot_id);
            packed.inner[idx] = led;
            ++packed.inner_count;
        }

        else if (led.pattern_id == msg::PatternId::OUTER) {
            // For v1, just store OUTER LEDs sequentially; no geometry enforced yet
            if (packed.outer_count < MAX_OUTER_LEDS) {
                packed.outer[packed.outer_count] = led;
                ++packed.outer_count;
            }
        }
        else {
            // UNKNOWN/other patterns ignored in v1
        }
    }

    return (packed.inner_count == 5); // Ensure all 5 INNER LEDs are present
}

void vbn::StaticPoseEstimator::transformLedstoPPF(PackedLeds& packed) const{
    CameraIntrinsics cam = m_cfg.CAM_INTRINSICS;
    float cx = cam.cx;
    float cy = cam.cy;

    // converting to principal point centered coordinates

    for(std::size_t i =0; i < packed.inner_count; ++i) {
        msg::Led2D& led = packed.inner[i];
        // Transform to camera frame (subpixel precision)
        led.u_px = (led.u_px - cx);
        led.v_px = (led.v_px - cy);
    }

    for (std::size_t i = 0; i < packed.outer_count; ++i) {
    msg::Led2D& led = packed.outer[i];
    led.u_px = (led.u_px - cx);
    led.v_px = (led.v_px - cy);
    }


}

void vbn::StaticPoseEstimator::undistortLedsPPF(PackedLeds& packed) const
{
    // NOTE (v1):
    //  - This is intentionally a NO-OP.
    //  - All LEDs in 'packed' are already in Principal-Point Frame (PPF),
    //    but we do not apply any lens distortion correction in v1.
    //  - Distortion parameters (k1,k2,k3,p1,p2) in CAM_INTRINSICS are
    //    currently ignored.
    //
    //  - If future testing shows that lens distortion significantly biases
    //    the pose estimate, this function is the single hook where a proper
    //    point-wise undistortion (Brown–Conrady inverse) should be added.
    //
    //  - The API stays stable: call site does not need to change when
    //    undistortion is enabled.

    (void)packed; // suppress unused-parameter warning for now
}

bool vbn::StaticPoseEstimator::computeLosAngles(const PackedLeds& packed,
                                             float& az,
                                             float& el) const {
    const CameraIntrinsics& cam = m_cfg.CAM_INTRINSICS;
    float fx = cam.fx;
    float fy = cam.fy;

    // INNER LEDs are packed by slot:
    // 0 = TOP, 1 = LEFT, 2 = BOTTOM, 3 = RIGHT, 4 = CENTER
    const msg::Led2D& T = packed.inner[0];
    const msg::Led2D& L = packed.inner[1];
    const msg::Led2D& B = packed.inner[2];
    const msg::Led2D& R = packed.inner[3];

    // 4-LED pattern centre in PPF (ũ, ṽ)
    const float u_c = 0.25f * (T.u_px + L.u_px + B.u_px + R.u_px);
    const float v_c = 0.25f * (T.v_px + L.v_px + B.v_px + R.v_px);

    // LOS angles in radians (analytic derivation):
    // Az = atan( u_c / fx ),  El = atan( -v_c / fy )
    az = std::atan(u_c/fx);
    el = std::atan(-v_c/fy);

    return true; // Indicate successful computation
}

// ===================================
// ==== POSE ESTIMATION ALGORITHMS ===
// ===================================

bool vbn::StaticPoseEstimator::estimatePoseAnalyticalInner(const PackedLeds& packed,
                                 float az, float el,
                                 float& roll, float& pitch, float& yaw,
                                 float& range_m) const{
    // Assumptions:
    // Central LED offset = pattern radius (D)
    // inner_count == 5, slots: 0:TOP,1:LEFT,2:BOTTOM,3:RIGHT,4:CENTER,
    // and coords already in PPF.

    auto safeAsin = [](float x) -> float {
        if (x >  1.0f) x =  1.0f;
        if (x < -1.0f) x = -1.0f;
        return std::asin(x);
    };

    const msg::Led2D& T = packed.inner[0];
    const msg::Led2D& L = packed.inner[1];
    const msg::Led2D& B = packed.inner[2];
    const msg::Led2D& R = packed.inner[3];
    const msg::Led2D& C = packed.inner[4];

    // Pattern centre in PPF
    const float u_c = 0.25f * (T.u_px + L.u_px + B.u_px + R.u_px);
    const float v_c = 0.25f * (T.v_px + L.v_px + B.v_px + R.v_px);

    // Relative coords wrt centre (pixels)
    const float x1 = T.u_px - u_c;
    const float y1 = T.v_px - v_c;

    const float x2 = L.u_px - u_c;
    const float y2 = L.v_px - v_c;

    const float x3 = B.u_px - u_c;
    const float y3 = B.v_px - v_c;

    const float x4 = R.u_px - u_c;
    const float y4 = R.v_px - v_c;

    // Central LED offset
    const float x5 = C.u_px - u_c;
    const float y5 = C.v_px - v_c;

    // Degeneracy checks (minimal, same spirit as Python)
    if (std::fabs(y4) < 1e-9f ||
        std::fabs(y5) < 1e-9f ||
        std::fabs(y1) < 1e-9f ||
        (std::fabs(y2) < 1e-12f && std::fabs(y1) < 1e-12f)) {
        return false;
    }

    // Roll computation
    float alpha = std::atan2(-y4, y3);
    float ca = std::cos(alpha);
    float sa = std::sin(alpha);

    // Yaw compqutation
    float arg_gamma = (-x5/y1) * ca;
    float gamma_plus_az = safeAsin(arg_gamma);
    float gamma = gamma_plus_az - az;

    // Pitch computation
    float cgpa = std::cos(gamma_plus_az);
    float sgpa = std::sin(gamma_plus_az);

    float num_b = cgpa * ca;
    float den_b = sgpa * sa + (x2/y5);

    if (std::fabs(den_b) < 1e-9f) {
        return false;
    }

    float arg_beta = num_b / den_b;
    float beta_plus_el = safeAsin(arg_beta);
    float beta = beta_plus_el - el;

    // Range computation
    const auto& cam  = m_cfg.CAM_INTRINSICS;
    const auto& geom = m_cfg.PATTERN_GEOMETRY;
    const float D    = geom.PATTERN_RADIUS;
    const float fx   = cam.fx;

    if (std::fabs(x4) < 1e-9f) {
        return false;
    }

    float Df = D * fx;
    float range = (Df/x4)*(ca*cgpa - sa*sgpa*std::sin(beta_plus_el));

    if(range < 0.0f) {
        return false;
    }

    // Map α,β,γ to roll,pitch,yaw in your chosen convention.
    // Here we take α=roll, β=pitch, γ=yaw (1-2-3 sequence in camera frame).
    roll  = alpha;
    pitch = beta;
    yaw   = gamma;
    range_m = range;

    return true;
}


bool vbn::StaticPoseEstimator::estimatePose(const PackedLeds& packed,
                                            float az, float el,
                                            float& roll, float& pitch, float& yaw,
                                            float& range_m) const {

    switch (m_cfg.ALGO) {
        case AlgoType::ANALYTICAL_INNER:
            return estimatePoseAnalyticalInner(packed, az, el, roll, pitch, yaw, range_m);
        
        // can be extended in future for more algorithms
        // case AlgoType::PNP_INNER:
        //     // future: pure PnP on inner pattern
        //     return estimatePosePnpInner(packed, az, el roll, pitch, yaw, range_m);

        default:
            // Unknown algorithm type
            return false;

    }

}

float vbn::StaticPoseEstimator::evaluateReprojectionError(const PackedLeds& packed,
                                                   float roll, float pitch, float yaw,
                                                   float range_m){
    
    const auto& cam = m_cfg.CAM_INTRINSICS;
    const auto& geom = m_cfg.PATTERN_GEOMETRY;
    const float fx = cam.fx;
    const float fy = cam.fy;
    const float D = geom.PATTERN_RADIUS;

    const msg::Led2D& T = packed.inner[0];
    const msg::Led2D& L = packed.inner[1];
    const msg::Led2D& B = packed.inner[2];
    const msg::Led2D& R = packed.inner[3];
    const msg::Led2D& C = packed.inner[4];

    const float u_c = 0.25f * (T.u_px + L.u_px + B.u_px + R.u_px);
    const float v_c = 0.25f * (T.v_px + L.v_px + B.v_px + R.v_px);

    float x_n = u_c / fx;
    float y_n = v_c / fy;
    float z_n = 1.0f;

    float norm = std::sqrt(x_n * x_n + y_n * y_n + z_n * z_n);
    if (norm < 1e-9f) {
        return 1e9f; // Large error for degenerate case
    }

    float dir_x = x_n / norm;
    float dir_y = y_n / norm;
    float dir_z = z_n / norm;

    // Translation: pattern origin in camera frame
    float t_x = range_m * dir_x;
    float t_y = range_m * dir_y;
    float t_z = range_m * dir_z;

    // Rotation matrix R_cam_pat from roll, pitch, yaw (1-2-3)
    const float cr = std::cos(roll);
    const float sr = std::sin(roll);
    const float cp = std::cos(pitch);
    const float sp = std::sin(pitch);
    const float cy = std::cos(yaw);
    const float sy = std::sin(yaw);

    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    float R00 = cy * cp;
    float R01 = cy * sp * sr - sy * cr;
    float R02 = cy * sp * cr + sy * sr;

    float R10 = sy * cp;
    float R11 = sy * sp * sr + cy * cr;
    float R12 = sy * sp * cr - cy * sr;

    float R20 = -sp;
    float R21 = cp * sr;
    float R22 = cp * cr;

    // Inner Pattern Geometry in Pattern Frame
    // Pattern Frame Definition:
    // +X right, +Y down, +Z inwards of pattern plane (+V bar)

    struct PatLed {
        float x;
        float y;
        float z;
    };

    PatLed pat_leds[5] = {
        {0.0f,  -D, 0.0f},   // TOP
        {-D, 0.0f, 0.0f},   // LEFT
        {0.0f, D, 0.0f},   // BOTTOM
        { D, 0.0f, 0.0f},    // RIGHT
        {0.0f,0.0f, -D}    // CENTER (offset towards camera, assuming H = D)
    };

    float sum_sq_error = 0.0f;
    float RMS_error = 0.0f;
    int count = 0;

    for(int i = 0; i < 5; ++i) {
        const PatLed& p_led = pat_leds[i];

        // Transform to camera frame: P_cam = R * P_pat + t
        float Xc = R00 * p_led.x + R01 * p_led.y + R02 * p_led.z + t_x;
        float Yc = R10 * p_led.x + R11 * p_led.y + R12 * p_led.z + t_y;
        float Zc = R20 * p_led.x + R21 * p_led.y + R22 * p_led.z + t_z;

        if (Zc < 1e-9f) {
            return 1e9f; // Large error for degenerate case
        }

        // Project to image plane (PPF)
        float u_proj = (fx * Xc) / Zc;
        float v_proj = (fy * Yc) / Zc;

        // Corresponding observed LED
        const msg::Led2D& obs_led = packed.inner[i];

        // Reprojection error
        float du = obs_led.u_px - u_proj;
        float dv = obs_led.v_px - v_proj;
        float error = std::sqrt(du * du + dv * dv);

        sum_sq_error += error*error;
        ++count;
    }  
    
    RMS_error = std::sqrt(sum_sq_error / count);

    return RMS_error;
}


bool vbn::StaticPoseEstimator::estimate(const msg::FeatureFrame& feature_frame, msg::PoseEstimate& out) {

    // PACKET LEDS
    PackedLeds packed{};
    if (!packetLeds(feature_frame, packed)) {
        // If we don't have a valid inner pattern, we cannot estimate pose in v1
        out.valid = 0; // Mark pose as invalid
        return false;
    }

    // FRAME TRANSFORMATION
    transformLedstoPPF(packed);
    undistortLedsPPF(packed);

    // LOS ANGLE COMPUTATION
    float az = 0.0f;
    float el = 0.0f;

    if(!computeLosAngles(packed, az, el)) {
        out.valid = 0; // Mark pose as invalid
        return false;
    }

    // POSE ESTIMATION
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float range_m = 0.0f;

    if(!estimatePose(packed, az, el, roll, pitch, yaw, range_m)) {
        out.valid = 0; // Mark pose as invalid
        return false;
    }

    // EVALUATE REPROJECTION ERROR
    float reproj_error = evaluateReprojectionError(packed, roll, pitch, yaw, range_m);

    if (reproj_error > m_cfg.MAX_REPROJ_ERROR_PX) {
        out.valid = 0; // Mark pose as invalid
        return false;
    }

    // POPULATE OUTPUT POSE ESTIMATE

    // Need to convert roll, pitch, yaw, range_m into quaternion and translation vector
    // Quaternion from roll, pitch, yaw (1-2-3 sequence)
    // Will do this in V2; for now, set a placeholder quaternion
    
    // Placeholder quaternion: identity (w,x,y,z)
    out.q_PbyC[0] = 1.0f;
    out.q_PbyC[1] = 0.0f;
    out.q_PbyC[2] = 0.0f;
    out.q_PbyC[3] = 0.0f;

    // Placeholder translation: along camera Z only
    out.t_PbyC[0] = 0.0f;
    out.t_PbyC[1] = 0.0f;
    out.t_PbyC[2] = range_m;

    out.az = az;
    out.el = el;
    out.roll = roll;
    out.pitch = pitch;
    out.yaw = yaw;
    out.range_m = range_m;

    out.reproj_rms_px = reproj_error;

    out.t_exp_end_us = feature_frame.t_exp_end_us;
    out.pattern_id = msg::PatternId::INNER;
    out.frame_id = feature_frame.frame_id;
    out.state = feature_frame.state;
    out.used_led_mask = 0x1F; // All 5 INNER LEDs used
    out.valid = 1; // Mark pose as valid

    return true;
}

