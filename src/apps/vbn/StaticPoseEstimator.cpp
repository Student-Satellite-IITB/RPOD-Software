#include "apps/vbn/StaticPoseEstimator.hpp"
#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry> 

namespace {
    using Vec2 = Eigen::Vector2f;
    using Vec3 = Eigen::Vector3f;
    using Mat3 = Eigen::Matrix3f;

    // Change of Basis Axis Definition
    // from Camera to Aerospace Convention
    // For DCM P*R*P.T for vectors P*t
    static const Mat3 P_CAM2AERO = (Mat3() <<
        0.f, 0.f, 1.f,
        1.f, 0.f, 0.f,
        0.f, 1.f, 0.f
    ).finished();
}

namespace vbn
{
    struct Pose{
        Mat3 R = Mat3::Identity(); // Pattern to Camera Frame Transformation
        Vec3 t = Vec3::Zero(); // Vector from Pattern Origin to Camera Origin
    };
} // namespace vbn


namespace vbn {
// Internal sanitisation of config parameters
// not public API
static inline StaticPoseEstimatorConfig sanitise(const StaticPoseEstimatorConfig& in) {
    StaticPoseEstimatorConfig cfg = in;

    const float D = cfg.PATTERN_GEOMETRY.PATTERN_RADIUS;
    const float H = cfg.PATTERN_GEOMETRY.PATTERN_OFFSET;

    // Check the validity of D and H; if not replace with defaults
    if (!(std::isfinite(D) && D > 0.0f)) cfg.PATTERN_GEOMETRY.PATTERN_RADIUS = 0.050f;
    if (!(std::isfinite(H) && H > 0.0f)) cfg.PATTERN_GEOMETRY.PATTERN_OFFSET = 0.020f;

    const float D2 = cfg.PATTERN_GEOMETRY.PATTERN_RADIUS;
    const float H2 = cfg.PATTERN_GEOMETRY.PATTERN_OFFSET;

    // Compute pseudo-inverse for your 5-point cross pattern:
    // Points: T(0,-D,0), L(-D,0,0), B(0,+D,0), R(+D,0,0), C(0,0,-H)
    const float inv2D = 1.0f / (2.0f * D);
    const float invH  = 1.0f / H;

    cfg.PATTERN_GEOMETRY.P_PINV = {
         0.0f,  -inv2D, 0.0f,  inv2D, 0.0f,
        -inv2D,  0.0f,  inv2D, 0.0f,  0.0f,
         0.0f,   0.0f,  0.0f,  0.0f, -invH
    };


    // If invalid parameters are set, replace with safe defaults
    if (cfg.MAX_REPROJ_ERROR_PX <= 0.0f) cfg.MAX_REPROJ_ERROR_PX = 1.0f;
    if (cfg.CAM_INTRINSICS.fx <= 0.0f) cfg.CAM_INTRINSICS.fx = 1.0f;
    if (cfg.CAM_INTRINSICS.fy <= 0.0f) cfg.CAM_INTRINSICS.fy = 1.0f;

    return cfg;
}

// Attitude Parametrisation
static inline void DCM2Euler321(const Mat3& C, float& roll, float& pitch, float& yaw){
    // Frame transformation
    // 3-2-1 Passive Sequence

    // pitch = asin(-c13)
    float s = -C(0,2); // c13 with 0-based indexing is C(0,2)

    // clamp for numeric safety
    s = std::max(-1.0f, std::min(1.0f, s));
    pitch = std::asin(s);

    const float cp = std::cos(pitch);

    if (std::fabs(cp) > 1e-6f) {
        // roll = atan2(c23, c33)
        roll = std::atan2(C(1,2), C(2,2));
        // yaw  = atan2(c12, c11)
        yaw  = std::atan2(C(0,1), C(0,0));
    } else {
        // Gimbal lock: cp ~ 0, yaw and roll coupled.
        // Common choice: set roll = 0 and solve yaw from other terms.
        roll = 0.0f;
        yaw  = std::atan2(-C(1,0), C(1,1));
    }
}

static inline void DCM2Quat_0123(const Mat3&R, float q[4]){
    // Sheppard's Method
    // Robust implementation as compared to direct DCM2Quat
    // Output quaternion (scalar-first):
    // q[0]=q0, q[1]=q1, q[2]=q2, q[3]=q3

    float q0, q1, q2, q3;

    const float tr = R(0,0) + R(1,1) + R(2,2);

    if (tr > 0.0f) {
    const float S = std::sqrt(tr + 1.0f) * 2.0f; // S = 4*q0
    q0 = 0.25f * S;
    q1 = (R(1,2) - R(2,1)) / S;
    q2 = (R(2,0) - R(0,2)) / S;
    q3 = (R(0,1) - R(1,0)) / S;

    } else if (R(0,0) > R(1,1) && R(0,0) > R(2,2)) {
        const float S = std::sqrt(1.0f + 2*R(0,0) - tr) * 2.0f; // S = 4*q1
        q0 = (R(1,2) - R(2,1)) / S;
        q1 = 0.25f * S;
        q2 = (R(0,1) + R(1,0)) / S;
        q3 = (R(0,2) + R(2,0)) / S;

    } else if (R(1,1) > R(2,2)) {
        const float S = std::sqrt(1.0f + 2*R(1,1) - tr) * 2.0f; // S = 4*q2
        q0 = (R(2,0) - R(0,2)) / S;
        q1 = (R(0,1) + R(1,0)) / S;
        q2 = 0.25f * S;
        q3 = (R(1,2) + R(2,1)) / S;

    } else {
        const float S = std::sqrt(1.0f + 2*R(2,2) - tr) * 2.0f; // S = 4*q3
        q0 = (R(0,1) - R(1,0)) / S;
        q1 = (R(0,2) + R(2,0)) / S;
        q2 = (R(1,2) + R(2,1)) / S;
        q3 = 0.25f * S;
    }

    // Normalize (good practice if R is slightly non-orthonormal numerically)
    const float n = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (n > 1e-12f) {
        q0 /= n;
        q1 /= n; 
        q2 /= n; 
        q3 /= n;
    } else {
        q0 = 1.0f; 
        q1 = 0.0f; 
        q2 = 0.0f; 
        q3 = 0.0f;
    }

    // Enforcing + q0 Shorter rotation convention
    if (q0 < 0.0f) { q0 = -q0; q1 = -q1; q2 = -q2; q3 = -q3; }

    q[0] = q0; q[1] = q1; q[2] = q2; q[3] = q3;
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

    for (std::size_t i = 0; i < feature_frame.feat_count; ++i) {
        const msg::Feature& led = feature_frame.feats[i];

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
        msg::Feature& led = packed.inner[i];
        // Transform to camera frame (subpixel precision)
        led.u_px = (led.u_px - cx);
        led.v_px = (led.v_px - cy);
    }

    for (std::size_t i = 0; i < packed.outer_count; ++i) {
    msg::Feature& led = packed.outer[i];
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
    const msg::Feature& T = packed.inner[0];
    const msg::Feature& L = packed.inner[1];
    const msg::Feature& B = packed.inner[2];
    const msg::Feature& R = packed.inner[3];

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
                                                           Pose& pose) const{
    // Assumptions:
    // Central LED offset = pattern radius (D)
    // inner_count == 5, slots: 0:TOP,1:LEFT,2:BOTTOM,3:RIGHT,4:CENTER,
    // and coords already in PPF.

    auto safeAsin = [](float x) -> float {
        if (x >  1.0f) x =  1.0f;
        if (x < -1.0f) x = -1.0f;
        return std::asin(x);
    };

    const msg::Feature& T = packed.inner[0];
    const msg::Feature& L = packed.inner[1];
    const msg::Feature& B = packed.inner[2];
    const msg::Feature& R = packed.inner[3];
    const msg::Feature& C = packed.inner[4];

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
    float roll = std::atan2(-y4, y3);
    float cr = std::cos(roll);
    float sr = std::sin(roll);

    // Yaw compqutation
    float arg_yaw = (-x5/y1) * cr;
    float yaw_plus_az = safeAsin(arg_yaw);
    float yaw = yaw_plus_az - az;

    // Pitch computation
    float cypa = std::cos(yaw_plus_az);
    float sypa = std::sin(yaw_plus_az);

    float num_b = cypa * cr;
    float den_b = sypa * sr + (x2/y5);

    if (std::fabs(den_b) < 1e-9f) {
        return false;
    }

    float arg_pitch = num_b / den_b;
    float pitch_plus_el = safeAsin(arg_pitch);
    float pitch = pitch_plus_el - el;

    // Range computation
    const auto& cam  = m_cfg.CAM_INTRINSICS;
    const auto& geom = m_cfg.PATTERN_GEOMETRY;
    const float D    = geom.PATTERN_RADIUS;
    const float fx   = cam.fx;

    if (std::fabs(x4) < 1e-9f) {
        return false;
    }

    float Df = D * fx;
    float Z = (Df/x4)*(cr*cypa - sr*sypa*std::sin(pitch_plus_el));

    if(Z < 0.0f) {
        return false;
    }    
    
    float cp = std::cos(pitch);
    float sp = std::sin(pitch);

    float cy = std::cos(yaw);
    float sy = std::sin(yaw);

    Mat3 R_C_P;
    R_C_P << 
        cp*cy, cp*sy, -sp,
        sr*sp*cy - cr*sy, sr*sp*sy + cr*cy, sr*cp,
        cr*sp*cy + sr*sy, cr*sp*sy - sr*cy, cr*cp;

    Vec3 t_PbyC;
    t_PbyC.x() = Z;
    t_PbyC.y() = Z*std::tan(az);
    t_PbyC.z() = -Z*std::tan(el)/std::cos(az);

    // Change of Basis to Aerospace convention
    // Euler angles here are derived according to aerospace convention directly
    // From Pirat reference
    // Translation vector is also already in aerospace convention

    // Output pose
    pose.R = R_C_P;
    pose.t = -t_PbyC;

    return true;
}

bool vbn::StaticPoseEstimator::genericAnalyticalPose(const PackedLeds& packed, float az, float el, Pose& pose) const{

    // Assumptions: Weak Perspective Projection

    using Mat53 = Eigen::Matrix<float, 5, 3>;
    using Vec5  = Eigen::Matrix<float, 5, 1>;
    using Mat35RM = Eigen::Matrix<float, 3, 5, Eigen::RowMajor>;

    const auto fx = m_cfg.CAM_INTRINSICS.fx;
    const auto fy = m_cfg.CAM_INTRINSICS.fy;
    
    const msg::Feature& T = packed.inner[0];
    const msg::Feature& L = packed.inner[1];
    const msg::Feature& B = packed.inner[2];
    const msg::Feature& R = packed.inner[3];
    const msg::Feature& C = packed.inner[4];

    // Pattern centre in PPF
    const float u_c = 0.25f * (T.u_px + L.u_px + B.u_px + R.u_px);
    const float v_c = 0.25f * (T.v_px + L.v_px + B.v_px + R.v_px);

    // Normalised center coordinates
    const float x_c = u_c/fx;
    const float y_c = v_c/fy;

    // Normalised LED coordinate stack
    Vec5 X,Y;
    X << T.u_px/fx - x_c,
         L.u_px/fx - x_c,
         B.u_px/fx - x_c,
         R.u_px/fx - x_c,
         C.u_px/fx - x_c;

    Y << T.v_px/fy - y_c,
         L.v_px/fy - y_c,
         B.v_px/fy - y_c,
         R.v_px/fy - y_c,
         C.v_px/fy - y_c;

    // Load precomputed pseudo-inverse (row-major 3x5)
    Eigen::Map<const Mat35RM> P_pinv(m_cfg.PATTERN_GEOMETRY.P_PINV.data());

    Vec3 a1 = P_pinv*X;
    Vec3 a2 = P_pinv*Y;
    

    if (!a1.allFinite() || !a2.allFinite()) {
        return false;
    }

    const float a1n = a1.norm();
    const float a2n = a2.norm();
    
    if (!(std::isfinite(a1n) && std::isfinite(a2n) && a1n > 1e-8f && a2n > 1e-8f)) return false;

    // === Recover Z0 using known norms:
    // ||b1||^2 = ||r1 - x_c r3||^2 = 1 + x_c^2
    // ||b2||^2 = 1 + y_c^2
    // and b1 = Z0*a1, b2 = Z0*a2
    const float Z01 = std::sqrt((1.0f + x_c*x_c) / (a1n*a1n));
    const float Z02 = std::sqrt((1.0f + y_c*y_c) / (a2n*a2n));
    if (!(std::isfinite(Z01) && std::isfinite(Z02) && Z01 > 0.0f && Z02 > 0.0f)) return false;

    const float Z0 = 0.5f * (Z01 + Z02);
    if (!(std::isfinite(Z0) && Z0 > 0.0f)) return false;

    // Unscale b1,b2
    const Vec3 b1 = Z0 * a1;
    const Vec3 b2 = Z0 * a2;

    // === Solve for r3 from:
    // b1·r3 = -x_c
    // b2·r3 = -y_c
    // ||r3|| = 1
    //
    // Particular solution: r3_0 = B^T (B B^T)^-1 d
    // where B=[b1^T; b2^T], d=[-x_c; -y_c]
    const float bb11 = b1.dot(b1);
    const float bb22 = b2.dot(b2);
    const float bb12 = b1.dot(b2);

    // BBt = [[bb11, bb12],[bb12, bb22]]
    const float detBBt = bb11*bb22 - bb12*bb12;
    if (!(std::isfinite(detBBt) && std::fabs(detBBt) > 1e-10f)) return false;

    const float inv00 =  bb22 / detBBt;
    const float inv01 = -bb12 / detBBt;
    const float inv10 = -bb12 / detBBt;
    const float inv11 =  bb11 / detBBt;

    const float d0 = -x_c;
    const float d1 = -y_c;

    // w = (BBt)^-1 d
    const float w0 = inv00*d0 + inv01*d1;
    const float w1 = inv10*d0 + inv11*d1;

    const Vec3 r3_0 = b1*w0 + b2*w1;

    // Null direction
    Vec3 n = b1.cross(b2);
    const float nn = n.norm();
    if (!(std::isfinite(nn) && nn > 1e-8f)) return false;
    n /= nn;

    // r3 = r3_0 ± alpha n, alpha = sqrt(1 - ||r3_0||^2)
    const float r30n2 = r3_0.squaredNorm();
    float alpha2 = 1.0f - r30n2;
    if (alpha2 < -1e-5f) return false;         // inconsistent due to noise/assumption violation
    if (alpha2 < 0.0f) alpha2 = 0.0f;          // clamp small negative
    const float alpha = std::sqrt(alpha2);

    auto buildR = [&](const Vec3& r3_in, Mat3& Rcp_out) -> bool {
        Vec3 r1 = b1 + x_c * r3_in;
        Vec3 r2 = b2 + y_c * r3_in;

        // Gram–Schmidt SO(3) enforcement
        float r1n = r1.norm();
        if (r1n < 1e-8f) return false;
        r1 /= r1n;

        r2 = r2 - (r1.dot(r2)) * r1;
        float r2n = r2.norm();
        if (r2n < 1e-8f) return false;
        r2 /= r2n;

        Vec3 r3 = r1.cross(r2);
        float r3n = r3.norm();
        if (r3n < 1e-8f) return false;
        r3 /= r3n;

        Rcp_out.row(0) = r1.transpose();
        Rcp_out.row(1) = r2.transpose();
        Rcp_out.row(2) = r3.transpose();

        if (Rcp_out.determinant() < 0.0f) {
            Rcp_out.row(2) *= -1.0f;
        }
        return Rcp_out.allFinite();
    };

    Mat3 Rcp_plus, Rcp_minus;
    const Vec3 r3_plus  = r3_0 + alpha * n;
    const Vec3 r3_minus = r3_0 - alpha * n;

    const bool ok_plus  = buildR(r3_plus,  Rcp_plus);
    const bool ok_minus = buildR(r3_minus, Rcp_minus);
    if (!ok_plus && !ok_minus) return false;

    // Choose sign: simplest heuristic → pick the one with r3.z > 0 if that matches your camera forward
    Mat3 R_C_P = ok_plus ? Rcp_plus : Rcp_minus;
    if (ok_plus && ok_minus) {
        // Prefer the solution whose r3 points roughly along +Zc
        if (Rcp_minus(2,2) > Rcp_plus(2,2)) R_C_P = Rcp_minus;
    }

    // Translation: use center bearing directly
    // t = [tx, ty, tz], with x_c=tx/tz, y_c=ty/tz
    Vec3 t_PbyC;
    t_PbyC.z() = Z0;
    t_PbyC.x() = x_c * Z0;
    t_PbyC.y() = y_c * Z0;

    if (!t_PbyC.allFinite()) return false;

    // Change of basis from CAM to Aerospace Convention
    R_C_P = P_CAM2AERO*R_C_P*P_CAM2AERO.transpose();
    t_PbyC = P_CAM2AERO*t_PbyC;

    // Output pose
    pose.R = R_C_P;
    pose.t = -t_PbyC;

    return true;
}


bool vbn::StaticPoseEstimator::estimatePose(const PackedLeds& packed,
                                            float az, float el,
                                            Pose& pose) const {

    switch (m_cfg.ALGO) {
        case AlgoType::ANALYTICAL_INNER:
            return estimatePoseAnalyticalInner(packed, az, el, pose);
        
        // can be extended in future for more algorithms
        // case AlgoType::PNP_INNER:
        //     // future: pure PnP on inner pattern
        //     return estimatePosePnpInner(packed, az, el roll, pitch, yaw, range_m);

        case AlgoType::ANALYTICAL_GENERIC:
            return genericAnalyticalPose(packed, az, el, pose);

        default:
            // Unknown algorithm type
            return false;

    }

}

float vbn::StaticPoseEstimator::evaluateReprojectionError(const PackedLeds& packed, const Pose& pose){

    Mat3 R_C_P = pose.R;
    Vec3 t_PbyC = -pose.t;

    // Axis transformations to camera convention
    R_C_P = P_CAM2AERO.transpose()*R_C_P*P_CAM2AERO;
    t_PbyC = P_CAM2AERO.transpose()*t_PbyC;

    const auto& cam = m_cfg.CAM_INTRINSICS;
    const auto& geom = m_cfg.PATTERN_GEOMETRY;
    const float fx = cam.fx;
    const float fy = cam.fy;
    const float D = geom.PATTERN_RADIUS;
    const float H = geom.PATTERN_OFFSET;

    // Inner Pattern Geometry in Pattern Frame
    // Pattern Frame Definition:
    // +X right, +Y down, +Z inwards of pattern plane (+V bar)
    
    const std::array<Vec3, 5> P_pat = {
        Vec3( 0.0f, -D,   0.0f),  // TOP
        Vec3(-D,    0.0f, 0.0f),  // LEFT
        Vec3( 0.0f,  D,   0.0f),  // BOTTOM
        Vec3( D,    0.0f, 0.0f),  // RIGHT
        Vec3( 0.0f, 0.0f, -H)     // CENTER (towards camera)
        // If using H: Vec3(0,0,-H)
    };

    Vec3 P_cam;

    float sum_sq_error = 0.0f;
    int count = 0;

    for(int i = 0; i < 5; ++i) {

        // Imaging geometry
        P_cam = R_C_P*P_pat[i] + t_PbyC;

        float Z = P_cam.z();
        if (Z <= 1e-9f) return 1e9f;

        // Project to image plane (PPF)
        float u_proj = fx * (P_cam.x() / Z);
        float v_proj = fy * (P_cam.y() / Z);

        // Corresponding observed LED
        const auto& obs_led = packed.inner[i];

        // Reprojection error
        float du = obs_led.u_px - u_proj;
        float dv = obs_led.v_px - v_proj;

        sum_sq_error += du*du + dv*dv;
        ++count;
    }  
    
    float RMS_error = std::sqrt(sum_sq_error / count);
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

    Pose pose;
    
    if(!estimatePose(packed, az, el, pose)) {
        out.valid = 0; // Mark pose as invalid
        return false;
    }

    DCM2Euler321(pose.R, roll, pitch, yaw);
    range_m = pose.t.norm();

    // EVALUATE REPROJECTION ERROR
    float reproj_error = evaluateReprojectionError(packed, pose);

    if (reproj_error > m_cfg.MAX_REPROJ_ERROR_PX) {
        out.valid = 0; // Mark pose as invalid
        return false;
    }

    // POPULATE OUTPUT POSE ESTIMATE

    const Mat3 R_C_P = pose.R;
    float q_C_P[4] = {1,0,0,0};

    // DCM to Quaternion conversion
    DCM2Quat_0123(R_C_P, q_C_P);
    out.q_C_P[0] = q_C_P[0];
    out.q_C_P[1] = q_C_P[1];
    out.q_C_P[2] = q_C_P[2];
    out.q_C_P[3] = q_C_P[3];
    
    // row-major pack
    int k = 0;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out.R_C_P[k++] = R_C_P(r,c);
        }
    }

    // Placeholder translation: along camera Z only
    out.t_CbyP[0] = pose.t.x();
    out.t_CbyP[1] = pose.t.y();
    out.t_CbyP[2] = pose.t.z();

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

