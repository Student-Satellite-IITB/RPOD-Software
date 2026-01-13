#pragma once
#include <cstdint>

#include "os/rtos.hpp"
#include "msg/PoseEstimate.hpp"
#include "msg/RNAVState.hpp"

namespace rnav{

// Pose of frame A wrt frame B: (A/B)
// Convention:
// - R: passive frame transform B -> A.
//      v^A = R_A_B * v^B
// - t: origin of A wrt origin of B, expressed in A.
//      (vector from B-origin to A-origin, expressed in A)

struct Vec3 {
    float x=0, y=0, z=0;
    static constexpr Vec3 Zero() { return {}; }
};

struct Quat {
    float w=1, x=0, y=0, z=0; // scalar-first by convention
    static constexpr Quat Identity() { return {}; }
};

struct Mat12 {
    float data[12*12] = {};
    static Mat12 Identity() {
        Mat12 I;
        for(int i=0;i<12;i++) I.data[i*12 + i] = 1.0f;
        return I;
    }
};

struct Pose{
    Vec3 t; // translation vector
    Quat q; // orientation quaternion (w,x,y,z)
};

struct RNAVInternalState{
    Vec3 r_BcbyBt; // position in Chaser Body frame (m)
    Vec3 v_BcbyBt; // velocity in Chaser Body frame (m/s)
    Quat q_BcbyBt; // orientation Chaser Body wrt Target Body (w,x,y,z)
    Vec3 w_BcbyBt; // angular velocity Chaser Body wrt Target Body (rad/s)
};

struct RNAVMeasurement{
    Vec3 r_BcbyBt; // position in Chaser Body frame (m)
    Quat q_BcbyBt; // orientation Chaser Body wrt Target Body (w,x,y,z)
};

// ------------------------------
// Queue types 
// ------------------------------
using PoseMeasQueue = Rtos::Queue<msg::PoseEstimate, 1>;
using StateEstimateQueue = Rtos::Queue<msg::RNAVState, 1>;

// Optional command queue (keep minimal)
enum class RNAVCmdType : uint8_t { Stop = 0, ResetFilter };
struct RNAVCmd { RNAVCmdType type = RNAVCmdType::Stop; };

using CmdQueue = Rtos::Queue<RNAVCmd, 8>;

struct RNAVFilterConfig{
    // Filter Configuration
    
    // Camera wrt Chaser Body frame
    Pose H_CAMbyBc{};
    // Target Pattern wrt Target Body frame
    Pose H_PATbyBt{};

    // Filter update rate
    uint32_t PropagateHz = 100; // Hz
    
    // Process noise covariance
    float sigma_a = 0.05f; // [m/s^2]
    float sigma_alpha = 0.05f; // [rad/s^2]

    // Measurement noise covariance Baseline
    // To be updated to be adaptive based on pose quality later
    float sigma_r = 0.1f; // [m]
    float sigma_theta = 0.01f; // [rad]

    // Robustness parameters
    bool enable_gating = false;
    float nis_gate = 30.0f; // NIS gating threshold (TO BE TUNED)

    uint64_t meas_timeout_us = 300000;  // -> DEGRADED if older than this
    uint64_t lost_timeout_us = 1000000; // -> LOST if older than this   
    
};

// ---------------------------------------------------------------------------
//  RNAVFilter
// ---------------------------------------------------------------------------
class RNAVFilter {
public:
    // Task entry wiring for OSAL (void* arg).
    // NOTE: The TaskCtx object must outlive the task (static or main-scope that never exits).
    struct TaskCtx {
        // Pointer to RNAVFilter object
        RNAVFilter* self = nullptr;
        // Pointers to Queues
        PoseMeasQueue* meas_in = nullptr;
        StateEstimateQueue* state_out = nullptr;

        CmdQueue* command_in = nullptr; // optional
    };

public:
    explicit RNAVFilter(const RNAVFilterConfig& cfg = {});

    static void TaskEntry(void* arg);

    void setConfig(const RNAVFilterConfig& cfg);

    // Core API: consume one PoseEstimate, produce one RNAVState.
    bool processMeasurement(const msg::PoseEstimate& meas, msg::RNAVState& out);

private:
    RNAVFilterConfig m_cfg{};
    RNAVInternalState m_x{};
    Mat12 m_P{};

    bool m_has_initialized = false;
    bool m_meas_available = false;

    uint64_t m_last_meas_us = 0;
    uint64_t m_last_fused_meas_us = 0; //measurement time of the last update applied

    // Run loop called by TaskEntry
    // command_in is optional (can be nullptr)
    void Run(PoseMeasQueue& meas_in, StateEstimateQueue& state_out, CmdQueue* command_in);

    bool Init(PoseMeasQueue& meas_in);
    void Propagate(float dt_s);
    bool Update(const RNAVMeasurement& z);

private:
    // Helper functions
    bool AcceptMeasurement(const msg::PoseEstimate& meas, uint64_t now_us);
    void FrameTransformation(const msg::PoseEstimate& meas, RNAVMeasurement& z);
};

} // namespace rnav
