#pragma once
#include <cstdint>
#include <cstddef>
#include <array>

namespace msg {

// Pixel coords: origin = top-left; u → right, v → down (in pixels).

// Track state helps PoseEstimator choose init/recover logic.
// Mode set by Detector+Tracker; Estimator may suggest downgrade for next frame.
enum class TrackState : uint8_t { TRACK = 0, LOST = 1 };

// Target Pattern IDs
enum class PatternId : uint8_t { UNKNOWN = 0, INNER = 1, OUTER = 2 };

// Bitmask of what's visible in THIS frame (detector's judgment).
enum : uint8_t {
    VISIBLE_NONE  = 0,
    VISIBLE_INNER = 1 << 0, // bit 0 set → INNER visible
    VISIBLE_OUTER = 1 << 1, // bit 1 set → OUTER visible
    VISIBLE_BOTH  = VISIBLE_INNER | VISIBLE_OUTER
};

// ----- 5-LED cross: fixed physical IDs and array slots -----
// Physical IDs:
// ID 1 = TOP, ID 2 = LEFT, ID 3 = BOTTOM, ID 4 = RIGHT, ID 5 = CENTER (off-plane)
// Array layout (zero-based) maps to those IDs exactly via LedSlot:

enum class LedSlot : uint8_t {
    TOP    = 0,
    LEFT   = 1,  
    BOTTOM = 2,  
    RIGHT  = 3,  
    CENTER = 4  
};

// One LED measurement in image pixels (subpixel allowed).
struct Feature {
    float u_px;        // centroid u in pixel coords (float)
    float v_px;        // centroid v in pixel coords
    int area;          // number of pixels in blob
    float intensity;   // sum of pixel intensities

    PatternId pattern_id; // PatternId enum as uint8_t
    uint8_t slot_id;    // cast of LedSlot for INNER; or 0..N-1 for OUTER
    uint8_t valid;     // 1 = this LED was seen this frame; 0 = missing
};

// Keep capacity small & deterministic. 8 is a good starting point.
// - INNER uses up to 5 slots (TOP/LEFT/BOTTOM/RIGHT/CENTER).
// - OUTER can use up to 8 asymmetric slots.
// - If BOTH are visible, you can include a mix (estimator will filter by pattern).
constexpr std::size_t MAX_FEATS = 100; //

struct FeatureFrame {               
    std::array<Feature, MAX_FEATS> feats; 
    uint8_t  feat_count;              // 0...MAX_LEDS: number of valid entries in leds[]
    // If out.valid == false then led_count = blob counts

    uint8_t visible_mask;            // bitmask of visible patterns (VISIBLE_*)
    // Which template the detector recommends as "primary" this frame;
    // estimator may still try both if visible_mask==VISIBLE_BOTH.
    PatternId pattern_id;          // dominant pattern ID (UNKNOWN/INNER/OUTER)

    // --- Measurement identity ---
    uint64_t t_exp_end_us;            // copy-through from ImageFrame (exposure end, µs)
    uint32_t frame_id;                // image frame counter (optional) copy-through from ImageFrame

    // Operational status
    TrackState state;                 // INIT/TRACK/LOST from detector/tracker

    // Quality metrics
    bool valid; // Valid for FeatureDetection (Identified LEDs)
    // Status status;
    // float sat_pct;                    // % pixels saturated in source frame (0..100), helps quality gating
};

} // namespace msg