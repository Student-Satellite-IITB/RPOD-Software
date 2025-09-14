#pragma once
#include <cstdint>
#include <cstddef>
#include <array>

namespace msg {

// Pixel coords: origin = top-left; u → right, v → down (in pixels).

// Track state helps PoseEstimator choose init/recover logic.
// Mode set by Detector+Tracker; Estimator may suggest downgrade for next frame.
enum class TrackState : uint8_t { INIT = 0, TRACK = 1, LOST = 2 };

// ----- 5-LED cross: fixed physical IDs and array slots -----
// Physical IDs:
// ID 1 = TOP, ID 2 = LEFT, ID 3 = BOTTOM, ID 4 = RIGHT, ID 5 = CENTER (off-plane)
// Array layout (zero-based) maps to those IDs exactly via LedSlot:
enum class LedSlot : uint8_t {
  TOP    = 0,  // physical ID 1: top (smallest v in image; "north")
  LEFT   = 1,  // physical ID 2: left  of center ("west")
  BOTTOM = 2,  // physical ID 3: bottom ("south")
  RIGHT  = 3,  // physical ID 4: right ("east")
  CENTER = 4   // physical ID 5: central, off-plane LED
};
constexpr std::size_t NUM_LEDS = 5;

// One LED measurement in image pixels (subpixel allowed).
struct Led2D {
  float u_px;        // column (x) in pixels, subpixel allowed
  float v_px;        // row (y) in pixels, subpixel allowed
  float strength;    // 0..1 detector confidence (e.g., normalized peak or blob SNR)
  uint8_t valid;     // 1 = this LED was seen this frame; 0 = missing
};

// Optional extra points (for far-range PnP if you enable more LEDs later)
constexpr std::size_t MAX_EXTRA = 3;
struct Extra2D {
  float u_px, v_px, strength;
  uint8_t id;        // 255 = unknown, else a stable index if you assign one
};

// POD, fixed-size message: deterministic to copy, no heap
struct FeatureFrame {               
  // Known 5-LED cross, fixed slots map directly to your 3D template indices.
  std::array<Led2D, NUM_LEDS> led; 
  // Optional extras (keep small, bounded)
  std::array<Extra2D, MAX_EXTRA> extra;
  uint8_t  extra_count;                 // 0..MAX_EXTRA  <-- add this

  uint64_t t_exp_end_us;            // copy-through from ImageFrame (exposure end, µs)
  uint32_t frame_id;                // copy-through from ImageFrame
  TrackState state;                 // INIT/TRACK/LOST from detector/tracker
  float sat_pct;                    // % pixels saturated in source frame (0..100), helps quality gating
  // (optional future fields: avg_blob_radius_px, motion_px_per_frame, etc.)
};

}// namespace msg