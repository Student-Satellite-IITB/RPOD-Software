#pragma once
#include <array>
#include <cstdint>
#include "msg/ImageFrame.hpp"
#include "msg/FeatureFrame.hpp"

namespace vbn {

// Tunables kept small and explicit for early bring-up.
struct FeatureDetectorConfig {
    uint8_t bin_thresh      = 180;   // GRAY8 threshold for LED blobs (0..255)
    int     min_blob_area   = 3;     // reject tiny noise blobs [pixels]
    int     max_blob_area   = 400;   // reject huge/glare blobs [pixels]
    float   assoc_gate_px   = 12.0f; // pixel gate for 2D association to last frame
    uint8_t sat_level       = 250;   // pixels >= sat_level counted as "saturated"
};

class FeatureDetector {
public:
    explicit FeatureDetector(const FeatureDetectorConfig& cfg = {});
    void setConfig(const FeatureDetectorConfig& cfg);
    void reset(); // forget any tracking history

    // Core API: consume one ImageFrame, produce one FeatureFrame.
    // Returns true if at least one LED was detected.
    bool detect(const msg::ImageFrame& img, msg::FeatureFrame& out);

    // RTOS task entry (optional). Pass pointer to FeatureDetectorConfig as arg.
    static void Run(void* arg);

private:
    FeatureDetectorConfig cfg_{};
    // Tiny tracker state (image space) to decide INIT/TRACK/LOST and keep IDs stable.
    struct Track {
        float   u_px = 0.f;
        float   v_px = 0.f;
        uint8_t valid = 0;
    };

    std::array<Track, msg::NUM_LEDS> last_{};
    uint32_t last_frame_id_ = 0;
};

}// namespace vbn