#pragma once
#include <array>
#include <cstdint>
#include "msg/ImageFrame.hpp"
#include "msg/FeatureFrame.hpp"

namespace vbn {

// ---------------------------------------------------------------------------
// Configuration for the FeatureDetector (tunable parameters, no state).
// ---------------------------------------------------------------------------
struct FeatureDetectorConfig {
    uint8_t BIN_THRESH        = 180;    // GRAY8 threshold for LED blobs (0..255)
    int     MIN_BLOB_AREA     = 3;      // reject tiny noise blobs [pixels]
    int     MAX_BLOB_AREA     = 400;    // reject huge/glare blobs [pixels]

    // Max acceptable geometric score for the 5-LED cross pattern.
    float PATTERN_MAX_SCORE = 1e2f;
    float MAX_OFFSET_SCORE = 0.70f; 

    float LED_TRACK_WINDOW_PX = 100;

    // ROI half-width scaling factor: ROI_radius = ROI_RADIUS_MARGIN * last_pattern_radius_px
    float   ROI_RADIUS_MARGIN = 2.0f;   // e.g. 2x the last measured pattern radius

    // Minimum border from image edges when defining ROI (in pixels).
    uint16_t ROI_BORDER_PX    = 8;
    
};


// ---------------------------------------------------------------------------
// FeatureDetector: stateless API from outside, stateful tracker inside.
// Call detect() once per image.
// ---------------------------------------------------------------------------
class FeatureDetector {
public:
    // Constructor with configuration
    explicit FeatureDetector(const FeatureDetectorConfig& cfg = {});

    // Update configuration at runtime
    void setConfig(const FeatureDetectorConfig& cfg);

    // Reset internal state (forget tracking history)
    void reset();

    // Core API: consume one ImageFrame, produce one FeatureFrame.
    bool detect(const msg::ImageFrame& img, msg::FeatureFrame& out);

private:
    FeatureDetectorConfig m_cfg{};

    std::array<msg::Led2D, msg::MAX_LEDS> m_last_leds{};

    msg::TrackState m_current_state = msg::TrackState::LOST;
    msg::TrackState m_last_state = msg::TrackState::LOST;

    float m_last_center_u_px = 0.0f;
    float m_last_center_v_px = 0.0f;
    float m_last_pattern_max_sep_px = 0.0f;

    int m_roi_u_min = 0;
    int m_roi_u_max = 0;
    int m_roi_v_min = 0;
    int m_roi_v_max = 0;

    static constexpr float ROI_MIN_HALF_WIDTH = 10.0f;

    void defineROI(const msg::ImageFrame& img);

    struct Blob {
    float u_cx;      // centroid u in pixel coords (float)
    float v_cx;      // centroid v in pixel coords
    float intensity; // sum of pixel intensities
    int   area;      // number of pixels in blob
    };

    static constexpr std::size_t MAX_BLOBS = 32;
    // Maximum image size for visited map
    // Have to ensure this is large enough
    static constexpr int VISITED_MAX_W = 1280;
    static constexpr int VISITED_MAX_H = 800;

    using BlobArray = std::array<Blob, MAX_BLOBS>; //type alias for arrray of blobs
    std::size_t detectBlobs(const msg::ImageFrame& img, BlobArray& blobs);
    std::size_t thresholdBlobs(BlobArray& blobs, std::size_t blob_count);

    msg::PatternId m_last_pattern_id = msg::PatternId::UNKNOWN;

    
    int InnerPatternArrange(std::array<Blob,5>& combo);
    float evaluateInnerCross(const std::array<Blob,5>& combo);
    using LedArray = std::array<msg::Led2D, msg::MAX_LEDS>; //type alias for array of leds
    bool identifyPattern(const BlobArray& blobs,
                         std::size_t blob_count,
                         LedArray& leds,
                         std::size_t& led_count,
                         msg::PatternId& pattern_id,
                         float& confidence);
    

};

}// namespace vbn