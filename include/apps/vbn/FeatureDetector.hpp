// FeatureDetector.hpp

#pragma once
#include <array>
#include <cstdint>

#include "msg/ImageFrame.hpp"
#include "msg/FeatureFrame.hpp"

namespace vbn {

enum class FDOutputMode : uint8_t {
    LEDS = 0,          // Nominal output expected by SPE
    RAW_BLOBS = 1,     // Debug mode: emit all thresholded connected components
    FILTERED_BLOBS = 2 // Debug mode: emit post-area-filter components
};

// ---------------------------------------------------------------------------
// Configuration for the FeatureDetector (tunable parameters, no state).
// ---------------------------------------------------------------------------
struct FeatureDetectorConfig {
    // Fixed threshold in native DN units
    // Used directly when ADAPTIVE_THRESH_ENABLE is false.
    uint16_t BIN_THRESH = 20;

    // Adaptive thresholding.
    // Threshold = mean + K*sigma computed over current ROI.
    // Final threshold is clamped to [ADAPTIVE_MIN_THRESH, ADAPTIVE_MAX_THRESH]
    // so behavior remains deterministic and bounded.
    bool ADAPTIVE_THRESH_ENABLE = false;
    float ADAPTIVE_K_SIGMA = 1.5f;
    uint16_t ADAPTIVE_MIN_THRESH = 8;
    uint16_t ADAPTIVE_MAX_THRESH = 1023;

    // Simple blob area heuristics.
    int MIN_BLOB_AREA = 100;   // reject tiny noise blobs [pixels]
    int MAX_BLOB_AREA = 20000; // reject huge/glare blobs [pixels]

    // Max acceptable geometric score for the 5-LED inner-cross pattern.
    float PATTERN_MAX_SCORE = 1e2f;

    // Max normalized center-offset score for CENTER LED.
    float MAX_OFFSET_SCORE = 0.70f;

    // ROI half-width scaling factor:
    // ROI_half_width = ROI_RADIUS_MARGIN * (last_pattern_max_sep_px/2)
    float ROI_RADIUS_MARGIN = 2.0f;

    // Margin from image borders when defining ROI.
    uint16_t ROI_BORDER_PX = 8;

    FDOutputMode OUTPUT_MODE = FDOutputMode::LEDS;
};

// ---------------------------------------------------------------------------
// FeatureDetector
//
// External API is frame-in/frame-out. Internally it stores tracking state
// (last center, last pattern scale, last tracking state) to shrink ROI and
// improve robustness/efficiency in TRACK mode.
// ---------------------------------------------------------------------------
class FeatureDetector {
public:
    explicit FeatureDetector(const FeatureDetectorConfig& cfg = {});

    // Update runtime configuration (sanitised internally).
    void setConfig(const FeatureDetectorConfig& cfg);

    // Reset tracker memory (forces next frame to LOST-style full-frame ROI).
    void reset();

    // Core API: consume one ImageFrame, produce one FeatureFrame.
    bool detect(const msg::ImageFrame& img, msg::FeatureFrame& out);

    // Minimal FDIR/status surface for diagnostics and telemetry.
    enum class Status : uint8_t {
        OK = 0,
        INVALID_IMAGE,
        NOT_ENOUGH_RAW_BLOBS,
        NOT_ENOUGH_LED_BLOBS,
        UNKNOWN_PATTERN,
    };

    Status lastStatus() const { return m_status; }
    static const char* StatusStr(Status s);

private:
    FeatureDetectorConfig m_cfg{};

    static constexpr float ROI_MIN_HALF_WIDTH = 10.0f;
    static constexpr std::size_t MAX_BLOBS = 32;
    static constexpr std::size_t MAX_LEDS = 10;

    // Compile-time guard against message capacity mismatch.
    static_assert(MAX_BLOBS <= msg::MAX_FEATS && MAX_LEDS <= msg::MAX_FEATS,
                  "MAX_LEDS or MAX_BLOBS violating FeatureFrame MAX_FEAT limit");

    using BlobArray = std::array<msg::Feature, MAX_BLOBS>;
    using LedArray = std::array<msg::Feature, MAX_LEDS>;

    // Combination candidate for INNER pattern (T,L,B,R,C).
    using InnerLedCandidates = std::array<msg::Feature, 5>;

    // Tracking state memory
    msg::TrackState m_current_state = msg::TrackState::LOST;
    msg::TrackState m_last_state = msg::TrackState::LOST;

    float m_last_center_u_px = 0.0f;
    float m_last_center_v_px = 0.0f;
    float m_last_pattern_max_sep_px = 0.0f;

    // ROI bounds (inclusive)
    int m_roi_u_min = 0;
    int m_roi_u_max = 0;
    int m_roi_v_min = 0;
    int m_roi_v_max = 0;

    // Static upper bounds for visited bitmap allocation.
    static constexpr int VISITED_MAX_W = 1280;
    static constexpr int VISITED_MAX_H = 800;

    // FDIR
    Status m_status = Status::OK;

    // ROI setup for current frame (LOST: full frame, TRACK: shrunk ROI).
    void defineROI(const msg::ImageFrame& img);

    // Computes adaptive threshold on current ROI only.
    uint16_t computeAdaptiveThreshold(const msg::ImageFrame& img) const;

    // Connected-component extraction in ROI with 8-neighborhood flood-fill.
    std::size_t detectBlobs(const msg::ImageFrame& img, BlobArray& blobs);

    // Area-based filtering.
    std::size_t thresholdBlobs(BlobArray& blobs, BlobArray& filtered_blobs, std::size_t blob_count) const;

    // Rearranges 5-candidate set into canonical order (T,L,B,R,C).
    bool InnerPatternArrange(InnerLedCandidates& combo);

    // Geometric score for one arranged inner-cross candidate.
    float evaluateInnerCross(const InnerLedCandidates& combo);

    // Brute-force choose best 5-blob inner pattern candidate.
    bool identifyPattern(const BlobArray& blobs,
                         std::size_t blob_count,
                         LedArray& leds,
                         std::size_t& led_count,
                         msg::PatternId& pattern_id,
                         float& match_score);

    
private: // Helpers
    // Read pixel value with boundary check and bit-depth handling.
    uint16_t readDN(const msg::ImageFrame& img, int u, int v) const;

    // Unified failure path to keep output fields deterministic.
    bool fail(Status s, msg::FeatureFrame& out);
};

} // namespace vbn
