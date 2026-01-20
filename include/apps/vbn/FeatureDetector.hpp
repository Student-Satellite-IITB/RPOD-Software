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
    uint16_t BIN_THRESH        = 20;      // Threshold in native DN units of the input image.
    // No automatic scaling applied based on bit depth
    int     MIN_BLOB_AREA     = 100;      // reject tiny noise blobs [pixels]
    int     MAX_BLOB_AREA     = 20000;    // reject huge/glare blobs [pixels]

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

    enum class Status : uint8_t {
        OK = 0,

        NOT_ENOUGH_RAW_BLOBS,
        NOT_ENOUGH_LED_BLOBS,
        UNKNOWN_PATTERN,

        //ADD MORE
    };

    Status lastStatus() const { return m_status; }


private:
    FeatureDetectorConfig m_cfg{};

    static constexpr float ROI_MIN_HALF_WIDTH = 10.0f;
    static constexpr std::size_t MAX_BLOBS = 32;
    static constexpr std::size_t MAX_LEDS = 10;
    static_assert(MAX_BLOBS <= msg::MAX_FEATS && MAX_LEDS <= msg::MAX_FEATS, "MAX_LEDS or MAX_BLOBS violating FeatureFrame MAX_FEAT limit");

    using BlobArray = std::array<msg::Feature, MAX_BLOBS>; //type alias for arrray of blobs
    using LedArray = std::array<msg::Feature, MAX_LEDS>; //type alias for array of leds
    using InnerLedCandidates = std::array<msg::Feature, 5>; // type alias for Inner LED Candidates

    LedArray m_leds;
    BlobArray m_blobs;

    LedArray m_last_leds{};

    msg::TrackState m_current_state = msg::TrackState::LOST;
    msg::TrackState m_last_state = msg::TrackState::LOST;

    float m_last_center_u_px = 0.0f;
    float m_last_center_v_px = 0.0f;
    float m_last_pattern_max_sep_px = 0.0f;

    int m_roi_u_min = 0;
    int m_roi_u_max = 0;
    int m_roi_v_min = 0;
    int m_roi_v_max = 0;

    // Maximum image size for visited map
    // Have to ensure this is large enough
    static constexpr int VISITED_MAX_W = 1280;
    static constexpr int VISITED_MAX_H = 800;

    msg::PatternId m_last_pattern_id = msg::PatternId::UNKNOWN;
    // FDIR
    Status m_status = Status::OK;


    void defineROI(const msg::ImageFrame& img);

    std::size_t detectBlobs(const msg::ImageFrame& img, BlobArray& blobs);
    std::size_t thresholdBlobs(BlobArray& blobs, std::size_t blob_count);
    
    bool InnerPatternArrange(InnerLedCandidates& combo);
    float evaluateInnerCross(const InnerLedCandidates& combo);
    bool identifyPattern(const BlobArray& blobs,
                         std::size_t blob_count,
                         LedArray& leds,
                         std::size_t& led_count,
                         msg::PatternId& pattern_id,
                         float& confidence);

    // Fail
    bool fail(Status s, msg::FeatureFrame& out);
    

};

}// namespace vbn