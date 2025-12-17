#include "apps/vbn/FeatureDetector.hpp"
#include "apps/queues.hpp"
#include <iostream>
#include <cmath>
// #include <opencv2/opencv.hpp>

namespace vbn {
static inline FeatureDetectorConfig sanitise(const FeatureDetectorConfig& in) {
    FeatureDetectorConfig cfg = in;

    if (cfg.BIN_THRESH > 255) cfg.BIN_THRESH = 255;

    if (cfg.MIN_BLOB_AREA < 1) cfg.MIN_BLOB_AREA = 1;
    if (cfg.MAX_BLOB_AREA < cfg.MIN_BLOB_AREA)
        cfg.MAX_BLOB_AREA = cfg.MIN_BLOB_AREA;

    if (cfg.PATTERN_MAX_SCORE <= 0.0f) cfg.PATTERN_MAX_SCORE = 1e4f;

    if (cfg.ROI_RADIUS_MARGIN < 1.0f) cfg.ROI_RADIUS_MARGIN = 1.0f; // at least 1x radius
    if (cfg.ROI_BORDER_PX > 1000) cfg.ROI_BORDER_PX = 1000;        // sanity cap

    return cfg;
}
} // namespace vbn

vbn::FeatureDetector::FeatureDetector(const FeatureDetectorConfig& cfg): m_cfg(sanitise(cfg)) {
    reset();
}

void vbn::FeatureDetector::setConfig(const FeatureDetectorConfig& cfg) {
  m_cfg = sanitise(cfg);
}

void vbn::FeatureDetector::reset() {
    // Clear tracker-derived quantities
    m_last_center_u_px       = 0.0f;
    m_last_center_v_px       = 0.0f;
    m_last_pattern_max_sep_px = 0.0f;

    m_last_state = msg::TrackState::LOST;

    // Invalidate all last_leds_
    for (auto& led : m_last_leds) {
        led.u_px      = 0.0f;
        led.v_px      = 0.0f;
        led.strength  = 0.0f;
        led.pattern_id= msg::PatternId::UNKNOWN;
        led.slot_id   = 0;
        led.valid     = 0;
    }

    // Default ROI = "whole image" until we see a frame.
    m_roi_u_min = m_roi_v_min = 0;
    m_roi_u_max = m_roi_v_max = 0;
}

void vbn::FeatureDetector::defineROI(const msg::ImageFrame& img) {
    const int w = static_cast<int>(img.width);
    const int h = static_cast<int>(img.height);

    const int border = static_cast<int>(m_cfg.ROI_BORDER_PX);

    // ----------------------------------------------------
    // CASE 1: LOST → use full frame (minus border)
    // ----------------------------------------------------
    if (m_current_state == msg::TrackState::LOST){

        m_roi_u_min = border;
        m_roi_v_min = border;

        m_roi_u_max = (w-1) - border;
        m_roi_v_max = (h-1) - border;

        return;
    }
  
    // ----------------------------------------------------
    // CASE 2: TRACK → shrink ROI around last pattern
    // ----------------------------------------------------  

    // Compute ROI half-width
    float roi_half_width = m_cfg.ROI_RADIUS_MARGIN * (m_last_pattern_max_sep_px/2);

    // Enforce minimum size
    if (roi_half_width < ROI_MIN_HALF_WIDTH) {
        roi_half_width = ROI_MIN_HALF_WIDTH;
    }

    const float cu = m_last_center_u_px;
    const float cv = m_last_center_v_px;

    float u_min = cu - roi_half_width;
    float u_max = cu + roi_half_width;
    float v_min = cv - roi_half_width;
    float v_max = cv + roi_half_width;


    // Clamp to image borders with margin
    if (u_min < border) u_min = static_cast<float>(border);
    if (v_min < border) v_min = static_cast<float>(border);

    if (u_max > ((w-1) - border)) u_max = static_cast<float>((w-1) - border);
    if (v_max > ((h-1) - border)) v_max = static_cast<float>((h-1) - border);

    // ROI defined
    m_roi_u_min = static_cast<int>(u_min);
    m_roi_u_max = static_cast<int>(u_max);
    m_roi_v_min = static_cast<int>(v_min);
    m_roi_v_max = static_cast<int>(v_max);

    return;
}

std::size_t vbn::FeatureDetector::detectBlobs(const msg::ImageFrame& img, BlobArray& blobs){
    
    const uint8_t* data = img.data;
    const int stride = static_cast<int>(img.stride);

    const uint8_t thresh = m_cfg.BIN_THRESH;

    // 8-connected neighbors
    const int du[8] = {-1, 0, 1, 1, 1, 0, -1, -1};
    const int dv[8] = {-1,-1,-1, 0, 1, 1,  1,  0};

    // Visited bitmap (static = persists across calls, but we will reset ROI region)
    // Static allocation for visited map
    static bool visited[VISITED_MAX_H][VISITED_MAX_W];

    // Local flood-fill stack
    struct Pix { int u; int v; };
    static Pix stack[4096];
    int sp = 0; //stack pointer

    std::size_t blob_count = 0;

    // clearing visited map for ROI
    for (int v = m_roi_v_min; v <= m_roi_v_max; ++v) {
        for (int u = m_roi_u_min; u <= m_roi_u_max; ++u) {         
            visited[v][u] = false;
        }
    }


    for (int v = m_roi_v_min; v <= m_roi_v_max; ++v) {
        for (int u = m_roi_u_min; u <= m_roi_u_max; ++u) {         
            
            const uint8_t pixel_val = data[v * stride + u];

            if (pixel_val < thresh || visited[v][u]) {
                continue;
            }

            // Safety check if too many blobs have been detected
            if (blob_count >= MAX_BLOBS) {
                return blob_count;
            }

            // Seeding Blob

            Blob& B = blobs[blob_count];
            // B is the current seeded blob where blob_count is the index
            // of the next blob to be detected due to the fact that
            // counting starts from 0 so if 1 blob is detected, it is already leading to index 1 which is the second blob

            B.u_cx = 0.0f;
            B.v_cx = 0.0f;
            B.intensity = 0.0f;
            B.area = 0;

            // clearing pixel stack
            sp = 0;
            // push seed pixel onto stack
            stack[sp++] = {u, v};
            // mark seed pixel as visited
            visited[v][u] = true;

            // Flood-fill loop

            while(sp > 0) {
                Pix p = stack[--sp]; //pop pixel from stack
                const int pu = p.u;
                const int pv = p.v;
                const uint8_t p_val = data[pv * stride + pu];

                // Accumulate blob properties
                // Weighter coordinates used for centroid calculation after flood-filling
                B.u_cx += static_cast<float>(pu)*std::pow(static_cast<float>(p_val),2);
                B.v_cx += static_cast<float>(pv)*std::pow(static_cast<float>(p_val),2);
                B.intensity += std::pow(static_cast<float>(p_val),2);
                B.area += 1;

                // Explore 8-connected neighbors
                for (int n = 0; n < 8; ++n){
                    const int nu = pu + du[n];
                    const int nv = pv + dv[n];

                    // Check if neighbour is in ROI
                    if( nu < m_roi_u_min || nu > m_roi_u_max ||
                        nv < m_roi_v_min || nv > m_roi_v_max ){
                        continue;
                    }

                    // Check if neighbour is already visited
                    if (visited[nv][nu]){
                        continue;
                    }

                    const uint8_t n_val = data[nv * stride + nu];

                    // Check if neighbour is above threshold
                    if (n_val < thresh){
                        continue;
                    }

                    if (sp < 4096) {
                        // Push neighbour onto stack
                        stack[sp++] = {nu, nv};
                        // Mark neighbour as visited
                        visited[nv][nu] = true;
                    }
                }
            }

            // Centroid extraction

            if (B.intensity > 0.0f) {
                B.u_cx /= B.intensity;
                B.v_cx /= B.intensity;
            } else {
                // Should not happen, but a fallback helps debugging
                B.u_cx = static_cast<float>(u);
                B.v_cx = static_cast<float>(v);
            }

            // One blob fully extracted
            blob_count++;
        }
    }
    return blob_count;
}

std::size_t vbn::FeatureDetector::thresholdBlobs(BlobArray& blobs, std::size_t blob_count){
    
    std::size_t led_blob_count = 0;

    for (std::size_t i = 0; i < blob_count; ++i) {
        const Blob& B = blobs[i];

        // Heurestic based conditions on blob filtering
        if (B.area < m_cfg.MIN_BLOB_AREA) continue;
        if (B.area > m_cfg.MAX_BLOB_AREA) continue;
        // Potential future heurestics can include moments and eccentricity checks
        // Or intensity based on distance

        // Potential LED blobs
        blobs[led_blob_count++] = B;
    }
    return led_blob_count;
}

bool vbn::FeatureDetector::InnerPatternArrange(std::array<Blob,5>& combo){
    //   index 0 -> TOP
    //   index 1 -> LEFT
    //   index 2 -> BOTTOM
    //   index 3 -> RIGHT
    //   index 4 -> CENTER

    int idx_top = 0;
    int idx_bottom = 0;

    for(int i = 1; i < 5; ++i){
        if (combo[i].v_cx < combo[idx_top].v_cx){
            idx_top = i;
        }
        if (combo[i].v_cx > combo[idx_bottom].v_cx){
            idx_bottom = i;
        }
    }

    // Array of remaining indices
    int rem[3];
    int r = 0; // counter for rem array

    for (int i = 0; i < 5; ++i){
        if (i != idx_top && i != idx_bottom){
            rem[r++] = i;
        }
    }

    if (r!=3){
        // Degenerate case
        // All LEDs are hoizontally aligned
        return false;
    }

    int idx_left = rem[0];
    int idx_right = rem[0];

    for (int k = 1; k < 3; ++k) {
        int i = rem[k];
        if (combo[i].u_cx < combo[idx_left].u_cx) {
            idx_left = i;
        }
        if (combo[i].u_cx > combo[idx_right].u_cx) {
            idx_right = i;
        }
    }

    int idx_center = -1;

    for (int k = 0; k < 3; ++k) {
        int i = rem[k];
        if (i != idx_left && i != idx_right) {

            // Condition that central LED projection lies in box with
            // corners defined by top, bottom, left, right LEDs
            
            if((combo[i].u_cx < combo[idx_right].u_cx) && (combo[i].u_cx > combo[idx_left].u_cx)){
                if((combo[i].v_cx > combo[idx_top].v_cx) && (combo[i].v_cx < combo[idx_bottom].v_cx)){
                    idx_center = i;
                    break;
                }
            }

        }
    }

    if (idx_center == -1) {
        // Degenerate case
        // All LEDs are vertically aligned
        return false;
    }
    
    // Arranged led_blob array
    std::array<Blob,5> arranged_leds;

    arranged_leds[0] = combo[idx_top];
    arranged_leds[1] = combo[idx_left];
    arranged_leds[2] = combo[idx_bottom];
    arranged_leds[3] = combo[idx_right];
    arranged_leds[4] = combo[idx_center];

    combo = arranged_leds;
    return true;
}

float vbn::FeatureDetector::evaluateInnerCross(const std::array<Blob,5>& combo){
    // combo is assumed to be in cannonical order:
    // index 0 -> TOP
    // index 1 -> LEFT
    // index 2 -> BOTTOM
    // index 3 -> RIGHT
    // index 4 -> CENTER
    
    float score = 0.0f;

    // Computing geometric properties for scoring

    const Blob& T = combo[0];
    const Blob& L = combo[1];
    const Blob& B = combo[2];
    const Blob& R = combo[3];
    const Blob& C = combo[4];

    auto dist = [](const Blob& a, const Blob& b) -> float {
        float du = a.u_cx - b.u_cx;
        float dv = a.v_cx - b.v_cx;
        return std::sqrt(du*du + dv*dv);
    };

    // Pattern center
    float u0 = 0.25f * (T.u_cx + L.u_cx + B.u_cx + R.u_cx);
    float v0 = 0.25f * (T.v_cx + L.v_cx + B.v_cx + R.v_cx);

    Blob center;
    center.u_cx = u0;
    center.v_cx = v0;

    // ----------- TRACKING STATE -----------------------
    if(m_current_state == msg::TrackState::TRACK){
        // Rules for pattern ID if we are in TRACK state

        // Pattern center doesnt change a lot from previous frame
        // LED Tracking Window
        float track_win = m_cfg.LED_TRACK_WINDOW_PX;

        float u0_prev = m_last_center_u_px;
        float v0_prev = m_last_center_v_px;
        
        // Instant reject if center moves too far away from last computed center
        if(fabsf(u0 - u0_prev) > track_win || fabsf(v0 - v0_prev) > track_win){
            // disqualify combination
            return 1e6f;   // or some m_cfg.INVALID_SCORE
        }
    }
    
    // ------------ LOST STATE ------------------------
    // Cross scale
    float arm_TB = dist(T, B)/2;
    float arm_LR = dist(L, R)/2;
    float L_arm  = 0.5f * (arm_TB + arm_LR) + 1e-6f;

    // Scoring criteria
    // Distance ratio score ensures that the distance ratios
    // of the pattern arms are near 1.

    // Distance ratio score is agnostic to central LED
    // There might be wrong LED blobs anywhere inside the bounding box
    // created by the T/L/B/R leds as edges these are equivalent if scored only based 
    // on distance ratio.
    // so offset score accounts for offset of the central LED


    // offset penalty
    float offset = dist(C, center);
    float offset_score = offset / L_arm;

    if(offset_score > m_cfg.MAX_OFFSET_SCORE){
        // Early out if offset is too large
        // Offset score under assumptions:
        // Orthographic projection, x rad rotation about principal axis
        // offset_score = 2sinx/(1+cosx)
        // for 30 deg --> 0.535 for 45 deg --> 0.82
        // Max offset chosen to be 0.70
        return 1e6f;
    }

    // distance ratio penalty
    float d[4];
    d[0] = dist(T, center);
    d[1] = dist(L, center);
    d[2] = dist(B, center);
    d[3] = dist(R, center);
    
    float ratio_score = 0.0f;

    for (int i = 0; i < 4; ++i) {
        for (int j = i + 1; j < 4; ++j) {
            const float r = d[i] / (d[j] + 1e-6f);
            const float diff = r - 1.0f;
            ratio_score += std::sqrt(diff * diff);
        }
    }

    score = offset_score + ratio_score;
    return score;
}

bool vbn::FeatureDetector::identifyPattern(const BlobArray& blobs,
                                      std::size_t blob_count,
                                      LedArray& leds,
                                      std::size_t& led_count,
                                      msg::PatternId& pattern_id,
                                      float& confidence){

    if (blob_count < 5) {
        // Write some FDIR logic which can identify partial patterns too maybe using previosu states
        // Identifies which LEDs have failed
        led_count = 0;
        pattern_id = msg::PatternId::UNKNOWN;
        confidence = 1e6f;
        return false; // Not enough blobs to form the pattern
    }

    // If blob count >= 5 take combinations of 5 blobs
    // and evaluate if they form the inner pattern or not

    // If blob count = 5 there is only 1 combination but that too might have 
    // a huge score if they are wrongly identified blobs so we need to score that combination too
    // and check if the score is smaller than a set PATTERN_MAX_SCORE

    // Initialise scoring
    float best_score = 1e9f;
    std::array<Blob,5> best_five;

    // -------------------------------------------------------------
    // COMBINATION SEARCH
    // -------------------------------------------------------------
    for (std::size_t i = 0; i < blob_count; ++i) {
        for (std::size_t j = i + 1; j < blob_count; ++j) {
            for (std::size_t k = j + 1; k < blob_count; ++k) {
                for (std::size_t l = k + 1; l < blob_count; ++l) {
                    for (std::size_t m = l + 1; m < blob_count; ++m) {

                        // The current 5-choice indices:
                        std::array<Blob,5> combo = {
                            blobs[i],
                            blobs[j],
                            blobs[k],
                            blobs[l],
                            blobs[m]
                        };

                        // Arrange into cannonical order
                        if(!InnerPatternArrange(combo)){
                            // try next combination
                            continue;
                        }

                        //Evaluate geometric quality of this 5-group
                        float score = evaluateInnerCross(combo);

                        if (score < best_score) {
                            best_score = score;
                            best_five = combo;
                        }

                    }
                }
            }
        }
    }

    if (best_score >= m_cfg.PATTERN_MAX_SCORE) {
        // pattern not recognized
        led_count = 0;
        pattern_id = msg::PatternId::UNKNOWN;
        confidence = best_score;
        return false;
    }

    // Place the LEDs into slots
    for (int s = 0; s < 5; ++s) {
        msg::Led2D& L = leds[s];
        const Blob& B = best_five[s];

        L.u_px       = B.u_cx;
        L.v_px       = B.v_cx;
        L.strength   = B.intensity / (255.0f * B.area + 1e-6f);
        L.pattern_id = msg::PatternId::INNER;
        L.slot_id    = static_cast<uint8_t>(s);  // TOP/LEFT/BOTTOM/RIGHT/CENTER
        L.valid      = 1;
    }

    led_count    = 5;
    pattern_id   = msg::PatternId::INNER;
    confidence   = best_score;
    return true;
}

bool vbn::FeatureDetector::detect(const msg::ImageFrame& img, msg::FeatureFrame& out) {

    // We start by assuming we havent lost last state
    m_current_state = m_last_state;

    // Filling FeatureFrame if detection fails
    // All failure conditions then only have to change the state and return false
    out.led_count   = 0;
    out.visible_mask = msg::VISIBLE_NONE;
    out.pattern_id  = msg::PatternId::UNKNOWN;

    out.t_exp_end_us = img.t_exp_end_us;
    out.frame_id    = img.frame_id;
    out.state       = m_last_state;

    for (std::size_t i = 0; i < msg::MAX_LEDS; ++i) {
        out.leds[i].u_px = 0.0f;
        out.leds[i].v_px = 0.0f;
        out.leds[i].strength = 0.0f;
        out.leds[i].pattern_id = msg::PatternId::UNKNOWN;
        out.leds[i].slot_id = 0;
        out.leds[i].valid = 0;
    }

    // ROI DEFINTION
    defineROI(img);
    
    // BLOB DETECTION
    BlobArray blobs{};
    const std::size_t num_blobs = detectBlobs(img, blobs);

    std::cout << "[FeatureDetector] Detected " << num_blobs << " blobs\n";

    if (num_blobs <5) {
        // Less than 5 blobs found → LOST
        m_current_state = msg::TrackState::LOST;
        out.state = m_current_state;
        return false;
    }
    
    // BLOB-AREA THRESHOLDING

    // threshold blobs modifies blobs in place so we back them up
    BlobArray blobs_raw = blobs; 

    std::size_t num_led_blobs = thresholdBlobs(blobs, num_blobs);

    std::cout << "[FeatureDetector] " << num_led_blobs << " blobs after area thresholding\n";

    // Simple heurestic: if num_led_blobs == 5, no area thresholding needed
    // It might be that the blobs are valid but smaller than min area
    // Even if not the pattern identification step will reject them cheaply
    if(num_led_blobs < 5){
        // Not enough blobs to identify pattern
        // We perform some FDIR logic

        if(num_blobs == 5){
            // All blobs are potential LED blobs
            // No area thresholding needed

            blobs = blobs_raw; // Restore raw blobs
            num_led_blobs = num_blobs;

        } else {
            // More than 5 blobs detected but none over min area threshold
            
            // Mark lost as this is a degraded state if we are in TRACKING mode
            // Dont return false immediately as we can try top-5 blobs for pattern identification
            m_current_state = msg::TrackState::LOST;

            // Future FDIR logic:
            // Pass on top-5 blobs by intensity to pattern identification step
            // Currently not implemnented so we return false here
            out.state = m_current_state;
            return false; // Detection was unsucessfull due to insufficient blobs
        }
    }

    // Idea for future FDIR logic here:
    // if num_led_blobs < 5 post thresholding relax area thresholds
    // and try again till some limit
    // Or use dynamic area thresholds based on range estimate if any

    // PATTERN-IDENTIFICATION
    LedArray leds{};
    // Initial estimate of number of leds
    std::size_t led_count = 0;
    msg::PatternId pattern_id = m_last_pattern_id;
    float confidence = 1e6f;
    
    bool found_pattern = identifyPattern(blobs, num_led_blobs, leds, led_count, pattern_id, confidence);
    if(!found_pattern){
        // Identify pattern fails
        
        std::cout << "[FeatureDetector] Pattern identification failed with confidence: " << confidence << "\n";

        // If in TRACK state, try once more
        if(m_current_state == msg::TrackState::TRACK){

            // Downgrade to LOST
            m_current_state = msg::TrackState::LOST;

            bool found_pattern = identifyPattern(blobs, num_led_blobs, leds, led_count, pattern_id, confidence);

            // attempting pattern identification again w/o tracking assumptions
            if(!found_pattern){
                // fails again

                // Maybe populate FeatureFrame here irrespetive
                // and SPE will check if detection returns true or false
                // and maybe valid status in the FeatureFrame to decide if to use it or not.
                // The FeatureFrame info like number of blobs detected etc can be passed onto the SPE
                // and higher so that they can make decisions based on assumed reason for failure.

                // We can populate items in FeatureFrame that can help identify where detection failed.

                out.state = m_current_state;
                return false; // Detection was unsucessfull
                // frame will be dropped
            }
        }

        out.state = m_current_state;
        return false; // Detection was unsucessfull
    }
    // Pattern identification was successful
    m_current_state = msg::TrackState::TRACK;

    // POPULATE FEATURE FRAME
    out.led_count = led_count;
    out.visible_mask = msg::VISIBLE_INNER;
    out.leds = leds;
    out.pattern_id = pattern_id;
    out.t_exp_end_us = img.t_exp_end_us;
    out.frame_id = img.frame_id;
    out.state = m_current_state;

    // TRACKING

    auto dist = [](const msg::Led2D& a, const msg::Led2D& b) -> float {
        float du = a.u_px - b.u_px;
        float dv = a.v_px - b.v_px;
        return std::sqrt(du*du + dv*dv);
    };

    const msg::Led2D T = leds[0]; // TOP
    const msg::Led2D L = leds[1]; // LEFT
    const msg::Led2D B = leds[2]; // BOTTOM
    const msg::Led2D R = leds[3]; // RIGHT

    float u0 = 0.5f * (L.u_px + R.u_px);
    float v0 = 0.5f * (T.v_px + B.v_px);

    m_last_center_u_px = u0;
    m_last_center_v_px = v0;

    float TB = dist(T, B);
    float LR = dist(L, R);
    
    m_last_pattern_max_sep_px = std::max(TB,LR);
    m_last_leds = leds;
    m_last_pattern_id = pattern_id;
    m_last_state = m_current_state;

    return true; // Indicates detection was "successful"
}