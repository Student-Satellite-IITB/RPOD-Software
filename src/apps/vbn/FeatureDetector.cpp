// FeatureDetector.cpp

#include "apps/vbn/FeatureDetector.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace vbn {

static inline FeatureDetectorConfig sanitise(const FeatureDetectorConfig& in) {
    FeatureDetectorConfig cfg = in;

    // Keep area bounds valid.
    if (cfg.MIN_BLOB_AREA < 1) cfg.MIN_BLOB_AREA = 1;
    if (cfg.MAX_BLOB_AREA < cfg.MIN_BLOB_AREA) cfg.MAX_BLOB_AREA = cfg.MIN_BLOB_AREA;

    // Keep score thresholds sane.
    if (cfg.PATTERN_MAX_SCORE <= 0.0f) cfg.PATTERN_MAX_SCORE = 1e4f;

    // ROI safety clamps.
    if (cfg.ROI_RADIUS_MARGIN < 1.0f) cfg.ROI_RADIUS_MARGIN = 1.0f;
    if (cfg.ROI_BORDER_PX > 1000) cfg.ROI_BORDER_PX = 1000;

    // Adaptive-threshold safety clamps.
    if (cfg.ADAPTIVE_K_SIGMA < 0.0f) cfg.ADAPTIVE_K_SIGMA = 0.0f;
    if (cfg.ADAPTIVE_MAX_THRESH < cfg.ADAPTIVE_MIN_THRESH) {
        cfg.ADAPTIVE_MAX_THRESH = cfg.ADAPTIVE_MIN_THRESH;
    }

    return cfg;
}

} // namespace vbn

vbn::FeatureDetector::FeatureDetector(const FeatureDetectorConfig& cfg)
    : m_cfg(sanitise(cfg)) {
    reset();
}

void vbn::FeatureDetector::setConfig(const FeatureDetectorConfig& cfg) {
    m_cfg = sanitise(cfg);
}

void vbn::FeatureDetector::reset() {
    // Reset tracking memory.
    m_last_center_u_px = 0.0f;
    m_last_center_v_px = 0.0f;
    m_last_pattern_max_sep_px = 0.0f;

    // Next frame starts in LOST behavior (full-frame ROI).
    m_last_state = msg::TrackState::LOST;
    m_current_state = msg::TrackState::LOST;

    m_roi_u_min = m_roi_v_min = 0;
    m_roi_u_max = m_roi_v_max = 0;

    m_status = Status::OK;
}

const char* vbn::FeatureDetector::StatusStr(FeatureDetector::Status s) {
    switch (s) {
        case Status::OK: return "OK";
        case Status::INVALID_IMAGE: return "INVALID_IMAGE";
        case Status::NOT_ENOUGH_RAW_BLOBS: return "NOT_ENOUGH_RAW_BLOBS";
        case Status::NOT_ENOUGH_LED_BLOBS: return "NOT_ENOUGH_LED_BLOBS";
        case Status::UNKNOWN_PATTERN: return "UNKNOWN_PATTERN";
        default: return "UNKNOWN";
    }
}

void vbn::FeatureDetector::defineROI(const msg::ImageFrame& img) {
    const int w = static_cast<int>(img.width);
    const int h = static_cast<int>(img.height);
    const int border = static_cast<int>(m_cfg.ROI_BORDER_PX);

    // ----------------------------------------------------
    // LOST -> Use full frame (minus border)
    // ----------------------------------------------------
    if (m_current_state == msg::TrackState::LOST) {
        m_roi_u_min = border;
        m_roi_v_min = border;
        m_roi_u_max = (w - 1) - border;
        m_roi_v_max = (h - 1) - border;
        return;
    }

    // ----------------------------------------------------
    // TRACK -> ROI around last center and last pattern scale
    // ----------------------------------------------------
    float roi_half_width = m_cfg.ROI_RADIUS_MARGIN * (m_last_pattern_max_sep_px / 2.0f);

    // Keep ROI from collapsing.
    if (roi_half_width < ROI_MIN_HALF_WIDTH) {
        roi_half_width = ROI_MIN_HALF_WIDTH;
    }

    const float cu = m_last_center_u_px;
    const float cv = m_last_center_v_px;

    float u_min = cu - roi_half_width;
    float u_max = cu + roi_half_width;
    float v_min = cv - roi_half_width;
    float v_max = cv + roi_half_width;

    // Clamp to image bounds with the requested border.
    if (u_min < border) u_min = static_cast<float>(border);
    if (v_min < border) v_min = static_cast<float>(border);

    if (u_max > ((w - 1) - border)) u_max = static_cast<float>((w - 1) - border);
    if (v_max > ((h - 1) - border)) v_max = static_cast<float>((h - 1) - border);

    m_roi_u_min = static_cast<int>(u_min);
    m_roi_u_max = static_cast<int>(u_max);
    m_roi_v_min = static_cast<int>(v_min);
    m_roi_v_max = static_cast<int>(v_max);
    return;
}

uint16_t vbn::FeatureDetector::readDN(const msg::ImageFrame& img, int u, int v) const {
    // Read one pixel in native DN units.
    // Helper: read pixel DN at (u,v) in native units (supports 8-bit and 16-bit containers)
    // Supports only 8-bit and 16-bit containers because we return uint16_t
    // - For 16-bit containers, we explicitly merge bytes to avoid any alignment/UB issues from
    //   reinterpret_cast<uint16_t*> on potentially unaligned rows.
    //
    // Assumptions / notes:
    // - This code assumes the buffer is **little-endian** (LSB at lower address), which is true for
    //   Raspberry Pi (ARM little-endian) and typical V4L2 memory buffers on Linux.
    // - bytes_per_px == 1: GRAY8-like container, returned DN is 0..255.
    // - bytes_per_px == 2: This covers GRAY16 and RAW10/RAW12 stored in 16-bit.
    //   We return the 16-bit sample by downshifting with bit_shift
    // Examples:
    // - RAW10 MSB-aligned in uint16 (10 bits in bits [15..6]) => bit_shift = 6
    // - RAW10 LSB-aligned in uint16 (10 bits in bits [9..0])  => bit_shift = 0
    // - True GRAY16 / RAW16                                   => bit_shift = 0

    const uint8_t* row = img.data + static_cast<size_t>(v) * img.stride;

    uint16_t sample = 0;
    if (img.bytes_per_px == 1) {
        // 8-bit container
        sample = static_cast<uint16_t>(row[u]);
    } else if (img.bytes_per_px == 2) {
        // bytes_per_px == 2 (e.g. RAW10/RAW12/GRAY16 stored in 16-bit container)
        // 16-bit container, stored as 2 bytes per pixel.
        const size_t i = static_cast<size_t>(u) * 2u;
        const uint16_t lo = static_cast<uint16_t>(row[i + 0]);
        const uint16_t hi = static_cast<uint16_t>(row[i + 1]);
        sample = static_cast<uint16_t>(lo | (hi << 8));
    } else {
        // Unsupported packing: caller already uses bounded fallback behavior.
        return 0;
    }

    // Convert container sample -> native DN (LSB aligned)
    // e.g., bitshift = 6 for MSB-aligned RAW10-in-16
    const uint8_t bitshift = img.bit_shift;
    uint16_t dn = static_cast<uint16_t>(sample >> bitshift);

    if (img.bit_depth < 16) {
        const uint16_t mask = static_cast<uint16_t>((1u << img.bit_depth) - 1u);
        dn &= mask;
    }

    return dn;
}

uint16_t vbn::FeatureDetector::computeAdaptiveThreshold(const msg::ImageFrame& img) const {

    // Compute mean + variance over ROI.
    uint64_t n = 0;
    double sum = 0.0;
    double sum2 = 0.0;

    for (int v = m_roi_v_min; v <= m_roi_v_max; ++v) {
        for (int u = m_roi_u_min; u <= m_roi_u_max; ++u) {
            const double dn = static_cast<double>(readDN(img, u, v));
            sum += dn;
            sum2 += (dn * dn);
            ++n;
        }
    }

    // Deterministic fallback if ROI had no valid samples.
    if (n == 0) {
        return m_cfg.BIN_THRESH;
    }

    const double mean = sum / static_cast<double>(n);
    double var = (sum2 / static_cast<double>(n)) - (mean * mean);
    if (var < 0.0) var = 0.0; // numerical safety

    const double sigma = std::sqrt(var);
    const double adaptive = mean + static_cast<double>(m_cfg.ADAPTIVE_K_SIGMA) * sigma;

    const uint16_t min_t = m_cfg.ADAPTIVE_MIN_THRESH;
    const uint16_t max_t = m_cfg.ADAPTIVE_MAX_THRESH;
    const uint16_t t = static_cast<uint16_t>(std::lround(adaptive));

    // Bounded deterministic threshold.
    return std::min<uint16_t>(max_t, std::max<uint16_t>(min_t, t));
}

std::size_t vbn::FeatureDetector::detectBlobs(const msg::ImageFrame& img, BlobArray& blobs) {

    // Fixed-threshold baseline unless adaptive mode is enabled.
    uint16_t thresh = m_cfg.BIN_THRESH;
    if (m_cfg.ADAPTIVE_THRESH_ENABLE) {
        thresh = computeAdaptiveThreshold(img);
    }

    // 8-connected neighborhood.
    const int du[8] = {-1, 0, 1, 1, 1, 0, -1, -1};
    const int dv[8] = {-1, -1, -1, 0, 1, 1, 1, 0};

    // Statically allocated visited bitmap
    // Persists across call with ROI regions being reset on each call.
    static bool visited[VISITED_MAX_H][VISITED_MAX_W];

    struct Pix {
        int u;
        int v;
    };

    // Local flood-fill stack (bounded).
    static Pix stack[4096];
    // Non-reentrant: uses static internal work buffers. Call from one task only.
    int sp = 0; //stack pointer

    std::size_t blob_count = 0;

    // Clear visited map only in current ROI (faster than clearing full frame).
    for (int v = m_roi_v_min; v <= m_roi_v_max; ++v) {
        for (int u = m_roi_u_min; u <= m_roi_u_max; ++u) {
            visited[v][u] = false;
        }
    }

    for (int v = m_roi_v_min; v <= m_roi_v_max; ++v) {
        for (int u = m_roi_u_min; u <= m_roi_u_max; ++u) {
            const uint16_t pixel_val = readDN(img, u, v);

            if (pixel_val < thresh || visited[v][u]) {
                continue;
            }

            // Cap number of extracted blobs to configured max.
            if (blob_count >= MAX_BLOBS) {
                // might need a degraded return condition here
                // maybe reduce threshold and try again, but for now we just return what we have.
                return blob_count;
            }

            // Seed blob at (u,v).
            msg::Feature& B = blobs[blob_count];
            B.u_px = 0.0f;
            B.v_px = 0.0f;
            B.area = 0;
            B.intensity = 0.0f;

            // clearing the pixel stack pointer and pushing the seed pixel, marking visited.
            sp = 0;
            stack[sp++] = {u, v};
            visited[v][u] = true;

            // Flood-fill loop.
            while (sp > 0) {
                Pix p = stack[--sp];
                const int pu = p.u;
                const int pv = p.v;
                const uint16_t p_val = readDN(img, pu, pv);

                // Accumulate blob properties
                // Intensity Weighted coordinates used for centroid calculation after flood-filling
                // // Current implementation : Intensity-squared weighing
                // const float w = static_cast<float>(p_val);
                // const float w2 = w * w;

                // B.u_px += static_cast<float>(pu) * w2;
                // B.v_px += static_cast<float>(pv) * w2;
                // B.area += 1;
                // B.intensity += w2;

                // Current implementation : Linear intensity weighing
                const float w = static_cast<float>(p_val);
                B.u_px += static_cast<float>(pu) * w;
                B.v_px += static_cast<float>(pv) * w;
                B.area += 1;
                B.intensity += w;

                // Explore 8-connected neighbors
                for (int n = 0; n < 8; ++n) {
                    const int nu = pu + du[n];
                    const int nv = pv + dv[n];

                    if (nu < m_roi_u_min || nu > m_roi_u_max || nv < m_roi_v_min || nv > m_roi_v_max) {
                        continue;
                    }

                    if (visited[nv][nu]) {
                        continue;
                    }

                    const uint16_t n_val = readDN(img, nu, nv);
                    if (n_val < thresh) {
                        continue;
                    }

                    if (sp < 4096) {
                        // Push neighbor pixel onto stack and mark visited.
                        stack[sp++] = {nu, nv};
                        visited[nv][nu] = true;
                    }
                }
            }

            // Centroid normalization.
            if (B.intensity > 0.0f) {
                B.u_px /= B.intensity;
                B.v_px /= B.intensity;
            } else {
                // Rare fallback for degenerate blob.
                B.u_px = static_cast<float>(u);
                B.v_px = static_cast<float>(v);
            }

            // One blob fully extracted
            blob_count++;
        }
    }

    return blob_count;
}

std::size_t vbn::FeatureDetector::thresholdBlobs(BlobArray& blobs,
                                                 BlobArray& filtered_blobs,
                                                 std::size_t blob_count) const {
    std::size_t led_blob_count = 0;

    for (std::size_t i = 0; i < blob_count; ++i) {
        const msg::Feature& B = blobs[i];
        // Heurestic conditions on blob filtering
        // Area-based pruning only.
        if (B.area < m_cfg.MIN_BLOB_AREA) continue;
        if (B.area > m_cfg.MAX_BLOB_AREA) continue;

        // Potential future heurestics can include moments and eccentricity checks
        // Or intensity based on distance

        filtered_blobs[led_blob_count++] = B;
    }

    return led_blob_count;
}

bool vbn::FeatureDetector::InnerPatternArrange(InnerLedCandidates& combo) {
    // Canonical order:
    //   0 -> TOP
    //   1 -> LEFT
    //   2 -> BOTTOM
    //   3 -> RIGHT
    //   4 -> CENTER

    int idx_top = 0;
    int idx_bottom = 0;

    for (int i = 1; i < 5; ++i) {
        if (combo[i].v_px < combo[idx_top].v_px) {
            idx_top = i;
        }
        if (combo[i].v_px > combo[idx_bottom].v_px) {
            idx_bottom = i;
        }
    }

    // Remaining three are [LEFT, RIGHT, CENTER] in unknown order.
    int rem[3];
    int r = 0;

    for (int i = 0; i < 5; ++i) {
        if (i != idx_top && i != idx_bottom) {
            rem[r++] = i;
        }
    }

    if (r != 3) {
        return false;
    }

    int idx_left = rem[0];
    int idx_right = rem[0];

    for (int k = 1; k < 3; ++k) {
        int i = rem[k];
        if (combo[i].u_px < combo[idx_left].u_px) {
            idx_left = i;
        }
        if (combo[i].u_px > combo[idx_right].u_px) {
            idx_right = i;
        }
    }

    int idx_center = -1;
    for (int k = 0; k < 3; ++k) {
        int i = rem[k];
        if (i != idx_left && i != idx_right) {
            // Center candidate must lie inside the T-L-B-R box.
            if ((combo[i].u_px < combo[idx_right].u_px) && (combo[i].u_px > combo[idx_left].u_px) &&
                (combo[i].v_px > combo[idx_top].v_px) && (combo[i].v_px < combo[idx_bottom].v_px)) {
                idx_center = i;
                break;
            }
        }
    }

    if (idx_center == -1) {
        return false;
    }

    InnerLedCandidates arranged_leds;
    arranged_leds[0] = combo[idx_top];
    arranged_leds[1] = combo[idx_left];
    arranged_leds[2] = combo[idx_bottom];
    arranged_leds[3] = combo[idx_right];
    arranged_leds[4] = combo[idx_center];

    combo = arranged_leds;
    return true;
}

float vbn::FeatureDetector::evaluateInnerCross(const InnerLedCandidates& combo) {
    // combo is assumed to already be in canonical T/L/B/R/C order.
    float score = 0.0f;

    const msg::Feature& T = combo[0];
    const msg::Feature& L = combo[1];
    const msg::Feature& B = combo[2];
    const msg::Feature& R = combo[3];
    const msg::Feature& C = combo[4];

    auto dist = [](const msg::Feature& a, const msg::Feature& b) -> float {
        float du = a.u_px - b.u_px;
        float dv = a.v_px - b.v_px;
        return std::sqrt(du * du + dv * dv);
    };

    // Cross center from arm LEDs.
    float u0 = 0.25f * (T.u_px + L.u_px + B.u_px + R.u_px);
    float v0 = 0.25f * (T.v_px + L.v_px + B.v_px + R.v_px);

    msg::Feature center;
    center.u_px = u0;
    center.v_px = v0;

    float arm_TB = dist(T, B) / 2.0f;
    float arm_LR = dist(L, R) / 2.0f;
    float L_arm = 0.5f * (arm_TB + arm_LR) + 1e-6f;

    // --------------------------------------------------------
    // SCORING CRITERIA:
    // --------------------------------------------------------
    // Distance ratio score ensures that the distance ratios
    // of the pattern arms are near 1.

    // Distance ratio score is agnostic to central LED
    // There might be wrong LED blobs anywhere inside the bounding box
    // created by the T/L/B/R leds as edges these are equivalent if scored only based 
    // on distance ratio.
    // so offset score accounts for offset of the central LED
    // Offset penalty for center LED.
    float offset = dist(C, center);
    float offset_score = offset / L_arm;

    if (offset_score > m_cfg.MAX_OFFSET_SCORE) {
        // Early out if offset is too large
        // Offset score under assumptions:
        // Orthographic projection, x rad rotation about principal axis
        // offset_score = 2sinx/(1+cosx)
        // for 30 deg --> 0.535 for 45 deg --> 0.82
        // Max offset chosen to be 0.70
        return 1e6f;
    }

    // Distance Ratio penalty over distances from (T,L,B,R) to center.
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
                                           float& match_score) {
    if (blob_count < 5) {
        led_count = 0;
        pattern_id = msg::PatternId::UNKNOWN;
        match_score = 1e6f;
        return false;
    }

    // If blob count >= 5 take combinations of 5 blobs
    // and evaluate if they form the inner pattern or not
    float best_score = 1e9f;
    InnerLedCandidates best_five;

    // Brute-force all 5-combinations from candidate blobs.
    for (std::size_t i = 0; i < blob_count; ++i) {
        for (std::size_t j = i + 1; j < blob_count; ++j) {
            for (std::size_t k = j + 1; k < blob_count; ++k) {
                for (std::size_t l = k + 1; l < blob_count; ++l) {
                    for (std::size_t m = l + 1; m < blob_count; ++m) {
                        InnerLedCandidates combo = {blobs[i], blobs[j], blobs[k], blobs[l], blobs[m]};

                        // Arrange into cannonical order
                        if (!InnerPatternArrange(combo)) {
                            // try next combination
                            continue;
                        }

                        // Evaluate geometric quality of this 5-group
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
        led_count = 0;
        pattern_id = msg::PatternId::UNKNOWN;
        match_score = 1e6f;
        return false;
    }

    // Fill output slots (canonical order already enforced).
    for (int s = 0; s < 5; ++s) {
        msg::Feature& L = leds[s];
        const msg::Feature& B = best_five[s];

        L.u_px = B.u_px;
        L.v_px = B.v_px;
        L.area = B.area;
        L.intensity = B.intensity;
        L.pattern_id = msg::PatternId::INNER;
        L.slot_id = static_cast<uint8_t>(s);
        L.valid = 1;
    }

    led_count = 5;
    pattern_id = msg::PatternId::INNER;
    match_score = best_score;
    return true;
}

bool vbn::FeatureDetector::detect(const msg::ImageFrame& img, msg::FeatureFrame& out) {
    // Basic input validity guard.
    if (!img.data || img.width > VISITED_MAX_W || img.height > VISITED_MAX_H || img.stride == 0) {
        return fail(Status::INVALID_IMAGE, out);
    }
    // Only support 10-bit MSB aligned ImageFrames as this is native to current imaging pipeline
    if (img.bytes_per_px != 2 || img.bit_depth != 10 || img.bit_shift != 6) {
        return fail(Status::INVALID_IMAGE, out);
    }

    // Start from previous frame tracking state.
    m_current_state = m_last_state;

    // Clear output frame deterministically.
    for (std::size_t i = 0; i < msg::MAX_FEATS; ++i) {
        out.feats[i].u_px = 0.0f;
        out.feats[i].v_px = 0.0f;
        out.feats[i].area = 0;
        out.feats[i].intensity = 0.0f;
        out.feats[i].pattern_id = msg::PatternId::UNKNOWN;
        out.feats[i].slot_id = 0;
        out.feats[i].valid = 0;
    }

    out.t_exp_end_us = img.t_exp_end_us;
    out.frame_id = img.frame_id;

    BlobArray blobs{};
    BlobArray blobs_filtered{};
    LedArray leds{};

    // 1) ROI DEFINITION: Define ROI based on track state.
    defineROI(img);

    // 2) BLOB DETECTION: Extract raw blobs by threshold + flood-fill.
    const std::size_t num_raw_blobs = detectBlobs(img, blobs);

    // ========= OUTPUT MODE: RAW BLOBS =========
    // Debug mode: raw blobs output.
    if (m_cfg.OUTPUT_MODE == FDOutputMode::RAW_BLOBS) {
        for (std::size_t i = 0; i < num_raw_blobs; ++i) {
            out.feats[i] = blobs[i];
        }
        out.feat_count = static_cast<uint8_t>(num_raw_blobs);
        out.visible_mask = msg::VISIBLE_NONE;
        out.pattern_id = msg::PatternId::UNKNOWN;
        out.state = m_current_state;
        out.valid = true;
        m_status = Status::OK;
        return out.feat_count > 0;
    }

    // 3) BLOB-FILTERING: Area thresholding
    std::size_t num_filtered_blobs = thresholdBlobs(blobs, blobs_filtered, num_raw_blobs);

    // ========= OUTPUT MODE: FILTERED BLOBS =========
    // Debug mode: filtered blobs output.
    if (m_cfg.OUTPUT_MODE == FDOutputMode::FILTERED_BLOBS) {
        for (std::size_t i = 0; i < num_filtered_blobs; ++i) {
            out.feats[i] = blobs_filtered[i];
        }
        out.feat_count = static_cast<uint8_t>(num_filtered_blobs);
        out.visible_mask = msg::VISIBLE_NONE;
        out.pattern_id = msg::PatternId::UNKNOWN;
        out.state = m_current_state;
        out.valid = true;
        m_status = Status::OK;
        return out.feat_count > 0;
    }
    // ========= OUTPUT MODE: LEDS (nominal flight) =========
    // Simple heurestic: if num_led_blobs == 5, no area thresholding needed
    // It might be that the blobs are valid but smaller than min area
    // Even if not the pattern identification step will reject them cheaply
    if (num_filtered_blobs < 5) {
        // Not enoigh blobs to identify pattern
        // Perform FDIR logic
        if (num_raw_blobs == 5) {
            // Exactly 5 raw blobs: allow direct pattern identification attempt.
            blobs_filtered = blobs;
            num_filtered_blobs = num_raw_blobs;
        } else if (num_raw_blobs > 5) {
            // More than 5 blobs detected but none over min area threshold
            
            // Marked lost as this is a degraded state if we are in TRACKING mode
            m_current_state = msg::TrackState::LOST;
            // Future FDIR logic:
            // Pass on top-5 blobs by intensity to pattern identification step
            // Currently not implemnented so we return false here
            return fail(Status::NOT_ENOUGH_LED_BLOBS, out);
        } else {
            return fail(Status::NOT_ENOUGH_RAW_BLOBS, out);
        }
    }

    // 4) PATTERN IDENTIFICATION: Pattern scoring
    std::size_t led_count = 0;
    msg::PatternId pattern_id = msg::PatternId::UNKNOWN;
    float match_score = 1e6f; // larger is worse

    bool found_pattern = identifyPattern(blobs_filtered, num_filtered_blobs,
                                         leds, led_count, pattern_id, match_score);

    if (!found_pattern) {
        return fail(Status::UNKNOWN_PATTERN, out);
    }

    // Success path.
    m_current_state = msg::TrackState::TRACK;

    for (std::size_t i = 0; i < led_count; ++i) {
        out.feats[i] = leds[i];
    }
    out.feat_count = static_cast<uint8_t>(led_count);
    out.visible_mask = msg::VISIBLE_INNER;
    out.pattern_id = pattern_id;
    out.t_exp_end_us = img.t_exp_end_us;
    out.frame_id = img.frame_id;
    out.state = m_current_state;
    out.valid = true;

    // Update tracker memory for next frame ROI prediction.
    auto dist = [](const msg::Feature& a, const msg::Feature& b) -> float {
        float du = a.u_px - b.u_px;
        float dv = a.v_px - b.v_px;
        return std::sqrt(du * du + dv * dv);
    };

    const msg::Feature T = leds[0];
    const msg::Feature L = leds[1];
    const msg::Feature B = leds[2];
    const msg::Feature R = leds[3];

    float u0 = 0.5f * (L.u_px + R.u_px);
    float v0 = 0.5f * (T.v_px + B.v_px);

    m_last_center_u_px = u0;
    m_last_center_v_px = v0;

    float TB = dist(T, B);
    float LR = dist(L, R);

    m_last_pattern_max_sep_px = std::max(TB, LR);
    m_last_state = m_current_state;
    m_status = Status::OK;

    return true;
}

bool vbn::FeatureDetector::fail(Status s, msg::FeatureFrame& out) {
    m_status = s;

    // On detection failure we explicitly downgrade tracker state.
    m_current_state = msg::TrackState::LOST;
    m_last_state = m_current_state;

    out.feat_count = 0;
    out.visible_mask = msg::VISIBLE_NONE;
    out.pattern_id = msg::PatternId::UNKNOWN;
    out.state = m_current_state;
    out.valid = false;
    return false;
}
