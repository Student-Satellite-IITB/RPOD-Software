#include "apps/vbn/FeatureDetector.hpp"
#include "apps/queues.hpp"
#include <iostream>
// #include <opencv2/opencv.hpp>

namespace vbn {
static inline FeatureDetectorConfig sanitise(const FeatureDetectorConfig& in) {
  FeatureDetectorConfig cfg = in;
  if (cfg.bin_thresh > 255) cfg.bin_thresh = 255;
  if (cfg.min_blob_area < 1) cfg.min_blob_area = 1;
  if (cfg.max_blob_area < cfg.min_blob_area) cfg.max_blob_area = cfg.min_blob_area;
  if (cfg.assoc_gate_px < 0.f) cfg.assoc_gate_px = 0.f;
  if (cfg.sat_level < 1) cfg.sat_level = 1;
  return cfg;
}
}

vbn::FeatureDetector::FeatureDetector(const FeatureDetectorConfig& cfg): cfg_(sanitise(cfg)) {
    reset();
}

void vbn::FeatureDetector::setConfig(const FeatureDetectorConfig& cfg) {
  cfg_ = sanitise(cfg);
}

void vbn::FeatureDetector::reset() {
  last_frame_id_ = 0;
  for (auto& t : last_) { t.u_px = 0.f; t.v_px = 0.f; t.valid = 0; }
}


//-------CORE API--------------------------------

bool vbn::FeatureDetector::detect(const msg::ImageFrame& img, msg::FeatureFrame& out) {
  // Placeholder implementation: no actual detection logic yet.
  // Just clears output and returns false.
  out = {};

  // pass-through timing/identity
  out.t_exp_end_us = img.t_exp_end_us;
  out.frame_id     = img.frame_id;

  // defaults for this stub
  out.sat_pct      = 0.0f;
  out.state        = msg::TrackState::LOST;
  out.extra_count  = 0;

  // TODO(step 3): compute saturation percentage into out.sat_pct
  // TODO(step 4): find bright blob candidates (bounded)
  // TODO(step 5): assign to fixed IDs (TOP, LEFT, BOTTOM, RIGHT, CENTER)
  // TODO(step 6): decide INIT/TRACK/LOST based on last_ + assoc gate
  // TODO(step 7): update last_ tracker state

  // return true only if we detected at least one LED (stub: false)
  return false;
}

// ==============================================
// =========== Feature Detection Task ===========
// ==============================================

// Static method to run the feature detection task
// This method will be called by the RTOS task system
void vbn::FeatureDetector::Run(void* arg) {

    // optional config through arg
    FeatureDetectorConfig cfg{};
    if (arg) cfg = *reinterpret_cast<const FeatureDetectorConfig*>(arg);

    FeatureDetector det{cfg};
    
    while (true) {
        msg::ImageFrame img{};
        imageFrameQueue.receive(img); // blocking wait

        msg::FeatureFrame out{};
        const bool ok = det.detect(img, out);

        if(ok) {
            if (!featureFrameQueue.try_send(out)) {
                std::cerr << "[FEAT_DET] queue full, drop frame_id=" << out.frame_id << "\n";
            }
            } else {
            // still useful to push? for now, skip pushing LOST/empty frames
            // std::cerr << "[FEAT_DET] no features for frame_id=" << img.frame_id << "\n";
            std::cout << "[FEAT_DET] no features for frame_id=" << img.frame_id << "\n";
            }

        // pacing is driven by input; keep it light
        Rtos::SleepMs(1); // optional tiny yield
    }
}