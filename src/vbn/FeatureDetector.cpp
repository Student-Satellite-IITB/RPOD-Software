#include "vbn/FeatureDetector.hpp"

FeatureDetector::FeatureDetector() : extractor_() {}

bool FeatureDetector::detect(const ImageFrame& img, FeatureFrame& features) {
    // Platform-agnostic processing could go here, e.g.:
    // Filtering keypoints, sorting, packing metadata
    return extractor_.extract(img, features);
}
