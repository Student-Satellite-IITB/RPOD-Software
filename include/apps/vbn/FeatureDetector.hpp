#pragma once
#include "types.hpp"
#include "platform/linux/FeatureExtractor.hpp"  // Include directly

class FeatureDetector {
public:
    FeatureDetector();  // constructor sets up the extractor
    bool detect(const ImageFrame& img, FeatureFrame& features);

private:
    FeatureExtractor extractor_;  // embedded, no dynamic allocation
};
