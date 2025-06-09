#pragma once

#include "platform/IFeatureExtractor.hpp"
#include "types/ImageFrame.hpp"
#include "types/FeatureFrame.hpp"

class FeatureDetector {
public:
    // Constructor with dependency injection
    FeatureDetector(IFeatureExtractor* extractor);

    // Runs feature detection using the underlying extractor
    bool detect(const ImageFrame& img, FeatureFrame& features);

private:
    IFeatureExtractor* extractor_;
};
