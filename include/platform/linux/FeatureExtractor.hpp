#pragma once
#include "types.hpp"

class FeatureExtractor {
public:
    bool extract(const ImageFrame& img, FeatureFrame& features);
};
