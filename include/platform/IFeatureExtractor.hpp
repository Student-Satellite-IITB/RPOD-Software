#pragma once

#include "types/ImageFrame.hpp"
#include "types/FeatureFrame.hpp"

class IFeatureExtractor {
public:
    virtual bool extract(const ImageFrame& img, FeatureFrame& features) = 0;
    virtual ~IFeatureExtractor() = default;
};
