#include "vbn/FeatureDetector.hpp"

FeatureDetector::FeatureDetector(IFeatureExtractor* extractor)
    : extractor_(extractor) {}

bool FeatureDetector::detect(const ImageFrame& img, FeatureFrame& features) {
    // Your core logic here — NO OpenCV, NO hardware specifics
    // Work on input.data, input.width, etc.

    // Only implement if there exists some platform-independent logic 
    // otherwise better to implement in platrform-specific code

     /* For example:
        Filtering out low-response keypoints
        Sorting by strength
        Packaging metadata (e.g., timestamp, source ID) */
        
    // return true;
    return extractor_->extract(img, features);
}
