// #include <opencv2/opencv.hpp>
// #include "platform/linux/FeatureExtractor.hpp"

// bool FeatureExtractor::extract(const ImageFrame& img, FeatureFrame& features) {
//     cv::Mat gray(img.height, img.width, CV_8UC1, img.data);

//     // Setup blob detector parameters
//     cv::SimpleBlobDetector::Params params;
//     params.filterByColor = true;
//     params.blobColor = 255;  // bright blobs

//     params.filterByArea = true;
//     params.minArea = 20;
//     params.maxArea = 5000;

//     params.filterByCircularity = true;
//     params.minCircularity = 0.7;

//     params.filterByInertia = false;
//     params.filterByConvexity = false;

//     auto detector = cv::SimpleBlobDetector::create(params);
//     std::vector<cv::KeyPoint> keypoints;
//     detector->detect(gray, keypoints);

//     for (const auto& kp : keypoints) {
//         features.keypoints.push_back({(int)kp.pt.x, (int)kp.pt.y, kp.response});
//     }

//     return !features.keypoints.empty();
// }
