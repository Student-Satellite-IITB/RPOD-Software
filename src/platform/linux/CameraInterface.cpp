#include "platform/CameraInterface.hpp"
#include <opencv2/opencv.hpp>

bool CameraOpenCV::getFrame(ImageFrame& frame) {
    cv::Mat img;
    cap.read(img);
    if (img.empty()) return false;

    frame.width = img.cols;
    frame.height = img.rows;
    frame.channels = img.channels();
    frame.timestamp_us = getTimeMicros();

    // Copy OpenCV data into abstracted frame buffer
    std::memcpy(frame.data, img.data, img.total() * img.elemSize());

    return true;
}
