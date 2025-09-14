#pragma once
#include "os/rtos.hpp"
#include "types.hpp"
#include "msg/ImageFrame.hpp"
#include "msg/FeatureFrame.hpp"

extern Rtos::Queue<msg::ImageFrame, 1> imageFrameQueue;
extern Rtos::Queue<msg::FeatureFrame, 1> featureFrameQueue;
extern Rtos::Queue<CommandPacket, 5> commandQueue;
extern Rtos::Queue<PoseEstimate, 1> poseEstimateQueue;
extern Rtos::Queue<RPODEvent, 1> eventQueue;