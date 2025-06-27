#pragma once
#include "os/rtos.hpp"
#include "types.hpp"

extern Rtos::Queue<ImageFrame, 1> imageFrameQueue;
extern Rtos::Queue<FeatureFrame, 1> featureFrameQueue;
extern Rtos::Queue<CommandPacket, 5> commandQueue;
extern Rtos::Queue<PoseEstimate, 1> poseEstimateQueue;