#pragma once
#include "os/rtos.hpp"
#include "types.hpp"
#include "msg/ImageFrame.hpp"
#include "msg/FeatureFrame.hpp"
#include "msg/PoseEstimate.hpp"

// VBN processing pipeline queues
extern Rtos::Queue<msg::ImageFrame, 1> imageFrameQueue;
extern Rtos::Queue<msg::FeatureFrame, 1> featureFrameQueue;
extern Rtos::Queue<msg::PoseEstimate, 1> poseEstimateQueue;

// Command and event queues
extern Rtos::Queue<CommandPacket, 5> commandQueue;
extern Rtos::Queue<RPODEvent, 1> eventQueue;