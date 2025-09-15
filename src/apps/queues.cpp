#include "apps/queues.hpp"

// VBN processing pipeline queues
Rtos::Queue<msg::ImageFrame, 1> imageFrameQueue(/*overwrite=*/true);
Rtos::Queue<msg::FeatureFrame, 1> featureFrameQueue(/*overwrite=*/true);
Rtos::Queue<msg::PoseEstimate, 1> poseEstimateQueue(/*overwrite=*/true);

// Command and event queues
Rtos::Queue<CommandPacket, 5> commandQueue;
Rtos::Queue<RPODEvent, 1> eventQueue;