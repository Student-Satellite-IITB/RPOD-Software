#include "apps/queues.hpp"

Rtos::Queue<msg::ImageFrame, 1> imageFrameQueue(/*overwrite=*/true);
Rtos::Queue<msg::FeatureFrame, 1> featureFrameQueue(/*overwrite=*/true);
Rtos::Queue<PoseEstimate, 1> poseEstimateQueue(/*overwrite=*/true);
Rtos::Queue<CommandPacket, 5> commandQueue;
Rtos::Queue<RPODEvent, 1> eventQueue;