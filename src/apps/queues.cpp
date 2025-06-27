#include "apps/queues.hpp"

Rtos::Queue<ImageFrame, 1> imageFrameQueue;
Rtos::Queue<FeatureFrame, 1> featureFrameQueue;
Rtos::Queue<CommandPacket, 5> commandQueue;