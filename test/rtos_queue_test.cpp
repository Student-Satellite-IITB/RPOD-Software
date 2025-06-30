#include "os/rtos.hpp"
#include <iostream>

// Define the queue type with int messages and size 5
Rtos::Queue<int, 5> queue;
constexpr int PRODUCER_TIMEOUT_MS = 1000;
constexpr int CONSUMER_TIMEOUT_MS = 1000;

void Producer(void*) {
    std::cout << "[Producer] Thread started\n";
    for (int i = 1; i <= 10; ++i) {
        if(queue.send(i, PRODUCER_TIMEOUT_MS)){ // Now blocks if queue is full
            std::cout << "[Producer] Sent: " << i << std::endl;
        }
        else std::cout << "[Producer] Send timed out!\n";
        Rtos::SleepMs(500);
    }
}

void Consumer(void*) { 
    std::cout << "[Consumer] Thread started\n";
    for (int i = 1; i <= 10; ++i) {
        int value;
        if (queue.receive(value, CONSUMER_TIMEOUT_MS)) {
            std::cout << "[Consumer] Received: " << value << std::endl;
            Rtos::SleepMs(500); // Simulate Processing
        } else {
            std::cout << "[Consumer] Receive timed out!\n";
        }
        Rtos::SleepMs(500);
    }
}

int main() {
    Rtos::Task producerTask;
    Rtos::Task consumerTask;

    producerTask.Create("Producer", Producer, nullptr);
    consumerTask.Create("Consumer", Consumer, nullptr);

    std::cout << "[Main] Waiting for threads...\n";
    producerTask.Join();
    std::cout << "[Producer] Finished sending all items.\n";
    consumerTask.Join();
    std::cout << "[Consumer] Finished processing all items.\n";

    std::cout << "[Main] Test complete.\n";
    return 0;
}
