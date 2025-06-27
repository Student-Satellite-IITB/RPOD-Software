#include "os/rtos.hpp"
#include <iostream>

// Define the queue type with int messages and size 5
Rtos::Queue<int, 5> queue;

void Producer(void*) {
    std::cout << "[Producer] Thread started\n";
    for (int i = 1; i <= 10; ++i) {
        queue.send(i);  // Now blocks if queue is full
        std::cout << "[Producer] Sent: " << i << std::endl;
        Rtos::SleepMs(50);  // Simulate slower production
    }
}

void Consumer(void*) {
    Rtos::SleepMs(1000); 
    std::cout << "[Consumer] Thread started\n";
    for (int i = 1; i <= 10; ++i) {
        int value;
        queue.receive(value);  // Now blocks until item is available
        std::cout << "[Consumer] Received: " << value << std::endl;
        Rtos::SleepMs(500);  // Simulate processing
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
