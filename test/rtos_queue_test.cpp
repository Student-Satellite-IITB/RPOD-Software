#include "os/rtos.hpp"
#include <iostream>
#include <string>

// Define the queue type with int messages and size 5
Rtos::Queue<int, 5> queue;

void Producer(void*) {
    std::cout << "[Producer] Thread started\n";
    for (int i = 1; i <= 10; ++i) {
        while (!queue.send(i)) {
            // Queue full, wait and retry
            Rtos::SleepMs(10);
        }
        // Note send() is non blocking, but our implementation
        // will wait for 10 ms if the queue is full and retry.
        // If space is not made in 10 ms, it will ignore and drop the values (not overwrite).
        // if(queue.send(i)){
        //     std::cout << "[Producer] Sent: " << i << std::endl;
        // }
        // else {
        //     std::cout << "[Producer] Queue full, dropping value: " << i << std::endl;
        // }
        std::cout << "[Producer] Sent: " << i << std::endl;
        Rtos::SleepMs(50);  // Simulate slower production
    }
}

void Consumer(void*) {
    //Rtos::SleepMs(1000);
    std::cout << "[Consumer] Thread started\n";
    int count = 0;
    while(1) {
        int value;
        if (queue.receive(value)) {
            count++;
            std::cout << "[Consumer] Received: " << value << std::endl;
            Rtos::SleepMs(50);  // Simulate processing time
        }
        if (count == 10) {
            break;  // Exit after processing all items
        }
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
