#include "os/rtos.hpp"
#include <iostream>


Rtos::BinarySemaphore sem;

void Consumer(void*) {
    std::cout << "[Consumer] Waiting for semaphore...\n";
    // sem.take();  // Should block until Producer gives
    // Non-blocking attempt
    if(sem.try_take()){
        std::cout << "[Consumer] Semaphore acquired immediately!\n";
    } else {
        std::cout << "[Consumer] Semaphore not available, waiting...\n";
        sem.take();  // Will block until Producer gives
        std::cout << "[Consumer] Semaphore acquired after waiting!\n";
    } 
}

void Producer(void*) {
    std::cout << "[Producer] Sleeping for 2 seconds before giving semaphore...\n";
    Rtos::SleepMs(2000);  // Simulate delay
    std::cout << "[Producer] Giving semaphore now.\n";
    sem.give();
}

int main() {
    std::cout << "[Main] Starting Binary Semaphore Test\n";

    Rtos::Task consumerTask;
    Rtos::Task producerTask;

    consumerTask.Create("Consumer", Consumer, nullptr);
    producerTask.Create("Producer", Producer, nullptr);

    consumerTask.Join();
    producerTask.Join();

    std::cout << "[Main] Binary Semaphore Test complete.\n";
    return 0;
}
