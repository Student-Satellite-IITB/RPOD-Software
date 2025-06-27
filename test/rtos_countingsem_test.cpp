#include "os/rtos.hpp"
#include <iostream>

Rtos::CountingSemaphore sem(3,3);  
// Allows 3 threads to enter the critical section
// Initial count is 3, meaning 3 permits are available

struct ThreadData {
    int id;
};

constexpr int NUM_WORKERS = 6;
ThreadData threadData[NUM_WORKERS];  // Static allocation

Rtos::Mutex mutex;  // Mutex to protect console output

void Worker(void* arg) {
    auto* data = static_cast<ThreadData*>(arg);
    int id = data->id;

    mutex.lock();  // Lock mutex for console output
    std::cout << "[Worker " << id << "] Waiting for permit...\n";
    mutex.unlock();  // Unlock after printing

    sem.take();  // Block until allowed in

    mutex.lock();  // Lock again for console output
    std::cout << "[Worker " << id << "] Acquired permit, working...\n";
    mutex.unlock();  // Unlock after printing
    
    Rtos::SleepMs(500);  // Simulate work
    
    mutex.lock();  // Lock for console output
    std::cout << "[Worker " << id << "] Releasing permit...\n";
    mutex.unlock();  // Unlock after printing

    sem.give();  // Release permit
}

int main() {
    Rtos::Task workers[NUM_WORKERS];

    for (int i = 0; i < NUM_WORKERS; ++i) {
        ThreadData* data = new ThreadData{i + 1};  // Unique ID for each worker
        workers[i].Create("Worker", Worker, data);
    }

    for (int i = 0; i < NUM_WORKERS; ++i) {
        workers[i].Join();
    }

    std::cout << "[Main] All workers completed.\n";
    return 0;
}
