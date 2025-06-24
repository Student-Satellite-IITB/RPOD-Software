// This is a test file for the RTOS Mutex implementation.
#include <iostream>
#include <cstdio>
#include "os/rtos.hpp"

#define NUM_THREADS 2

volatile long int a = 0;
Rtos::Mutex globalLock;

// Tried unecesary code to create an array of tasks
// which might not be needed in a real RTOS implementation.

// Rtos::Task t1, t2;
Rtos::Task tasks[NUM_THREADS];

void counterTask(void* arg) {
    long localSum = 0;
    for (int i = 0; i < 10000000; ++i) {
        localSum++;
    }

    globalLock.lock();
    a += localSum;
    globalLock.unlock();
}


int main() {
    // t1.Create("Counter1", counterTask, nullptr);
    // t2.Create("Counter2", counterTask, nullptr);
    
    // Create threads
    for (int i = 0; i < NUM_THREADS; ++i) {
        tasks[i].Create("Worker", counterTask, nullptr);
    }

    // Wait for tasks to complete (in real RTOS this would be done via signals or join logic)
    // Here we sleep enough time for tasks to finish
    // Rtos::SleepMs(5000);

    for (int i = 0; i < NUM_THREADS; ++i) {
        tasks[i].Join();
    }

    std::cout << "Final value of a: " << a << std::endl;

    return 0;
}