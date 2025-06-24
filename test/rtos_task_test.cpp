// This is a test file for the RTOS Task implementation.
#include <iostream>
#include <cstdio>
#include "os/rtos.hpp"

Rtos::Task t1, t2;

void task1(void* arg) {
    while (true) {
        printf("Hello from Task 1\n");
        Rtos::SleepMs(1000); // 1 second
    }
}

void task2(void* arg) {
    while (true) {
        printf("Hello from Task 2\n");
        Rtos::SleepMs(1500); // 1.5 seconds
    }
}


int main() {
    t1.Create("Task1", task1, nullptr);
    t2.Create("Task2", task2, nullptr);

    // Main thread sleeps forever
    while (true) {
        Rtos::SleepMs(10000);
    }

    return 0;
}