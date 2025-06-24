#include "os/rtos.hpp"
#include <pthread.h>
#include <unistd.h>   // for usleep
#include <iostream>   // for std::cerr

namespace Rtos {

// Sleep utility
void SleepMs(int ms) {
    usleep(ms * 1000);  // Convert ms to microseconds
}

// =======================
// Task Implementation
// =======================

// Wrapper to convert function pointer to pthread-style
struct ThreadArgs {
    void (*fn)(void*);
    void* arg;
};

// Static thread entry point
void* threadEntryPoint(void* ptr) {
    ThreadArgs* args = static_cast<ThreadArgs*>(ptr);
    args->fn(args->arg);
    delete args;
    return nullptr;
}

// Platform-specific handle
struct Task::TaskHandle {
    pthread_t thread;
    bool created = false;
    bool joined = false;
};

// Constructor
Task::Task() {
    handle_ = new TaskHandle{};
}

// Destructor
Task::~Task() {
    if (handle_ && !handle_->joined && handle_->created) {
        pthread_detach(handle_->thread);  // detach if not joined
    }
    delete handle_;
}

// Create a new thread
void Task::Create(const char* /*name*/, void (*fn)(void*), void* arg) {

    auto* args = new ThreadArgs{fn, arg};

    if (pthread_create(&handle_->thread, nullptr, threadEntryPoint, args) == 0) {
        handle_->created = true;
    } else {
        std::cerr << "Failed to create task\n";
        delete args;
    }
}

void Task::Join() {
    if (handle_ && handle_->created && !handle_->joined) {
        pthread_join(handle_->thread, nullptr);
        handle_->joined = true;
    }
}

// =======================
// Mutex Implementation
// =======================

struct Mutex::MutexHandle {
    pthread_mutex_t native;
};

Mutex::Mutex() {
    handle_ = new MutexHandle;
    if (pthread_mutex_init(&handle_->native, nullptr) != 0) {
        std::cerr << "Mutex init failed\n";
    }
}

Mutex::~Mutex() {
    pthread_mutex_destroy(&handle_->native);
    delete handle_;
}

void Mutex::lock() {
    pthread_mutex_lock(&handle_->native);
}

void Mutex::unlock() {
    pthread_mutex_unlock(&handle_->native);
}

}  // namespace Rtos
