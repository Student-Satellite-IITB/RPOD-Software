#include "os/rtos.hpp"
#include <pthread.h>
#include <unistd.h>   // for usleep
#include <iostream>   // for std::cerr
#include <semaphore.h>

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

// =======================
// Binary Semaphore Implementation
// =======================

struct BinarySemaphore::SemaphoreHandle {
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    bool available;  // acts like a binary flag
};

BinarySemaphore::BinarySemaphore() {
    handle_ = new SemaphoreHandle;
    pthread_mutex_init(&handle_->mutex, nullptr);
    pthread_cond_init(&handle_->cond, nullptr);
    handle_->available = false;  // starts as "not given"
}

BinarySemaphore::~BinarySemaphore() {
    pthread_cond_destroy(&handle_->cond);
    pthread_mutex_destroy(&handle_->mutex);
    delete handle_;
}

void BinarySemaphore::take() {
    pthread_mutex_lock(&handle_->mutex);
    while (!handle_->available) {
        pthread_cond_wait(&handle_->cond, &handle_->mutex);
    }
    handle_->available = false;  // consume the semaphore
    pthread_mutex_unlock(&handle_->mutex);
}

bool BinarySemaphore::try_take() {
    bool acquired = false;
    pthread_mutex_lock(&handle_->mutex);
    if (handle_->available) {
        handle_->available = false;
        acquired = true;
    }
    pthread_mutex_unlock(&handle_->mutex);
    return acquired;
}

void BinarySemaphore::give() {
    pthread_mutex_lock(&handle_->mutex);
    handle_->available = true;
    pthread_cond_signal(&handle_->cond);  // wake one waiting thread
    pthread_mutex_unlock(&handle_->mutex);
}

// =======================
// Counting Semaphore Implementation
// =======================

struct CountingSemaphore::CountingSemHandle {
    sem_t sem;
    unsigned maxCount;
};

CountingSemaphore::CountingSemaphore(size_t maxCount, size_t initialCount) {
    handle_ = new CountingSemHandle;
    handle_->maxCount = static_cast<unsigned>(maxCount);

    if (initialCount > maxCount) {
        std::cerr << "[CountingSemaphore] Error: Initial count > max count\n";
        initialCount = maxCount;  // clamp
    }

    if (sem_init(&handle_->sem, 0, static_cast<unsigned>(initialCount)) != 0) {
        std::cerr << "[CountingSemaphore] sem_init failed\n";
    }
}

CountingSemaphore::~CountingSemaphore() {
    sem_destroy(&handle_->sem);
    delete handle_;
}

void CountingSemaphore::take() {
    while (sem_wait(&handle_->sem) != 0) {
        if (errno != EINTR) {
            std::cerr << "[CountingSemaphore] sem_wait failed\n";
            return;
        }
    }
}

bool CountingSemaphore::try_take() {
    return (sem_trywait(&handle_->sem) == 0);
}

void CountingSemaphore::give() {
    int val;
    sem_getvalue(&handle_->sem, &val);

    if (static_cast<unsigned>(val) < handle_->maxCount) {
        sem_post(&handle_->sem);
    } else {
        std::cerr << "[CountingSemaphore] give() called when full\n";
    }
}
}  // namespace Rtos
