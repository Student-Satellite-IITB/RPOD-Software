#pragma once

namespace Rtos {

void SleepMs(int ms);

//== Task abstraction ==//
// This class provides a simple task wrapper
class Task {
public:
    Task();
    ~Task();

    void Create(const char* name, void (*fn)(void*), void* arg);
    void Join();

private:
    struct TaskHandle;
    TaskHandle* handle_;
};

//== Mutex abstraction ==//
// This class provides a simple mutex wrapper

class Mutex {
public:
    Mutex();
    ~Mutex();

    void lock();
    void unlock();

private:
    struct MutexHandle;
    MutexHandle* handle_;
};
} // namespace Rtos