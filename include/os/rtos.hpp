#pragma once
#include <cstddef> // Required for size_t

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

//== Binary Semaphore abstraction ==//
class BinarySemaphore {
public:
    BinarySemaphore();
    ~BinarySemaphore();

    void take();            // Blocks until available
    bool try_take();        // Non-blocking
    void give();            // Releases the semaphore

private:
    struct SemaphoreHandle;
    SemaphoreHandle* handle_;
};


//== Counting Semaphore abstraction ==//
class CountingSemaphore {
public:
    /**
     * @param maxCount    Maximum count (e.g. queue capacity)
     * @param initialCount  Starting count
     */
    CountingSemaphore(size_t maxCount, size_t initialCount);
    ~CountingSemaphore();

    void take();      // block until count>0, then --count
    bool try_take();  // non‐blocking: if count>0 then --count, else false
    void give();      // ++count, wake one waiter if present

private:
    struct CountingSemHandle;
    CountingSemHandle* handle_;
};

//== Queue abstraction ==//
// Fixed-size statically allocated queue
//
// This templated Queue<T, N> is implemented using a circular buffer,
// and synchronized using the OSAL Mutex and BinarySemaphore primitives.
// It is designed to be portable across platforms — works on Linux (POSIX) 
// and can be reused with minimal change on embedded RTOS environments.
//
// POSIX does not provide native queue support, so this implementation is
// built manually using mutexes and semaphores. On FreeRTOS, however, native
// queue APIs (xQueueCreate, xQueueSend, xQueueReceive) are available.
//
// For development simplicity and portability, this template is currently
// implemented entirely in the header. If performance becomes critical,
// this implementation can be optionally replaced or specialized with native
// RTOS APIs in a platform-specific file (e.g., rtos_freertos.cpp).
//
// Future Improvement:
// - Specialize Queue<T, N> for FreeRTOS to use native xQueue APIs
//   if tighter memory or timing constraints demand it.
// - Add timeout-based send/receive methods if required.
//
//
template <typename T, size_t Capacity>
class Queue {
public:
    Queue() : head(0), tail(0) {}

    void send(const T& item) {
        spaceAvailable.take();  // Wait for space
        lock.lock();
        buffer[head] = item;
        head = (head + 1) % Capacity;
        lock.unlock();
        dataAvailable.give();   // Signal data is available
    }

    bool try_send(const T& item) {
        if (!spaceAvailable.try_take()) return false;
        lock.lock();
        buffer[head] = item;
        head = (head + 1) % Capacity;
        lock.unlock();
        dataAvailable.give();
        return true;
    }

    void receive(T& item) {
        dataAvailable.take();  // Wait for data
        lock.lock();
        item = buffer[tail];
        tail = (tail + 1) % Capacity;
        lock.unlock();
        spaceAvailable.give(); // Signal space is available
    }

    bool try_receive(T& item) {
        if (!dataAvailable.try_take()) return false;
        lock.lock();
        item = buffer[tail];
        tail = (tail + 1) % Capacity;
        lock.unlock();
        spaceAvailable.give();
        return true;
    }

private:
    T buffer[Capacity];
    size_t head, tail;

    Mutex lock;
    CountingSemaphore spaceAvailable{Capacity, Capacity};  // Initially full space
    CountingSemaphore dataAvailable{Capacity, 0};          // Initially no data
};
} // namespace Rtos