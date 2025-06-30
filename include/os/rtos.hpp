#pragma once
#include <cstddef> // Required for size_t

namespace Rtos {

constexpr int MAX_TIMEOUT = -1; // Infinite block

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

    bool take(int timeout_ms = -1);            // Blocks until available
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

    bool take(int timeout_ms = -1);      // block until count>0, then --count
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
    Queue(bool overwrite = false) : head(0), tail(0), overwrite_(overwrite) {}

    bool send(const T& item, int timeout_ms = -1) {
        bool isFull = false;

        if(!overwrite_){
            if(!spaceAvailable.take(timeout_ms)){
                return false;
            };  // Wait for space
        }
        else{
            if(!spaceAvailable.try_take()){
                isFull = true; // Queue is full, will overwrite
                lock.lock();
                tail = (tail + 1) % Capacity; // Overwrite oldest item
                lock.unlock();
            }
        }
        
        lock.lock();
        buffer[head] = item;
        head = (head + 1) % Capacity;
        wasOverwritten = isFull; // Track if last item was overwritten
        lock.unlock();
        if(!isFull) {
            dataAvailable.give(); // Signal data is available
        }
        return true;
    }

    bool try_send(const T& item) {
        bool isFull = false;

        if(!overwrite_){
            if (!spaceAvailable.try_take()) return false;
        }
        else{
            if(!spaceAvailable.try_take()){
                isFull = true; // Queue is full, will overwrite
                lock.lock();
                tail = (tail + 1) % Capacity; // Overwrite oldest item
                lock.unlock();
            }
        }

        lock.lock();
        buffer[head] = item;
        head = (head + 1) % Capacity;
        wasOverwritten = isFull; // Track if last item was overwritten
        lock.unlock();

        if(!isFull) {
            dataAvailable.give(); // Signal data is available
        }
        return true;
    }

    bool receive(T& item, int timeout_ms = -1) {
        if(dataAvailable.take(timeout_ms)){  // Wait for data
            lock.lock();
            item = buffer[tail];
            tail = (tail + 1) % Capacity;
            lock.unlock();
            spaceAvailable.give(); // Signal space is available
            return true;
        }
        else return false;
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


    // Implementation for fidning out 
    // if last send was overwritten
    // may or may not be needed (can remove if not)

    bool wasLastSendOverwritten() {
        lock.lock();
        bool flag = wasOverwritten;
        lock.unlock();
        return flag;
    }

private:
    T buffer[Capacity];
    size_t head, tail;
    bool overwrite_;  // Whether to overwrite oldest item when full
    bool wasOverwritten = false; // Track if last item was overwritten
    Mutex lock;
    CountingSemaphore spaceAvailable{Capacity, Capacity};  // Initially full space
    CountingSemaphore dataAvailable{Capacity, 0};          // Initially no data
};
} // namespace Rtos