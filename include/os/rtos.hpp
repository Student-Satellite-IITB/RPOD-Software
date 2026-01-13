#pragma once
#include <cstddef> // Required for size_t
#include <cstdint> // Required for uint64_t

namespace Rtos {

constexpr int MAX_TIMEOUT = -1; // Infinite block

uint64_t NowUs();
void SleepMs(int ms);

// Absolute deadline sleep in same timebase as NowUs()
void SleepUntilUs(uint64_t deadline_us);
// Convenience wrapper
inline void SleepUntilMs(uint64_t deadline_ms) {
    SleepUntilUs(deadline_ms * 1000ULL);
}

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
    Queue(bool overwrite = false) : head(0), tail(0), count_(0), overwrite_(overwrite) {}

    bool send(const T& item, int timeout_ms = -1) {

        if(!overwrite_){
            if(!spaceAvailable.take(timeout_ms)){
                return false;
            };  // Wait for space

            lock.lock();
            buffer[head] = item;
            head = (head + 1) % Capacity;
            count_++;                      // <= NEW: keep count in sync
            wasOverwritten = false;
            lock.unlock();

            dataAvailable.give();
            return true;
        }
        else{
            // overwrite_ == true : do NOT touch spaceAvailable at all
            lock.lock();
            const bool wasFull = (count_ == Capacity);
            if (wasFull) {
                // drop oldest (advance tail), count_ stays == Capacity
                tail = (tail + 1) % Capacity;
                wasOverwritten = true;
            } else {
                // room available
                count_++;
                wasOverwritten = false;
            }

            buffer[head] = item;
            head = (head + 1) % Capacity;

            lock.unlock();

            // Only signal new data if we increased occupancy (i.e., not overwriting)
            if (!wasFull) dataAvailable.give();
            return true;
        }        
    }

    bool try_send(const T& item) {

        if(!overwrite_){
            if (!spaceAvailable.try_take()) return false;

            lock.lock();
            buffer[head] = item;
            head = (head + 1) % Capacity;
            count_++;
            wasOverwritten = false;
            lock.unlock();

            dataAvailable.give();
            return true;
        }
        else{
            // overwrite_ == true : always succeeds
            lock.lock();

            const bool wasFull = (count_ == Capacity);
            if (wasFull) {
                tail = (tail + 1) % Capacity;
                wasOverwritten = true;
            } else {
                count_++;
                wasOverwritten = false;
            }

            buffer[head] = item;
            head = (head + 1) % Capacity;

            lock.unlock();

            if (!wasFull) dataAvailable.give();
            return true;
        }
    }

    bool receive(T& item, int timeout_ms = -1) {
        if(dataAvailable.take(timeout_ms)){  // Wait for data
            lock.lock();
            item = buffer[tail];
            tail = (tail + 1) % Capacity;
            if (count_ > 0) count_--;
            lock.unlock();
            if (!overwrite_) spaceAvailable.give();  // Signal space is available only in non-overwrite
            return true;
        }
        else return false;
    }

    bool try_receive(T& item) {
        if (!dataAvailable.try_take()) return false;
        lock.lock();
        item = buffer[tail];
        tail = (tail + 1) % Capacity;
        if (count_ > 0) count_--;
        lock.unlock();
        if (!overwrite_) spaceAvailable.give(); // Signal space is available only in non-overwrite
        return true;
    }


    // Implementation for finding out 
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
    size_t count_;                 // Occupancy (0..Capacity)
    bool overwrite_;  // Whether to overwrite oldest item when full
    bool wasOverwritten = false; // Track if last item was overwritten
    Mutex lock;
    CountingSemaphore spaceAvailable{Capacity, Capacity};  // Initially full space
    CountingSemaphore dataAvailable{Capacity, 0};          // Initially no data
};
} // namespace Rtos