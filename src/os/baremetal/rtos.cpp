// src/os/baremetal/rtos.cpp
//
// Baremetal OSAL implementation for STM32 bringup (NO RTOS).
//
// Purpose:
// - Provide minimal implementations of the Rtos::* API so embedded modules can
//   compile + link during early bringup (superloop / HAL_Delay-based timing).
//
// Important:
// - This implementation is *STM32 HAL dependent*.
// - It intentionally uses STM32 HAL time primitives (HAL_GetTick / HAL_Delay).
// - It is NOT a real RTOS: Task/Mutex/Semaphore are stubs (no scheduling, no blocking).
//
// Assumptions:
// - CubeMX has initialized HAL (HAL_Init) and SysTick is running.
// - HAL_GetTick() returns milliseconds since boot.
//
// Timebase note:
// - Rtos::NowUs() here returns "microseconds" derived from HAL_GetTick() * 1000,
//   which has only millisecond resolution. Good enough for coarse timeouts,
//   NOT suitable for precise estimator timing.
//
// Upgrade path:
// - When FreeRTOS is enabled on STM32, replace this file in the STM32 build with
//   src/os/freertos/rtos.cpp (real implementations using FreeRTOS primitives).
// - Alternatively, implement a true microsecond timer using a hardware timer
//   (e.g., TIMx) and update NowUs()/SleepUntilUs() accordingly.

#include "os/rtos.hpp"

// STM32 HAL dependency (HAL_GetTick, HAL_Delay). We include the CubeMX umbrella header.
// If you want a cleaner dependency, include "stm32h7xx_hal.h" instead, but keep it explicit.
extern "C" {
#include "main.h"
}

namespace Rtos {

uint64_t NowUs()
{
    // Millisecond tick -> "microseconds" with ms resolution.
    return static_cast<uint64_t>(HAL_GetTick()) * 1000ULL;
}

void SleepMs(int ms)
{
    HAL_Delay(static_cast<uint32_t>(ms));
}

void SleepUntilUs(uint64_t deadline_us)
{
    // Coarse spin-wait; consider HAL_Delay(1) to yield CPU time to interrupts.
    while (NowUs() < deadline_us) {
        // If you want to reduce busy-waiting:
        // HAL_Delay(1);
    }
}

// ---- Task (stub) ----
Task::Task() : handle_(nullptr) {}
Task::~Task() = default;

void Task::Create(const char* /*name*/, void (*/*fn*/)(void*), void* /*arg*/)
{
    // No tasking in baremetal mode.
    // If code calls this, it is a logic/config error: you're expecting RTOS.
}

void Task::Join() {}

// ---- Mutex (stub) ----
Mutex::Mutex() : handle_(nullptr) {}
Mutex::~Mutex() = default;

void Mutex::lock()   {}
void Mutex::unlock() {}

// ---- BinarySemaphore (stub) ----
BinarySemaphore::BinarySemaphore() : handle_(nullptr) {}
BinarySemaphore::~BinarySemaphore() = default;

bool BinarySemaphore::take(int /*timeout_ms*/) { return true; }
bool BinarySemaphore::try_take()               { return true; }
void BinarySemaphore::give()                   {}

// ---- CountingSemaphore (stub) ----
CountingSemaphore::CountingSemaphore(size_t /*maxCount*/, size_t /*initialCount*/) : handle_(nullptr) {}
CountingSemaphore::~CountingSemaphore() = default;

bool CountingSemaphore::take(int /*timeout_ms*/) { return true; }
bool CountingSemaphore::try_take()               { return true; }
void CountingSemaphore::give()                   {}

} // namespace Rtos