#pragma once

namespace rpod::system::rcu {

// One-time init after CubeMX peripheral init
void init();

// For superloop mode (optional)
void loop();

// For RTOS mode: create tasks, start scheduler (typically never returns)
[[noreturn]] void start();

} // namespace rpod::system::rcu