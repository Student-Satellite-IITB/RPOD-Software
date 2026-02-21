#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Called once after HAL_Init(), SystemClock_Config(), MX_*_Init()
void rcu_init(void);

// If using a superloop build (no scheduler), call this inside while(1)
void rcu_loop(void);

// If using RTOS, call this once; it should start scheduler and never return.
void rcu_start(void);

#ifdef __cplusplus
}
#endif