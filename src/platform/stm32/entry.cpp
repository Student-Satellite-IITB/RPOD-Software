#include "platform/stm32/entry.h"
#include "system/rcu/rcu.hpp"

// C ABI glue functions to call into the C++ RCU implementation

extern "C" void rcu_init(void)
{
    rpod::system::rcu::init();
}

extern "C" void rcu_loop(void)
{
    rpod::system::rcu::loop();
}

extern "C" void rcu_start(void)
{
    rpod::system::rcu::start();
}