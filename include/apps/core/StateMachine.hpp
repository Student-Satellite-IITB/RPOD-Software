#pragma once
#include "types.hpp"
#include <string>


class StateMachine {
public:
    StateMachine();

    // Called every cycle to update state based on events
    void update(RPODEvent event);
    RPODState getState() const;
    const char* getStateName() const;
    static void Run(void* arg);

private:
    RPODState state;
};
