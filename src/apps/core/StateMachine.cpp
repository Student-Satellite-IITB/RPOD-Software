#include "apps/core/StateMachine.hpp"
#include "os/rtos.hpp"
#include "apps/queues.hpp"
#include <cstdio>  // For printf (optional)

// Constructor
StateMachine::StateMachine()
    : state(RPODState::IDLE) {}

// State transition logic
void StateMachine::update(RPODEvent event) {
    // Implement event-based state transition logic here

}

// Get the current state
RPODState StateMachine::getState() const {
    return state;
}

// Return a human-readable state name for logging/Debugging
const char* StateMachine::getStateName() const {
    switch (state) {
        case RPODState::IDLE: return "IDLE";
        case RPODState::SEARCH:return "SEARCH";
        case RPODState::FAR_APPROACH: return "FAR_APPROACH";
        case RPODState::CLOSE_APPROACH: return "CLOSE_APPROACH";
        case RPODState::SOFT_DOCKING: return "SOFT_DOCKING";
        case RPODState::HARD_DOCKING: return "HARD_DOCKING";
        case RPODState::DOCKED: return "DOCKED";
        default: return "UNKNOWN";
    }
}

void StateMachine::Run(void* arg){
    StateMachine machine;
    RPODEvent event;

    while (true)
    {
        if(eventQueue.receive(event)){
            machine.update(event);
        }
    }
}