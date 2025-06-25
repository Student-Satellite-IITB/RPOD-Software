#include "apps/core/StateMachine.hpp"
#include <cstdio>  // For printf (optional)

// Constructor
StateMachine::StateMachine()
    : state(RPODState::IDLE) {}

// State transition logic
void StateMachine::update(bool farPoseDetected,
                          bool closePoseDetected,
                          bool softDockingDetected,
                          bool hardDockingConfirmed) {
    switch (state) {
        case RPODState::IDLE:
            if (farPoseDetected) {
                state = RPODState::FAR_APPROACH;
                printf("[StateMachine] Transition: IDLE → FAR_APPROACH\n");
            }
            break;

        case RPODState::FAR_APPROACH:
            if (closePoseDetected) {
                state = RPODState::CLOSE_APPROACH;
                printf("[StateMachine] Transition: FAR_APPROACH → CLOSE_APPROACH\n");
            }
            break;

        case RPODState::CLOSE_APPROACH:
            if (softDockingDetected) {
                state = RPODState::SOFT_DOCKING;
                printf("[StateMachine] Transition: CLOSE_APPROACH → SOFT_DOCKING\n");
            }
            break;

        case RPODState::SOFT_DOCKING:
            if (hardDockingConfirmed) {
                state = RPODState::HARD_DOCKING;
                printf("[StateMachine] Transition: SOFT_DOCKING → HARD_DOCKING\n");
            }
            break;

        case RPODState::HARD_DOCKING:
            // Automatically transition to final state after hard docking
            state = RPODState::DOCKED;
            printf("[StateMachine] Transition: HARD_DOCKING → DOCKED\n");
            break;

        case RPODState::DOCKED:
            // No further transitions
            // Note: In a real system, you might reset or handle errors here
            // as it will get stuck here indefinitely in case of no reset logic.
            break;
    }
}

// Get the current state
RPODState StateMachine::getState() const {
    return state;
}

// Return a human-readable state name for logging/Debugging
const char* StateMachine::getStateName() const {
    switch (state) {
        case RPODState::IDLE: return "IDLE";
        case RPODState::FAR_APPROACH: return "FAR_APPROACH";
        case RPODState::CLOSE_APPROACH: return "CLOSE_APPROACH";
        case RPODState::SOFT_DOCKING: return "SOFT_DOCKING";
        case RPODState::HARD_DOCKING: return "HARD_DOCKING";
        case RPODState::DOCKED: return "DOCKED";
        default: return "UNKNOWN";
    }
}
