#pragma once

#include <string>

enum class RPODState {
    IDLE,
    FAR_APPROACH,
    CLOSE_APPROACH,
    SOFT_DOCKING,
    HARD_DOCKING,
    DOCKED
};

class StateMachine {
public:
    StateMachine();

    // Called every cycle to update state based on events
    void update(bool farPoseDetected,
                bool closePoseDetected,
                bool softDockingDetected,
                bool hardDockingConfirmed);

    RPODState getState() const;
    const char* getStateName() const;

private:
    RPODState state;
};
