#pragma once

namespace RPOD {

/**
 * @brief CaptureController manages the capture process during docking.
 * 
 * It interfaces with low-level sensors and actuators to detect soft docking
 * and trigger hard docking. The controller updates internal flags each cycle,
 * which can be queried by the higher-level state machine.
 */
class CaptureController {
public:
    CaptureController();

    /**
     * @brief Run one control cycle: poll sensors, update flags, command motor if needed.
     */
    void run();

    /**
     * @brief Returns true if soft docking has been detected 
     */
    bool isSoftDocked() const;

    /**
     * @brief Returns true if hard docking has been completed (motor rotated successfully).
     */
    bool isHardDocked() const;

private:
    // Internal state flags set by run()
    bool softDockingDetected;
    bool hardDockingConfirmed;

    /**
     * @brief Polls sensor or GPIO input to check if soft docking is achieved.
     * 
     * Placeholder — actual implementation depends on hardware.
     */
    bool detectSoftDocking();

    /**
     * @brief Commands the motor to perform hard docking rotation.
     * 
     * Returns true if the operation was successful.
     * Placeholder — actual implementation depends on hardware.
     */
    bool performHardDocking();

    /**
     * @brief Confirms if hard docking was successful.
     * 
     * This could check for a specific sensor state or motor position.
     * Placeholder — actual implementation depends on hardware.
     */
    bool confirmHardDocking();
};

} // namespace RPOD
