#include "apps/core/CommandDataHandler.hpp"
#include "apps/queues.hpp"  // for commandQueue
#include <iostream>

CommandDataHandler::CommandDataHandler() {
    // Constructor logic (if needed)
}

// Handle one command
void CommandDataHandler::handleCommand(const CommandPacket& cmd) {
    std::cout << "[CDH] CMD_RECIVED: Command ID = "
              << static_cast<int>(cmd.commandId) << "\n";
    // Process the command based on its ID
    Rtos::SleepMs(500);  // Simulate processing delay
    std::cout << "[CDH] CMD_PROCESSED: Command ID = "
              << static_cast<int>(cmd.commandId) << "\n";
    // Placeholder: Add command-specific logic here
    // e.g., switch(cmd.command_id) { case SET_MODE: ... }
}

// ==============================================
// =========== Command and Data Handler Tasks ===========
// ==============================================
// Static run function to be passed to RTOS task creation

void CommandDataHandler::CommandHandlerRun(void* arg) {
    CommandDataHandler handler;

    while (true) {
        CommandPacket cmd;

        // Block until a command is available from the queue
        commandQueue.receive(cmd);

        // Process the command
        handler.handleCommand(cmd);
    }
}

void CommandDataHandler::TelemetryHandlerRun(void* arg) {
    while (true) {
        // Placeholder for telemetry handling logic
        // This could involve sending telemetry data to a server or logging it
        std::cout << "[CDH] TELEMETRY: Sending telemetry data...\n";
        
        Rtos::SleepMs(1000);  // 1Hz telemetry rate
    }
}

// ===============================================