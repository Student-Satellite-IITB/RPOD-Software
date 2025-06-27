#pragma once
#include "types.hpp"


class CommandDataHandler {
public:
    CommandDataHandler();  
    // Add functions
    static void CommandHandlerRun(void* arg); // Static function to run the command handler task
    static void TelemetryHandlerRun(void* arg); // Static function to run the telemetry handler task

private:
    // Add private members if needed
    // For example, a queue to handle incoming commands
    // Helper method to handle command packets
    void handleCommand(const CommandPacket& cmd);

    // Can add shared telemetry state / buffers here later
};
