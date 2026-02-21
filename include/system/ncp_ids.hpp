// system/ncp_ids.hpp
//
// Navika Communication Protocol (NCP) system-level identifiers.
//
// This header defines the system architecture contract for inter-compute and
// inter-subsystem messaging:
//   - Node IDs: who is talking
//   - Link IDs: which physical/logical link carries traffic (UART/CAN/RF gateway)
//   - Port IDs: which service/API is being addressed (demultiplexing key)
//
// Design intent:
//   1) NCP remains mission-agnostic: it routes packets by (DST_NODE, DST_PORT)
//      and validates integrity (CRC), but does NOT know what payload bytes mean.
//   2) Payload meaning is owned by RPOD-Software under `include/wire/`
//      (encode/decode functions + fixed wire formats).
//   3) Port IDs are system-stable. Changing a payload wire format is an API change.
//      Prefer one of:
//        - Keep payload backwards-compatible on the same port, OR
//        - Allocate a new port for the new wire format version.
//   4) Node IDs and Port IDs must be unique across the network. Directionality
//      (who sends vs who registers the handler) is defined by the system design.
//
// Operational note:
//   - The NCP RX callback (`NcpEndpoint::ON_PACKET`) receives a non-owning view
//     (`ByteBufConst`) into NCP's scratch buffer. Handlers must decode/copy the
//     payload during the callback; do not store the pointer for later use.

#pragma once
#include <cstdint>

namespace sys::ncp {

// Node IDs
static constexpr uint8_t NODE_RCU  = 0x01;
static constexpr uint8_t NODE_RSOM = 0x02;

// Link IDs (as used in NcpLinkInfo.LINK_ID)
static constexpr uint8_t LINK_UART_RCU_RSOM = 1;  // your main UART interconnect

// Ports (service IDs)
static constexpr uint8_t PORT_POSE_EST  = 10; // RSOM -> RCU
static constexpr uint8_t PORT_RNAV_STATE = 11; // RCU -> RSOM

// Optional future
static constexpr uint8_t PORT_SYNC = 1;
static constexpr uint8_t PORT_CMD  = 2;

} // namespace sys::ncp