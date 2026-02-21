// wire/pose_estimate.hpp
//
// Wire payload format: PoseEstimate (RSOM -> RCU)
//
// This file defines the byte-level contract for PoseEstimate as carried inside
// an NCP packet payload (ByteBufConst). NCP provides addressing (node/port) and
// integrity (CRC). This layer defines what the payload bytes *mean*.
//
// Encoding assumptions:
//   - All multi-byte integers are little-endian on the wire.
//   - float is IEEE-754 32-bit and is copied as 4 raw bytes.
//   - Quaternion order is (w, x, y, z).
//   - Translation is in meters.
//
// Fixed payload size: 40 bytes (POSE_EST_BYTES).
//
// Byte layout (offsets in bytes):
//   0..7    : uint64_t t_exp_end_us      // exposure-end timestamp [microseconds]
//   8..19   : float    t_CbyP[3]         // translation (x,y,z) [meters]
//   20..35  : float    q_C_P[4]          // quaternion (w,x,y,z) [-]
//   36..39  : float    reproj_rms_px     // reprojection RMS [pixels]
//
// Lifetime note (RX):
//   - NcpDecoded.payload is a non-owning view into NCP’s scratch buffer.
//     Decode/copy the payload during the port handler callback; do not retain pointers.

#pragma once
#include <cstddef>
#include <cstdint>

#include "msg/PoseEstimate.hpp"

namespace wire {

// Fixed wire size (bytes) for PoseEstimate payload
static constexpr size_t POSE_EST_BYTES = 40;

// Encode msg::PoseEstimate -> payload bytes for NCP
// Returns number of bytes written (POSE_EST_BYTES) or 0 on failure.
size_t encode_pose_estimate(const msg::PoseEstimate& in, uint8_t* out, size_t cap);

// Decode payload bytes -> msg::PoseEstimate
// Returns true on success.
bool decode_pose_estimate(const uint8_t* data, size_t len, msg::PoseEstimate& out);

} // namespace wire