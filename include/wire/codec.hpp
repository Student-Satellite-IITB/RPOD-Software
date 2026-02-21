#pragma once
#include <cstdint>

// Wire encoding is little-endian for all multi-byte integers.
// Floats are IEEE-754 32-bit and copied as raw bytes.

namespace wire {

inline void put_u64(uint8_t*& p, uint64_t v){
  for(int i=0;i<8;i++) *p++ = (uint8_t)(v >> (8*i));
}

inline uint64_t get_u64(const uint8_t*& p){
  uint64_t v = 0;
  for(int i=0;i<8;i++) v |= (uint64_t(p[i]) << (8*i));
  p += 8;
  return v;
}

} // namespace wire