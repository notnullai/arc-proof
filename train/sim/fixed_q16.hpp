#pragma once
/* Shared Q16.16 fixed-point helpers for train-side plants (joint, drone,
   car). Header-only, no dependencies. */
#include <cstdint>

namespace arc {
namespace sim {

inline int32_t q16_mul(int32_t a, int32_t b) {
    return (int32_t)(((int64_t)a * b) >> 16);
}
inline int32_t q16_abs(int32_t v) { return v < 0 ? -v : v; }

}  // namespace sim
}  // namespace arc
