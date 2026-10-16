#pragma once
/* Host-side Arc HAL for the drone simulation.
   Wires the DronePlant to the arc_hal vtable. */
#include "arc/arc_hal.h"

namespace arc {
namespace sim {
struct DronePlant;
struct DroneConfig;
}
}

/* Create an arc_hal wired to a DronePlant. The plant must outlive the HAL.
   reward_out is filled with 3 Q16.16 reward components each tick. */
arc_hal arc_hal_drone_sim_create(arc::sim::DronePlant* plant,
                                 int32_t* reward_out = nullptr);
