/* ESC pulse-width math for the Arc drone HAL (Plan Step 4a): Q16.16
   thrust setpoint -> microsecond pulse -> FlexPWM VAL3 counts. Extracted
   from platforms/teensy/hal_teensy_drone.cpp so the mapping is host-tested
   (tests/test_esc_pulse.cpp) without hardware. Integer-only, no
   allocation, no I/O. */
#pragma once
#include <stdint.h>
#include "hal_teensy_drone_config.h"

namespace esc_pulse {

/* Q16.16 setpoint -> [kPulseMinUs, kPulseMaxUs] pulse width in
   microseconds. Unidirectional thrust: inputs clamp to [0, 65536] (zero to
   full-scale, saturating outside). */
static inline uint32_t setpoint_to_pulse_us(int32_t sp_q16) {
    const uint32_t c = (sp_q16 <= 0)    ? 0u
                     : ((sp_q16 >= 65536) ? 65536u
                                          : (uint32_t)sp_q16);
    return hal_teensy_drone::kPulseMinUs +
           (uint32_t)(((uint64_t)c *
                       (uint64_t)(hal_teensy_drone::kPulseMaxUs -
                                  hal_teensy_drone::kPulseMinUs)) /
                      65536u);
}

/* Microseconds of pulse -> FlexPWM VAL3 counts: counts = us *
   (ipbus_hz / prescale_div) / 1e6. At 150 MHz / 8 (kPwmIpBusHz /
   kPrescaleDiv): 1000 us -> 18750, 1500 -> 28125, 2000 -> 37500 (exact
   integer counts). */
static inline uint32_t pulse_us_to_counts(uint32_t us) {
    const uint64_t f_clk =
        (uint64_t)hal_teensy_drone::kPwmIpBusHz /
        (uint64_t)hal_teensy_drone::kPrescaleDiv;
    return (uint32_t)(((uint64_t)us * f_clk) / 1000000ull);
}

} /* namespace esc_pulse */
