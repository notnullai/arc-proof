/* Host test for the drone ESC pulse math (Plan Step 4a), extracted into
   platforms/teensy/esc_pulse.h. Verifies the exact count values that were
   previously assumed correct only on silicon: at IPBus 150 MHz with
   prescale /8 (18.75 MHz counter), 1000/1500/2000 us pulses are exactly
   18750/28125/37500 counts. Also covers clamp endpoints and mid-range
   linearity of the Q16.16 -> us mapping. */
#include <stdint.h>
#include <stdio.h>
#include "esc_pulse.h"
#include "hal_teensy_drone_config.h"

using hal_teensy_drone::kPulseMaxUs;
using hal_teensy_drone::kPulseMinUs;
using hal_teensy_drone::kPwmIpBusHz;
using hal_teensy_drone::kPrescaleDiv;

static int failures = 0;

static void check_eq_u32(const char* name, uint32_t got, uint32_t want) {
    if (got != want) {
        printf("FAIL %s: got %u want %u\n", name, got, want);
        ++failures;
    }
}

int main() {
    /* Counts: 1 kHz / 1.5 kHz / 2 kHz at 150 MHz / 8. */
    check_eq_u32("1000us->counts",
                 esc_pulse::pulse_us_to_counts(1000u), 18750u);
    check_eq_u32("1500us->counts",
                 esc_pulse::pulse_us_to_counts(1500u), 28125u);
    check_eq_u32("2000us->counts",
                 esc_pulse::pulse_us_to_counts(2000u), 37500u);

    /* Config sanity: the arithmetic must match the config the HAL uses. */
    check_eq_u32("f_clk", (uint32_t)(kPwmIpBusHz / kPrescaleDiv), 18750000u);
    check_eq_u32("pulse span", kPulseMaxUs - kPulseMinUs, 1000u);

    /* Setpoint clamp endpoints. */
    check_eq_u32("sp<0",
                 esc_pulse::setpoint_to_pulse_us(-1), kPulseMinUs);
    check_eq_u32("sp=0",
                 esc_pulse::setpoint_to_pulse_us(0), kPulseMinUs);
    check_eq_u32("sp=65536",
                 esc_pulse::setpoint_to_pulse_us(65536), kPulseMaxUs);
    check_eq_u32("sp>s full",
                 esc_pulse::setpoint_to_pulse_us(100000), kPulseMaxUs);

    /* Half scale: 32768/65536 * 1000 us = 500 us -> min + 500. */
    check_eq_u32("sp=32768",
                 esc_pulse::setpoint_to_pulse_us(32768),
                 kPulseMinUs + 500u);

    if (failures == 0) {
        printf("PASS test_esc_pulse\n");
        return 0;
    }
    printf("FAIL test_esc_pulse (%d checks)\n", failures);
    return 1;
}
