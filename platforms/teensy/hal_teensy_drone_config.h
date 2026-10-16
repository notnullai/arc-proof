#pragma once
/* Pin and peripheral configuration for the Arc drone ESC PWM profile
   (Teensy 4.0). All values are wiring-adjustable here because the frame
   arrives later; on-arrival checks are listed in docs/DRONE_TO_REAL_PLAN.md
   Step 4a (first: confirm the ESCs accept 400 Hz PWM — DShot-only ESCs
   supersede this profile with Step 4b and an owner re-scope). */
#include <stdint.h>

namespace hal_teensy_drone {

/* --- ESC PWM: FlexPWM1 submodules 0-3, channel A of each.
   Pin -> (submodule, channel, mux ALT) from PJRC Teensy4 pwm.c
   pwm_pin_info[] (proven on silicon); mux pad index from the NXP
   MIMXRT1062 header (iomuxc_mux(idx) = IOMUXC_BASE + 0x14 + idx*4). --- */
enum {
    kEsc1PwmPin = 37, kEsc1MuxIdx = 106, kEsc1MuxVal = 1, /* SM0_A (SD_B0_00) */
    kEsc2PwmPin = 35, kEsc2MuxIdx = 108, kEsc2MuxVal = 1, /* SM1_A (SD_B0_02) */
    kEsc3PwmPin = 39, kEsc3MuxIdx = 110, kEsc3MuxVal = 1, /* SM2_A (SD_B0_04) */
    kEsc4PwmPin = 8,  kEsc4MuxIdx = 90,  kEsc4MuxVal = 6, /* SM3_A (B1_00)   */
};

/* --- 400 Hz one-shot PWM, unidirectional 1000-2000 us pulse.
   Counter clock = IPBus 150 MHz (default after reset) / 2^kPrescaleDivLog2.
   PRSC=/8 -> 18.75 MHz; 18.75 MHz / 400 Hz = 46875 counts, which fits the
   16-bit VAL registers (must be < 65536). */
enum {
    kPwmFreqHz = 400,
    kPwmIpBusHz = 150000000,
    kPrescaleLog2 = 3,
    kPrescaleDiv = 8,
    kPulseMinUs = 1000,   /* zero thrust */
    kPulseMaxUs = 2000,   /* full thrust */
};

/* --- E-stop input (active-low, same wiring convention as the base).
   Pin 10 = pad GPIO_B0_00 = GPIO7 bit 0 (PJRC core_pins.h: PIN10_BIT 0,
   PIN10_PORTREG GPIO7_DR; B0 pads are on the fast GPIO7 bank). --- */
enum { kEstopGpioBit = 0, kEstopMuxIdx = 74, kEstopMuxVal = 5 };

/* --- UART (LPUART1) for the bench console (used by the on-target tests). */
enum { kUartTxPin = 1, kUartRxPin = 0, kUartBaud = 115200 };

/* --- Watchdog (WDOG1, LPO clock). --- */
enum { kWdogTimeoutMs = 100 };

/* --- Safety defaults for the drone profile (safety_contract.md drone
   example: fault ramps to hover throttle, not zero). --- */
enum {
    kMaxSetpointQ16    = 65536,   /* 1.0 Q16.16 -> 2000 us */
    kMaxRatePerTickQ16 = 10000,
    kTickBudgetUs      = 1000,
    kWatchdogMisses    = 3,
    kSafeSetpointQ16   = 32768,   /* hover -> 1500 us */
};

}  // namespace hal_teensy_drone
