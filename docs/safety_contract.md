# Safety Contract

## Overview

The Arc safety contract has **two disjoint mechanisms** protecting against **disjoint failure classes**. They must be documented, wired, and tested separately. Anything else is a false claim.

---

## 1. Software-Detected Faults (Loop Alive)

The loop is running; software can act.

### Fault Classes

| Class | Detection | Response |
|-------|-----------|----------|
| HAL E-stop asserted | `hal->estop_asserted()` | Software ramp |
| Runtime error | Any `arc_*` returns non-OK | Software ramp |
| Rate limit violation | `arc_safety_post_tick` clamp | Clamp + software ramp |
| Tick overrun | `arc_safety_heartbeat` > `tick_budget_us` | Watchdog counter → software ramp |
| Explicit `arc_safety_estop(s, true)` | API call | Software ramp |

### Software Ramp

Each DoF interpolates from current setpoint → `safe_setpoint_q16[g]` over `safe_ramp_ticks[g]` ticks:

```
if ramp_remaining > 0:
    step = (target - current) / ramp_remaining
    current += step
    ramp_remaining--
else:
    current = target
```

- `safe_ramp_ticks[g] = 0` → instant (brake)
- `safe_ramp_ticks[g] > 0` → linear interpolation over N ticks
- Loop continues running, writing ramped values

### Configuration

```c
typedef struct {
    int32_t  max_setpoint_q16;           // Hard clamp ±
    int32_t  max_rate_per_tick_q16;      // Rate limit per tick
    int32_t* safe_setpoint_q16;          // [n_dofs] Target for each DoF
    int32_t* safe_ramp_ticks;            // [n_dofs] 0 = instant, N = ticks
    uint64_t tick_budget_us;             // Max allowed tick time
    int      watchdog_misses;            // Consecutive overruns before fault
    int32_t* safe_setpoints_buf;         // Pre-allocated buffer (n_dofs)
} arc_safety_config;
```

**Per-DoF examples:**
- Wheeled base: all `safe_setpoint_q16 = 0` (stop)
- Drone: `safe_setpoint_q16 = hover_throttle` (hover)
- Arm with brakes: `safe_setpoint_q16 = hold_position` (hold)

---

## 2. Hardware-Detected Faults (Loop Dead)

The loop is **not** running; software **cannot** act.

### Fault Classes

| Class | Detection | Response |
|-------|-----------|----------|
| Loop hang (memory fault, interrupt storm, corrupted variable) | HW WDT timeout | MCU reset |
| Clock stop | HW WDT timeout | MCU reset |
| Power glitch | HW WDT / brownout | MCU reset |
| Hardware E-stop | Direct line to motor driver | Motor driver fallback |

### Response

- **HW WDT fires** → MCU resets (or halts)
- **Motor driver fallback** engages — brake, coast, or disable (board property)
- **Software ramp does NOT execute and CANNOT execute**

### Critical Distinction

| | Software Fault | Hardware Fault |
|---|---|---|
| Loop state | Alive | Dead |
| Software action | Yes (ramp) | No (impossible) |
| Response time | ~1-10 ticks | ~WDT timeout (ms) |
| Guarantee | Graceful degradation | Last line of defense |

**The software ramp is graceful degradation for controlled faults. The hardware fallback is the last line of defense and is a property of the board, not the runtime.**

---

## 3. Safety Layer API

```c
arc_status arc_safety_create(const arc_safety_config* cfg, void* arena, size_t arena_size, arc_safety** out);
arc_status arc_safety_pre_tick(arc_safety* s, const arc_engine* e);
arc_status arc_safety_post_tick(arc_safety* s, int32_t* setpoints_q16, int n_dofs);
arc_status arc_safety_heartbeat(arc_safety* s, uint64_t now_us);
arc_status arc_safety_kick(arc_safety* s);
void       arc_safety_estop(arc_safety* s, bool asserted);
void       arc_safety_clear(arc_safety* s);
bool       arc_safety_ok(const arc_safety* s);
```

### Guarantees

| Guarantee | Scope |
|-----------|-------|
| Bounded setpoints | `[-max_setpoint_q16, +max_setpoint_q16]` |
| Bounded rate | `max_rate_per_tick_q16` per tick |
| Watchdog | `watchdog_misses` consecutive overruns → fault |
| E-stop observation | `hal->estop_asserted()` checked each tick |
| Deterministic failure response | Same inputs → same safe trajectory |

### Non-Guarantees

| Non-Goal | Reason |
|----------|--------|
| Policy correctness | Outside scope |
| Hardware E-stop replacement | Board-level responsibility |
| HAL write audit | Caller responsibility |

---

## 4. Integration Checklist (Bench-Testable)

### Hardware WDT

- [ ] Assert HW WDT fires when loop deliberately hung (infinite loop in tick)
- [ ] Measure time from WDT fire to motor disable; document it

### Motor Driver Fallback

- [ ] Assert motor driver enters fallback when power cut
- [ ] Verify fallback is brake/coast/disable per board spec

### E-Stop Wiring

- [ ] Assert E-stop line wired to **both** HAL (`estop_asserted`) **and** motor driver directly

### Software Ramp

- [ ] Assert ramp executes correctly for each software fault class:
  - [ ] HAL E-stop asserted
  - [ ] Runtime error (non-OK status)
  - [ ] Rate limit violation
  - [ ] Watchdog trip
  - [ ] Explicit `arc_safety_estop(s, true)`

### Determinism

- [ ] Same fault sequence → same safe trajectory (bit-identical)

---

## 5. Failure Mode Matrix

| Fault | Software Ramp? | HW WDT? | Motor Driver Fallback? |
|-------|----------------|---------|------------------------|
| HAL E-stop | ✓ | | |
| Runtime error | ✓ | | |
| Rate limit | ✓ | | |
| Tick overrun (1x) | | | |
| Tick overrun (N×) | ✓ | | |
| Loop hang | | ✓ | ✓ |
| Clock stop | | ✓ | ✓ |
| Power glitch | | ✓ | ✓ |
| HW E-stop | | | ✓ |

---

## 6. Configuration Guidelines

### `tick_budget_us`

- Measure worst-case `arc_step` on target (100k iterations)
- Add 20-50% margin
- Typical: 500-2000 µs for 64 hidden neurons @ 100 MHz

### `watchdog_misses`

- 1 = zero tolerance (any overrun trips)
- 3 = tolerates 2 sporadic overruns
- 5 = noisy environments
- Typical: 3

### `max_rate_per_tick_q16`

- Mechanical limit: actuator slew rate / tick period
- Electrical limit: current limit / tick period
- Set to 80% of hard limit

### `safe_ramp_ticks`

- 0 = instant (electronic brake)
- 10-100 = mechanical ramp (prevents jerk)
- Match to mechanical time constants

---

## 7. Test Coverage

| Test | File | Covers |
|------|------|--------|
| Clamp | test_safety.cpp | `max_setpoint_q16` |
| Rate limit | test_safety.cpp | `max_rate_per_tick_q16` |
| Watchdog | test_safety.cpp | `tick_budget_us`, `watchdog_misses` |
| E-stop | test_safety.cpp | `estop_asserted` |
| Determinism | test_safety.cpp | Same inputs → same outputs |
| Loop estop ramp | test_loop_fault.cpp | Fault mid-run → ramp from last commanded |
| Watchdog overrun in `arc_run` | test_fault_injection.cpp | Consecutive overruns → fault → ramp → safe hold |
| Plasticity gate set by loop | test_fault_injection.cpp | `fault_gated` set on fault, cleared on recovery |
| Estop release / clear | test_fault_injection.cpp | Latched until `arc_safety_clear`; normal resumes |
| Sensor dropout | test_fault_injection.cpp | Stale frame: loop completes, setpoints clamped |
| Single late tick | test_fault_injection.cpp | One miss tolerated, counter decays |
| Tilt-fault ramp + snapshot | test_tilt_fault_response.cpp | Roll > limit → hover ramp, snapshot-before-fault |
| Motor-loss survival (bench profile) | test_motor_loss_survival.cpp | No fault at motor kill; bands hold with real rate limit |
| Loader robustness | test_arc_load_fuzz.cpp | Truncations, magic, header bombs, bitflips: no crash |
| Trained prior determinism | test_brain_prior_load.cpp | Published priors load + deterministic over reset |
| HW WDT | On-target | Board bring-up |
| Motor fallback | On-target | Board bring-up |

Hardware-path tests (WDT, motor driver fallback) are on-target integration tests, run per board revision.