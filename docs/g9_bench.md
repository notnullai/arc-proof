# G9 board bench: one Teensy 4.0 run

Gap G9 (`docs/gap_register.md`) is the last timing gap. The tick's cycle cost on a Cortex-M7 is an analytic bound (`tools/mcu_cycle_bound.py`, `docs/performance.md`): the exact instruction mix from a QEMU trace, with every instruction charged its worst case on a *stated* M7 envelope, because Arm publishes no M7 cycle table. This kit measures the same ticks with DWT_CYCCNT on your own Teensy and says whether the bound holds. Nothing is built: you flash the shipped image and run one script.

## What to have

- A **Teensy 4.0** (i.MX RT1062, Cortex-M7 600 MHz) and its **USB micro-B cable** (for power and flashing).
- A **3.3 V USB-serial adapter** (FTDI TTL-232R-3V3, CP2102 or similar) and 2 jumper wires. The bench prints on LPUART6 (Teensy pin 1), not on the Teensy's USB.
  Wiring: adapter **RX → Teensy pin 1**, adapter **GND → Teensy GND**. Leave the adapter's VCC unconnected.
- A loader: **teensy_loader_cli** or the **Teensy Loader** from PJRC's Teensyduino.
- **Python 3**. **pyserial** (`pip install pyserial`) is optional; without it `--port` reads the OS serial device directly. You can also capture the port with any terminal and pass the file with `--log`.

## Check and flash the shipped image

```bash
sha256sum -c SHA256SUMS
```
```bash
teensy_loader_cli --mcu=TEENSY40 -w -v targets/g9_bench_teensy40.hex
```

Press the Teensy's program button when the loader waits. With the GUI loader: File → Open HEX File → `targets/g9_bench_teensy40.hex`, then press the button. The board reboots into the bench. The run takes about a second; then the report goes out (about 5 s at 115200 baud) and repeats, so connecting late is fine.

## The one run command

```bash
python tools/g9_bench.py --port COM5 --out g9_bench_my_teensy40.json
```

Use your adapter's port (`COM5`, `/dev/ttyUSB0`). The script also checks the image you flashed against its linker map, `targets/g9_bench_teensy40.map`, and its boot header in `targets/g9_bench_teensy40.hex`.

To see the whole pipeline before the board arrives, run the QEMU stand-in (`qemu-system-arm` needed). It reports instructions, not cycles, so it never closes G9:

```bash
python tools/g9_bench.py --qemu targets/g9_bench_mps2_m7
```

## What the bench runs

Six tick sequences of the QEMU timing images in `targets/`, from the same sources. Each is 2000 ticks and gives 1,999 timed tick intervals.

| workload | timing image | G9 bound row (M7 envelope) |
|---|---|---|
| `flight_dense`: flight default 832 / 96, inference, dense stress | `mcu_drone_loop_timing_x500_832_inference_dense` | **135,284** cycles |
| `learn_sliced_dense`: learning demo 480 / 96, plasticity ÷10, dense | `mcu_drone_loop_timing_x500_480_sliced_dense` | **195,408** cycles |
| `learn_sliced_worst`: learning demo, plasticity ÷10, **worst case**: every synapse of the slice flips, every branch the expensive way (G8) | `mcu_drone_loop_timing_x500_480_sliced_worst_walk` (the ceiling-walk arming, the worst case of the fast pass) | **211,745** cycles |
| `learn_dense`: learning demo, not amortised | `mcu_drone_loop_timing_x500_480_dense` | none (CPI only) |
| `flight_plain`, `golden_twin`: one-bit input, golden hash | `…_832_inference`, `mcu_drone_loop_timing` | none (hash anchor) |

How the image runs them:
- Code is in ITCM and every tick's data in DTCM, both zero-wait.
- Interrupts stay masked for the whole run.
- An unaligned access inside a timed tick faults instead of slowing silently (CCR.UNALIGN_TRP).
- Each tick reads DWT_CYCCNT at the same points where the timing images read SysTick.
- The worst-case twin arms its state between ticks, outside the timed window, and counts every armed call that failed to flip all 920 synapses of its slice. The script rejects the run if any call missed.

## Hypotheses, stated before the run

- **H1: the M7 envelope bounds the tick.** Prediction: worst tick ≤ 135,284 cycles (flight), ≤ 195,408 (learning, dense) and ≤ 211,745 (learning, worst case). One tick over its row refutes the envelope.
- **H2: one cycle per instruction**, the figure behind G8's clock floors. Prediction: worst cycles ≤ the bench's exact instruction count (73,612 / 97,432 / 105,945), i.e. CPI ≤ 1.00.
- **Start value, not tested:** on this integer, TCM-resident code we expect CPI between 0.6 and 0.9, from M7 dual issue and single-cycle TCM. That would put the flight worst tick near 44k–66k cycles and the worst-case learning tick near 64k–95k, leaving the envelope 2–3× loose. Your run can prove this wrong in either direction.

## Reading the verdict

The script prints, per workload:
- the hash check;
- p50 / p90 / worst cycles;
- CPI (worst, mean, and per tick against the stand-in's instruction counts in `docs/g9_bench_standin.json`).

Then one `ROW … PASS/FAIL` line per bound row, both hypotheses as SUPPORTED or REFUTED, and a last line that is one of:

- **`G9 CLOSES`**: the run is valid and all three worst ticks are inside their rows. If the line adds "CPI > 1", H2 is refuted: G9 still closes, but G8's "1 cycle/instr" clock floors must be restated with the measured CPI.
- **`G9 REOPENS`**: a worst tick exceeds its row, so the M7 envelope is wrong.
- **`INVALID RUN`**: nothing is concluded. Fix what it lists and run again. Causes:
  - a hash that differs from the committed one. That is a bit-exactness failure on silicon, a finding bigger than G9; please report it.
  - code or data outside ITCM / DTCM, in the map or in the firmware's own read-back.
  - a flashed image that is not the shipped map.
  - interrupts not masked.
  - the worst-case twin missed an armed call.
  - a `G9 FAULT` line.
  - a missing or corrupted report.

The JSON written by `--out` is the record of your run: every tick's cycle count and every check.

If nothing arrives:
- **No `G9 BOOT` line:** check the wiring, the port and 115200 baud.
- **`G9 BOOT` but no `G9 RUN`:** the 600 MHz clock switch hung. Report it; cycle counts in TCM do not depend on the clock.

## Checked before shipping, and what only a board can check

- **Hashes:** all six match the host twin, the stand-in, and the QEMU timing images in `targets/` (M3 and M7, `docs/performance.md`). The worst-case twin armed 1,499 calls, missed none, and flipped 920 synapses in each.
- **Same tick as the bound:** the stand-in's own cycle bounds (`tools/mcu_cycle_bound.py`) are 135,186, 195,306 and 211,745 cycles, against the rows' 135,284, 195,408 and 211,745 (re-run 2026-10-25 after the G8 fast pass).
- **No unaligned access:** none happened in any timed tick, with the trap armed.
- **Image layout:** 33.0 KB of code in one ITCM bank and 413 KB of data in 15 DTCM banks. Flash holds only the boot header, the reset code and the vector table (940 bytes).
- **Boot header:** "FCFB" config block, IVT entry at the reset handler, boot data covering the image.
- **Not checked until a board runs it:** that the boot ROM accepts this header (it follows PJRC's Teensy 4.0 layout), and the LPUART6, FlexRAM and clock register values on silicon.
