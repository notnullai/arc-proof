# Fly the flight default in Gazebo yourself

This page takes a clean Linux x86_64 machine to the same PX4 SITL numbers we
publish in `docs/px4_sitl_summary_table.md` and `docs/bridge_loss_method.md`:
the Arc flight default flying the Gazebo x500 in hover, in wind, and through
an Arc process loss with handover to an unmodified PX4. Everything it needs
is in this repository plus the public PX4 image; nothing is fetched from us.

## What you run, and what it is

| file | what it is |
|---|---|
| `bin/linux-x86_64/px4_bridge` | the Arc PX4 bridge: release build, symbols stripped, Arc engine and C/C++ runtime statically linked (no shared-library dependencies) |
| `bin/windows-x64/px4_bridge.exe` | the same bridge for Windows x64 (static C runtime, depends on `KERNEL32.dll` only) |
| `models/prior_x500_candidate_v5.arc002` | the flight default: candidate v5, 832 input bits, the station cascade compiled into ternary weights, flown with plasticity off (no learning claim) |
| `tools/px4/*.py` | the harness, readable Python: scenario runner, PX4 estimator streamer, Gazebo motor publisher and disturbance injector (these two run inside the PX4 container), MAVLink parameter helpers, summarizers |
| `tests/fixtures/px4_ekf_ground.bin` | 922 PX4 estimator records from a disarmed gz_x500 on the ground, for the offline determinism check |

Both binaries were built from the private repository at commit `396cc0f`, the
source the published runs used. **No engine or bridge source is published.**
The binaries are stripped, which removes symbol names and debug information;
it does not hide the machine code. **A binary can still be disassembled.**
We say so plainly so nobody mistakes stripping for protection.

Data path per run: PX4 estimator (MAVLink, UDP 14550) → `ekf_stream.py` →
OS pipe → `px4_bridge` (1 kHz tick on sim time) → OS pipe →
`gz_motor_pub.py` inside the container → Gazebo rotor commands. PX4 is the
estimator only; its motor outputs stay unrouted unless a handover happens.

## Requirements

- Linux x86_64 with at least 4 free cores and 8 GB RAM. The tick is locked to
  sim time but the pipes run in real time, so a loaded host shows up as wall
  stalls (summary column `max wall stall ms`; a run with a stall over 1 s is
  not valid, rerun it on an idle machine).
- Docker Engine, Python 3 (verified with 3.12), about 4 GB of disk for the PX4 image (3.1 GB).
- UDP port 14550 free on the host (PX4 sends MAVLink to the host through
  `host.docker.internal`).

## 1. Set up (once)

```sh
sudo apt-get update && sudo apt-get install -y docker.io git python3-venv
sudo usermod -aG docker "$USER"     # then log out and back in

git clone https://github.com/notnullai/arc-proof && cd arc-proof
sha256sum -c SHA256SUMS             # every binary, the model and the fixture: OK

python3 -m venv .venv && . .venv/bin/activate
pip install -r tools/px4/requirements.txt

# the PX4 + Gazebo Harmonic image our runs used, pinned by digest
docker pull px4io/px4-sitl-gazebo@sha256:29a5a1b5af77e5d39b67bc6664d36acead63f9dfb829475216522b1feec8036e
docker tag px4io/px4-sitl-gazebo@sha256:29a5a1b5af77e5d39b67bc6664d36acead63f9dfb829475216522b1feec8036e \
           px4io/px4-sitl-gazebo:latest
```

## 2. Offline check: same bytes in, same commands out (1 second, no Gazebo)

```sh
bin/linux-x86_64/px4_bridge --brain models/prior_x500_candidate_v5.arc002 --bits 832 \
    --plasticity off --hover 769 --every 4 < tests/fixtures/px4_ekf_ground.bin > /dev/null
```

Expected on stderr (only `max_wall_tick_us` varies by machine):

```
PX4_BRIDGE_STATS ok=1 ticks=19000 records=922 rejected=0 gaps=0 dropout_ticks=0 estop_ticks=0 fault_ticks=0 ... hash=00cffd5d35dc57d9 ...
```

`hash` is over every rotor command the bridge produced. The Windows binary
prints the same `hash=00cffd5d35dc57d9` for the same file.

## 3. Fly it

Each run starts a fresh PX4 + Gazebo container named `px4gz` (it **removes
any existing container of that name**; pass `--container NAME` to use
another), waits for the EKF, flies 60 s at 1 m and writes `TAG.bin` (the true
estimator stream), `TAG.csv` (per-tick bridge log), `TAG.json` (metrics) and a
row in `results.md`. With `--arm reflex` and no `--brain`, the runner flies the
flight default (`tools/px4/flight_default.py`: this model, 832 bits,
plasticity off). One run takes about 2 minutes.

Hover and wind, 3 runs each as published:

```sh
mkdir -p runs/me
for i in 1 2 3; do python3 tools/px4/run_scenario.py --arm reflex --scenario hover --out runs/me --tag hover_$i; done
for i in 1 2 3; do python3 tools/px4/run_scenario.py --arm reflex --scenario wind  --out runs/me --tag wind_$i;  done
python3 tools/px4/summarize_runs.py runs/me
```

Wind = lateral force pulses of 3.0 N + 1.5 N on the model, 1 s long, every
6 s from 15 s (the defaults `--wind-n 3 --wind-every 6 --wind-len 1 --wind-from 15`).

Arc process loss with handover to PX4 (kill = crash, hang = process suspended):

```sh
mkdir -p runs/loss
for i in 1 2 3; do python3 tools/px4/run_scenario.py --arm reflex --seconds 50 --scenario bridge_loss \
    --loss-at 30 --loss-kind kill --standby px4 --out runs/loss --tag kill_$i 2>&1 | tee runs/loss/kill_$i.log; done
for i in 1 2 3; do python3 tools/px4/run_scenario.py --arm reflex --seconds 50 --scenario bridge_loss \
    --loss-at 30 --loss-kind hang --standby px4 --out runs/loss --tag hang_$i 2>&1 | tee runs/loss/hang_$i.log; done
python3 tools/px4/summarize_bridge_loss.py runs/loss
```

Keep the bridge-loss runs in their own directory: that summarizer reads the
`.log` next to every `.json`. Remove the last container with `docker rm -f px4gz`.

## 4. What you should get

These are scenario statistics on float physics, not bit-exact: PX4's EKF and
Gazebo run against the wall clock, so each run differs a little. Published
ranges are min..max over 3 runs, and a run of yours landing slightly outside
one is that noise. A fault, a crash, a tilt fault or a wall stall over 1 s is
not noise.

Hover, flight default (`docs/px4_sitl_summary_table.md`, row `hover | reflex | raw, plasticity off, prior_x500_candidate_v5.arc002 (832 bits)`):

| metric | published (3 runs) |
|---|---|
| takeoff s | 1.02..1.06 |
| alt mean m (target 1.0) | 1.000 |
| alt jitter rms m | 0.023..0.027 |
| station max m (settled) | 0.235..0.303 |
| tilt max ° | 3.3..3.4 |
| survival s (of 60) | 58.9..59.0, 3 / 3 survived |
| fault ticks | 0 |

Wind, flight default (same table, row `wind | reflex | raw, plasticity off, prior_x500_candidate_v5.arc002 (832 bits)`):

| metric | published (3 runs) |
|---|---|
| alt mean m (before the first pulse) | 0.991..1.005 |
| disturbed: alt err max m | 0.092..0.106 |
| disturbed: station max m | 2.960..3.049 |
| disturbed: tilt max ° | 17.6..18.4 |
| survival s | 58.9..59.1, 3 / 3 survived |
| fault ticks | 0 |
| wind recovery max s (unrecovered) | 19.40..31.06 (2..8). The recovery band is strict (0.25 m station and 0.15 m altitude, held 2 s) and a pulse every 6 s often lands before the vehicle is back in it |

Arc process loss (`docs/bridge_loss_method.md`): every loss survived with the
PX4 fallback (13 / 13 published; 0 / 1 without it). Detection 3..27 ms after
a kill (stream closed) and 201..203 ms after a hang (200 ms watchdog); route +
arm + takeoff 95..330 ms; altitude lost 0.16..0.70 m from a ~1.1 m hover;
after takeover PX4 climbs to 2.0 m and holds. Those published rows were
measured with the trained reflex and the Arc cascade flying; the same runs
with the flight default flying are being added to that page.

### Checked from these files (2026-10-19)

One hover each, from a clone of this repository, with the binaries, model and harness above and the pinned image:

| host | takeoff s | alt mean m | alt jitter rms m | station max m | tilt max ° | survival s | fault ticks | max wall stall ms |
|---|---|---|---|---|---|---|---|---|
| Linux: Ubuntu 24.04 (WSL2 kernel), Docker Engine 29.1.3 (`docker.io`), Python 3.12 | 1.24 | 1.000 | 0.025 | 0.236 | 3.7 | 58.1 | 0 | 23.0 |
| Windows 11 x64, Docker Desktop, Python 3.12 | 1.04 | 1.000 | 0.024 | 0.248 | 3.3 | 58.3 | 0 | 0.6 |

Both survived with 0 faults. Takeoff and tilt on the Linux run, and survival on both, sit just outside the 3-run published ranges: that is the run-to-run noise described above.
The offline check printed `hash=00cffd5d35dc57d9` on both.

## Windows

The same steps work from PowerShell with Docker Desktop: use `python` for
`python3`, skip the `apt-get` and `usermod` lines, and the runner picks
`bin/windows-x64/px4_bridge.exe` by itself. Offline check:

```bat
cmd /c "bin\windows-x64\px4_bridge.exe --brain models\prior_x500_candidate_v5.arc002 --bits 832 --plasticity off --hover 769 --every 4 < tests\fixtures\px4_ekf_ground.bin > NUL"
```

## Not covered here

- macOS and Linux ARM64 hosts: no bridge binary ships for them yet (the PX4
  image is x86_64 too).
- Learning: the flight default is compiled, not trained, and flies with
  plasticity off. The trained prior is the learning demonstration and is not
  in this repository.
- Hardware: SITL only. `docs/gap_register.md` lists what simulation does not
  settle.
