#!/bin/bash
# G10 Gazebo campaign (docs/rotor_loss.md section 4): sequential, a fresh container per run, each run only
# after no other run_scenario.py is running and host CPU has been under 25 % (Docker is shared).
# Usage: tools/px4/run_g10.sh <out dir> <bridge.exe> [<static linux bridge>]   (repo root, Git Bash on Windows)
# With the Linux bridge, the Arc side runs inside the container (G6): no Windows pipe on the command path, which
# stalled 348 ms in one host-side run (docs/rotor_loss.md 4).
set -u
OUT=${1:-runs/g10}
BRIDGE=$(cygpath -w "$(realpath "${2:-build/Release/px4_bridge.exe}")")   # absolute: Windows CreateProcess
RL=models/prior_x500_rotor_loss.arc002
SIDE=()
[ -n "${3:-}" ] && SIDE=(--arc-side container --bridge-linux "$3")
mkdir -p "$OUT"

wait_clear() {
  while true; do
    busy=$(powershell -NoProfile -Command "(Get-CimInstance Win32_Process -Filter \"Name like 'python%'\" | Where-Object { \$_.CommandLine -match 'run_scenario' } | Measure-Object).Count")
    # our own finished run leaves its Gazebo container idling: remove it before measuring (every run restarts it)
    [ "${busy//[$'\r\n ']/}" = "0" ] && docker rm -f px4gz >/dev/null 2>&1
    cpu=$(powershell -NoProfile -Command "[int]((Get-Counter '\\Processor(_Total)\\% Processor Time' -SampleInterval 3 -MaxSamples 4).CounterSamples | Measure-Object CookedValue -Average).Average")
    if [ "${busy//[$'\r\n ']/}" = "0" ] && [ "${cpu//[$'\r\n ']/}" -lt 25 ]; then echo "clear: cpu ${cpu//[$'\r\n ']/} %"; return; fi
    echo "waiting: run_scenario running ${busy//[$'\r\n ']/}, cpu ${cpu//[$'\r\n ']/} %"; sleep 30
  done
}

run() {   # tag, then run_scenario arguments
  local tag=$1; shift
  if [ -f "$OUT/$tag.json" ]; then echo "skip $tag (done)"; return; fi
  wait_clear
  echo "=== $tag: $*"
  python tools/px4/run_scenario.py --arm reflex --bridge "$BRIDGE" "${SIDE[@]}" --out "$OUT" --tag "$tag" "$@" > "$OUT/$tag.log" 2>&1
  tail -2 "$OUT/$tag.log"
}

# normal flight with the mode on (flight default unchanged: compare with docs/px4_sitl.md G9)
run g10_hover_rl   --scenario hover --rotor-loss $RL
run g10_wind_rl    --scenario wind  --rotor-loss $RL
# motor loss at 10 m, stock x500 (T/W 1.69): flight default alone, then with the mode, each rotor
run g10_ml_v5_m1   --scenario motor_loss --alt 10 --fail-at 40 --fail-motor 1 --seconds 70
for m in 0 1 2 3; do
  run g10_ml_rl_m$m --scenario motor_loss --alt 10 --fail-at 40 --fail-motor $m --seconds 70 --rotor-loss $RL
done
# motor loss at 10 m, x500 with motorConstant x1.2 (T/W 2.03)
run g10_ml_v5_s12_m1 --scenario motor_loss --alt 10 --fail-at 40 --fail-motor 1 --seconds 70 --motor-scale 1.2
for m in 0 1 2 3; do
  run g10_ml_rl_s12_m$m --scenario motor_loss --alt 10 --fail-at 40 --fail-motor $m --seconds 70 --motor-scale 1.2 --rotor-loss $RL
done
# --fail-motor is the gz MotorFailure index, 1-based (0 = none): 1-4 = gz rotors 0-3; the m0 runs above are
# failure-free control flights with the mode on. The fourth rotor, and 30 m (more height for the spin-up):
run g10_ml_rl_m4     --scenario motor_loss --alt 10 --fail-at 40 --fail-motor 4 --seconds 70 --rotor-loss $RL
run g10_ml_rl_s12_m4 --scenario motor_loss --alt 10 --fail-at 40 --fail-motor 4 --seconds 70 --motor-scale 1.2 --rotor-loss $RL
for m in 1 2 3 4; do
  run g10_ml_rl_s12_a30_m$m --scenario motor_loss --alt 30 --fail-at 75 --fail-motor $m --seconds 110 --motor-scale 1.2 --rotor-loss $RL
done
# the last run leaves its Gazebo container running: remove it (Docker is shared)
docker rm -f px4gz >/dev/null 2>&1
