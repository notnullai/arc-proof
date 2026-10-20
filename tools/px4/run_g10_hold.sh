#!/bin/bash
# G10 hold re-measurement (docs/rotor_loss.md 4): the x1.2 airframe from 30 m and 10 m, each rotor, with the
# mode's own height and climb rate (baro + accelerometer). Same rules as run_g10.sh: sequential, a fresh
# container per run, each run only after no other run_scenario.py is running and host CPU is under 25 %.
# Usage: tools/px4/run_g10_hold.sh <out dir> <bridge.exe> <static linux bridge>   (repo root, Git Bash on Windows)
set -u
OUT=${1:-runs/g10_hold}
BRIDGE=$(cygpath -w "$(realpath "${2:-build/Release/px4_bridge.exe}")")
RL=models/prior_x500_rotor_loss.arc002
SIDE=(--arc-side container --bridge-linux "$3")
mkdir -p "$OUT"

wait_clear() {
  while true; do
    busy=$(powershell -NoProfile -Command "(Get-CimInstance Win32_Process -Filter \"Name like 'python%'\" | Where-Object { \$_.CommandLine -match 'run_scenario' } | Measure-Object).Count")
    [ "${busy//[$'\r\n ']/}" = "0" ] && docker rm -f px4gz >/dev/null 2>&1
    cpu=$(powershell -NoProfile -Command "[int]((Get-Counter '\\Processor(_Total)\\% Processor Time' -SampleInterval 3 -MaxSamples 4).CounterSamples | Measure-Object CookedValue -Average).Average")
    if [ "${busy//[$'\r\n ']/}" = "0" ] && [ "${cpu//[$'\r\n ']/}" -lt 25 ]; then echo "clear: cpu ${cpu//[$'\r\n ']/} %"; return; fi
    echo "waiting: run_scenario running ${busy//[$'\r\n ']/}, cpu ${cpu//[$'\r\n ']/} %"; sleep 30
  done
}

run() {
  local tag=$1; shift
  if [ -f "$OUT/$tag.json" ]; then echo "skip $tag (done)"; return; fi
  wait_clear
  echo "=== $tag: $*"
  python tools/px4/run_scenario.py --arm reflex --bridge "$BRIDGE" "${SIDE[@]}" --out "$OUT" --tag "$tag" "$@" > "$OUT/$tag.log" 2>&1
  tail -2 "$OUT/$tag.log"
}

for m in 1 2 3 4; do
  run g10h_s12_a30_m$m --scenario motor_loss --alt 30 --fail-at 75 --fail-motor $m --seconds 110 --motor-scale 1.2 --rotor-loss $RL
done
for m in 1 2 3 4; do
  run g10h_s12_m$m --scenario motor_loss --alt 10 --fail-at 40 --fail-motor $m --seconds 70 --motor-scale 1.2 --rotor-loss $RL
done
# repeats of the 30 m rotor-4 case (one of the first four lost its attitude in the transient)
for k in 2 3; do
  run g10h_s12_a30_m4_r$k --scenario motor_loss --alt 30 --fail-at 75 --fail-motor 4 --seconds 110 --motor-scale 1.2 --rotor-loss $RL
done
# the last run leaves its Gazebo container running: remove it (Docker is shared)
docker rm -f px4gz >/dev/null 2>&1
