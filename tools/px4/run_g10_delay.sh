#!/bin/bash
# G10 delay batch (docs/rotor_loss.md 5): the direct rig (G11/G12: --record-clock own --physics-hz 1000) with an
# extra sensor delay injected in the record stream (run_scenario.py --sensor-delay-ms), rotor-loss and
# normal-flight runs with the mode on. Same rules as run_g11.sh: sequential, a fresh container per run, each run
# only after no other run_scenario.py is running and host CPU has been under 25 %; every run kept; witness on.
# Usage: tools/px4/run_g10_delay.sh <out dir> [<static linux bridge, default bin/linux-x86_64/px4_bridge>] [delay_ms ...]   (repo root, Git Bash on Windows)
set -u
OUT=${1:-runs/g10_delay}
BL=${2:-bin/linux-x86_64/px4_bridge}
shift 2 2>/dev/null
DELAYS=${*:-20 40}
RL=models/prior_x500_rotor_loss.arc002
CT=(--arc-side container --bridge-linux "$BL" --witness --rig direct --record-clock own --physics-hz 1000)
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

run() {   # tag, then run_scenario arguments
  local tag=$1; shift
  if [ -f "$OUT/$tag.json" ]; then echo "skip $tag (done)"; return; fi
  wait_clear
  echo "=== $tag $(date +%T): $*"
  python tools/px4/run_scenario.py --out "$OUT" --tag "$tag" "$@" > "$OUT/$tag.log" 2>&1
  tail -1 "$OUT/$tag.log"
}

ML10=(--scenario motor_loss --alt 10 --fail-at 40 --seconds 70)
ML30=(--scenario motor_loss --alt 30 --fail-at 75 --seconds 110)
for d in $DELAYS; do
  D=(--sensor-delay-ms $d)
  # normal flight: the mode on (no failure), and v5 alone
  run g10d${d}_hover_rl --arm reflex "${CT[@]}" "${D[@]}" --scenario hover --rotor-loss $RL
  run g10d${d}_wind_rl  --arm reflex "${CT[@]}" "${D[@]}" --scenario wind  --rotor-loss $RL
  run g10d${d}_hover_v5 --arm reflex "${CT[@]}" "${D[@]}" --scenario hover
  # rotor loss: x1.2 from 10 m and 30 m, stock from 10 m
  for m in 1 3; do run g10d${d}_s12_a10_m$m --arm reflex "${CT[@]}" "${D[@]}" "${ML10[@]}" --fail-motor $m --motor-scale 1.2 --rotor-loss $RL; done
  for m in 2 4; do run g10d${d}_s12_a30_m$m --arm reflex "${CT[@]}" "${D[@]}" "${ML30[@]}" --fail-motor $m --motor-scale 1.2 --rotor-loss $RL; done
  run g10d${d}_a10_m1 --arm reflex "${CT[@]}" "${D[@]}" "${ML10[@]}" --fail-motor 1 --rotor-loss $RL
done
# the last run leaves its Gazebo container running: remove it (Docker is shared)
docker rm -f px4gz >/dev/null 2>&1
