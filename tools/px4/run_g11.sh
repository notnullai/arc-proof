#!/bin/bash
# G11 direct rig campaign (docs/rotor_loss.md 4, docs/px4_sitl_method.md, direct rig): the Arc side reads Gazebo's IMU topic and
# writes the motor topic directly (run_scenario.py --rig direct), PX4 stays the estimator and the fallback.
# Same rules as run_g10.sh: sequential, a fresh container per run, each run only after no other run_scenario.py
# is running and host CPU has been under 25 %; every run kept. Every run carries the Gazebo witness (--witness)
# for tools/px4/rig_latency.py.
# Usage: tools/px4/run_g11.sh <out dir> <static linux bridge> [phase ...]   (repo root, Git Bash on Windows)
#   phases: before (the MAVLink rig with the witness, the latency baseline), normal, rotor, g6 (default: all)
set -u
OUT=${1:-runs/g11}
BL=${2:-bin/linux-x86_64/px4_bridge}
shift 2 2>/dev/null
PHASES=${*:-before normal rotor g6}
RL=models/prior_x500_rotor_loss.arc002
CT=(--arc-side container --bridge-linux "$BL" --witness)
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
  echo "=== $tag $(date +%T): $*"
  python tools/px4/run_scenario.py --out "$OUT" --tag "$tag" "$@" > "$OUT/$tag.log" 2>&1
  tail -1 "$OUT/$tag.log"
}

ML10=(--scenario motor_loss --alt 10 --fail-at 40 --seconds 70)
ML30=(--scenario motor_loss --alt 30 --fail-at 75 --seconds 110)
for ph in $PHASES; do case $ph in
before)   # latency baseline: the MAVLink rig as in G10 (x1.2 airframe, the mode on), with the witness
  for m in 1 2; do run g11b_s12_a10_m$m --arm reflex "${CT[@]}" --rig mavlink "${ML10[@]}" --fail-motor $m --motor-scale 1.2 --rotor-loss $RL; done
  for m in 1 2 3 4; do run g11b_s12_a30_m$m --arm reflex "${CT[@]}" --rig mavlink "${ML30[@]}" --fail-motor $m --motor-scale 1.2 --rotor-loss $RL; done ;;
normal)   # v5 regressions (G9 matrix: hover, wind 3 N every 6 s, 60 s, 1 m) and normal flight with the mode on
  for i in 1 2 3; do run g11_hover_v5_$i --arm reflex "${CT[@]}" --rig direct --scenario hover; done
  for i in 1 2 3; do run g11_wind_v5_$i  --arm reflex "${CT[@]}" --rig direct --scenario wind; done
  run g11_hover_rl --arm reflex "${CT[@]}" --rig direct --scenario hover --rotor-loss $RL
  run g11_wind_rl  --arm reflex "${CT[@]}" --rig direct --scenario wind  --rotor-loss $RL
  run g11_ctl_a10_rl     --arm reflex "${CT[@]}" --rig direct "${ML10[@]}" --fail-motor 0 --rotor-loss $RL
  run g11_ctl_s12_a10_rl --arm reflex "${CT[@]}" --rig direct "${ML10[@]}" --fail-motor 0 --motor-scale 1.2 --rotor-loss $RL ;;
rotor)    # rotor failure: v5 alone, the stock x500 with the mode, the x1.2 airframe from 10 m and 30 m
  run g11_ml_v5_m1     --arm reflex "${CT[@]}" --rig direct "${ML10[@]}" --fail-motor 1
  run g11_ml_v5_s12_m1 --arm reflex "${CT[@]}" --rig direct "${ML10[@]}" --fail-motor 1 --motor-scale 1.2
  for m in 1 2 3 4; do run g11_ml_rl_a10_m$m --arm reflex "${CT[@]}" --rig direct "${ML10[@]}" --fail-motor $m --rotor-loss $RL; done
  i=0
  for m in 1 2 3 4 1 3; do i=$((i+1)); run g11_s12_a10_m${m}_$i --arm reflex "${CT[@]}" --rig direct "${ML10[@]}" --fail-motor $m --motor-scale 1.2 --rotor-loss $RL; done
  for m in 1 2 3 4 2 4; do i=$((i+1)); run g11_s12_a30_m${m}_$i --arm reflex "${CT[@]}" --rig direct "${ML30[@]}" --fail-motor $m --motor-scale 1.2 --rotor-loss $RL; done ;;
g6)       # G6 handover matrix (docs/bridge_loss_method.md): v5, PX4 standby, plant-clock watchdog, 50 s, loss at 30 s
  G6=(--arm reflex "${CT[@]}" --rig direct --standby px4 --watchdog-clock sim --seconds 50)
  for i in 1 2 3 4 5 6; do run g11_g6_kill_$i "${G6[@]}" --scenario bridge_loss --loss-at 30 --loss-kind kill; done
  for i in 1 2 3; do run g11_g6_hang_$i "${G6[@]}" --scenario bridge_loss --loss-at 30 --loss-kind hang; done
  for i in 1 2 3 4; do run g11_g6_nofault_$i "${G6[@]}" --scenario hover; done ;;
esac; done
# the last run leaves its Gazebo container running: remove it (Docker is shared)
docker rm -f px4gz >/dev/null 2>&1
echo "G11 DONE $(date)"
