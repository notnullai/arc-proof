#!/bin/bash
# G6 transport fix: handover matrix on the flight default (candidate v5) with the Arc side in the container
# (companion-computer layout), the MAVLink takeover, the standby keep-alive and the plant-clock watchdog.
# Sequential, fresh container per run; waits for host CPU < 25 % before each run; a run with a host stall
# (max wall tick > 1 s) is kept as evidence (_stall) and re-run once.
# Linux bridge: static build of HEAD ccf4fed (build-linux/px4_bridge, sha256 be6abe5f...).
cd /d/arc
OUT=runs/g6; V5="--arm reflex --brain models/prior_x500_candidate_v5.arc002 --bits 832 --plasticity off"
CT="--arc-side container --bridge-linux build-linux/px4_bridge --standby px4 --watchdog-clock sim --seconds 50"
HB="--arc-side host --standby px4 --watchdog-clock sim --seconds 50"
cpu() { powershell -NoProfile -Command "[int](Get-CimInstance Win32_Processor | Measure-Object -Property LoadPercentage -Average).Average" | tr -d '\r'; }
idle() { while :; do c=$(cpu); [ "$c" -lt 25 ] && { echo "cpu $c% $(date +%T)"; return; }; sleep 15; done; }
stall() { python -c "import json;d=json.load(open('$OUT/$1.json'));print(1 if int(d.get('bridge',{}).get('max_wall_tick_us','0'))>1000000 else 0)" 2>/dev/null || echo 1; }
disk() { df -BG /d | awk 'NR==2{gsub("G","",$4); print $4}'; }
one() {  # one <tag> <args...>
  local t=$1; shift
  for n in 1 2; do
    [ -f $OUT/$t.json ] && { echo "$t done"; return; }
    [ "$(disk)" -lt 20 ] && { echo "D: low ($(disk) GB), stopping"; exit 1; }
    docker ps >/dev/null 2>&1 || { echo "docker down, stopping"; exit 1; }
    idle
    python -u tools/px4/run_scenario.py --out $OUT --tag $t "$@" > $OUT/$t.out 2>&1; rc=$?
    if [ "$(stall $t)" = "1" ]; then for f in $OUT/$t.*; do mv "$f" "${f/$t/${t}_stall$n}"; done; echo "$t rc=$rc STALL (kept)"; else echo "$t rc=$rc $(date +%T)"; return; fi
  done
}
echo "start $(date)"
for i in 1 2 3 4 5 6; do one g6_kill_v5_$i $V5 $CT --scenario bridge_loss --loss-at 30 --loss-kind kill; done
for i in 1 2 3; do one g6_hang_v5_$i $V5 $CT --scenario bridge_loss --loss-at 30 --loss-kind hang; done
for i in 1 2 3 4; do one g6_nofault_v5_$i $V5 $CT --scenario hover; done
# same-day baseline with the same witnesses: the Arc side on the Windows host (the h4 layout)
for i in 1 2; do one g6_hostbase_v5_$i $V5 $HB --scenario hover; done
echo "MATRIX DONE $(date)"
