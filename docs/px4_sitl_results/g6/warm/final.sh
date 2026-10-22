#!/bin/bash
# interleaved cold/warm A/B (kills) on both rigs, then the final G6 matrices; one process, idle host per run
cd /d/arc-g13
B="--arm reflex --arc-side container --bridge-linux bin/linux-x86_64/px4_bridge --witness --standby px4 --watchdog-clock sim --seconds 50 --scenario bridge_loss --loss-at 30 --loss-kind kill"
for i in 1 2 3 4 5 6; do for k in warm cold; do
  t=g13f_mav_${k}_kill_$i; [ -f runs/g13_ab5_mav/$t.json ] || bash /d/arc-g13/runs/scripts/one.sh runs/g13_ab5_mav $t $B --rig mavlink --takeover $k
  t=g13f_dir_${k}_kill_$i; [ -f runs/g13_ab5_dir/$t.json ] || bash /d/arc-g13/runs/scripts/one.sh runs/g13_ab5_dir $t $B --rig direct --record-clock own --physics-hz 1000 --takeover $k
done; done
echo AB DONE
P=g13 EXTRA="--record-clock own --physics-hz 1000" tools/px4/run_g11.sh runs/g13 bin/linux-x86_64/px4_bridge g6
P=g13 tools/px4/run_g11.sh runs/g13_mav bin/linux-x86_64/px4_bridge g6mav
echo FINAL DONE
