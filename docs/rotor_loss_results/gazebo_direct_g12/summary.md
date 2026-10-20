# G12 direct rig: record clock and 1 kHz world (tools/px4/run_g11.sh, campaign.log)

g12_* = --record-clock own --physics-hz 1000 (both changes); g12own_* = --record-clock own at the stock 250 Hz
(the bridge clock alone). Same airframes, heights, failure times and bridge build (static Linux, this branch).
g12own_s12_a10_m3_3 stopped after "injector READY" with no result and no error (log kept), re-run as
g12own_s12_a10_m3_3b. g12_wind_rl had one 5.15 s wind pulse (the clear injected 4 s late, wall clock, across
two Docker VM stalls), re-run as g12_wind_rl_2. g12_ml_v5_m1: the summarizer places the onset on the climb-out
(alt at failure 0.45 m); the witness shows the vehicle level at 10 m until the failure at sim 54.37 s, then the
flip as on G11 (true tilt 169 deg).

## Records (tools/px4/summarize_rotor_loss.py; tilt from the record's attitude)

| run | arm | motor x | failed motor (gz index + 1) | alt at failure m | onset s | detected after ms | confirmed after ms | rotor | outcome | lowest alt m | touchdown m/s | max tilt deg | catch tilt deg (1.5 s) | mean spin rad/s | max drift m | flown after s | last 20 s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| g12_ml_rl_a10_m1 | v5 + rotor-loss | 1.00 | 1 | 9.98 | 37.06 | 6 | 21 | 0 | touchdown | 0.06 | 5.03 | 32.3 | 32.3 | 7.4 | 2.19 | 3.2 | - |
| g12_ml_rl_a10_m2 | v5 + rotor-loss | 1.00 | 2 | 9.97 | 37.08 | 6 | 21 | 1 | touchdown | 0.09 | 4.97 | 31.7 | 31.7 | 7.4 | 2.15 | 3.3 | - |
| g12_ml_rl_a10_m3 | v5 + rotor-loss | 1.00 | 3 | 9.99 | 37.08 | 6 | 21 | 2 | touchdown | 0.08 | 4.93 | 29.5 | 29.5 | 7.5 | 2.16 | 3.3 | - |
| g12_ml_rl_a10_m4 | v5 + rotor-loss | 1.00 | 4 | 10.03 | 37.05 | 6 | 21 | 3 | touchdown | 0.05 | 4.84 | 27.0 | 27.0 | 7.6 | 1.75 | 3.4 | - |
| g12_ml_v5_m1 | v5 alone | 1.00 | 1 | 0.45 | 32.00 | - | - | - | touchdown | 0.08 | 2.91 | 79.7 | 79.7 | 21.8 | 4.03 | 0.8 | - |
| g12_ml_v5_s12_m1 | v5 alone | 1.20 | 1 | 9.98 | 37.05 | - | - | - | touchdown | -0.07 | 13.25 | 171.0 | 171.0 | 1.6 | 2.0 | 1.5 | - |
| g12_s12_a10_m1_1 | v5 + rotor-loss | 1.20 | 1 | 9.98 | 37.06 | 6 | 21 | 0 | held | 8.12 | - | 179.9 | 27.2 | 14.6 | 1.95 | 31.1 | 9.2-9.9 m, xy 1.4 m |
| g12_s12_a10_m1_5 | v5 + rotor-loss | 1.20 | 1 | 10.02 | 37.02 | 6 | 21 | 0 | held | 8.13 | - | 179.9 | 24.0 | 14.5 | 1.89 | 31.1 | 9.3-10.1 m, xy 1.6 m |
| g12_s12_a10_m2_2 | v5 + rotor-loss | 1.20 | 2 | 9.99 | 37.04 | 6 | 21 | 1 | held | 8.31 | - | 179.9 | 27.6 | 14.6 | 1.88 | 31.1 | 9.4-10.1 m, xy 1.6 m |
| g12_s12_a10_m3_3 | v5 + rotor-loss | 1.20 | 3 | 10.02 | 37.09 | 6 | 21 | 2 | held | 7.94 | - | 179.8 | 24.5 | 14.6 | 1.95 | 31.1 | 9.3-9.9 m, xy 1.4 m |
| g12_s12_a10_m3_6 | v5 + rotor-loss | 1.20 | 3 | 9.99 | 37.07 | 6 | 21 | 2 | held | 8.12 | - | 179.8 | 25.4 | 14.6 | 1.76 | 31.1 | 9.3-10.0 m, xy 1.2 m |
| g12_s12_a10_m4_4 | v5 + rotor-loss | 1.20 | 4 | 9.99 | 37.10 | 6 | 21 | 3 | held | 8.45 | - | 179.3 | 24.1 | 14.6 | 2.0 | 31.1 | 9.6-10.3 m, xy 1.3 m |
| g12_s12_a30_m1_7 | v5 + rotor-loss | 1.20 | 1 | 30.01 | 69.93 | 6 | 21 | 0 | held | 28.18 | - | 179.9 | 28.7 | 14.7 | 1.79 | 36.3 | 29.4-30.0 m, xy 1.1 m |
| g12_s12_a30_m2_11 | v5 + rotor-loss | 1.20 | 2 | 29.99 | 69.23 | 6 | 21 | 1 | held | 28.21 | - | 179.8 | 23.9 | 14.6 | 1.96 | 32.9 | 29.4-30.1 m, xy 1.3 m |
| g12_s12_a30_m2_8 | v5 + rotor-loss | 1.20 | 2 | 30.02 | 69.84 | 6 | 21 | 1 | held | 28.5 | - | 179.9 | 25.5 | 14.7 | 1.9 | 36.4 | 29.4-30.2 m, xy 1.6 m |
| g12_s12_a30_m3_9 | v5 + rotor-loss | 1.20 | 3 | 30.0 | 69.99 | 6 | 21 | 2 | held | 28.27 | - | 179.6 | 27.6 | 14.6 | 1.86 | 34.4 | 29.5-30.2 m, xy 1.3 m |
| g12_s12_a30_m4_10 | v5 + rotor-loss | 1.20 | 4 | 29.98 | 69.99 | 6 | 21 | 3 | held | 27.95 | - | 179.6 | 25.0 | 14.7 | 2.0 | 36.3 | 29.2-29.9 m, xy 1.2 m |
| g12_s12_a30_m4_12 | v5 + rotor-loss | 1.20 | 4 | 29.98 | 69.36 | 6 | 21 | 3 | held | 27.86 | - | 179.7 | 28.2 | 14.7 | 1.85 | 35.9 | 28.9-29.8 m, xy 1.0 m |
| g12own_s12_a10_m1_1 | v5 + rotor-loss | 1.20 | 1 | 10.01 | 37.07 | 4 | 20 | 0 | held | 8.58 | - | 179.8 | 28.3 | 14.5 | 1.92 | 31.1 | 9.6-10.4 m, xy 1.6 m |
| g12own_s12_a10_m1_5 | v5 + rotor-loss | 1.20 | 1 | 10.0 | 37.10 | 4 | 20 | 0 | held | 7.96 | - | 179.7 | 28.4 | 14.5 | 1.82 | 29.1 | 9.2-10.1 m, xy 1.8 m |
| g12own_s12_a10_m2_2 | v5 + rotor-loss | 1.20 | 2 | 10.01 | 37.11 | 4 | 20 | 1 | held | 8.31 | - | 179.6 | 29.9 | 14.5 | 1.96 | 31.1 | 9.4-9.9 m, xy 1.7 m |
| g12own_s12_a10_m3_3b | v5 + rotor-loss | 1.20 | 3 | 9.97 | 37.05 | 4 | 20 | 2 | held | 8.53 | - | 179.0 | 29.5 | 14.5 | 1.83 | 31.1 | 9.6-10.3 m, xy 1.4 m |
| g12own_s12_a10_m3_6 | v5 + rotor-loss | 1.20 | 3 | 10.0 | 37.07 | 4 | 20 | 2 | held | 8.33 | - | 178.9 | 28.4 | 14.5 | 1.9 | 29.1 | 9.5-10.1 m, xy 1.6 m |
| g12own_s12_a10_m4_4 | v5 + rotor-loss | 1.20 | 4 | 9.98 | 37.08 | 4 | 20 | 3 | held | 8.05 | - | 179.8 | 27.8 | 14.6 | 2.05 | 31.1 | 9.1-10.1 m, xy 1.4 m |

## Gazebo witness (tools/px4/rig_latency.py; sim clock, true pose and gyro)

Onset = the first true IMU sample above 20 rad/s^2 (1 ms resolution at 1 kHz, 4 ms at 250 Hz); acts = cut + one
physics step; failure to acts = from the failure command's witness stamp (resolution-independent).

| run | rig | alt m | step ms | onset s (sim) | first command change ms | opposite rotor cut ms | acts ms | failure to acts ms | true catch tilt deg (1.5 s) | peak roll/pitch rate rad/s (0.3 s) |
|---|---|---|---|---|---|---|---|---|---|---|
| g12_ml_rl_a10_m1 | direct | 10.0 | 1.0 | 67.201 | 8.0 | 8.0 | 9.0 | 10.0 | 32.2 | 1.25 |
| g12_ml_rl_a10_m2 | direct | 10.0 | 1.0 | 67.138 | 8.0 | 8.0 | 9.0 | 10.0 | 31.7 | 1.22 |
| g12_ml_rl_a10_m3 | direct | 10.0 | 1.0 | 67.166 | 7.0 | 7.0 | 8.0 | 9.0 | 29.4 | 1.1 |
| g12_ml_rl_a10_m4 | direct | 10.0 | 1.0 | 67.154 | 8.0 | 8.0 | 9.0 | 11.0 | 27.0 | 1.09 |
| g12_ml_v5_m1 | direct | 10.0 | 1.0 | 54.371 | 23.0 | 527.0 | 528.0 | 529.0 | 169.1 | 12.16 |
| g12_ml_v5_s12_m1 | direct | 10.0 | 1.0 | 66.968 | 14.0 | 521.0 | 522.0 | 523.0 | 170.0 | 12.22 |
| g12_s12_a10_m1_1 | direct | 10.0 | 1.0 | 67.091 | 7.0 | 7.0 | 8.0 | 10.0 | 27.2 | 1.08 |
| g12_s12_a10_m1_5 | direct | 10.0 | 1.0 | 66.964 | 7.0 | 7.0 | 8.0 | 9.0 | 24.0 | 1.02 |
| g12_s12_a10_m2_2 | direct | 10.0 | 1.0 | 67.154 | 7.0 | 7.0 | 8.0 | 9.0 | 27.5 | 1.1 |
| g12_s12_a10_m3_3 | direct | 10.0 | 1.0 | 67.223 | 7.0 | 7.0 | 8.0 | 9.0 | 24.6 | 1.04 |
| g12_s12_a10_m3_6 | direct | 10.0 | 1.0 | 66.875 | 7.0 | 7.0 | 8.0 | 9.0 | 25.4 | 1.09 |
| g12_s12_a10_m4_4 | direct | 10.0 | 1.0 | 67.131 | 7.0 | 7.0 | 8.0 | 9.0 | 24.0 | 1.02 |
| g12_s12_a30_m1_7 | direct | 30.0 | 1.0 | 99.962 | 7.0 | 7.0 | 8.0 | 9.0 | 28.7 | 1.16 |
| g12_s12_a30_m2_11 | direct | 30.0 | 1.0 | 99.446 | 7.0 | 7.0 | 8.0 | 9.0 | 23.9 | 1.07 |
| g12_s12_a30_m2_8 | direct | 30.0 | 1.0 | 100.012 | 7.0 | 7.0 | 8.0 | 9.0 | 25.4 | 1.09 |
| g12_s12_a30_m3_9 | direct | 30.0 | 1.0 | 100.287 | 7.0 | 7.0 | 8.0 | 9.0 | 27.7 | 1.14 |
| g12_s12_a30_m4_10 | direct | 30.0 | 1.0 | 100.015 | 7.0 | 7.0 | 8.0 | 10.0 | 24.9 | 1.06 |
| g12_s12_a30_m4_12 | direct | 30.0 | 1.0 | 112.376 | 7.0 | 7.0 | 8.0 | 9.0 | 28.1 | 1.18 |
| g12own_s12_a10_m1_1 | direct | 10.0 | 4.0 | 67.304 | 4.0 | 4.0 | 8.0 | 12.0 | 28.3 | 1.16 |
| g12own_s12_a10_m1_5 | direct | 10.0 | 4.0 | 67.24 | 4.0 | 4.0 | 8.0 | 12.0 | 28.3 | 1.21 |
| g12own_s12_a10_m2_2 | direct | 10.0 | 4.0 | 67.244 | 4.0 | 4.0 | 8.0 | 12.0 | 29.9 | 1.2 |
| g12own_s12_a10_m3_3b | direct | 10.0 | 4.0 | 67.24 | 4.0 | 4.0 | 8.0 | 16.0 | 29.4 | 1.23 |
| g12own_s12_a10_m3_6 | direct | 10.0 | 4.0 | 67.244 | 4.0 | 4.0 | 8.0 | 12.0 | 28.4 | 1.21 |
| g12own_s12_a10_m4_4 | direct | 10.0 | 4.0 | 67.14 | 4.0 | 4.0 | 8.0 | 16.0 | 27.6 | 1.09 |
