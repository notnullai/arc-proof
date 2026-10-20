# G11 direct rig: v5 regressions (G9 matrix), tools/px4/run_g11.sh phase normal

g11_hover_v5_2_streamer_exit_crash: the first g11_hover_v5_2, kept; the direct streamer crashed at exit (pybind11, 0.6 s before the end of the stream, after the 60 s window), fixed and re-run. Not in the table.

| scenario | arm | fault policy | n | takeoff s | alt mean m | alt jitter rms m | station max m | tilt max ° | dist. alt err max m | dist. station max m | dist. tilt max ° | wind recovery max s (unrec.) | survival s | first fault / impact s | fault ticks | dropout ticks | max wall stall ms |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hover | reflex | raw, plasticity off, prior_x500_candidate_v5.arc002 (832 bits) | 3 | 0.88..0.99 | 0.999..1.000 | 0.015..0.017 | 0.233..0.251 | 3.0..3.1 | - | - | - | - | 57.4..57.5 | - | 0 | 0 | 0.3..0.4 |
| wind | reflex | raw, plasticity off, prior_x500_candidate_v5.arc002 (832 bits) | 3 | 1.01..1.13 | 1.001..1.004 | 0.021..0.029 | 2.736..2.908 | 17.6 | 0.086..0.105 | 3.085..3.209 | 17.6..17.7 | 0.38..0.71 (1) | 57.2..57.4 | - | 0 | 0 | 0.3..0.7 |
