# Updates

## 0.9.0 - Python folder layout (`fw_sitl` package)
- Move utilities into `python/fw_sitl/`; shells → `python/scripts/`; spawn files → `python/assets/`; tests → `python/tests/`.
- Entrypoints stay at `python/run_straight_flight_*.py` with `fw_sitl` imports + path bootstrap; old shell paths are thin shims.
- `sim_lifecycle` exposes `PYTHON_ROOT` / `SCRIPTS_DIR`; shell scripts resolve assets under `../assets` and repo `Dockerfiles/`.

## 0.8.1 - Wrap engage retry reconnect as EngageError
- `engage_offboard_with_retries`: convert reconnect/reboot/restart failures to `EngageError` (runners keep plant hints).
- Drop unused JSBSim imports (`Path`, `DEFAULT_LOOKAHEAD_M`); remove unused `sys` from `mavlink_io`.

## 0.8.0 - Split straight-flight Python modules
- Extract `path_geometry`, `mavlink_io`, `sim_lifecycle`, `cli_common`, `straight_flight_core` from the JSBSim runner.
- Thin `run_straight_flight_{jsbsim,yasim}.py` entrypoints (CLI + plant engage policy); YASim uses shared hold loop.
- Tests import path helpers from `path_geometry`; shims unchanged.

## 0.7.6 - Fix --viz fall / early exit / missing plot
- Cause: FG delays EKF health → arm denied ~10–20 s (plane falls); late lock marked unhealthy → reboot retries → `Engage failed` → exit 1 with no plot.
- `--viz`: `accept_unhealthy=True`, single attempt (no reboot loop), softer healthy thresholds; extra arm bypasses (`COM_ARM_SDCARD/HFLT_CHK/MAG_STR=0`).
- Clearer stderr when engage fails before any hold/plot.

## 0.7.5 - Stop ~50 m NED Z jumps (GPS alt vs baro)
- Cause: EKF GPS altitude fusion vs baro height reference snapped local Z mid-hold (~50 m cliff) while XY stayed smooth.
- `EKF2_HGT_REF=0` (baro), `EKF2_GPS_CTRL=5` (lon/lat+vel, no GPS alt); softer baro gate/noise.
- After arm: `settle_path_altitude` waits for stable Z before locking `z_hold` / starting history; re-lock on |Δz|>15 m during hold.
- Path summary reports max|Δz|/|Δxy|.

## 0.7.4 - Stop mid-hold NED jumps (OFFBOARD←local-pos invalid)
- Cause: position OFFBOARD is marked lost when `local_position_invalid` flickers (PX4 `offboardCheck`), even with live setpoints → failsafe cascade to ALTCTL + EKF/NED cliffs on plots.
- Soften EKF GNSS / dead-reckon (`EKF2_GPS_*` noise/gates, `EKF2_GPS_MODE=1`, `EKF2_NOAID_TOUT` max); `COM_OF_LOSS_T=60`, `COM_OBL_RC_ACT=Hold`, fix invalid `NAV_RCL_ACT=0`.
- Hold loop: restore OFFBOARD immediately on mode leave (no spam while in 6); on >40 m NED step, re-lock path origin so setpoints do not yank after an EKF snap.

## 0.7.3 - Make JSBSim --viz flight match headless
- Root cause: `--viz` inherited FG/docker stdout into the Python process (100s of MB), stalling OFFBOARD setpoints → mode flips / EKF sawtooth on plots; `accept_unhealthy` also locked drifted EKF.
- `start_sim --viz`: log to `/tmp/jsbsim_viz_runner_<pid>.log` (never inherit); headless still DEVNULL.
- `--viz` engage: same healthy path-lock as headless (`accept_unhealthy=False`), 60 s arm wait, still no full container restart.
- FG viz patch V3: `nice -n 15`, 15 Hz FPS cap, disable clouds/sound — keep JSBSim lockstep closer to headless.

## 0.7.2 - Fix --viz black screen / 10s restart loop
- Cause: late arm (>3.5 s) marked unhealthy → `full_sim_restart` killed JSBSim+FG every ~12 s; FG never finished loading (black view).
- `--viz`: `accept_unhealthy`, 45 s arm timeout, `full_sim_restart=False` (autopilot reboot only).
- FG viz patch V2: `--timeofday=noon` (less black/night default).

## 0.7.1 - Final-review viz I/O + bridge hint
- `start_sim`: with `--viz`, inherit stdout/stderr (headless still DEVNULL).
- Viz path `jsbsim_bridge` missing check now prints same rebuild hint as headless.

## 0.7.0 - JSBSim FG viz + YASim rename
- Rename headless → `run_straight_flight_jsbsim.py` / keep `runSimJsbsimRascal.sh`; add `--viz` (FG `--fdm=null` on same JSBSim plant).
- Rename FG YASim path → `run_straight_flight_yasim.py` + `runSimYasimRascal.sh`; old names shim.
- `Dockerfiles/patch_px4_jsbsim_fg_viz.sh`: TerraSync off + FG logs for JSBSim viz launch.

## 0.6.4 - FG no longer restarts on engage
- Cause: engage treated FG as failed (`have_pos` required `|z|>5`, 12 s arm timeout), then killed the FG container and respawned.
- Fix: any `LOCAL_POSITION_NED` counts as pose; FG uses 45 s arm timeout, `accept_unhealthy`, and autopilot reboot only (`full_sim_restart=False`). Headless JSBSim still full-sim restarts.
- Engage wait logs STATUSTEXT arm/failsafe lines + periodic armed/have_pos.

## 0.6.3 - FG history plot parity + post-engage stream refresh
- `run_straight_flight.py`: same `FlightHistory` recording/plot as headless (NED pos/vel, attitude, along/cross-track, 3D); recreate + `request_streams` after engage so retries do not leave an empty plot.
- Headless: same post-engage history refresh.

## 0.6.2 - FG straight flight matches headless locked-line hold
- `run_straight_flight.py`: drop carrot-from-current-XY; reuse headless ASAP arm, path lock at arm, FW position-only closest-on-line hold, SITL param/reboot prep, engage retries, and `flight_history` along/cross plot.

## 0.6.1 - Straight-flight plot clarity + FW position-only SP
- Explain/fix plot confusion: raw `x(N)` is North, not along-track; sawtooth ↔ brief OFFBOARD→ALTCTL turns + EKF jumps.
- `flight_history`: along-track / cross-track panel + path summary; log mode flips during hold.
- FW OFFBOARD type_mask is position-only (PX4 ignores velocity on fixed-wing); `EKF2_GPS_CHECK=0` for SITL.

## 0.6.0 - Fix headless straight-flight drift
- Root causes: (1) carrot setpoint from current XY never corrected cross-track; (2) late arm / path lock after EKF wander; (3) OFFBOARD exit on position/offboard-loss failsafes.
- `run_straight_flight_headless.py`: ASAP OFFBOARD force-arm with ahead-on-yaw bridge; lock origin/course/Z at arm (course←yaw unless `--course-deg`); stream closest-on-line path + tangent; on unhealthy arm (slow / drifted EKF) restart the JSBSim container and retry; INT32-safe params + reboot; softened `COM_POS_FS_*` / `COM_OF_LOSS_T`; `jsb_spawn`/default speed ~30 m/s.
- Verified 3/3 × ~40 s holds after healthy engage: cross-track RMS ~11–27 m (was ~800 m), along-track ~825–860 m, mode 6 stable.
- `flight_history.last_att_rad`; unit tests for path projection (+ bank helpers retained).

## 0.5.1 - 3D trajectory figure
- `flight_history.plot`: second matplotlib window with 3D North-East-Up path (start/end markers).

## 0.5.0 - Post-flight history plot (headless)
- `python/flight_history.py`: buffer LOCAL_POSITION_NED + ATTITUDE; matplotlib 3-panel plot (pos/vel/attitude).
- `run_straight_flight_headless.py`: record during hold; show plot after finish (Ctrl+C or `--duration`); `--no-plot` to skip. SIGINT/SIGTERM break the loop (plot then stop sim) instead of exiting immediately.

## 0.4.7 - kill.sh at flight script start
- `run_straight_flight.py` / `run_straight_flight_headless.py`: call `kill.sh` (`--fg` / `--jsbsim`) at start of `main()` before connect/sim start (also with `--no-sim`); keep runner `--kill` in `start_sim` and atexit cleanup.

## 0.4.6 - kill.sh for sim containers
- `python/kill.sh`: remove FG (`--fg`, default), JSBSim (`--jsbsim`), or both (`--all`).

## 0.4.5 - Detach Docker from debug console TTY
- Python sim start uses `stdin=DEVNULL` so runners do not `docker run -it` on the IDE/debug TTY (rerun was typing into the live PX4 shell).
- Shell runners honor `PX4_SITL_NO_DOCKER_TTY` to force non-TTY docker.

## 0.4.4 - Kill leftover Docker before sim restart
- `run_straight_flight.py` / `run_straight_flight_headless.py`: call sim `--kill` before starting so debugger stop/rerun does not leave the previous container running.
- Register `atexit` cleanup (idempotent with SIGINT/SIGTERM handlers).

## 0.4.3 - FG straight flight uses ahead waypoint + Z hold
- `run_straight_flight.py`: drop BODY_NED velocity-only (crab + no alt hold); same LOCAL_NED ahead-waypoint + locked Z + course velocity as headless.
- Defaults: 30 m/s (matches `fg_spawn.env`), `--lookahead` 300 m, `--rate` 20 Hz, force-arm for in-air SITL.

## 0.4.2 - OFFBOARD ahead waypoint + locked altitude
- Headless: velocity-only OFFBOARD left FW in MODE_OTHER (no TECS); now stream LOCAL_NED position ahead on course at engage `z` + course velocity (`--lookahead`, default 300 m).
- Enables PX4 FW `AUTO_PATH` altitude/course hold; force-arm immediately for in-air SITL; lock `z` on first real local sample if engage was early.

## 0.4.1 - FW OFFBOARD uses fixed LOCAL_NED course
- Headless runner: stop using BODY_NED velocity (causes FW crabbing); lock course from heading at engage (or `--course-deg`).
- `MAV_CMD_DO_CHANGE_SPEED` sets airspeed target to the same `|v|`.

## 0.4.0 - Headless engage ASAP at 1.5×Vstall
- `run_straight_flight_headless.py`: no warmup by default; stream setpoints → OFFBOARD → arm (force-arm fallback) immediately after heartbeat.
- Default speed `1.5 * Rascal Vstall` (10 kt → ~7.72 m/s); `--rate` default 20 Hz.
- `jsb_spawn.xml` IC airspeed aligned to 1.5×Vstall (~25.3 ft/s).

## 0.3.1 - Prebuild jsbsim_bridge in image
- Dockerfiles 1.5.0: bake `jsbsim_bridge` into `px4-noble-sim-ros` (runtime `--rm` was discarding compile).
- `runSimJsbsimRascal.sh`: no runtime apt/build; requires rebuilt image.

## 0.3.0 - Headless JSBSim straight flight
- `python/runSimJsbsimRascal.sh` — Docker headless Rascal via `HEADLESS=1 make px4_sitl jsbsim_rascal`.
- `python/jsb_spawn.xml` — in-air IC (~500 m AGL, ~30 m/s) mounted over default LSZH scene.
- `python/run_straight_flight_headless.py` — OFFBOARD straight hold using the JSBSim runner.
- Smoke: heartbeat on UDP 14540; VFR_HUD showed ~32 m/s airspeed and airborne altitude (no FG window).

## 0.2.1 - Git repo
- Initialized root git repository (`main`); `.gitignore` present. `Dockerfiles/` remains its own nested git checkout.

## 0.2.0 - Fix airspeed; Rascal RPM path
- Bridge patch: `HIL_SENSOR.id=0` + zero-init; runner re-applies patch and rebuilds `flightgear_bridge`.
- Rascal RPM via `/engines/engine/rpm` → `RAW_RPM` on GCS.
- Verified under OFFBOARD: `VFR_HUD.airspeed` finite, `VFR_HUD.throttle` / `SERVO_OUTPUT_RAW` / `ACTUATOR_OUTPUT_STATUS` move with thrust, `RAW_RPM.frequency` non-zero. Prefer `--udp 14540` so QGC keeps 14550.

## 0.1.0 - Project bootstrap
- Host helpers under `python/` for FlightGear Rascal SITL + OFFBOARD straight flight.
