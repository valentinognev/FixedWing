# Updates

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
