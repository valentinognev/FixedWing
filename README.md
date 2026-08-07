# FixedWing

## Idea
PX4 fixed-wing SITL with FlightGear (Rascal) via the Noble Docker image, plus host helpers to spawn in-air and fly a straight OFFBOARD velocity hold. Headless JSBSim Rascal path for faster-than-realtime runs without FG graphics.

## Architecture
- `Dockerfiles/` — nested PX4/FlightGear/JSBSim sim image (`px4-noble-sim-ros`, PX4 v1.17, FG 2024.1.6). See that folder’s README.
- `python/runSimFlightGearRascal.sh` — host runner: Docker `--net=host`, mounts `fg_spawn.env` → `FG_ARGS_EX`.
- `python/kill.sh` — stop FG and/or JSBSim containers (`--fg` default, `--jsbsim`, `--all`).
- `python/run_straight_flight.py` — optional FG sim start + OFFBOARD **LOCAL_NED** ahead waypoint at locked Z + course velocity (same control as headless; default 30 m/s).
- `python/runSimJsbsimRascal.sh` — headless JSBSim Rascal (`HEADLESS=1 make px4_sitl jsbsim_rascal`); mounts `jsb_spawn.xml` over LSZH IC (in-air ~500 m / 1.5×Vstall). Container name `px4-noble-jsbsim-rascal` (does not clash with FG).
- `python/run_straight_flight_headless.py` — connects ASAP, OFFBOARD + arm (force if needed); **LOCAL_NED** waypoint ahead on a fixed course (heading at engage, or `--course-deg`) at **locked Z** (altitude hold) plus course velocity — required so PX4 FW enables position/TECS (velocity-only is a no-op). Speed = **1.5×Rascal Vstall** (~7.72 m/s); `--lookahead` default 300 m. After the hold ends, opens a matplotlib history of position/velocity/attitude (`--no-plot` to skip).
- `python/flight_history.py` — MAVLink sample buffer + post-flight time-history and 3D trajectory plots used by the headless runner.
- QGC: UDP 14550. Prefer `--udp 14540` for the Python scripts so they do not fight QGC for the GCS port.
- Airspeed in QGC: `VFR_HUD.airspeed`. Thrust: `VFR_HUD.throttle` / servos / `ACTUATOR_OUTPUT_STATUS`. RPM: `RAW_RPM` (FG path).

## Reading order for agents
1. Read this `README.md` (mandatory if present).
2. Read `UPDATES.md` (mandatory) for the change history and current state before working.
3. Read `Dockerfiles/README.md` before touching the image or FlightGear/PX4 glue.
