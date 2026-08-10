# FixedWing

## Idea
PX4 fixed-wing SITL (Rascal) via the Noble Docker image: JSBSim plant (default headless; optional FG `--fdm=null` viz) and YASim FlightGear plant, plus host helpers for in-air spawn and OFFBOARD locked-line hold.

## Architecture
- `Dockerfiles/` — nested PX4/FlightGear/JSBSim sim image (`px4-noble-sim-ros`, PX4 v1.17, FG 2024.1.6). See that folder’s README.
- `python/fw_sitl/` — shared package: `path_geometry`, `mavlink_io`, `sim_lifecycle`, `cli_common`, `straight_flight_core`, `flight_history`.
- `python/run_straight_flight_jsbsim.py` / `run_straight_flight_yasim.py` — thin plant entrypoints (CLI + engage policy); YASim uses shared hold. JSBSim optional `--viz`. Softened failsafes; plant-specific engage retries. Default ~30 m/s, `--lookahead` 500 m. Plot after hold (`--no-plot` to skip). Old names `run_straight_flight_headless.py` / `run_straight_flight.py` are thin shims.
- `python/scripts/` — `runSimJsbsimRascal.sh` (headless default; `--viz` → FG `--fdm=null`), `runSimYasimRascal.sh`, `kill.sh`. Compat shims remain at `python/runSim*.sh` / `python/kill.sh`.
- `python/assets/` — `jsb_spawn.xml`, `fg_spawn.env` (mounted by sim scripts).
- `python/tests/` — unit tests (`test_path_setpoint.py`).
- QGC: UDP 14550. Prefer `--udp 14540` for the Python scripts so they do not fight QGC for the GCS port.
- Airspeed in QGC: `VFR_HUD.airspeed`. Thrust: `VFR_HUD.throttle` / servos / `ACTUATOR_OUTPUT_STATUS`. RPM: `RAW_RPM` (YASim FG path).

## Reading order for agents
1. Read this `README.md` (mandatory if present).
2. Read `UPDATES.md` (mandatory) for the change history and current state before working.
3. Read `Dockerfiles/README.md` before touching the image or FlightGear/PX4 glue.
