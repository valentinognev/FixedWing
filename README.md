# FixedWing

## Idea
PX4 fixed-wing SITL (Rascal) via the Noble Docker image: JSBSim plant (default headless; optional FG `--fdm=null` viz) and YASim FlightGear plant, plus host helpers for in-air spawn and OFFBOARD locked-line hold.

## Architecture
- `Dockerfiles/` — nested PX4/FlightGear/JSBSim sim image (`px4-noble-sim-ros`, PX4 v1.17, FG 2024.1.6). See that folder’s README.
- `python/run_straight_flight_jsbsim.py` — JSBSim plant OFFBOARD locked-line hold; optional `--viz` (FG window, same FDM). Reboot-applied SITL params → ASAP force-arm → lock origin/course/Z → closest-on-line LOCAL_NED path. Softened failsafes; unhealthy engage → full JSBSim restart + retry (keeps `--viz`). Default ~30 m/s, `--lookahead` 500 m. Plot after hold (`--no-plot` to skip). Old name `run_straight_flight_headless.py` is a thin shim.
- `python/run_straight_flight_yasim.py` — YASim FlightGear plant: same locked-line hold / `flight_history` plot as JSBSim runner. Imports shared core from `run_straight_flight_jsbsim`. Old name `run_straight_flight.py` is a thin shim.
- `python/runSimJsbsimRascal.sh` — JSBSim Rascal SITL; default headless (`HEADLESS=1`); `--viz` mounts `Dockerfiles/patch_px4_jsbsim_fg_viz.sh`, unsets HEADLESS, X11 so FG shows `--fdm=null`. Mounts `jsb_spawn.xml` over LSZH IC (~500 m AGL / ~30 m/s). Container `px4-noble-jsbsim-rascal`.
- `python/runSimYasimRascal.sh` — YASim FlightGear FDM Rascal: Docker `--net=host`, mounts `fg_spawn.env` → `FG_ARGS_EX`. Old name `runSimFlightGearRascal.sh` is a thin exec shim.
- `python/kill.sh` — stop YASim/FG and/or JSBSim containers (`--fg` default, `--jsbsim`, `--all`).
- `python/flight_history.py` — shared MAVLink sample buffer + post-flight time-history (NED/along-cross/vel/attitude) and 3D trajectory plots used by both runners.
- QGC: UDP 14550. Prefer `--udp 14540` for the Python scripts so they do not fight QGC for the GCS port.
- Airspeed in QGC: `VFR_HUD.airspeed`. Thrust: `VFR_HUD.throttle` / servos / `ACTUATOR_OUTPUT_STATUS`. RPM: `RAW_RPM` (YASim FG path).

## Reading order for agents
1. Read this `README.md` (mandatory if present).
2. Read `UPDATES.md` (mandatory) for the change history and current state before working.
3. Read `Dockerfiles/README.md` before touching the image or FlightGear/PX4 glue.
