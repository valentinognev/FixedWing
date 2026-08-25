# FixedWing

## Idea
PX4 fixed-wing SITL testbed for OFFBOARD guidance. Primary workload is a **balloon race**: fly through a sequence of colored spheres using camera/HSV track (or synthetic imagery) and a selectable chase law. Secondary workload is **straight-flight** locked-line hold on the same plants. Plants swap under one Docker image (`px4-noble-sim-ros`, PX4 v1.17) and a shared `python/fw_sitl` package.

## Architecture

Control stack (outer → inner):

1. **Guidance** — `race_guidance` + balloon pass logic; `flightSetup.json` selects plant, duration, `cmd_mode`, controller, `homing_law`. A pass is enter 3D `pass_radius_m` then recede 2 m (`PASS_THROUGH_HYST_M`); `|cam_el| > 12°` keeps homing. `race_*` look-at/homing is HSV `dir_cam` only; off-blob is path-hold at current altitude plus bank onto geometric bearing.
2. **Chase controllers** (`fw_sitl/controllers/`) — registry: `pure_pursuit_quat` (PP `a_des`→attitude→governor), `race_quat` (quaternion LOS `q_des_from_los`), `race_euler` (same LOS; in-view keeps cascade I-state under `cmd_mode=attitude`). In-view `race_*` speed/altitude follow camera LOS elevation only (`CAM_LOS_ALT_PROXY_M=80` × sin(el); no NED range). Off-blob search still uses caller range. In-view seeker is `guidance.homing_law` / `FW_HOMING_LAW` (see Config). Path-hold freezes z at search start; HSV drop reseeds z from the last camera proxy.
3. **Attitude cascade** — `px4_att_cascade` (Euler PID → `q_cmd` / body rates). `cmd_mode=attitude` packs quat|euler; `rates` sends body rates from φ̇θ̇ψ̇; `velocity` is locked-line TECS path setpoints.
4. **Plant gains** — `platforms/<family>/{plant_id}.jsonc` → `load_plant_gains(plant_id, controller=…)`; shared top-level + `controllers.*` blocks; PX4 `FW_*` overlay at arm.
5. **PX4 SITL** — OFFBOARD over MAVLink; optional mavlink-server fan-out.
6. **FDM** — JSBSim (headless / FG viz), YASim+FlightGear, Gazebo; X-Plane code remains in-tree but is off the race menu.

Package layout:

| Path | Role |
|------|------|
| `Dockerfiles/` | Nested PX4/FG/JSBSim/Gazebo image + patches. See that folder’s README. |
| `python/fw_sitl/` | Shared library (geometry, MAVLink, plants, race, straight flight). |
| `python/fw_sitl/platforms/{jsbsim,yasim,gz,xplane}/` | Plant JSONC + backend camera/pose/overlay glue. |
| `python/fw_sitl/controllers/` | Selectable outer chase laws. |
| `python/controlCallibration/` | Host SID. `--waveform chirp\|sine` (default chirp; sine is instead of chirp). Procedure in `procedure.json` (`sine_phases` 3/60/2 s; per-axis `f0_hz`/`f1_hz`/`f_sine_hz` on rates and attitude — rates `f1_hz` p 8 / q 2 / r 1.2 Hz; attitude `f1_hz` roll 4 / pitch 1.5 / yaw 1.2 Hz; `window_s` q/pitch 2.0, p/r 0.5, roll/yaw 1.0; `max_angle_deg` roll 10° / pitch 12° / yaw 40° bounce-at-wall on `--layer rates` except channel `r` (heading not in envelope); `start_angle_deg` 5° wings-level gate; `hold_initial_timeout_s` 90 s recover-from-dive before the first axis, then p/q/r all overlay; q/r amplitude 0.08). Plant from `flightSetup.json` (`--gz`/`--yasim`/`--viz`/`--jsbsim`/`--model`). GZ Cessna one-click `FW_PR_P` 0.50 / `FW_YR_P` 0.45. History/FFT/step PNG + `hints.json`. Verdict uses mean-step `curve_peak` not window `peak_mean`. Envelope abort recaptures and retries that axis (3 attempts) then skips. Live rates/attitude overlay do not abort on Δalt (TECS off; climb expected). Live attitude `gt` is unwrapped onto `cmd`'s 2π branch (PX4 ATTITUDE yaw wraps). Empty skip → no CSV/plots. Does not write plant JSONC. See `UPDATES.md` 0.54.2. |
| `run_control_calibration.sh` | Root shim → `python -m controlCallibration run`. |
| `python/flightSetup.json` | Balloon-race config (JSONC); also default SID plant. |
| `python/scripts/` | Sim launchers, race orchestrator, kill, fetch helpers. |
| `python/tests/` | Unit tests (host: `cd python && python3 -m unittest discover -s tests`). |

## Runtime (balloon race)

`./run_balloon_race.sh` → tmux session `balloon_race`:

| Pane | Process |
|------|---------|
| sim | `runSim{JsbsimRascal,YasimRascal,GzPlane}.sh` in Docker |
| control | `run_balloon_control.py --udp 14540` |
| image | `run_balloon_image_source.py --udp 14541` (`synth`\|`fg`\|`gz`) |
| camera | `run_balloon_camera.py` (HSV track + overlay) |
| pose | `--gz` only: `gz_pose_bridge` → ZMQ (~40 Hz mesh pose) |

**ZMQ** (`flightSetup.json` → `zmq`): `image`, `color`, `track`, `pose` (one PUB binder per endpoint).

**MAVLink**: PX4 GCS **18570** → remote **14550**; mavlink-server fans to control **14540** and image **14541**. QGC connects as UDP client to 14550. Race requires fan-out + real autopilot HEARTBEAT before spawning control/image. Straight flight leaves fan-out off by default.

Teardown: `./kill.sh --all` (or `--gz` / `--fg` / `--jsbsim` as needed).

## Plants

| Flag / `sim.platform` | Plant id | Notes |
|-----------------------|----------|--------|
| (default) `jsbsim` | `jsbsim_rascal` | Headless JSBSim Rascal; primary tuning target |
| `--viz` | `jsbsim_rascal_viz` | Same FDM + FG `--fdm=null` window; FG GT rebase |
| `--yasim` | `yasim_rascal` | YASim FG Rascal; FG GT rebase |
| `--gz` | `gz_rc_cessna` (or `--model advanced_plane`) | Gazebo; EKF−origin_bias NED |
| (disabled) | `xplane_cessna172` | In-tree; `sim.platform=xplane` / `--xplane` exit 2 |

Straight flight: `run_straight_flight_{jsbsim,yasim,gz}.py`.

## Config

**`python/flightSetup.json`**

- `balloons[]` / `spawn` — home-relative NED (`+z` down); heading 0=N, 90=E.
- `sim.platform` — `jsbsim`\|`viz`\|`yasim`\|`gz`; `sim.gz_model`; `sim.duration_s` (`0` = no limit; CLI `--duration` overrides).
- `camera` — FOV, mount `azimuth_deg`/`elevation_deg`, resolution, FG window pattern.
- `guidance.controller` — `pure_pursuit_quat`\|`race_quat`\|`race_euler`.
- `guidance.homing_law` — in-view HSV seeker (`race_*` only). `FW_HOMING_LAW` overrides JSON when set. Parser default `lookat`; shipped JSON `bias`. Pitch still ±20°.

| `homing_law` | What it does |
|--------------|----------------|
| `lookat` | Point body +X at the blob. |
| `pd_lead` | Look-at plus PD lead on az/el rates (`kd=0.35 s`). |
| `pn` | Rate-only PNG (`N=4`); ignores boresight. Misses balloon 0 in GZ. |
| `bias` | Look-at plus same-sign **12°** elev intercept; slow on steep el. Best B3 (~8 m). |
| `el_first` | Wings-level, pitch-only until abs(el) > 8°. |
| `bang` | Saturate ±20° pitch when abs(el) > 3°. Tightest B0/B1. |
| `area_slow` | Look-at; speed from blob `area_px` (ref 400 px). |
| `fpa_thrust` | Look-at plus thrust ∝ `sin(el)` (gain 0.35). |
| `filter` | LPF `dir_cam` (`τ=0.25 s`) then look-at. |
| `apn` | PN plus +5° nose-up. Misses balloon 0 in GZ. |

- `guidance.cmd_mode` — `velocity`\|`attitude`\|`rates`; `attitude_format` quat\|euler (meaningful for `attitude`).
- `guidance.laps` — `0` = cycle until duration; `N>0` = end after N circuits.
- `verification.*` — offline parity thresholds for `compare_balloon_runs.py`.

**Plant JSONC** — `python/fw_sitl/platforms/<family>/{plant_id}.jsonc`. Family from plant-id prefix (`jsbsim_`, `yasim_`, `gz_`, `xplane_`). Top-level: airspeeds, lookahead, `px4_inner`. Per-controller outer gains under `controllers.<id>`; PP-only aero/governor keys may be inherited by `race_*` from sibling `pure_pursuit_quat`.

Code defaults for missing keys: controller `pure_pursuit_quat`, homing_law `lookat`. Check the checked-in `flightSetup.json` for the active race defaults (may differ).

## Frames

- **NED** — balloons, spawn, chase targets, CSV/plots (`+z` down; race balloons at local `z≈0` cruise).
- **Body FRD** — in-view homing closes camera LOS vs body +X (HSV `dir_cam` → mount). Off-blob search uses path-hold, not geometric elevation.
- **Camera** — OpenCV optical (+Z boresight); mount azimuth+/elevation+ vs body FRD (`camera_model.py`).
- **`--viz`/`--yasim`** — PX4 EKF dead-reckons and drifts; guidance rebases from FG telnet GT (`--ekf-fix gps` disabled / exit 2).
- **`--gz`** — race NED = EKF − constant origin bias locked from first good mesh pose (ZMQ pose stream, not per-tick `docker exec` poll).

## Run

```bash
./run_balloon_race.sh                  # headless JSBSim (or sim.platform in setup)
./run_balloon_race.sh --gz
./run_balloon_race.sh --yasim
./run_balloon_race.sh --viz
./run_balloon_race.sh --duration 0     # no time limit
./kill.sh --all

./run_control_calibration.sh --layer rates
./run_control_calibration.sh --layer attitude --gz
./run_control_calibration.sh --layer rates --gz --waveform sine
./run_control_calibration.sh --layer attitude --gz --waveform sine
./run_control_calibration.sh --layer accel_z --inject pitch --dry-run --no-plot

cd python && python3 -m unittest discover -s tests
FW_SITL_E2E=1 ./python/scripts/run_race_quat_e2e.sh   # opt-in live SITL
FW_SITL_E2E=1 ./python/scripts/run_race_euler_e2e.sh  # GZ race_euler, production 10 m course
```

Plant for SID comes from `flightSetup.json`; override with `--gz`/`--yasim`/`--viz`/`--jsbsim`/`--model`. `--waveform sine` is **instead of** chirp (60 s tone per axis from `procedure.json`). Artifacts in `/tmp/fw_calib_<stamp>/` (`_history` / `_step` always, `_fft` unless the log is too short). Interactive matplotlib unless `--no-plot`. Envelope abort recaptures and retries that axis (3 attempts) then skips.

Full `unittest discover` blocks on race-plot `plt.show(block=True)`. Calibration tests: `MPLBACKEND=Agg python3 -m unittest tests.test_calibration_* tests.test_flight_history`.

## Known limits

- `race_euler` + `px4_att_cascade`: GZ `./run_balloon_race.sh --gz` with `FW_HOMING_LAW=bias` hits balloon 0/1 at **&lt;2 m** 3D; balloon 3 stays **~8.5 m high** (20° pitch cap; HSV drops at cam_el≈−32°). Ten-law sweep: `bias`/`bang` best; `pn`/`apn` miss balloon 0. Closest three-pass balance (`012817` 12° bias + area-slow): **5.92 / 5.39 / 5.88 m**, just outside 5 m. Load-pitch on down-LOS is skipped in shared `q_des_from_los`.
- X-Plane plant is residual (code/tests present; race menu disabled).
- FG/YASim: EKF is not ground truth — always use GT rebase for chase/plots.
- OpenCV ≥5 (some conda envs) blacks out `balloon_camera`; race launcher prefers an `opencv-python<5` env when needed.
- Calibration live `gt`/`px4` are the same MAVLink ATTITUDE sample, not FDM truth. `accel_z`/`vel_z` live GT is `nan` (`no_data`) — use `--dry-run` for those layers.
- `--layer rates` commands body rates (little visible Euler wobble). Use `--layer attitude` to see the aircraft pitch/roll/yaw.

## Reading order for agents
1. Read this `README.md` (mandatory if present).
2. Read `UPDATES.md` (mandatory) for recent change history before working.
3. Read `Dockerfiles/README.md` before touching the image or FlightGear/PX4 glue.
