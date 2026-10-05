# FixedWing

## Idea
PX4 fixed-wing SITL testbed for OFFBOARD guidance. Primary workload is a **balloon race**: fly through a sequence of colored spheres using camera/HSV track (or synthetic imagery) and a selectable chase law. Secondary workload is **straight-flight** locked-line hold on the same plants. Plants swap under one Docker image (`px4-noble-sim-ros`, PX4 v1.17) and a shared `python/fw_sitl` package.

## Architecture

Control stack (outer → inner):

1. **Guidance** — `race_guidance` + balloon pass logic; `flightSetup.json` selects plant, duration, `cmd_mode`, controller, `homing_law`. A pass is enter 3D `pass_radius_m` then recede 2 m (`PASS_THROUGH_HYST_M`); `|cam_el| > 12°` keeps homing. `race_*` look-at/homing is HSV `dir_cam` only; off-blob is path-hold at the **next balloon's z** plus bank onto **this tick's** geometric bearing (even if HSV `last_in_view` is still true after `lookat_clears`). Origin/course refresh every off-blob tick; `z_hold` still freezes per balloon (live `204638` froze the first south heading and flew N 500→−1600 past B2). Off-blob banks **track** with chase crab (90°, GS≥2) and will not climb below recover speed. HSV drop on the same balloon reseeds z from the last camera proxy only when **co-altitude** (`|Δz|≤10 m`); an altitude step keeps balloon z (live `112813` froze ~16 m low on a shallow 80·sin(el) proxy). `lookat_clears` enters look-at on a step at |el|≥`PlantGains.lookat_el_min_rad` (default 12°) correct sign and drops below 8°. `--viz`/`--yasim` `race_*` use 16° so path-hold banks a 20–40 m step instead of slamming ±20° HSV pitch.
2. **Chase controllers** (`fw_sitl/controllers/`) — registry: `pure_pursuit_quat` (PP `a_des`→attitude→governor), `race_quat` (quaternion LOS `q_des_from_los`), `race_euler` (same LOS; in-view keeps cascade I-state under `cmd_mode=attitude`). In-view `race_*` speed follows camera LOS elevation (`CAM_LOS_ALT_PROXY_M=80` × sin(el) is path-reseed only). In-view throttle holds speed + `vz` damp, not 80·sin(el) altitude error. Look-at pitch also damps with NED `vz` only when `|el|≤8°` (steep intercepts are not fought). `race_quat` applies that D **after** the pitch LPF (`PlantGains.pitch_vz_gain`; GZ 0.08); `race_euler` still damps on `q_des` before the cascade. `bias` fades the 12° intercept to 0 as blob el → 0 (no ±12° relay at level). Off-blob search still uses caller range. In-view seeker is `guidance.homing_law` / `FW_HOMING_LAW` (see Config).
3. **Attitude cascade** — `px4_att_cascade` (Euler PID → `q_cmd` / body rates). `cmd_mode=attitude` packs quat|euler; `rates` sends body rates from φ̇θ̇ψ̇; `velocity` is locked-line TECS path setpoints.
4. **Plant gains** — `platforms/<family>/{plant_id}.jsonc` → `load_plant_gains(plant_id, controller=…)`; shared top-level + `controllers.*` blocks; PX4 `FW_*` overlay at arm.
5. **PX4 SITL** — OFFBOARD over MAVLink; optional mavlink-server fan-out.
6. **FDM** — JSBSim (headless / FG viz), YASim+FlightGear, Gazebo; X-Plane code remains in-tree but is off the race menu.

Package layout:

| Path | Role |
|------|------|
| `Dockerfiles/` | Nested PX4/FG/JSBSim/Gazebo image + patches. See that folder’s README. |
| `python/fw_sitl/` | Shared library (geometry, MAVLink, plants, race, straight flight). |
| `python/f16/` | Host-only F-16 + LQR + GCAS. Aero is `stevens` or `morelli` via `subf16_derivative(..., model=)` / `run_f16.py --aero` (default `morelli`). Aerodynamic coefficients for both models load at runtime from data/planes/f16/{morelli,stevens}.json under coefficients. Wings-level trim for a run is the bilinear lookup in data/planes/f16/{morelli,stevens}.json at the maneuver speed (straight-level 153.0096 m/s, GCAS 164.592 m/s) and the setup spawn altitude. That state is the initial condition and the LQR equilibrium. The gain matrix stays the paper K_lqr. Plant state, controls, and derivatives are SI (m, m/s, rad, N; Nz/Ny in g). units.py is the only converter. Stevens is Appendix A.8 tables plus `aero_stevens.damp_table` (radians in, indexes with `aerobench_deg`); Morelli is the polynomial only — Stevens damping is not added. AeroBench and f16dynamics stay imperial behind compare.py. CSV is NED metres. No PX4, MAVLink, ZMQ, Docker, or fw_sitl coupling. C++ parity via python/scripts/build_f16dynamics.sh; runner gcas_long is 120 s and the Python compare is 115 s (AeroBench stops below 60.96 m/s); C++ rows SKIP when unbuilt. |
| `python/plane/` | Host-only 13-state airplane. Aero is `morelli_coefficients` and `data/planes/<id>/morelli.json`. `data/planes/linear/morelli.json` is synthetic: nonlinear Morelli slots are 0. `cd python && python3 run_plane.py --plane linear --duration 1 --csv /tmp/linear.csv`. No PX4. Cessna 172 is `data/planes/cessna172` and is selected with `--plane cessna172 --model`. |
| `python/aero_convert/` | Converts sibling AID solver output into the Morelli-schema files under `data/planes/cessna172/`. |
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
| camera | `run_balloon_camera.py` (HSV track + overlay; PX4 `Arming denied` STATUSTEXT in red until armed) |
| pose | `--gz` only: `gz_pose_bridge` → ZMQ (~40 Hz mesh pose) |

**ZMQ** (`flightSetup.json` → `zmq`): `image`, `color`, `track`, `pose` (one PUB binder per endpoint).

**MAVLink**: PX4 GCS **18570** → remote **14550**; mavlink-server fans to control **14540** and image **14541**. QGC connects as UDP client to 14550. Race requires fan-out + real autopilot HEARTBEAT before spawning control/image. Straight flight leaves fan-out off by default.

Teardown: `./kill.sh --all` (or `--gz` / `--fg` / `--jsbsim` as needed).

## Plants

| Flag / `sim.platform` | Plant id | Notes |
|-----------------------|----------|--------|
| (default) `jsbsim` | `jsbsim_rascal` | Headless JSBSim Rascal; typical-spawn first circuit ~4.4 / 3.5 / 3.5 m. 5 m is not a plant bar |
| `--viz` | `jsbsim_rascal_viz` | Same FDM + FG `--fdm=null`; HSV pinhole is FG grab ~106°×90°; `lookat` 16°. Live `viz_lookat16` B0 **2.98** / B1 **3.32 m**, B2 CPA 16.6 m (spawn 533 m, co-alt; pre-FOV-fix) |
| `--yasim` | `yasim_rascal` | YASim FG Rascal; FG GT rebase; `race_quat`+`pn` 28/18, bank 0.48 rad, `lookat` 16°. Live `yas_lookat16c` B0 **8.80 m**, B1 open; `yas_conv2` B0 **3.83 m** still tighter |
| `--gz` | `gz_rc_cessna` (or `--model advanced_plane`) | Gazebo; EKF−origin_bias NED. Default Cessna. `advanced_plane` is `race_quat`+`pn` tuning (5 m first-circuit gate open) |
| (disabled) | `xplane_cessna172` | In-tree; `sim.platform=xplane` / `--xplane` exit 2 |

Straight flight: `run_straight_flight_{jsbsim,yasim,gz}.py`.

## Config

**`python/flightSetup.json`**

- `balloons[]` / `spawn` — home-relative NED (`+z` down); heading 0=N, 90=E.
- `sim.platform` — `jsbsim`\|`viz`\|`yasim`\|`gz`; `sim.gz_model`; `sim.duration_s` (`0` = no limit; CLI `--duration` overrides).
- `camera` — FOV, mount `azimuth_deg`/`elevation_deg`, resolution, FG window pattern.
- `guidance.controller` — `pure_pursuit_quat`\|`race_quat`\|`race_euler`. Shipped JSON is `race_quat` (open-loop `q_des` + cascade reset). `race_euler` stays in-tree (Euler-error close).
- `guidance.homing_law` — in-view HSV seeker (`race_*` only). `FW_HOMING_LAW` overrides JSON when set. Parser default `lookat`; shipped JSON `pn`. Pitch still ±20°.

| `homing_law` | What it does |
|--------------|----------------|
| `lookat` | Point body +X at the blob. |
| `pd_lead` | Look-at plus PD lead on az/el rates (`kd=0.35 s`). |
| `pn` | NED PN: camera LOS × attitude → λ, λ̇ in NED (not body/camera az/el), a = N V λ̇ (`N=4`, |a|≤2g, LPF τ=0.15 s), look-lead along inertial λ with **τ=0.25 s**. Needs `q_act` + `speed_mps`. |
| `bias` | Look-at plus same-sign **12°** elev intercept faded to 0 by |el|<8°; slow on steep el. Best B3 (~8 m). |
| `el_first` | Wings-level, pitch-only until abs(el) > 8°. |
| `bang` | Saturate ±20° pitch when abs(el) > 3°. Tightest B0/B1. |
| `area_slow` | Look-at; speed from blob `area_px` (ref 400 px). |
| `fpa_thrust` | Look-at plus thrust ∝ `sin(el)` (gain 0.35). |
| `filter` | LPF `dir_cam` (`τ=0.25 s`) then look-at. |
| `apn` | Same as `pn` plus +1 g upward in NED (a_z − g; not a fixed +5°). |

- `guidance.cmd_mode` — `velocity`\|`attitude`\|`rates`; `attitude_format` quat\|euler (meaningful for `attitude`).
- `guidance.laps` — `0` = cycle until duration; `N>0` = end after N circuits.
- `verification.*` — offline parity thresholds for `compare_balloon_runs.py`.

**Plant JSONC** — `python/fw_sitl/platforms/<family>/{plant_id}.jsonc`. Family from plant-id prefix (`jsbsim_`, `yasim_`, `gz_`, `xplane_`). Top-level: airspeeds, lookahead, `px4_inner`. Per-controller outer gains under `controllers.<id>`; PP-only aero/governor keys may be inherited by `race_*` from sibling `pure_pursuit_quat`. GZ Cessna `race_quat` shares the 0.65/14 center-through outer set with `race_euler` (LOS pitch 20° / 0.35 rad, `kp_elev` 1.5, approach 8 m/s, `slow_range` 280 m) except `pitch_vz_gain` 0.08 vs default 0.03. `gz_advanced_plane` uses the same outer structure at 20 m/s trim (`cruise_thrust` 0.5, `speed_mps` 20). JSBSim Rascal `race_quat` uses that outer set at 18 m/s (`approach` 16 / `slow_range` 140 / thrust 0.62); `--viz` same FDM with `pitch_vz_gain` 0.05 (GT vz) and `lookat_el_min_rad` 16°. YASim at 28 m/s (`approach` 18 / `slow_range` 280 / thrust 0.63 / `min_thrust` 0.40 / bank 0.48 rad / `pitch_vz_gain` 0.04 / `lookat_el_min_rad` 16°).

Code defaults for missing keys: controller `pure_pursuit_quat`, homing_law `lookat`. Checked-in `flightSetup.json` ships `sim.platform` `jsbsim`, `race_quat` + `pn` + `attitude_format` euler.

## Frames

- **NED** — balloons, spawn, chase targets, CSV/plots (`+z` down; race balloons at local `z≈0` cruise). `pn`/`apn` differentiate λ here.
- **Body FRD** — lookat/bias/bang close camera LOS vs body +X (HSV `dir_cam` → mount). Off-blob search path-holds the next balloon's z and banks onto geometric bearing.
- **Camera** — OpenCV optical (+Z boresight); mount azimuth+/elevation+ vs body FRD (`camera_model.py`).
- **`--viz`/`--yasim`** — PX4 EKF dead-reckons and drifts; guidance rebases from FG telnet GT (`--ekf-fix gps` disabled / exit 2). HSV uses the FG grab pinhole: FG `field-of-view` is vertical (`camera.hfov_deg` 90° written there) so the 4:3 crop is **HFOV≈106.3° / VFOV=90°**, not synth 90×70. Geometric UV is from the eyepoint (`fg_eye_forward_m`, default 5 m body +X), not the CG.
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

cd python && python3 run_f16.py          # GCAS upright; bad setup exits 2; NED-metre CSV; --anim / --fg only after the CSV is on disk
cd python && python3 run_f16.py --aero stevens   # same runner; Appendix A.8 aero instead of Morelli polynomials
./run_f16_straight_level.sh  # host-only straight_level (3.0 s); --csv / --duration optional; extra args to run_f16.py
./run_f16_gcas_upright.sh  # host-only gcas_upright (3.51 s); --csv / --duration optional; extra args to run_f16.py
./run_f16_gcas_inverted.sh  # host-only gcas_inverted (10.0 s); --csv / --duration optional; extra args to run_f16.py
./run_f16_gcas_long.sh  # host-only gcas_long (120.0 s); --csv / --duration optional; extra args to run_f16.py
./run_trim_survey.sh  # wings-level trim grid; default plane f16, model morelli; --plane / --model
./python/scripts/run_f16_reference_e2e.sh  # opt-in AeroBench parity; C++ f16dynamics skips when unbuilt
cd python && python3 run_plane.py --plane linear --duration 1 --csv /tmp/linear.csv  # host-only Morelli 6DOF; nonlinear slots zeroed
cd python && python3 run_plane.py --plane cessna172 --model tornado --duration 5 --csv /tmp/c172.csv
cd python && python3 -m unittest discover -s tests
cd python && MPLBACKEND=Agg python3 -m unittest tests.test_plane_cessna172_json -v  # c172 aero acceptance suite; signs + tornado magnitudes + static margin + trim + modes; flow5 cm[1] is the declared 4.9 % band outlier, asserted against provenance.invariant_band_note at 5 %
FW_SITL_E2E=1 ./python/scripts/run_race_quat_e2e.sh   # opt-in live SITL (50 m smoke course)
FW_SITL_E2E=1 ./python/scripts/run_race_quat_production_e2e.sh  # production 10 m course, typical-spawn first circuit
FW_SITL_E2E=1 ./python/scripts/run_race_euler_e2e.sh  # GZ race_euler, production 10 m course
```

Plant for SID comes from `flightSetup.json`; override with `--gz`/`--yasim`/`--viz`/`--jsbsim`/`--model`. `--waveform sine` is **instead of** chirp (60 s tone per axis from `procedure.json`). Artifacts in `/tmp/fw_calib_<stamp>/` (`_history` / `_step` always, `_fft` unless the log is too short). Interactive matplotlib unless `--no-plot`. Envelope abort recaptures and retries that axis (3 attempts) then skips.

Full `unittest discover` blocks on race-plot `plt.show(block=True)`. Calibration tests: `MPLBACKEND=Agg python3 -m unittest tests.test_calibration_* tests.test_flight_history`.

## Known limits

- `python/plane/` is host-only open-loop Morelli 6DOF (`run_plane.py`). The shipped `linear` airplane zeros every nonlinear Morelli slot. No PX4, MAVLink, Docker, or FlightGear.
- F-16 GCAS is host-only (`cd python && python3 run_f16.py`). The runner starts upright, inverted, and long GCAS from the same wings-level trim row (540 ft/s at the spawn altitude), so those runs do not use the AeroBench dive attitudes. AeroBench trajectory parity still integrates paper_x0, the historical benchmark state. Parity horizons are 3.51 s upright and 10 s inverted. Unittest compares our plant to the Python reference. The in-tree ODE is SI. Parity converts at compare.py. Altitude gate is 15.24 m. Final RMS stays 5.0 on the imperial-equivalent residual. `f16dynamics` skips when unbuilt. JSBSim spawn vt is 29.99232 m/s. FG --vc stays 58.3 knots. FGNetFDM speeds stay ft/s. Balloon elevation-ft stays feet.
- C++ parity via `python/scripts/build_f16dynamics.sh`. Runner `gcas_long` is 120 s; the Python compare is 115 s. C++ rows SKIP when unbuilt. AeroBench run_f16_sim raises below 60.96 m/s (vt about 60.8 m/s near t = 118 s).
- `jsbsim_rascal` `race_quat`+`pn` headless, **typical spawn** (~310 m to B0, |D|≲80 m): 120 s `/tmp/balloon_race_20260904_220720.csv` first circuit **4.44 / 3.47 / 3.46 m** (lap-2 B2 **6.63 m**). 200 s `/tmp/balloon_race_20260904_220218.csv` first circuit **5.22 / 6.60 / 3.72 m**. 5 m first-circuit is **not** a plant achievement. `/tmp/balloon_race_jsb_baseline.csv` **2.67 / 1.44 / 2.22 m** was a 629 m / D≈241 m intercept and is retracted. Production e2e `223744` spawn 696 m / D=303, B0 1.32 m incomplete — honesty gate failed (not published). `--viz` `lookat_el_min` 16° `/tmp/balloon_race_viz_lookat16.csv`: first circuit B0 **2.98** / B1 **3.32 m**, B2 CPA **16.6 m** no pass (spawn 533 m / co-alt D≈263; vz flips **6**/120 s). Prior `viz_conv` 6.89 / 1.98 no B2; user `221031` B0 **7.49 m** only. `yasim_rascal` `lookat` 16° live `yas_lookat16c` B0 **8.80 m** t=91, B1 CPA 150 m, vz flips 33. Earlier `yas_conv2` B0 **3.83 m** (vz 32) still the tightest B0; B1 zoom still open. Tune2 `/tmp/balloon_race_yas_tune2.csv` B0 **3.92 m**, B1 zoom-climb CPA **30 m**. Production e2e: `FW_SITL_E2E=1 ./python/scripts/run_race_quat_production_e2e.sh` (first three passes, typical-spawn gate, no 5 m miss claim).
- `race_quat` + `px4_att_cascade`: residual miss is systematically **high** (NED +z down; negative ΔD = above the balloon). Live GZ NED PN 2026-09-04 `/tmp/balloon_race_hyst.csv` (+ `.pkl`): first circuit **2.91 / 0.20 / 4.62 m** (B0 XY 0.22, ΔD −2.91, t=49.0; B1 XY 0.20, ΔD +0.02, t=66.2; B2 XY 1.04, ΔD −4.50, t=80.8; gate ≤5 m). Matches end-of-tune `vz_tune2` 3.23 / 0.86 / 4.23 and `111023` 2.60 / 0.35 / 3.45. Pre-fix `112813` B1 **4.20 m under** (stuck z≈16 m on camera proxy). 200 s lap-2 B2 5.84 m still high; 20 Hz vz sign-flips **71 / 120 s** vs `vz_tune2` **15**. Descending B0/B2 still high; B1 climb is the accurate one. Next lever: shared `q_des_from_los` skips load-pitch on down-LOS.
- `gz_advanced_plane` `--gz --model advanced_plane` `race_quat`+`pn`: live `204638` B0 ~0.5 m / B1 ~1 m / B2 never (HSV miss; frozen south path-hold). Pre-fix energy races (tune1 `/tmp/balloon_race_adv_tune1.csv`) B0 **0.57 m** / B1 **1.02 m** / B2 heading fly-by CPA **86.7 m**. 5 m first-circuit gate still open pending a live re-run after 0.71.1.
- `race_euler` remains in-tree as the Euler-error close (`pn_ned_001310` 5.23 / 0.60 / 2.80 m). Older `bias` B0/B1 **&lt;2 m**, B3 **~8.5 m**. 0.63 body-frame `pn` missed balloon 0.
- X-Plane plant is residual (code/tests present; race menu disabled).
- FG/YASim: EKF is not ground truth — always use GT rebase for chase/plots.
- OpenCV ≥5 (some conda envs) blacks out `balloon_camera`; race launcher prefers an `opencv-python<5` env when needed.
- Calibration live `gt`/`px4` are the same MAVLink ATTITUDE sample, not FDM truth. `accel_z`/`vel_z` live GT is `nan` (`no_data`) — use `--dry-run` for those layers.
- `--layer rates` commands body rates (little visible Euler wobble). Use `--layer attitude` to see the aircraft pitch/roll/yaw.

## Reading order for agents
1. Read this `README.md` (mandatory if present).
2. Read `UPDATES.md` (mandatory) for recent change history before working.
3. Read `Dockerfiles/README.md` before touching the image or FlightGear/PX4 glue.
