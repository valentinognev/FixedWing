# X-Plane 12 demo plant for balloon race

Date: 2026-08-21  
Status: approved (architecture 1, stay on PX4 v1.17, visual balloons + HSV, Salzburg spawn)

## Goal

Add **`--xplane`** to `./run_balloon_race.sh`: PX4 v1.17 SITL + host-installed **X-Plane 12 demo** (bind-mounted into the existing Noble container) flying Laminar **Cessna 172 SP**, with three visual-only balloons in the X-Plane world and **mss window grab** for HSV chase. Other plants unchanged.

## Non-goals

- Baking the ~25 GB demo into a Docker image or git.
- Bumping PX4 off v1.17.0 (unless a later live run proves the v1.17 + copied airframe cannot connect).
- X-Plane on the host desktop with PX4-only Docker (rejected; sim runs in the container).
- Two-container PX4 / X-Plane split.
- Defeating the demo 15-minute control lock (each race is a new flight).
- Colliding balloons, scenery DSF, or WED.
- Matching JSBSim/FG/GZ trajectories (no cross-plant RMS gate).
- Aircraft other than Cessna 172 SP.

## Decisions (locked)

- Approach **1**: one `px4-noble-sim-ros` container, bind-mount `/home/valentin/X-Plane 12`.
- PX4 **v1.17**: copy `5001_xplane_cessna172` into the container PX4 tree at runtime (and bake the same files on the next Noble rebuild). Do not `git checkout` PX4 main.
- Visual plant: balloons via `XPLMCreateInstance`; camera = mss grab of the X-Plane window (not an onboard sensor).
- Spawn: in-air over **Salzburg (LOWS)** by default (~500 m AGL). Other demo airports are an env/config override, same code path.
- `--xplane` exclusive with `--viz` / `--yasim` / `--gz`. Race default stays headless JSBSim.
- Plant id `xplane_cessna172`.

## Architecture

```text
./run_balloon_race.sh --xplane
  → kill.sh --all
  → runSimXplaneCessna.sh --mavlink-server --setup …
       docker: px4-noble-xplane-cessna
       mount:  $HOME/X-Plane 12 → /opt/xplane12
       plugins: px4xplane + fixedwing_balloons
       X-Plane-x86_64  (Cessna_172SP, new flight, LOWS, in-air)
       PX4: PX4_SIMULATOR=xplane PX4_SYS_AUTOSTART=5001
  → balloon_scene --xplane   (UDP place spheres; before HEARTBEAT)
  → wait HEARTBEAT 14540
  → control --xplane --no-sim
  → image --mode xp   (mss, window pattern X-Plane)
  → camera (HSV; park off XP window)
  → pose pane (plugin UDP → ZMQ pose, geodetic)
```

| Path | Plant | Camera | Balloons | Container |
|------|--------|--------|----------|-----------|
| JSBSim | `jsbsim_rascal` | synth | none | `px4-noble-jsbsim-rascal` |
| `--viz` | `jsbsim_rascal_viz` | FG mss | FG add-model | `px4-noble-jsbsim-rascal` |
| `--yasim` | `yasim_rascal` | FG mss | FG add-model | `px4-noble-sim-ros` |
| `--gz` | `gz_rc_cessna` | gz `race_cam` | gz models | `px4-noble-gz-plane` |
| **`--xplane`** | `xplane_cessna172` | XP mss | XPLM instances | `px4-noble-xplane-cessna` |

Same Noble image. Distinct container name. MAVLink fan-out unchanged: 14550 → 14540 / 14541.

## Components

### Host X-Plane install

- Path default `XP12_HOME=/home/valentin/X-Plane 12` (override `XP12_HOME`).
- Required: `X-Plane-x86_64`, `Aircraft/Laminar Research/Cessna 172 SP/Cessna_172SP.acf`, `Global Scenery/X-Plane 12 Demo Areas`.
- Missing any of those → runner exits 1 before `docker run`, prints the path.
- Do not copy the sim into git or the image.

### `python/scripts/runSimXplaneCessna.sh`

Mirror `runSimGzPlane.sh` contracts:

- `--name "${PX4_XP_DOCKER_NAME:-px4-noble-xplane-cessna}"`
- Image `px4-noble-sim-ros:latest`; refuse missing image (same rebuild hint as GZ).
- `--net=host --privileged --gpus all`; X11 (`DISPLAY`, `.X11-unix`, `xhost +local:docker`). Missing `DISPLAY` → fail.
- GPU: `--gpus all`; immediate failure retries without GPU only if elapsed < 30 s and not 137/143 (same helper idea as GZ). Vulkan demo is not expected to be useful on CPU; still fail loud if the window never appears.
- Bind-mounts:
  - `$XP12_HOME` → `/opt/xplane12` (rw; plugins are installed here)
  - `python/` → `/opt/fixedwing/python`
  - `python/assets/xplane` → `/opt/fixedwing/xplane`
- `--setup PATH` (default `python/flightSetup.json`): spawn NED/heading + speeds.
- `--mavlink-server` / `MAVLINK_FANOUT` (race sets 1): same sidecar/log/fail-loud as JSBSim.
- `--kill` removes container + sidecar.
- Inside the container, in order:
  1. Install Linux **px4xplane** into `/opt/xplane12/Resources/plugins/px4xplane` if missing (download pinned GitHub release zip; set `config_name = Cessna172`).
  2. Install **fixedwing_balloons** `.xpl` + OBJ into `Resources/plugins/fixedwing_balloons`.
  3. Copy `5001_xplane_cessna172` into the PX4 **build** `etc/init.d-posix/airframes/` (and source ROMFS). Patch `px4-rc.simulator` only if v1.17 does not already fall through to `px4-rc.mavlinksim`.
  4. Start X-Plane (new flight, Cessna 172 SP, LOWS, in-air pose + airspeed).
  5. Start PX4: `PX4_SIMULATOR=xplane PX4_SYS_AUTOSTART=5001` using the existing SITL binary (TCP 4560). `PX4_SIM_HOSTNAME=127.0.0.1`.
  6. Balloon plugin issues `px4xplane/toggleEnable` once X-Plane is in flight (no Plugins menu).

X-Plane args (minimum): `--no_sound`, windowed (not exclusive fullscreen so mss + `balloon_camera` can sit beside it), `--flight_is_new`. Aircraft: `Aircraft/Laminar Research/Cessna 172 SP/Cessna_172SP.acf`. Airport: `LOWS` unless `XP12_AIRPORT` is set.

### Origin / spawn

Constants (Salzburg Airport / LOWS, demo scenery that this machine already loaded):

- `XP_ORIGIN_LAT_DEG = 47.7933`
- `XP_ORIGIN_LON_DEG = 13.0044`
- `XP_GROUND_ALT_M = 430.0`
- `XP_AIRCRAFT_MSL_M = 930.0`  (ground + 500 m AGL)

`spawn.ned` / `heading_deg` in `flightSetup.json` are home-relative in this frame (same as other plants). `spawn_ic` grows `--xp-geodetic` (or equivalent) that prints lat/lon/alt/heading for the runner.

In-air IC must include forward airspeed ≈ plant `speed_mps` (first cut 40 m/s, matching PX4 Cessna trim). Pose-only spawn that stalls is a bug.

`balloon_scene.spawn_xp_from_setup` uses **live** aircraft lat/lon/MSL from the plugin if available (FG live-origin lesson); fallback is the LOWS constants. Balloon NED `z≈0` is cruise MSL, not EKF `pos_d`. Visual-only (no collision).

### `fixedwing_balloons` plugin

Small XPLM plugin (C++, Linux `lin.xpl`), UDP **49091** on localhost:

```json
{"cmd":"clear"}
{"cmd":"place","name":"balloon_255_0_0","lat":47.79,"lon":13.00,"alt_msl_m":930.0,"diameter_m":10,"rgb":[255,0,0]}
{"cmd":"sitl_connect"}
{"cmd":"pose_query"}
```

Replies:

```json
{"ok":true,"cmd":"clear"}
{"ok":true,"cmd":"pose","lat":...,"lon":...,"alt_msl_m":...,"roll_deg":...,"pitch_deg":...,"heading_deg":...,"vn_mps":...,"ve_mps":...,"vd_mps":...}
```

- `place` / `clear`: `XPLMLoadObject` + `XPLMCreateInstance` / destroy. OBJ8 spheres under the plugin folder (`balloon_sphere.obj`), tint via instance dataref or three colored OBJs. Diameter from `BalloonSpec.diameter_m`.
- `sitl_connect`: `XPLMCommandOnce("px4xplane/toggleEnable")` if not already connected.
- Pose: X-Plane datarefs (`sim/flightmodel/position/latitude` etc.). Host pose pane polls or the plugin streams at ~20 Hz after `sitl_connect`.
- Also **set aircraft lat/lon/alt/attitude/airspeed** once at start from spawn geodetic (in-air). Terrain probe: if `XPLMProbeTerrainXYZ` misses, log and refuse place (demo ocean).

Build: X-Plane SDK headers downloaded in the image or at first run; compile in-container. Sources live in this repo under `python/assets/xplane/plugin/`.

### px4xplane

- Pin a Linux release (v4.1.3 or newer with a `.xpl` / `64/lin.xpl`). Do not vendor X-Plane; vendoring the **plugin zip** in `python/bin/` (gitignored) or fetching at first run like mavlink-server is OK.
- `config_name = Cessna172` in `64/config.ini`.
- TCP **4560** to PX4 in the same netns (`--net=host`).

### Camera

- `run_balloon_image_source.py --mode xp`: reuse `fg_camera` grabbers with `camera.xp_window_pattern` default `X-Plane|X-System` (skip tiny helper windows, same size gate as FG).
- Sync view: datarefs via the balloon plugin or X-Plane commands — forward / chase-free, FOV 90°, hide cockpit if a draw-mask equivalent exists; otherwise rely on an external view offset.
- `balloon_camera` uses `fit_window_outside_rect` against the XP window (mss must not grab HighGUI).
- Skip `track_centroid_near_expected` geom-gate for `--xplane` (same as `--viz`/`--yasim`/`--gz`).

### Control / gains / pose

- `plant_id_from_flags(..., xplane=True)` → `xplane_cessna172`. Mutually exclusive with gz/yasim/viz.
- First-cut outer loop: Cessna-sized, **trim 40 m/s**, approach 28 m/s, `slow_range_m` 280, `max_roll` 0.40 rad, `visual_lock_kp_alt` on. Inner overlay starts from PX4 `5001_xplane_cessna172` (`FW_RR_*`, `FW_AIRSPD_*` 28/40/65). Live 60 s run may retune (same process as other plants); one allowed retune in the first live gate.
- `--xplane` engage: skip reboot, 60 s arm, `accept_unhealthy` like `--viz`/`--gz` (in-air attach).
- Pose: host `run_balloon_xp_pose.py` reads plugin UDP and publishes ZMQ. Control uses that geodetic converted with the same NED origin as balloon place (not GZ ENU `PoseSample` x/y/z). CSV/plots/pass use sim NED. If EKF stays within ~20 m of XP after settle, still plot both; do not use raw EKF as pass position.
- GPS/mag: the Cessna airframe **enables** sim GPS/mag. Do **not** force `SYS_HAS_MAG=0` / `EKF2_GPS_MODE=1` on this plant. If mag faults on in-air spawn (JSBSim 0.35.1 lesson), fall back to pose rebase only for `--xplane` and record it in UPDATES; do not re-enable `--ekf-fix gps` for FG.

### Teardown

- `kill.sh --xplane` / `--all`: remove `px4-noble-xplane-cessna` + `-mavlink` sidecar + host mavlink-server. `xhost -local:docker` with `--all`/`--xplane`. Never `rm` `$XP12_HOME`.

## Error handling

| Failure | Behavior |
|---------|----------|
| No `XP12_HOME` / binary / Cessna / demo scenery | exit 1 before docker, path on stderr |
| No `DISPLAY` | exit 1 |
| Image missing | exit 1, print `Dockerfiles/PX4_noble_sim_build.sh` |
| px4xplane missing after fetch | exit 1 |
| No HEARTBEAT in 120 s | dump sim pane, kill tmux, exit 1 |
| Balloon UDP timeout (90 s) | abort like FG/GZ spawn |
| Terrain probe miss | abort spawn; hint LOWS / demo tiles |
| Demo timer mid-race | not handled; races are 60 s default on a new flight |

## Testing

Unit / contract (host `cd python && python3 -m unittest …`):

- `plant_id_from_flags(xplane=True) == "xplane_cessna172"`; exclusive with gz/yasim/viz.
- Launcher: `--xplane` exclusive; spawn-before-heartbeat; `--mode xp`; `kill.sh --xplane` in `--all`.
- `ned_to_geodetic` of spawn `(0,0,0)` at LOWS origin → lat/lon ≈ 47.7933 / 13.0044, alt 930 m.
- Balloon UDP JSON roundtrip (mock socket).
- Window pattern matches `X-Plane` and rejects tiny helpers.
- `5001_xplane_cessna172` shipped under `python/assets/xplane/` contains `SYS`-relevant `FW_AIRSPD_TRIM` 40.

Live (manual, not CI):

1. `runSimXplaneCessna.sh` — XP window, Cessna airborne over Salzburg terrain (not ocean), PX4 `Simulator connected on TCP port 4560`.
2. `./run_balloon_race.sh --xplane --duration 60` — three colored balloons, `balloon_camera` HSV, CSV written. Pass count is not a merge gate for v1; retune once if needed.

## Docs

- Root `README.md`: fifth plant; `--xplane`; bind-mount; LOWS; demo limits.
- Root `UPDATES.md`: feature `subver` bump (0.36.0).
- `Dockerfiles/README.md` + `Dockerfiles/UPDATES.md` only if the Noble image bake gains the xplane airframe patch.

## Risks

| Risk | Mitigation |
|------|------------|
| v1.17 ROMFS lacks 5001 | copy airframe into build `etc` at runtime |
| px4xplane needs menu click | plugin `sitl_connect` → `px4xplane/toggleEnable` |
| In-air mag/GPS pre-arm fault | skip mag-off; if arm denied, rebase-only + UPDATES note |
| Demo scenery / ocean | LOWS default + terrain probe |
| Vulkan in Docker | GPU + X11 like FG; fail loud |
| mss grabs balloon_camera | `fit_window_outside_rect` |
| Cessna 40 m/s vs 200 m triangle | approach slowdown; first-cut gains; one live retune |
| 15 min demo lock | new flight per race; 60 s default |

## Decision log

- Install: host demo bind-mount (user has `/home/valentin/X-Plane 12`).
- Camera: full visual A (balloons in world + mss).
- Spawn: Salzburg / other demo tiles (user).
- PX4: stay v1.17, copy Cessna airframe (user 1).
- Wiring: one Noble container (user 1).
