# Gazebo PX4 plane plant for balloon race

Date: 2026-08-13  
Status: approved for implementation after user review of this file

## Goal

Add a **Gazebo Jetty + PX4 v1.17 plane** plant beside JSBSim Rascal. Balloon race can fly that plant with balloons in the Gazebo world, an **onboard camera sensor** feeding the existing ZMQ tracker, and the Gazebo GUI for the operator. Rascal (JSBSim default, YASim leftover) stays unchanged.

## Non-goals

- Gazebo GUI window grab for tracker pixels.
- Host-side `gz.transport` Python (Jetty bindings stay in the container).
- Runway takeoff / climb-then-race.
- Plant ABC / shared backend refactor.
- Spawning Gazebo balloons on the JSBSim/FG path.
- Matching JSBSim vs Gazebo trajectories (no RMS gate across plants).
- Changing PX4 airframe IDs (`4003` / `4008`).

## Decisions (locked)

- Approach **A**: additive plant (new sim runner + race `--gz`).
- Tracker: gz camera sensor → ZMQ `ImageFrame`. GUI is human-only.
- Spawn: in-air ~500 m AGL, heading north, with **initial forward airspeed** (pose-only spawn is rejected).
- Default model `gz_rc_cessna` (`PX4_SYS_AUTOSTART=4003`); `--model advanced_plane` → `gz_advanced_plane` (`4008`).
- Race default remains JSBSim + synth. `--gz` is opt-in. `--gz` and `--viz` are mutually exclusive; `--model` without `--gz` is invalid (exit 2, one-line error).

## Architecture

```text
[JSBSim default — unchanged]
  ./run_balloon_race.sh
    → runSimJsbsimRascal.sh --mavlink-server
    → image --mode synth

[JSBSim + FG — unchanged]
  ./run_balloon_race.sh --viz
    → runSimJsbsimRascal.sh --mavlink-server --viz
    → image --mode fg + FG telnet balloons

[Gazebo plane]
  ./run_balloon_race.sh --gz [--model advanced_plane]
    → runSimGzPlane.sh --mavlink-server [--model …]
    → PX4 SITL + gz-sim GUI + overlay camera + initial V
    → image --mode gz  (docker exec in-container bridge → ZMQ image PUB)
    → camera unchanged (SUB image, PUB track)
    → control --no-sim --gz --spawn-gz-balloons
```

| Path | Plant | Viz | Camera pixels | Balloons | Container |
|------|--------|-----|---------------|----------|-----------|
| JSBSim | `jsbsim_rascal` | none or FG `--fdm=null` | synth or FG grab | FG `geo.put_model` | `px4-noble-jsbsim-rascal` |
| YASim | `flightgear_rascal` | FG FDM | n/a (straight flight) | n/a | `px4-noble-sim-ros` |
| **Gazebo** | `gz_rc_cessna` / `gz_advanced_plane` | gz-sim GUI | onboard sensor | gz create/remove | `px4-noble-gz-plane` |

Same Noble image (`px4-noble-sim-ros`). Distinct container name so JSBSim and Gazebo can coexist.

MAVLink fan-out unchanged: GCS `14550` → control `14540` + image-source `14541`. Gazebo sim default `MAVLINK_FANOUT=0`; race sets `1`.

## Components

### `python/scripts/runSimGzPlane.sh`

Docker run of `px4-noble-sim-ros`:

- `--name "${PX4_GZ_DOCKER_NAME:-px4-noble-gz-plane}"`
- `--net=host --privileged --gpus all`
- X11 like JSBSim `--viz` (`DISPLAY`, `.X11-unix`, `xhost +local:docker`). Missing `DISPLAY` → fail (not warn-and-continue).
- NVIDIA: pass `--gpus all`. If `docker run` fails, print rebuild/toolkit hint and exit non-zero.
- Bind-mount:
  - `python/assets/gz` → `/opt/fixedwing/gz` (models + generated camera SDF)
  - `python/` → `/opt/fixedwing/python` (bridge + spawn helpers run in-container)
- Resource path: prepend `/opt/fixedwing/gz/models` on `GZ_SIM_RESOURCE_PATH` so overlay `rc_cessna` / `advanced_plane` win over stock PX4-gazebo-models.
- Launch: `PX4_GZ_WORLD=default`, `PX4_GZ_MODEL_POSE="${POSE}"` with default `0,0,500,0,0,1.570796` (ENU metres / rad; 500 m AGL, yaw +90° = heading **north** so balloon 0 at NED +N is ahead). Prefer env that `sitl_run.sh` already honors; do not fork PX4 make.
- `--model rc_cessna|advanced_plane` (default `rc_cessna`) → `make px4_sitl gz_rc_cessna` or `gz_advanced_plane`.
- `--setup PATH` (default `python/flightSetup.json`): camera size/HFOV and `guidance.speed_mps` for overlay. Race `--gz` passes the same `--setup` it already gives image/control.
- `--mavlink-server` / `--no-mavlink-server` / `MAVLINK_FANOUT`: copy JSBSim contract (fail loud, host fetch, sidecar name `${CONTAINER}-mavlink`, log `/tmp/mavlink-server-fanout.log`).
- `--kill` removes container + sidecar.
- `--help` documents flags.

Before `make`, run a small in-container overlay step: copy stock model SDF from the PX4 checkout, inject camera + initial velocity (see Overlay). Do not vendor full meshes.

### Overlay SDF (camera + airspeed)

Runtime, from stock `Tools/simulation/gz/models/{rc_cessna,advanced_plane}/model.sdf`:

1. **Camera** named `race_cam` on a `camera_link` offset **+X body / forward** by `camera.fg_eye_forward_m` (reuse that field; default 5 m) so the fuselage is out of frame. Image `width`/`height` and `<horizontal_fov>` **must equal** `CameraSpec` (default 640×480, HFOV 90°). Gazebo derives VFOV from HFOV × aspect; do not crop/resize to force `vfov_deg`. Tracker LOS still uses `CameraSpec` including `vfov_deg` (same as synth/FG). Small VFOV mismatch (~70° config vs ~74° from 4:3 + 90° HFOV) is accepted.
2. **Initial linear velocity** `guidance.speed_mps` (default 30 m/s) along the model’s body +X in world frame at spawn yaw (default heading north ⇒ world `vy=+30`). Pose-only spawn is a bug. Implementation may inject SDF velocity and/or set the canonical link velocity immediately after spawn via gz-sim APIs. **Acceptance:** control `--gz` aborts before engage if `VFR_HUD.airspeed` (else groundspeed) stays &lt; 15 m/s for 5 s after MAVLink connect (“in-air spawn has no airspeed”). Overlay is still required so the plane is not already stalled when control attaches.
3. Keep stock visual/collision/plugin so PX4 `SIM_GZ_*` bindings still attach.

Camera topic is discovered by sensor name `race_cam` (do not hard-code a full `/world/...` path beyond world name `default` + model name).

### In-container camera bridge

`python/fw_sitl/gz_camera.py`:

- Subscribe gz-transport `gz.msgs.Image` for `race_cam`.
- Convert to RGB8 `ImageFrame` and **bind** the existing ZMQ image PUB (`FlightSetup.zmq.image`, topic `image`).
- `--net=host` ⇒ host `run_balloon_camera.py` connects as today.
- Timeout: if no camera image within 30 s of start, exit non-zero (race image pane dies; remain-on-exit keeps the error).
- Python gz bindings run **inside** the image. If Noble `gz-jetty` lacks `gz.transport` / `gz.msgs` for CPython, add the matching `python3-gz-*` packages to `Dockerfiles/PX4NobleSimNvidia.dockerfile` (image rebuild required once). Do **not** add gz packages to host `python/requirements.txt`.

### `python/run_balloon_image_source.py --mode gz`

Host process is the race image pane. It `docker exec`s the gz container running `gz_camera` (one ZMQ PUB). It does not subscribe to gz on the host and does not start a second PUB. Missing container → exit 1 with the container name.

### Balloons

Assets: `python/assets/gz/models/balloon_R_G_B/` sphere SDFs, diffuse RGB matching filename / `BalloonSpec.color` (same map as FG: `255_0_0`, `0_255_0`, `0_0_255`). Diameter from `BalloonSpec.diameter_m` (SDF scale or geometry).

`spawn_balloons_gz(balloons, *, origin_enu, container, world="default")` in `balloon_scene.py`:

- Convert each balloon **home-relative NED** `(n, e, d)` to Gazebo ENU about the aircraft spawn:
  - `x = origin_x + e`
  - `y = origin_y + n`
  - `z = origin_z - d`
  - Default origin `(0, 0, 500)` matching `PX4_GZ_MODEL_POSE`.
- **Clear then place:** remove existing models named `balloon_*` (and any prior race names), then create. No accumulation across re-runs (same contract as FG 0.17.5).
- Invoke Gazebo create/remove **inside the container** (`docker exec … gz service` or in-container Python). Host does not need `gz` CLI.
- Call **after OFFBOARD engage**, background thread, using **rebased** race NED (same altitudes control chases). Failure → stderr warning plus non-zero from the spawn helper; control may keep flying (assisted geometry) but must print that world balloons are missing.

### Race launcher `python/scripts/run_balloon_race.sh`

| Flag | Effect |
|------|--------|
| (none) | JSBSim + synth (unchanged) |
| `--viz` | JSBSim + FG (unchanged) |
| `--gz` | Gazebo plant |
| `--model NAME` | only valid with `--gz` (default `rc_cessna`); without `--gz` → exit 2 |
| `--viz` and `--gz` together | exit 2 |

`--gz` wiring:

- `SIM_CMD=runSimGzPlane.sh --mavlink-server --setup ${SETUP}` (+ `--model` if set)
- `IMG_CMD=… --mode gz`
- `CTL_CMD=… --no-sim --gz --spawn-gz-balloons`
- Fan-out / HEARTBEAT / control-first / tiled panes / remain-on-exit: unchanged
- Fan-out liveness: also treat `${PX4_GZ_DOCKER_NAME}-mavlink` as a sidecar (today only JSBSim name is checked)

Root `run_balloon_race.sh` stays a shim. Root `kill.sh`: default still `--jsbsim`; `--gz` and `--all` include the Gazebo stack. `python/scripts/kill.sh --gz` removes `px4-noble-gz-plane` + sidecar + host mavlink-server (same pattern as `--jsbsim`). `--all` kills FG + JSBSim + Gazebo.

### Control `python/run_balloon_control.py`

- `--gz`: plant is Gazebo. When control owns sim (not race `--no-sim`), `DEFAULT_SIM` is `runSimGzPlane.sh` and kill target is `--gz`. Race always passes `--no-sim`, so this matters for solo control debug.
- `--spawn-gz-balloons`: after engage + Z rebase, background `spawn_balloons_gz`.
- Engage policy for `--gz` matches `--viz`: `accept_unhealthy=True`, `full_sim_restart=False`, 60 s arm timeout, skip reboot when `--gz --no-sim`.
- No FG telnet on this path.

### Straight flight

`python/run_straight_flight_gz.py`: thin locked-line hold like YASim, `DEFAULT_SIM=runSimGzPlane.sh`, `KILL_TARGET=--gz`. GUI always (this plant has no headless mode in this spec). Optional `--model` passed to the sim script.

## Pose / frames (explicit)

- Gazebo world: ENU, default PX4 `default` world, ground z≈0.
- `PX4_GZ_MODEL_POSE="0,0,500,0,0,1.570796"`: x=0 east, y=0 north, z=500 up, yaw=+π/2 so body +X (forward) is Gazebo +Y (**north**). Matches balloon 0 at NED `(300,0,0)`. PX4 local NED origin is still EKF home (≈ aircraft); balloon NED in `flightSetup.json` stays home-relative with z≈0 at cruise.
- Balloon ENU uses the **Gazebo spawn origin**, not PX4 local NED metres, except that spawn uses the same **rebased NED deltas** control chases. A balloon at NED `(300,0,0)` with origin `(0,0,500)` is Gazebo `(x=0, y=300, z=500)`.
- Initial velocity: `speed_mps` along body +X ⇒ default world `(vx,vy,vz)=(0, 30, 0)`. Same `guidance.speed_mps` for overlay and OFFBOARD.

## Error handling

| Failure | Behavior |
|---------|----------|
| `--viz` and `--gz`, or `--model` without `--gz` | exit 2, one-line error |
| No `DISPLAY` | sim script exit 1 |
| `docker run` / GPU | exit 1, hint nvidia-container-toolkit / image tag |
| No HEARTBEAT in 120 s | race abort (existing) |
| No gz camera frame in 30 s | image process exit 1 |
| Balloon create/remove fails | spawn helper non-zero; control prints warning |
| Airspeed &lt; 15 m/s for 5 s after control connect (`--gz`) | control exit 1 (stall-fall) |
| Image `--mode gz` and container down | exit 1 |

Single ZMQ image bind: only the in-container bridge (started by `--mode gz`) binds `zmq.image`. Sim script must not also bind that port.

## Testing

Contract tests (no live Gazebo), extend existing style:

- `run_balloon_race.sh`: `--gz` sets sim/image/control strings above; `--viz --gz` exits 2; `--no-sim` still passed to control; heartbeat-before-peers still holds.
- `runSimGzPlane.sh`: default model cessna; `--model advanced_plane`; `MAVLINK_FANOUT` default 0; `--mavlink-server`; `--kill`; `PX4_GZ_MODEL_POSE` z=500 and yaw=+π/2; overlay injects `race_cam` and nonzero velocity; GPU/X11 flags present.
- `run_balloon_race.sh --model` without `--gz` exits 2.
- `kill.sh --gz` / `--all` include gz container name.
- `ned_to_gz_enu((300,0,0), origin=(0,0,500)) == (0, 300, 500)`; `((0,80,-15), …) == (80, 0, 515)`.
- `spawn_balloons_gz` source/contract: clear before create; names `balloon_255_0_0` etc.
- Balloon SDF materials match filename RGB (0..1).
- Overlay/camera generator: HFOV/width/height from `CameraSpec`.
- `run_balloon_image_source.py` accepts `--mode gz`.
- Control: `spawn_balloons_gz(` appears after `engage_offboard_with_retries`; `--gz` engage uses viz-like timeouts.

Live smoke (manual, not CI gate):

1. `python/scripts/runSimGzPlane.sh` — gz GUI, plane airborne, HEARTBEAT on 14550, airspeed &gt; 15 m/s.
2. `./run_balloon_race.sh --gz` — three colored balloons in the world, camera pane shows RGB frames, tracker can lock a blob, OFFBOARD engages. Pass count is not a merge gate for v1.

## Docs

- Root `README.md`: third plant; `--gz`; container name; camera=sensor not GUI grab.
- Root `UPDATES.md`: feature `subver` bump.
- `Dockerfiles/README.md` only if the image gains `python3-gz-*` or a documented `gz_rc_cessna` bake.
- This file is the design record.

## Risks

| Risk | Mitigation |
|------|------------|
| In-air spawn with V=0 stall-falls | overlay/post-spawn velocity + 15 m/s gate |
| Stock SDF updates break overlay inject | copy from image PX4 tree at runtime; test camera name `race_cam` |
| gz Python bindings missing in image | Dockerfile package add + rebuild |
| GPU/X11 | same class as FG viz; fail loud |
| Camera sees fuselage | `fg_eye_forward_m` mount; not GUI follow-cam |
| NED vs ENU balloon miss | unit-tested conversion; rebase after engage |
| gz-transport discovery from `docker exec` | exec into the sim container, not a new netns |

## Decision log

- Use: balloon race with Gazebo viz/camera (user 3).
- Pixels: sensor + GUI for human (user 3).
- Spawn: in-air like Rascal (user 1).
- Model: Cessna default, advanced selectable (user 3).
- Implementation: additive plant A (user A).
