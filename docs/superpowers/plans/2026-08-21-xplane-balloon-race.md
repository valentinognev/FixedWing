# X-Plane 12 Balloon Race Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `./run_balloon_race.sh --xplane` that flies PX4 v1.17 + bind-mounted X-Plane 12 demo Cessna 172 over Salzburg with visual balloons and mss HSV chase.

**Architecture:** One `px4-noble-sim-ros` container (`px4-noble-xplane-cessna`) bind-mounts `$XP12_HOME`. Runtime copy of airframe `5001_xplane_cessna172`. Plugins: px4xplane + `fixedwing_balloons` (UDP 49091). Camera reuses FG mss grab. Pose is geodetic from the balloon plugin, converted with LOWS NED origin.

**Tech Stack:** PX4 v1.17 SITL, X-Plane 12 demo, px4xplane, XPLM, Python `fw_sitl`, tmux race launcher, mss, ZMQ.

## Global Constraints

- PX4 stays **v1.17.0**; do not checkout PX4 main.
- Do not copy `/home/valentin/X-Plane 12` into git or the Docker image.
- `--xplane` is mutually exclusive with `--viz` / `--yasim` / `--gz`.
- Default origin: LOWS lat **47.7933**, lon **13.0044**, ground **430 m**, aircraft MSL **930 m**.
- Cessna file: `Aircraft/Laminar Research/Cessna 172 SP/Cessna_172SP.acf`.
- Balloon plugin UDP port **49091**.
- Plant id **`xplane_cessna172`**. First-cut trim **40 m/s**, approach **28 m/s**, `max_roll` **0.40**.
- Do not `git commit` unless the user asks.
- Host tests: `cd python && python3 -m unittest discover -s tests`.
- Spec: `docs/superpowers/specs/2026-08-21-xplane-balloon-race-design.md`.

---

## File map

| Path | Role |
|------|------|
| `python/fw_sitl/xp_origin.py` | LOWS constants + geodetic helpers |
| `python/fw_sitl/xp_balloon.py` | UDP JSON client for `fixedwing_balloons` |
| `python/fw_sitl/xp_camera.py` | thin wrapper: FG grabbers + XP window pattern |
| `python/assets/xplane/5001_xplane_cessna172` | PX4 airframe copied into container ROMFS/build etc |
| `python/assets/xplane/plugin/` | `fixedwing_balloons` C++ sources + OBJ8 |
| `python/scripts/runSimXplaneCessna.sh` | docker + XP + PX4 |
| `python/scripts/fetch_px4xplane.sh` | download Linux plugin zip |
| `python/run_balloon_xp_pose.py` | plugin pose → ZMQ |
| `python/fw_sitl/plant_gains.py` | `xplane_cessna172` table + flag |
| `python/scripts/run_balloon_race.sh` | `--xplane` |
| `python/scripts/kill.sh` | `--xplane` |
| `python/run_balloon_{control,image_source}.py` | flags / `--mode xp` |
| `python/fw_sitl/balloon_scene.py` | `spawn_xp_from_setup` |
| `python/fw_sitl/spawn_ic.py` | `--xp-geodetic` |
| `python/fw_sitl/flight_setup.py` | `xp_window_pattern` |
| `README.md` / `UPDATES.md` | 0.36.0 |

---

### Task 1: Plant id and gains

**Files:**
- Modify: `python/fw_sitl/plant_gains.py`
- Modify: `python/tests/test_plant_gains.py`

**Interfaces:**
- Consumes: existing `PlantGains`, `plant_id_from_flags`
- Produces: `plant_id_from_flags(*, xplane: bool = False, ...) -> str`; `KNOWN_PLANT_IDS` includes `"xplane_cessna172"`; `load_plant_gains("xplane_cessna172")`

- [ ] **Step 1: Write the failing tests** in `test_plant_gains.py`:

```python
def test_xplane(self) -> None:
    self.assertEqual(plant_id_from_flags(xplane=True), "xplane_cessna172")

def test_xplane_and_gz_rejected(self) -> None:
    with self.assertRaises(ValueError):
        plant_id_from_flags(xplane=True, gz=True)

def test_xplane_cessna172_race_snapshot(self) -> None:
    p = load_plant_gains("xplane_cessna172")
    self.assertEqual(p.plant_id, "xplane_cessna172")
    self.assertAlmostEqual(p.speed_mps, 40.0)
    self.assertAlmostEqual(p.approach_speed_mps, 28.0)
    self.assertAlmostEqual(p.fw_airspd_trim, 40.0)
    self.assertAlmostEqual(p.bank_max_roll_rad, 0.40)
```

Add `"xplane_cessna172"` to `_PLANTS` in that test file.

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd python && python3 -m unittest tests.test_plant_gains.TestPlantIdFromFlags.test_xplane tests.test_plant_gains.TestPlantGainsRegistry.test_xplane_cessna172_race_snapshot
```

Expected: FAIL (`xplane` unexpected / unknown plant).

- [ ] **Step 3: Minimal implementation**

`plant_id_from_flags`: add `xplane: bool = False`; count it in `nflags`; if set return `"xplane_cessna172"`. Error text: `"gz, yasim, viz, and xplane are mutually exclusive"`.

Table (first cut; PX4 inner from 5001 where it maps onto `PlantGains.px4_inner`):

- `speed_mps=40`, `approach_speed_mps=28`, `slow_range_m=280`, `fw_airspd_min=28`, `fw_airspd_trim=40`, `fw_airspd_max=65`
- `bank_max_roll_rad=0.40`, `bank_kp_heading=1.2`, `cruise_thrust=0.60`, `visual_lock_kp_alt=0.028`
- Inner: `FW_RR_P=0.42`, `FW_RR_I=0.11`, `FW_RR_FF=3.2`, `FW_PR_P=0.365`, `FW_THR_TRIM=0.60` (subset of 5001)

- [ ] **Step 4: Re-run tests — PASS**

---

### Task 2: LOWS origin + spawn_ic

**Files:**
- Create: `python/fw_sitl/xp_origin.py`
- Modify: `python/fw_sitl/spawn_ic.py`
- Create: `python/tests/test_xp_origin.py`

**Interfaces:**
- Produces:

```python
XP_ORIGIN_LAT_DEG = 47.7933
XP_ORIGIN_LON_DEG = 13.0044
XP_GROUND_ALT_M = 430.0
XP_AIRCRAFT_MSL_M = 930.0

def xp_geodetic(spawn: SpawnSpec) -> tuple[float, float, float]:
    """NED → (lat_deg, lon_deg, alt_msl_m) at LOWS cruise origin."""
```

- [ ] **Step 1: Failing test**

```python
def test_spawn_zero_is_lows_cruise(self) -> None:
    from fw_sitl.flight_setup import SpawnSpec
    from fw_sitl.xp_origin import xp_geodetic, XP_ORIGIN_LAT_DEG, XP_ORIGIN_LON_DEG, XP_AIRCRAFT_MSL_M
    lat, lon, alt = xp_geodetic(SpawnSpec(ned=(0.0, 0.0, 0.0), heading_deg=10.0))
    self.assertAlmostEqual(lat, XP_ORIGIN_LAT_DEG, places=4)
    self.assertAlmostEqual(lon, XP_ORIGIN_LON_DEG, places=4)
    self.assertAlmostEqual(alt, XP_AIRCRAFT_MSL_M, places=1)
```

- [ ] **Step 2: Run — FAIL** (module missing)

- [ ] **Step 3: Implement `xp_origin.xp_geodetic` via `balloon_scene.ned_to_geodetic`**

- [ ] **Step 4: `spawn_ic` `--xp-geodetic` prints `lat,lon,alt,heading` CSV (one line). Test the argv parser like `--gz-pose`.**

- [ ] **Step 5: Tests PASS**

---

### Task 3: Balloon UDP protocol (host)

**Files:**
- Create: `python/fw_sitl/xp_balloon.py`
- Modify: `python/fw_sitl/balloon_scene.py`
- Create: `python/tests/test_xp_balloon.py`

**Interfaces:**
- Produces:

```python
XP_BALLOON_PORT = 49091

def encode_clear() -> bytes  # b'{"cmd":"clear"}'
def encode_place(name: str, lat: float, lon: float, alt_msl_m: float, diameter_m: float, rgb: tuple[int,int,int]) -> bytes
def encode_sitl_connect() -> bytes
def parse_reply(raw: bytes) -> dict
def spawn_xp_from_setup(setup, *, host="127.0.0.1", port=49091, timeout_s=90.0) -> int
```

`spawn_xp_from_setup`: `pose_query` (or live pose) → origin lat/lon/MSL (fallback LOWS) → `clear` → `place` each balloon with `rebase_balloons_to_local_z(..., local_z=0)` then `ned_to_geodetic`. Return 0/1.

- [ ] **Step 1: Tests for encode/parse and spawn using a mock UDP socket (no X-Plane)**
- [ ] **Step 2: FAIL**
- [ ] **Step 3: Implement**
- [ ] **Step 4: PASS**

---

### Task 4: Launcher, kill, image `--mode xp`, control `--xplane`

**Files:**
- Modify: `python/scripts/run_balloon_race.sh`
- Modify: `python/scripts/kill.sh` and root `kill.sh` if it forwards
- Modify: `python/run_balloon_image_source.py`
- Modify: `python/run_balloon_control.py`
- Modify: `python/fw_sitl/flight_setup.py` (`xp_window_pattern` default `X-Plane|X-System`)
- Create: `python/tests/test_xplane_race_contracts.py`

**Interfaces:**
- `--xplane` sets `MODE=xp`, `CONTAINER_NAME=px4-noble-xplane-cessna`, `SIM_CMD=runSimXplaneCessna.sh --mavlink-server --setup …`
- After fan-out: `python -m fw_sitl.balloon_scene --setup … --xplane`
- Control: `--xplane --spawn-xp-balloons`; skip geom-gate; skip reboot; plant xplane
- Image: `--mode xp` calls `run_xp_publisher` (wrap `run_fg_publisher` with XP pattern, no FG telnet view sync — or skip telnet if host unreachable)
- Kill: `--xplane` removes `px4-noble-xplane-cessna` + sidecar; `--all` includes it
- Exclusive errors: `--xplane and --gz are mutually exclusive` (and viz/yasim)

- [ ] **Step 1: Contract tests (read scripts / `--help` / bash exclusive flags) — FAIL**
- [ ] **Step 2: Wire scripts and Python flags — PASS**

`runSimXplaneCessna.sh` may still be a stub that `--help`s in this task; Task 5 fills it. Contract test should require the file exists and mentions `X-Plane` / bind-mount `/opt/xplane12`.

---

### Task 5: `runSimXplaneCessna.sh` + airframe file + px4xplane fetch

**Files:**
- Create: `python/assets/xplane/5001_xplane_cessna172` (copy from PX4-Autopilot main `ROMFS/.../5001_xplane_cessna172`)
- Create: `python/scripts/fetch_px4xplane.sh`
- Create: `python/scripts/runSimXplaneCessna.sh`

**Interfaces:**
- Env: `XP12_HOME` default `$HOME/X-Plane 12`; `PX4_XP_DOCKER_NAME`; `XP12_AIRPORT` default `LOWS`
- Preflight: test `-x "$XP12_HOME/X-Plane-x86_64"` and Cessna `.acf` and `Global Scenery/X-Plane 12 Demo Areas`
- Docker: same GPU/X11/net-host pattern as `runSimGzPlane.sh`
- Copy airframe into `/home/valentin/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/` (and `px4_sitl_nolockstep` if that is the baked binary — detect which exists)
- Launch XP then PX4 as in the spec

- [ ] **Step 1: Test: `bash -n python/scripts/runSimXplaneCessna.sh`; contract test asserts missing XP12_HOME path is checked (`X-Plane-x86_64` string in script). FAIL until script exists.**
- [ ] **Step 2: Implement script + fetch helper. `bash -n` PASS.**

---

### Task 6: `fixedwing_balloons` plugin

**Files:**
- Create: `python/assets/xplane/plugin/fixedwing_balloons.cpp` (and `CMakeLists.txt` or Makefile)
- Create: `python/assets/xplane/plugin/balloon_sphere.obj` (simple OBJ8 sphere ~1 m, scaled by diameter)
- Modify: `runSimXplaneCessna.sh` to build `lin.xpl` in-container if missing

**Interfaces:** UDP 49091, JSON cmds from Task 3. `sitl_connect` → `XPLMCommandOnce("px4xplane/toggleEnable")`. Place/clear instances. Pose query from datarefs. Optional: apply spawn lat/lon/alt/psi + airspeed once.

- [ ] **Step 1: Unit-test JSON handling by extracting encode/decode to a tiny C helper is optional; host tests already cover the protocol. Compile-smoke: Makefile exists and lists `XPLMCreateInstance`.**
- [ ] **Step 2: Implement plugin. SDK: download Laminar X-Plane SDK zip into `/tmp` in the container (not git).**

---

### Task 7: Pose pane + control NED from X-Plane

**Files:**
- Create: `python/run_balloon_xp_pose.py`
- Create: `python/fw_sitl/xp_pose_bridge.py`
- Modify: `python/run_balloon_control.py` (`--xplane` pose subscriber → `geodetic_to_ned`)
- Modify: launcher to split a pose pane like `--gz`

**Interfaces:**
- `XpPoseSample(stamp, lat, lon, alt_msl_m, roll_deg, pitch_deg, heading_deg)`
- Convert with live or LOWS origin; overwrite history NED like GZ/FG
- Skip geom-gate when `args.xplane`

- [ ] **Step 1: Test geodetic → NED roundtrip at LOWS**
- [ ] **Step 2: Implement bridge + control wiring**

---

### Task 8: Docs

**Files:**
- Modify: `README.md`, `UPDATES.md` → **0.36.0**
- Modify: `Dockerfiles/README.md` only if the image bake actually gains the airframe patch this round (runtime copy does not require it)

- [ ] **Step 1: README Architecture bullet for `--xplane`; UPDATES 0.36.0 on top**
- [ ] **Step 2: `cd python && python3 -m unittest discover -s tests` — all PASS**

---

## Spec coverage

| Spec item | Task |
|-----------|------|
| `--xplane` exclusive launcher | 4 |
| Bind-mount, no bake of demo | 5 |
| v1.17 + 5001 copy | 5 |
| LOWS spawn | 2, 5 |
| Visual balloons + UDP plugin | 3, 6 |
| mss camera | 4 (`--mode xp`) |
| plant gains | 1 |
| pose ZMQ / CSV sim NED | 7 |
| kill.sh | 4 |
| README/UPDATES | 8 |
| Demo new flight | 5 |
