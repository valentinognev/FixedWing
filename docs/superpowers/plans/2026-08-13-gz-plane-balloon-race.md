# Gazebo PX4 Plane Balloon Race Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a Gazebo Jetty PX4 plane plant (Cessna default) beside JSBSim Rascal so `./run_balloon_race.sh --gz` flies in-air with world balloons, an onboard camera on the ZMQ tracker bus, and the Gazebo GUI for the operator.

**Architecture:** Additive plant: new `runSimGzPlane.sh` on the existing Noble image, overlay SDF (forward `race_cam` + spawn velocity), in-container gz→ZMQ bridge via `docker exec`, `spawn_balloons_gz` after engage. JSBSim/YASim paths stay default/unchanged.

**Tech Stack:** bash, Docker `--net=host` + NVIDIA + X11, PX4 v1.17 `gz_rc_cessna` / `gz_advanced_plane`, Gazebo Jetty (`gz-sim`), Python 3 unittest, pymavlink, pyzmq, numpy.

**Spec:** `docs/superpowers/specs/2026-08-13-gz-plane-balloon-race-design.md`

## Global Constraints

- Do not change JSBSim/YASim default race or PX4 airframe IDs `4003` / `4008`.
- Tracker pixels come from gz camera sensor `race_cam`, never a Gazebo GUI grab.
- Host Python must not import `gz.transport`; camera/spawn gz CLI run inside `px4-noble-gz-plane`.
- Default spawn pose string is `0,0,500,0,0,1.570796` (500 m AGL, yaw +π/2 = north).
- `--gz` and `--viz` are mutually exclusive (exit 2); `--model` without `--gz` is exit 2.
- Single ZMQ image PUB: only the in-container `gz_camera` bridge binds `FlightSetup.zmq.image`.
- Tests: `cd python && python3 -m unittest <module> -v` (this repo does not use pytest as the runner).
- **Commits:** this repo forbids `git commit` unless the user explicitly asks; skip commit steps until asked.

---

## File map

| File | Responsibility |
|------|----------------|
| `python/fw_sitl/gz_pose.py` | NED→Gazebo ENU; body-+X velocity in ENU from yaw |
| `python/fw_sitl/gz_overlay.py` | Inject `race_cam` + spawn-velocity plugin into stock plane SDF |
| `python/assets/gz/systems/race_spawn_velocity.py` | One-shot gz-sim Python system: set link world velocity |
| `python/assets/gz/models/balloon_*/` | Colored sphere models (`255_0_0`, `0_255_0`, `0_0_255`) |
| `python/fw_sitl/balloon_scene.py` | `spawn_balloons_gz`, `gz_balloon_model_name`, create/remove argv |
| `python/scripts/runSimGzPlane.sh` | Docker Gazebo plane SITL + overlay + fan-out |
| `python/scripts/kill.sh` | `--gz` / `--all` include gz container + sidecar |
| `kill.sh` | Forward `--gz` (default remains `--jsbsim`) |
| `python/fw_sitl/gz_camera.py` | In-container gz Image → ZMQ `ImageFrame` |
| `python/run_balloon_image_source.py` | `--mode gz` → `docker exec` `gz_camera.py` |
| `python/run_balloon_control.py` | `--gz`, `--spawn-gz-balloons`, airspeed gate |
| `python/scripts/run_balloon_race.sh` | `--gz` / `--model` / `--setup` passthrough |
| `python/run_straight_flight_gz.py` | Thin locked-line hold on Gazebo plant |
| `python/fw_sitl/mavlink_io.py` | `wait_min_airspeed` |
| `Dockerfiles/PX4NobleSimNvidia.dockerfile` | `python3-gz-*` + `python3-zmq` if missing in image |
| `README.md` / `UPDATES.md` | Third plant; `--gz` |

---

### Task 1: NED→ENU pose and balloon sphere assets

**Files:**
- Create: `python/fw_sitl/gz_pose.py`
- Create: `python/assets/gz/models/balloon_255_0_0/model.sdf`
- Create: `python/assets/gz/models/balloon_255_0_0/model.config`
- Create: `python/assets/gz/models/balloon_0_255_0/model.sdf`
- Create: `python/assets/gz/models/balloon_0_255_0/model.config`
- Create: `python/assets/gz/models/balloon_0_0_255/model.sdf`
- Create: `python/assets/gz/models/balloon_0_0_255/model.config`
- Test: `python/tests/test_gz_pose.py`

**Interfaces:**
- Consumes: none
- Produces:
  - `ned_to_gz_enu(ned: tuple[float, float, float], origin_enu: tuple[float, float, float]) -> tuple[float, float, float]`
  - `world_velocity_enu(speed_mps: float, yaw_rad: float) -> tuple[float, float, float]`
  - `DEFAULT_GZ_ORIGIN_ENU = (0.0, 0.0, 500.0)`
  - `DEFAULT_GZ_YAW_RAD = math.pi / 2`
  - Assets dir `python/assets/gz/models/balloon_R_G_B/` with sphere radius `diameter_m/2` (default 5 m) and diffuse RGB matching the filename (0..1)

- [ ] **Step 1: Write the failing test**

Create `python/tests/test_gz_pose.py`:

```python
#!/usr/bin/env python3
from __future__ import annotations

import math
import re
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.gz_pose import (
    DEFAULT_GZ_ORIGIN_ENU,
    DEFAULT_GZ_YAW_RAD,
    ned_to_gz_enu,
    world_velocity_enu,
)

ASSETS_GZ = _PYTHON_ROOT / "assets" / "gz" / "models"


class TestGzPose(unittest.TestCase):
    def test_ned_north_is_gz_y(self) -> None:
        x, y, z = ned_to_gz_enu((300.0, 0.0, 0.0), DEFAULT_GZ_ORIGIN_ENU)
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 300.0, places=6)
        self.assertAlmostEqual(z, 500.0, places=6)

    def test_ned_east_up_is_gz_x_and_higher_z(self) -> None:
        x, y, z = ned_to_gz_enu((0.0, 80.0, -15.0), DEFAULT_GZ_ORIGIN_ENU)
        self.assertAlmostEqual(x, 80.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(z, 515.0, places=6)

    def test_default_heading_north_velocity(self) -> None:
        vx, vy, vz = world_velocity_enu(30.0, DEFAULT_GZ_YAW_RAD)
        self.assertAlmostEqual(vx, 0.0, places=5)
        self.assertAlmostEqual(vy, 30.0, places=5)
        self.assertAlmostEqual(vz, 0.0, places=5)
        self.assertAlmostEqual(DEFAULT_GZ_YAW_RAD, math.pi / 2, places=6)


class TestGzBalloonAssets(unittest.TestCase):
    def test_materials_match_filename_rgb(self) -> None:
        cases = (
            ("balloon_255_0_0", (1.0, 0.0, 0.0)),
            ("balloon_0_255_0", (0.0, 1.0, 0.0)),
            ("balloon_0_0_255", (0.0, 0.0, 1.0)),
        )
        diffs: list[tuple[float, float, float]] = []
        for stem, rgb in cases:
            text = (ASSETS_GZ / stem / "model.sdf").read_text(encoding="utf-8")
            self.assertIn("<radius>5</radius>", text.replace(" ", ""))
            cfg = (ASSETS_GZ / stem / "model.config").read_text(encoding="utf-8")
            self.assertIn(stem, cfg)
            got: list[float] = []
            for ch, exp in zip(("r", "g", "b"), rgb):
                m = re.search(rf"<{ch}>([0-9.]+)</{ch}>", text)
                self.assertIsNotNone(m, f"{stem} {ch}")
                assert m is not None
                val = float(m.group(1))
                got.append(val)
                self.assertAlmostEqual(val, exp, places=5)
            diffs.append((got[0], got[1], got[2]))
        self.assertEqual(len(set(diffs)), 3)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/valentin/Projects/FixedWing/python && python3 -m unittest tests.test_gz_pose -v`

Expected: FAIL with `ModuleNotFoundError: No module named 'fw_sitl.gz_pose'` (or missing asset files).

- [ ] **Step 3: Implement pose helpers and balloon SDFs**

`python/fw_sitl/gz_pose.py`:

```python
"""Gazebo ENU helpers for the PX4 gz plane plant."""
from __future__ import annotations

import math

DEFAULT_GZ_ORIGIN_ENU = (0.0, 0.0, 500.0)
DEFAULT_GZ_YAW_RAD = math.pi / 2
DEFAULT_GZ_POSE = "0,0,500,0,0,1.570796"


def ned_to_gz_enu(
    ned: tuple[float, float, float],
    origin_enu: tuple[float, float, float] = DEFAULT_GZ_ORIGIN_ENU,
) -> tuple[float, float, float]:
    """Home-relative NED (n,e,d) → Gazebo ENU (x=east, y=north, z=up)."""
    north, east, down = ned
    ox, oy, oz = origin_enu
    return (ox + east, oy + north, oz - down)


def world_velocity_enu(speed_mps: float, yaw_rad: float) -> tuple[float, float, float]:
    """Body +X airspeed in Gazebo ENU. yaw=0 → +X (east); yaw=π/2 → +Y (north)."""
    return (
        float(speed_mps) * math.cos(yaw_rad),
        float(speed_mps) * math.sin(yaw_rad),
        0.0,
    )
```

For each color stem `balloon_255_0_0` / `balloon_0_255_0` / `balloon_0_0_255` with RGB `(1,0,0)` / `(0,1,0)` / `(0,0,1)`, write `model.config`:

```xml
<?xml version="1.0" ?>
<model>
  <name>balloon_255_0_0</name>
  <sdf version="1.9">model.sdf</sdf>
</model>
```

and `model.sdf` (swap name + `<r><g><b>`):

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="balloon_255_0_0">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>5</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>
            <r>0.4</r><g>0</g><b>0</b><a>1</a>
          </ambient>
          <diffuse>
            <r>1</r><g>0</g><b>0</b><a>1</a>
          </diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>5</radius>
          </sphere>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

Ambient = 0.4 × diffuse per channel (same ratio as FG balloon tests). If Gazebo rejects nested `<r>` and wants `1 0 0 1` on one line, keep the tagged form first; if a later live smoke fails parse, switch both SDF and the regex test to `1 0 0 1` together.

- [ ] **Step 4: Run tests and make sure they pass**

Run: `cd /home/valentin/Projects/FixedWing/python && python3 -m unittest tests.test_gz_pose -v`

Expected: all tests PASS.

- [ ] **Step 5: Commit** (skip unless the user asked)

---

### Task 2: Overlay injector (`race_cam` + spawn velocity)

**Files:**
- Create: `python/fw_sitl/gz_overlay.py`
- Create: `python/assets/gz/systems/race_spawn_velocity.py`
- Test: `python/tests/test_gz_overlay.py`

**Interfaces:**
- Consumes: `world_velocity_enu` from Task 1 (not required inside overlay; overlay takes vx,vy,vz)
- Produces:
  - `canonical_link_name(sdf: str) -> str` — `"base_link"` if present, else first `<link name="...">`
  - `inject_race_cam(sdf: str, *, width: int, height: int, hfov_deg: float, eye_forward_m: float, update_rate_hz: float = 10.0) -> str`
  - `inject_spawn_velocity_plugin(sdf: str) -> str`
  - `apply_plane_overlay(sdf: str, *, width: int, height: int, hfov_deg: float, eye_forward_m: float, update_rate_hz: float = 10.0) -> str` — cam then velocity plugin
  - Module `race_spawn_velocity.velocity_from_env(env: Mapping[str, str]) -> tuple[float, float, float]`

- [ ] **Step 1: Write the failing test**

Create `python/tests/test_gz_overlay.py`:

```python
#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.gz_overlay import (
    apply_plane_overlay,
    canonical_link_name,
    inject_race_cam,
)
from assets.gz.systems.race_spawn_velocity import velocity_from_env  # may fail; see Step 3 import path
```

Do **not** import via `assets.gz...` (not a package). Import by adding `python/assets/gz/systems` to `sys.path` in the test:

```python
import sys
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
_SYS_DIR = _PYTHON_ROOT / "assets" / "gz" / "systems"
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))
if str(_SYS_DIR) not in sys.path:
    sys.path.insert(0, str(_SYS_DIR))

from fw_sitl.gz_overlay import apply_plane_overlay, canonical_link_name, inject_race_cam
from race_spawn_velocity import velocity_from_env

_STOCK = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="rc_cessna">
    <link name="base_link">
      <inertial><mass>1</mass></inertial>
    </link>
  </model>
</sdf>
"""


class TestGzOverlay(unittest.TestCase):
    def test_canonical_prefers_base_link(self) -> None:
        sdf = '<sdf><model><link name="wing"/><link name="base_link"/></model></sdf>'
        self.assertEqual(canonical_link_name(sdf), "base_link")

    def test_inject_race_cam_matches_cameraspec(self) -> None:
        out = inject_race_cam(
            _STOCK, width=640, height=480, hfov_deg=90.0, eye_forward_m=5.0
        )
        self.assertIn('name="race_cam"', out)
        self.assertIn('type="camera"', out)
        self.assertIn("<width>640</width>", out)
        self.assertIn("<height>480</height>", out)
        self.assertIn("<parent>base_link</parent>", out)
        self.assertIn("5 0 0 0 0 0", out)
        hfov = math.radians(90.0)
        self.assertIn(f"{hfov:.6f}", out)

    def test_apply_overlay_adds_velocity_plugin(self) -> None:
        out = apply_plane_overlay(
            _STOCK, width=640, height=480, hfov_deg=90.0, eye_forward_m=5.0
        )
        self.assertIn("race_cam", out)
        self.assertIn("PythonSystemLoader", out)
        self.assertIn("race_spawn_velocity", out)

    def test_velocity_from_env_north(self) -> None:
        vx, vy, vz = velocity_from_env(
            {"FW_GZ_SPAWN_VX": "0", "FW_GZ_SPAWN_VY": "30", "FW_GZ_SPAWN_VZ": "0"}
        )
        self.assertEqual((vx, vy, vz), (0.0, 30.0, 0.0))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/valentin/Projects/FixedWing/python && python3 -m unittest tests.test_gz_overlay -v`

Expected: FAIL (`gz_overlay` missing).

- [ ] **Step 3: Implement overlay + velocity module**

`python/fw_sitl/gz_overlay.py`:

```python
"""Inject race camera + spawn-velocity plugin into a PX4 Gazebo plane SDF."""
from __future__ import annotations

import math
import re

_LINK_NAME = re.compile(r'<link\s+name="([^"]+)"')


def canonical_link_name(sdf: str) -> str:
    names = _LINK_NAME.findall(sdf)
    if not names:
        raise ValueError("no <link name=...> in SDF")
    if "base_link" in names:
        return "base_link"
    return names[0]


def inject_race_cam(
    sdf: str,
    *,
    width: int,
    height: int,
    hfov_deg: float,
    eye_forward_m: float,
    update_rate_hz: float = 10.0,
) -> str:
    parent = canonical_link_name(sdf)
    hfov = math.radians(float(hfov_deg))
    snippet = f"""
    <link name="race_cam_link">
      <pose>{float(eye_forward_m):.3f} 0 0 0 0 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <sensor name="race_cam" type="camera">
        <always_on>1</always_on>
        <update_rate>{float(update_rate_hz):.1f}</update_rate>
        <camera>
          <horizontal_fov>{hfov:.6f}</horizontal_fov>
          <image>
            <width>{int(width)}</width>
            <height>{int(height)}</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>5000</far>
          </clip>
        </camera>
      </sensor>
    </link>
    <joint name="race_cam_joint" type="fixed">
      <parent>{parent}</parent>
      <child>race_cam_link</child>
    </joint>
"""
    idx = sdf.rfind("</model>")
    if idx < 0:
        raise ValueError("SDF missing </model>")
    return sdf[:idx] + snippet + sdf[idx:]


def inject_spawn_velocity_plugin(sdf: str) -> str:
    snippet = """
    <plugin filename="gz-sim-python-system-loader-system"
            name="gz::sim::systems::PythonSystemLoader">
      <module_name>race_spawn_velocity</module_name>
    </plugin>
"""
    idx = sdf.rfind("</model>")
    if idx < 0:
        raise ValueError("SDF missing </model>")
    return sdf[:idx] + snippet + sdf[idx:]


def apply_plane_overlay(
    sdf: str,
    *,
    width: int,
    height: int,
    hfov_deg: float,
    eye_forward_m: float,
    update_rate_hz: float = 10.0,
) -> str:
    with_cam = inject_race_cam(
        sdf,
        width=width,
        height=height,
        hfov_deg=hfov_deg,
        eye_forward_m=eye_forward_m,
        update_rate_hz=update_rate_hz,
    )
    return inject_spawn_velocity_plugin(with_cam)
```

`python/assets/gz/systems/race_spawn_velocity.py`:

```python
"""One-shot spawn velocity for in-air gz plane (loaded by PythonSystemLoader)."""
from __future__ import annotations

import os
from collections.abc import Mapping

def velocity_from_env(env: Mapping[str, str] | None = None) -> tuple[float, float, float]:
    src = os.environ if env is None else env
    return (
        float(src.get("FW_GZ_SPAWN_VX", "0")),
        float(src.get("FW_GZ_SPAWN_VY", "0")),
        float(src.get("FW_GZ_SPAWN_VZ", "0")),
    )


try:
    import gz.sim as gz_sim
    from gz.sim import EntityComponentManager, UpdateInfo
    from gz.sim.components import LinearVelocityReset, Model, Name, CanonicalLink
except ImportError:
    gz_sim = None  # type: ignore


if gz_sim is not None:

    class RaceSpawnVelocity(gz_sim.System, gz_sim.ISystemPreUpdate):
        def __init__(self) -> None:
            gz_sim.System.__init__(self)
            self._done = False

        def pre_update(self, _info: UpdateInfo, ecm: EntityComponentManager) -> None:
            if self._done:
                return
            vx, vy, vz = velocity_from_env()
            applied = False

            def _on_model(entity: int) -> bool:
                nonlocal applied
                name_c = ecm.component(entity, Name)
                if name_c is None:
                    return True
                name = str(name_c.data()) if hasattr(name_c, "data") else str(name_c)
                if name not in {"rc_cessna", "advanced_plane"}:
                    return True
                link = ecm.component(entity, CanonicalLink)
                if link is None:
                    return True
                link_entity = int(link.data()) if hasattr(link, "data") else int(link)
                ecm.set_component_data(
                    link_entity, LinearVelocityReset([vx, vy, vz])
                )
                applied = True
                return False

            ecm.each(Model, _on_model)
            if applied:
                self._done = True
```

gz-sim Python component APIs differ by version. Keep `velocity_from_env` and the plugin XML as the contract tests. If Jetty's `LinearVelocityReset` / `each` names differ, adjust **only** the `if gz_sim is not None` class so tests still pass. If PythonSystemLoader cannot set velocity at runtime, add a fallback in Task 4: after the model exists, `gz service` is not required; print `Error: spawn velocity plugin did not load` if `FW_GZ_SPAWN_VY` is set and later airspeed gate trips (Task 6).

- [ ] **Step 4: Run tests**

Run: `cd /home/valentin/Projects/FixedWing/python && python3 -m unittest tests.test_gz_overlay -v`

Expected: PASS.

- [ ] **Step 5: Commit** (skip unless asked)

---

### Task 3: `spawn_balloons_gz` (clear then create)

**Files:**
- Modify: `python/fw_sitl/balloon_scene.py`
- Test: `python/tests/test_gz_balloons.py`

**Interfaces:**
- Consumes: `ned_to_gz_enu`, `DEFAULT_GZ_ORIGIN_ENU` (Task 1); balloon asset dirs
- Produces:
  - `gz_balloon_model_name(color: tuple[int, int, int]) -> str` → `balloon_255_0_0`
  - `gz_remove_argv(world: str, name: str) -> list[str]`
  - `gz_create_argv(world: str, name: str, sdf_filename: str, pose_enu: tuple[float, float, float]) -> list[str]`
  - `spawn_balloons_gz(balloons, *, origin_enu, world, container, models_dir, runner) -> None`
  - `runner(argv: list[str]) -> None` default wraps `docker exec {container} ...`

- [ ] **Step 1: Write the failing test**

Create `python/tests/test_gz_balloons.py`:

```python
#!/usr/bin/env python3
from __future__ import annotations

import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.balloon_scene import (
    gz_balloon_model_name,
    gz_create_argv,
    gz_remove_argv,
    spawn_balloons_gz,
)
from fw_sitl.flight_setup import BalloonSpec
from fw_sitl.gz_pose import DEFAULT_GZ_ORIGIN_ENU


class TestGzBalloons(unittest.TestCase):
    def test_names(self) -> None:
        self.assertEqual(gz_balloon_model_name((255, 0, 0)), "balloon_255_0_0")
        self.assertEqual(gz_balloon_model_name((0, 255, 0)), "balloon_0_255_0")
        self.assertEqual(gz_balloon_model_name((0, 0, 255)), "balloon_0_0_255")

    def test_remove_then_create_order(self) -> None:
        calls: list[list[str]] = []

        def runner(argv: list[str]) -> None:
            calls.append(argv)

        balloons = (
            BalloonSpec(ned=(300.0, 0.0, 0.0), color=(255, 0, 0), diameter_m=10.0),
            BalloonSpec(ned=(0.0, 80.0, -15.0), color=(0, 255, 0), diameter_m=10.0),
        )
        models = Path("/opt/fixedwing/gz/models")
        spawn_balloons_gz(
            balloons,
            origin_enu=DEFAULT_GZ_ORIGIN_ENU,
            world="default",
            container="px4-noble-gz-plane",
            models_dir=models,
            runner=runner,
        )
        remove_i = [i for i, a in enumerate(calls) if a[:2] == ["gz", "service"] and "/remove" in " ".join(a)]
        create_i = [i for i, a in enumerate(calls) if "/create" in " ".join(a)]
        self.assertGreaterEqual(len(remove_i), 1)
        self.assertEqual(len(create_i), 2)
        self.assertLess(max(remove_i), min(create_i))
        joined = " ".join(calls[create_i[0]])
        self.assertIn("balloon_255_0_0", joined)
        self.assertIn("y: 300", joined.replace("y:", "y: "))
        joined2 = " ".join(calls[create_i[1]])
        self.assertIn("balloon_0_255_0", joined2)
        self.assertIn("x: 80", joined2.replace("x:", "x: "))
        self.assertIn("z: 515", joined2.replace("z:", "z: "))

    def test_argv_shapes(self) -> None:
        rm = gz_remove_argv("default", "balloon_255_0_0")
        self.assertEqual(rm[0], "gz")
        self.assertIn("/world/default/remove", rm)
        cr = gz_create_argv(
            "default",
            "balloon_255_0_0",
            "/opt/fixedwing/gz/models/balloon_255_0_0/model.sdf",
            (0.0, 300.0, 500.0),
        )
        self.assertIn("/world/default/create", cr)
        self.assertIn("gz.msgs.EntityFactory", cr)


if __name__ == "__main__":
    unittest.main()
```

The pose substring asserts should use the exact `--req` payload from `gz_create_argv` (implementer: if spacing differs, assert on the function output, not a guessed string). Safer extra asserts:

```python
req = cr[cr.index("--req") + 1]
self.assertIn("name: \"balloon_255_0_0\"", req)
self.assertIn("x: 0", req)
self.assertIn("y: 300", req)
self.assertIn("z: 500", req)
```

Put those on `gz_create_argv` directly in `test_argv_shapes`.

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/valentin/Projects/FixedWing/python && python3 -m unittest tests.test_gz_balloons -v`

Expected: FAIL (`spawn_balloons_gz` missing).

- [ ] **Step 3: Implement in `balloon_scene.py`**

Add imports: `from collections.abc import Callable, Sequence` and `from fw_sitl.gz_pose import DEFAULT_GZ_ORIGIN_ENU, ned_to_gz_enu`.

```python
GZ_CONTAINER_MODELS = Path("/opt/fixedwing/gz/models")
DEFAULT_GZ_CONTAINER = "px4-noble-gz-plane"
DEFAULT_GZ_WORLD = "default"


def gz_balloon_model_name(color: tuple[int, int, int]) -> str:
    r, g, b = (int(color[0]), int(color[1]), int(color[2]))
    return f"balloon_{r}_{g}_{b}"


def gz_remove_argv(world: str, name: str) -> list[str]:
    return [
        "gz",
        "service",
        "-s",
        f"/world/{world}/remove",
        "--reqtype",
        "gz.msgs.Entity",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        "3000",
        "--req",
        f'name: "{name}" type: MODEL',
    ]


def gz_create_argv(
    world: str,
    name: str,
    sdf_filename: str,
    pose_enu: tuple[float, float, float],
) -> list[str]:
    x, y, z = pose_enu
    req = (
        f'name: "{name}" sdf_filename: "{sdf_filename}" '
        f"pose {{ position {{ x: {x} y: {y} z: {z} }} }}"
    )
    return [
        "gz",
        "service",
        "-s",
        f"/world/{world}/create",
        "--reqtype",
        "gz.msgs.EntityFactory",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        "3000",
        "--req",
        req,
    ]


def _docker_exec_runner(container: str) -> Callable[[list[str]], None]:
    def run(argv: list[str]) -> None:
        import subprocess

        cmd = ["docker", "exec", container, *argv]
        subprocess.run(cmd, check=True)
    return run


def spawn_balloons_gz(
    balloons: Sequence[BalloonSpec],
    *,
    origin_enu: tuple[float, float, float] = DEFAULT_GZ_ORIGIN_ENU,
    world: str = DEFAULT_GZ_WORLD,
    container: str = DEFAULT_GZ_CONTAINER,
    models_dir: Path = GZ_CONTAINER_MODELS,
    runner: Callable[[list[str]], None] | None = None,
    clear_existing: bool = True,
) -> None:
    """Clear then place colored sphere models at NED converted to Gazebo ENU."""
    exec_fn = runner if runner is not None else _docker_exec_runner(container)
    names = [gz_balloon_model_name(b.color) for b in balloons]
    if clear_existing:
        for name in names:
            try:
                exec_fn(gz_remove_argv(world, name))
            except Exception as exc:  # noqa: BLE001
                print(f"GZ balloon clear {name}: {exc}")
    for spec, name in zip(balloons, names):
        pose = ned_to_gz_enu(spec.ned, origin_enu)
        sdf_path = str(models_dir / name / "model.sdf")
        print(f"GZ balloon {name} ENU={pose} sdf={sdf_path}")
        exec_fn(gz_create_argv(world, name, sdf_path, pose))
```

- [ ] **Step 4: Run tests**

Run: `cd /home/valentin/Projects/FixedWing/python && python3 -m unittest tests.test_gz_balloons tests.test_gz_pose tests.test_balloon_scene -v`

Expected: PASS (existing FG balloon tests still pass).

- [ ] **Step 5: Commit** (skip unless asked)

---

### Task 4: `runSimGzPlane.sh` + `kill.sh --gz`

**Files:**
- Create: `python/scripts/runSimGzPlane.sh`
- Modify: `python/scripts/kill.sh`
- Modify: `kill.sh` (root) — no logic change beyond `--gz` already forwarding via `"$@"` when args present; document in usage if there is a comment. Default with no args stays `--jsbsim`.
- Modify: `python/tests/test_mavlink_fanout_contracts.py` (extend; add gz contract class in same file or `python/tests/test_gz_sim_contracts.py`)

**Interfaces:**
- Consumes: `apply_plane_overlay`, `world_velocity_enu`, `load_flight_setup`, `DEFAULT_GZ_POSE`
- Produces: container `px4-noble-gz-plane`; flags `--model`, `--setup`, `--mavlink-server`, `--kill`; env `PX4_GZ_MODEL_POSE`, `FW_GZ_SPAWN_V*`

- [ ] **Step 1: Write the failing contract tests**

Create `python/tests/test_gz_sim_contracts.py`:

```python
#!/usr/bin/env python3
from __future__ import annotations

import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
_REPO = _PYTHON_ROOT.parent
_SIM = _PYTHON_ROOT / "scripts" / "runSimGzPlane.sh"
_KILL = _PYTHON_ROOT / "scripts" / "kill.sh"
_RACE = _PYTHON_ROOT / "scripts" / "run_balloon_race.sh"


class TestGzSimContracts(unittest.TestCase):
    def test_sim_script_exists_and_defaults(self) -> None:
        text = _SIM.read_text(encoding="utf-8")
        self.assertIn("px4-noble-gz-plane", text)
        self.assertIn("0,0,500,0,0,1.570796", text)
        self.assertIn("gz_rc_cessna", text)
        self.assertIn("gz_advanced_plane", text)
        self.assertIn("--gpus all", text)
        self.assertIn("GZ_SIM_RESOURCE_PATH", text)
        self.assertIn("apply_plane_overlay", text)
        self.assertIn("FW_GZ_SPAWN_VY", text)
        self.assertIn('MAVLINK_FANOUT="${MAVLINK_FANOUT:-0}"', text)
        self.assertIn("--mavlink-heartbeat-frequency 0", text)
        self.assertIn("/opt/fixedwing/python", text)
        self.assertIn("/opt/fixedwing/gz", text)
        self.assertIn("DISPLAY", text)
        self.assertIn("exit 1", text)

    def test_kill_gz(self) -> None:
        text = _KILL.read_text(encoding="utf-8")
        self.assertIn("--gz", text)
        self.assertIn("px4-noble-gz-plane", text)
        self.assertIn("kill_gz_stack", text)
        all_block = text[text.index("--all"):]
        self.assertIn("kill_gz_stack", all_block)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/valentin/Projects/FixedWing/python && python3 -m unittest tests.test_gz_sim_contracts -v`

Expected: FAIL (missing `runSimGzPlane.sh`).

- [ ] **Step 3: Implement `kill.sh --gz`**

In `python/scripts/kill.sh`:

- Add `GZ_NAME="${PX4_GZ_DOCKER_NAME:-px4-noble-gz-plane}"`
- Usage line: `--gz` Remove Gazebo plane container
- Copy `kill_jsbsim_stack` as `kill_gz_stack` using `GZ_NAME` (container + `${GZ_NAME}-mavlink` + host mavlink-server pkill, same as JSBSim)
- Case `--gz) kill_gz_stack ;;`
- `--all)` also call `kill_gz_stack`

Root `kill.sh` already `exec`s extra args; `--gz` works. With no args it still kills JSBSim only (spec).

- [ ] **Step 4: Implement `runSimGzPlane.sh`**

Create `python/scripts/runSimGzPlane.sh` executable (`chmod +x`). Copy **verbatim** from `python/scripts/runSimJsbsimRascal.sh` the functions `mavlink_server_usable`, `resolve_mavlink_server_bin`, `ensure_host_mavlink_server`, and `start_mavlink_fanout` (they already use `CONTAINER_NAME`, `IMAGE_TAG`, `PYTHON_ROOT`, `MAVLINK_SERVER_SCRIPT`). Then use this unique header and docker body:

```bash
#!/bin/bash
# Start PX4 SITL + Gazebo plane (GUI). Default model: rc_cessna. In-air spawn.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PYTHON_ROOT}/.." && pwd)"
CONTAINER_NAME="${PX4_GZ_DOCKER_NAME:-px4-noble-gz-plane}"
IMAGE_TAG="${PX4_SITL_DOCKER_VER:-px4-noble-sim-ros:latest}"
MAVLINK_SERVER_SCRIPT="${REPO_ROOT}/Dockerfiles/scripts/start_mavlink_server.sh"
MAVLINK_SERVER_PID=""
MAVLINK_FANOUT="${MAVLINK_FANOUT:-0}"
GZ_MODEL="rc_cessna"
SETUP="${PYTHON_ROOT}/flightSetup.json"
POSE="${PX4_GZ_MODEL_POSE:-0,0,500,0,0,1.570796}"
HOST_GZ_ASSETS="${PYTHON_ROOT}/assets/gz"
HOST_PYTHON="${PYTHON_ROOT}"

cleanup_on_exit() {
	if [[ -n "${MAVLINK_SERVER_PID:-}" ]]; then
		kill "${MAVLINK_SERVER_PID}" 2>/dev/null || true
		wait "${MAVLINK_SERVER_PID}" 2>/dev/null || true
		MAVLINK_SERVER_PID=""
	fi
	docker rm -f "${CONTAINER_NAME}-mavlink" 2>/dev/null || true
	docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true
	xhost -local:docker 2>/dev/null || true
}

while [[ $# -gt 0 ]]; do
	case "$1" in
		--help|-h)
			echo "Usage: $0 [--model rc_cessna|advanced_plane] [--setup PATH] [--mavlink-server|--no-mavlink-server] [--kill]"
			exit 0
			;;
		--model)
			GZ_MODEL="$2"
			shift
			;;
		--setup)
			SETUP="$2"
			shift
			;;
		--mavlink-server) MAVLINK_FANOUT=1 ;;
		--no-mavlink-server) MAVLINK_FANOUT=0 ;;
		--kill) cleanup_on_exit; exit 0 ;;
		*) echo "Unknown option: $1" >&2; exit 1 ;;
	esac
	shift
done

if [[ "${GZ_MODEL}" != "rc_cessna" && "${GZ_MODEL}" != "advanced_plane" ]]; then
	echo "Error: --model must be rc_cessna or advanced_plane (got ${GZ_MODEL})" >&2
	exit 1
fi
if [[ ! -f "${SETUP}" ]]; then
	echo "Error: missing setup ${SETUP}" >&2
	exit 1
fi
if [[ -z "${DISPLAY:-}" ]]; then
	echo "Error: DISPLAY is not set (Gazebo GUI required)" >&2
	exit 1
fi

MAKE_TGT="gz_rc_cessna"
if [[ "${GZ_MODEL}" == "advanced_plane" ]]; then
	MAKE_TGT="gz_advanced_plane"
fi

# paste mavlink helpers here, then:
trap cleanup_on_exit EXIT INT TERM
docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true
if ! start_mavlink_fanout; then
	echo "Error: mavlink fan-out requested (MAVLINK_FANOUT=1) but failed to start" >&2
	exit 1
fi

xhost + 2>/dev/null || true
xhost +local:docker 2>/dev/null || true
XAUTH_FILE="${XAUTHORITY:-$HOME/.Xauthority}"
XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"
mkdir -p "${XDG_RUNTIME_DIR}"
chmod 700 "${XDG_RUNTIME_DIR}"

DOCKER_IT=(-i)
if [[ -t 0 && -z "${PX4_SITL_NO_DOCKER_TTY:-}" ]]; then
	DOCKER_IT=(-it)
fi

echo "Starting ${IMAGE_TAG} Gazebo ${GZ_MODEL} pose=${POSE} setup=${SETUP}"
docker run "${DOCKER_IT[@]}" --rm \
	--net=host --privileged --gpus all \
	--name "${CONTAINER_NAME}" \
	--env="DISPLAY=${DISPLAY}" \
	--env="QT_X11_NO_MITSHM=1" \
	--env="XAUTHORITY=${XAUTH_FILE}" \
	--env="XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR}" \
	--env="PX4_GZ_WORLD=default" \
	--env="PX4_GZ_MODEL_POSE=${POSE}" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--volume="${XDG_RUNTIME_DIR}:${XDG_RUNTIME_DIR}" \
	--volume="${HOST_GZ_ASSETS}:/opt/fixedwing/gz:rw" \
	--volume="${HOST_PYTHON}:/opt/fixedwing/python:ro" \
	--volume="${SETUP}:/opt/fixedwing/flightSetup.json:ro" \
	${XAUTH_FILE:+--volume="${XAUTH_FILE}:${XAUTH_FILE}:ro"} \
	"${IMAGE_TAG}" \
	/bin/bash -lc "set -euo pipefail
		cd /home/valentin/PX4-Autopilot
		export PYTHONPATH=/opt/fixedwing/python:/opt/fixedwing/gz/systems\${PYTHONPATH:+:\$PYTHONPATH}
		STOCK=Tools/simulation/gz/models/${GZ_MODEL}/model.sdf
		if [[ ! -f \"\${STOCK}\" ]]; then
			echo \"missing stock SDF \${STOCK}\" >&2
			exit 1
		fi
		mkdir -p /tmp/fw_gz_overlay/models/${GZ_MODEL}
		cp -a Tools/simulation/gz/models/${GZ_MODEL}/. /tmp/fw_gz_overlay/models/${GZ_MODEL}/
		python3 - <<'PY'
from pathlib import Path
import math, sys
sys.path.insert(0, '/opt/fixedwing/python')
from fw_sitl.flight_setup import load_flight_setup
from fw_sitl.gz_overlay import apply_plane_overlay
from fw_sitl.gz_pose import world_velocity_enu, DEFAULT_GZ_YAW_RAD
setup = load_flight_setup(Path('/opt/fixedwing/flightSetup.json'))
stock = Path('/tmp/fw_gz_overlay/models/${GZ_MODEL}/model.sdf')
text = stock.read_text()
out = apply_plane_overlay(
    text,
    width=setup.camera.width_px,
    height=setup.camera.height_px,
    hfov_deg=setup.camera.hfov_deg,
    eye_forward_m=setup.camera.fg_eye_forward_m,
    update_rate_hz=setup.camera.rate_hz,
)
stock.write_text(out)
vx, vy, vz = world_velocity_enu(setup.guidance.speed_mps, DEFAULT_GZ_YAW_RAD)
Path('/tmp/fw_gz_vel.env').write_text(f'export FW_GZ_SPAWN_VX={vx}\\nexport FW_GZ_SPAWN_VY={vy}\\nexport FW_GZ_SPAWN_VZ={vz}\\n')
print(f'overlay {stock} v=({vx:.1f},{vy:.1f},{vz:.1f})')
PY
		# shellcheck disable=SC1091
		source /tmp/fw_gz_vel.env
		export GZ_SIM_RESOURCE_PATH=/tmp/fw_gz_overlay/models:/opt/fixedwing/gz/models\${GZ_SIM_RESOURCE_PATH:+:\$GZ_SIM_RESOURCE_PATH}
		export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/fixedwing/gz/systems\${GZ_SIM_SYSTEM_PLUGIN_PATH:+:\$GZ_SIM_SYSTEM_PLUGIN_PATH}
		make px4_sitl ${MAKE_TGT}
	"
```

Fix the typo ` mar			shift` if it appears — the `--setup` case is `SETUP="$2"; shift` only.

The inner Python heredoc interpolates `${GZ_MODEL}` from the host bash (unquoted `'PY'` delimiter would not; use `PY` without quotes **or** pass model as argv). **Use quoted `'PY'` and pass the model path as an argument** to avoid a broken overlay:

```bash
python3 - /tmp/fw_gz_overlay/models/${GZ_MODEL}/model.sdf <<'PY'
from pathlib import Path
import sys
sys.path.insert(0, "/opt/fixedwing/python")
from fw_sitl.flight_setup import load_flight_setup
from fw_sitl.gz_overlay import apply_plane_overlay
from fw_sitl.gz_pose import world_velocity_enu, DEFAULT_GZ_YAW_RAD
stock = Path(sys.argv[1])
setup = load_flight_setup(Path("/opt/fixedwing/flightSetup.json"))
stock.write_text(apply_plane_overlay(
    stock.read_text(),
    width=setup.camera.width_px,
    height=setup.camera.height_px,
    hfov_deg=setup.camera.hfov_deg,
    eye_forward_m=setup.camera.fg_eye_forward_m,
    update_rate_hz=setup.camera.rate_hz,
))
vx, vy, vz = world_velocity_enu(setup.guidance.speed_mps, DEFAULT_GZ_YAW_RAD)
Path("/tmp/fw_gz_vel.env").write_text(
    f"export FW_GZ_SPAWN_VX={vx}\nexport FW_GZ_SPAWN_VY={vy}\nexport FW_GZ_SPAWN_VZ={vz}\n"
)
print(f"overlay {stock} v=({vx:.1f},{vy:.1f},{vz:.1f})")
PY
```

If `docker run --gpus all` fails, the script already exits non-zero; prepend a hint:

```bash
# after docker run failure the shell's set -e exits; document in README.
```

Keep `set -e` so GPU failure is loud.

- [ ] **Step 5: Run contract tests**

Run: `cd /home/valentin/Projects/FixedWing/python && python3 -m unittest tests.test_gz_sim_contracts tests.test_mavlink_fanout_contracts -v`

Expected: PASS. If `test_sim_script_exists_and_defaults` fails on a substring, align the script (do not weaken the test unless the string is a false positive).

- [ ] **Step 6: Commit** (skip unless asked)

---

### Task 5: gz camera bridge + `--mode gz`

**Files:**
- Create: `python/fw_sitl/gz_camera.py`
- Modify: `python/run_balloon_image_source.py`
- Modify: `Dockerfiles/PX4NobleSimNvidia.dockerfile` (only if in-container `python3 -c 'from gz.transport import Node'` fails on the current image — add Jetty `python3-gz-transport*` `python3-gz-msgs*` `python3-gz-sim*` and `python3-zmq` to the existing `gz-jetty` apt block)
- Test: `python/tests/test_gz_camera.py`

**Interfaces:**
- Consumes: `ImagePublisher`, `ImageFrame`, `FlightSetup.zmq.image`
- Produces:
  - `gz_image_to_rgb(width, height, step, data: bytes, pixel_format: str) -> bytes` (RGB8)
  - `find_race_cam_topic(topic_names: list[str], sensor: str = "race_cam") -> str`
  - `run_gz_publisher_via_docker(setup, *, container: str) -> int` — `docker exec` in-container `gz_camera.py`
  - CLI `gz_camera.py --endpoint --sensor --timeout-s`

- [ ] **Step 1: Write the failing test**

Create `python/tests/test_gz_camera.py`:

```python
#!/usr/bin/env python3
from __future__ import annotations

import sys
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.gz_camera import find_race_cam_topic, gz_image_to_rgb


class TestGzCamera(unittest.TestCase):
    def test_finds_sensor_image_topic(self) -> None:
        topics = [
            "/world/default/model/rc_cessna/link/race_cam_link/sensor/race_cam/image",
            "/clock",
        ]
        t = find_race_cam_topic(topics)
        self.assertIn("race_cam", t)
        self.assertTrue(t.endswith("/image") or "image" in t)

    def test_rgb_int8_passthrough(self) -> None:
        rgb = bytes([255, 0, 0, 0, 255, 0])
        out = gz_image_to_rgb(2, 1, 6, rgb, "RGB_INT8")
        self.assertEqual(out, rgb)

    def test_image_source_mode_gz(self) -> None:
        text = (_PYTHON_ROOT / "run_balloon_image_source.py").read_text(encoding="utf-8")
        self.assertIn("gz", text)
        self.assertIn("run_gz_publisher", text)
        self.assertIn("docker", text.lower())
        self.assertIn("px4-noble-gz-plane", text)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/valentin/Projects/FixedWing/python && python3 -m unittest tests.test_gz_camera -v`

Expected: FAIL.

- [ ] **Step 3: Implement `gz_camera.py` and image-source mode**

`python/fw_sitl/gz_camera.py`:

```python
"""In-container Gazebo camera → ZMQ ImageFrame (RGB8)."""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

# Allow `python3 /opt/fixedwing/python/fw_sitl/gz_camera.py` with PYTHONPATH=.../python
_PYTHON_ROOT = Path(__file__).resolve().parent.parent
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.zmq_bus import ImageFrame, ImagePublisher


def find_race_cam_topic(topic_names: list[str], sensor: str = "race_cam") -> str:
    hits = [t for t in topic_names if sensor in t and "image" in t and "info" not in t]
    if not hits:
        raise RuntimeError(f"no gz topic for sensor {sensor!r} in {topic_names[:12]}")
    hits.sort(key=len)
    return hits[0]


def gz_image_to_rgb(
    width: int, height: int, step: int, data: bytes, pixel_format: str
) -> bytes:
    fmt = (pixel_format or "RGB_INT8").upper()
    arr = np.frombuffer(data, dtype=np.uint8)
    if fmt in {"RGB_INT8", "R8G8B8", "RGB8"}:
        if arr.size < width * height * 3:
            raise ValueError("short RGB buffer")
        return bytes(arr[: width * height * 3])
    if fmt in {"BGR_INT8", "B8G8R8", "BGR8"}:
        img = arr[: width * height * 3].reshape((height, width, 3))
        return np.ascontiguousarray(img[:, :, ::-1]).tobytes()
    if fmt in {"RGBA_INT8", "R8G8B8A8"}:
        img = arr[: width * height * 4].reshape((height, width, 4))
        return np.ascontiguousarray(img[:, :, :3]).tobytes()
    raise ValueError(f"unsupported gz pixel format {pixel_format!r}")


def _import_gz():
    try:
        from gz.transport import Node
        return Node
    except ImportError:
        pass
    for mod in ("gz.transport14", "gz.transport13", "gz.transport12"):
        try:
            return __import__(mod, fromlist=["Node"]).Node
        except ImportError:
            continue
    raise ImportError(
        "gz.transport Node not found; add python3-gz-transport* to Noble image"
    )


def run_bridge(*, endpoint: str, sensor: str, timeout_s: float) -> int:
    Node = _import_gz()
    try:
        from gz.msgs.image_pb2 import Image
    except ImportError:
        from gz.msgs11.image_pb2 import Image  # type: ignore

    node = Node()
    deadline = time.time() + timeout_s
    topic = None
    while time.time() < deadline and topic is None:
        try:
            names = list(node.topic_list())
        except Exception:
            names = []
        try:
            topic = find_race_cam_topic(names, sensor)
        except RuntimeError:
            time.sleep(0.5)
    if topic is None:
        print(f"Error: no gz camera topic for {sensor} within {timeout_s:.0f}s", file=sys.stderr)
        return 1

    pub = ImagePublisher(endpoint)
    got = {"n": 0}

    def _cb(msg: Image) -> None:
        rgb = gz_image_to_rgb(
            int(msg.width),
            int(msg.height),
            int(getattr(msg, "step", 0) or 0),
            bytes(msg.data),
            str(getattr(msg, "pixel_format_type", "") or getattr(msg, "pixel_format", "RGB_INT8")),
        )
        frame = ImageFrame(
            stamp=time.time(),
            width=int(msg.width),
            height=int(msg.height),
            rgb=rgb,
        )
        pub.publish(frame)
        got["n"] += 1

    if not node.subscribe(Image, topic, _cb):
        print(f"Error: subscribe failed {topic}", file=sys.stderr)
        return 1
    print(f"gz_camera subscribed {topic} → {endpoint}")
    t0 = time.time()
    while True:
        time.sleep(0.05)
        if got["n"] == 0 and (time.time() - t0) > timeout_s:
            print(f"Error: no camera image within {timeout_s:.0f}s", file=sys.stderr)
            return 1


def run_gz_publisher_via_docker(
    setup,
    *,
    container: str = "px4-noble-gz-plane",
    timeout_s: float = 30.0,
) -> int:
    import subprocess

    cmd = [
        "docker",
        "exec",
        "-i",
        container,
        "env",
        "PYTHONPATH=/opt/fixedwing/python",
        "python3",
        "/opt/fixedwing/python/fw_sitl/gz_camera.py",
        "--endpoint",
        setup.zmq.image,
        "--sensor",
        "race_cam",
        "--timeout-s",
        str(timeout_s),
    ]
    print("Starting gz camera bridge:", " ".join(cmd))
    return int(subprocess.call(cmd))


def main() -> int:
    p = argparse.ArgumentParser(description="gz race_cam → ZMQ image")
    p.add_argument("--endpoint", required=True)
    p.add_argument("--sensor", default="race_cam")
    p.add_argument("--timeout-s", type=float, default=30.0)
    args = p.parse_args()
    return run_bridge(
        endpoint=args.endpoint, sensor=args.sensor, timeout_s=args.timeout_s
    )


if __name__ == "__main__":
    raise SystemExit(main())
```

`gz.msgs.Image` field names (`pixel_format_type` vs enum) vary; `gz_image_to_rgb` stays unit-tested. If subscribe API is `node.subscribe(topic, Image, cb)` argument order reversed, match Jetty docs inside `run_bridge` only.

In `run_balloon_image_source.py`:

- `choices=("synth", "fg", "gz")`
- add `--container` default `px4-noble-gz-plane`
- `from fw_sitl.gz_camera import run_gz_publisher_via_docker`
- if `args.mode == "gz": return run_gz_publisher_via_docker(setup, container=args.container)`

- [ ] **Step 4: Image python-gz packages if needed**

From the host: `docker run --rm px4-noble-sim-ros:latest python3 -c "from gz.transport import Node; import zmq"`

If that fails, in `Dockerfiles/PX4NobleSimNvidia.dockerfile` on the `gz-jetty` apt install list, add the Jetty Python bindings and `python3-zmq` (names as `apt-cache search python3-gz` inside a throwaway container). Rebuild is a nested-repo change; mention it in `Dockerfiles/UPDATES.md` only if the Dockerfile actually changes.

- [ ] **Step 5: Run tests**

Run: `cd /home/valentin/Projects/FixedWing/python && python3 -m unittest tests.test_gz_camera -v`

Expected: PASS.

- [ ] **Step 6: Commit** (skip unless asked)

---

### Task 6: Race `--gz` + control spawn/airspeed

**Files:**
- Modify: `python/scripts/run_balloon_race.sh`
- Modify: `python/run_balloon_control.py`
- Modify: `python/fw_sitl/mavlink_io.py` (`wait_min_airspeed`)
- Modify: `python/tests/test_mavlink_fanout_contracts.py`
- Test: `python/tests/test_gz_race_contracts.py`

**Interfaces:**
- Consumes: `spawn_balloons_gz` (Task 3), `wait_min_airspeed`, `runSimGzPlane.sh` (Task 4)
- Produces: race flags `--gz` `--model`; control `--gz` `--spawn-gz-balloons`; airspeed abort before engage

- [ ] **Step 1: Write failing tests**

`python/tests/test_gz_race_contracts.py`:

```python
#!/usr/bin/env python3
from __future__ import annotations

import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
_RACE = _PYTHON_ROOT / "scripts" / "run_balloon_race.sh"
_CTL = _PYTHON_ROOT / "run_balloon_control.py"
_MAV = _PYTHON_ROOT / "fw_sitl" / "mavlink_io.py"


class TestGzRaceContracts(unittest.TestCase):
    def test_race_gz_wiring(self) -> None:
        text = _RACE.read_text(encoding="utf-8")
        self.assertIn("--gz", text)
        self.assertIn("runSimGzPlane.sh", text)
        self.assertIn("--mode gz", text)
        self.assertIn("--spawn-gz-balloons", text)
        self.assertIn("--viz and --gz are mutually exclusive", text)
        self.assertIn('CTL_CMD+=" --no-sim"', text)
        self.assertIn("--setup", text)
        self.assertIn("px4-noble-gz-plane", text)

    def test_control_gz_after_engage(self) -> None:
        ctl = _CTL.read_text(encoding="utf-8")
        self.assertIn("spawn_balloons_gz(", ctl)
        self.assertGreater(
            ctl.index("spawn_balloons_gz("),
            ctl.index("engage_offboard_with_retries"),
        )
        self.assertIn("wait_min_airspeed", ctl)
        self.assertIn("--spawn-gz-balloons", ctl)
        self.assertIn("accept_unhealthy", ctl)

    def test_wait_min_airspeed_exists(self) -> None:
        self.assertIn("def wait_min_airspeed", _MAV.read_text(encoding="utf-8"))
        self.assertIn("in-air spawn has no airspeed", _MAV.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
```

Add a **subprocess** test in the same file (or race script unit via bash):

```python
    def test_viz_and_gz_exit_2(self) -> None:
        import subprocess
        r = subprocess.run(
            ["bash", str(_RACE), "--viz", "--gz"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(r.returncode, 2)

    def test_model_without_gz_exit_2(self) -> None:
        import subprocess
        r = subprocess.run(
            ["bash", str(_RACE), "--model", "rc_cessna"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(r.returncode, 2)
```

These must parse flags **before** `tmux` / `set -e` paths that need tmux. Put the mutual-exclusion check immediately after the `while` parse loop.

Also extend `test_mavlink_fanout_contracts.py` `mavlink_fanout_up` contract: race script must grep `px4-noble-gz-plane-mavlink` as well as the JSBSim sidecar name.

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/valentin/Projects/FixedWing/python && python3 -m unittest tests.test_gz_race_contracts -v`

Expected: FAIL.

- [ ] **Step 3: `wait_min_airspeed` in `mavlink_io.py`**

```python
def wait_min_airspeed(
    master: mavutil.mavfile,
    *,
    min_mps: float = 15.0,
    timeout_s: float = 5.0,
) -> float:
    """Abort in-air gz spawn if VFR_HUD airspeed (else groundspeed) never reaches min_mps."""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
        100000,
        0,
        0,
        0,
        0,
        0,
    )
    deadline = time.time() + timeout_s
    last = 0.0
    while time.time() < deadline:
        msg = master.recv_match(type="VFR_HUD", blocking=True, timeout=0.5)
        if msg is None:
            continue
        last = float(getattr(msg, "airspeed", 0.0) or 0.0)
        if last < 1e-3:
            last = float(getattr(msg, "groundspeed", 0.0) or 0.0)
        if last >= min_mps:
            return last
    raise RuntimeError(
        f"in-air spawn has no airspeed (last={last:.1f} m/s, need >={min_mps:.0f})"
    )
```

Add a small unittest in `python/tests/test_gz_race_contracts.py` that imports the function (existence is enough; do not require a live MAVLink).

- [ ] **Step 4: Race launcher**

In `python/scripts/run_balloon_race.sh`:

- `GZ=0`, `GZ_MODEL="rc_cessna"`
- usage: `--gz`, `--model rc_cessna|advanced_plane`
- parse `--gz) GZ=1; MODE="gz" ;;` and `--model) GZ_MODEL="$2"; shift ;;`
- **Immediately after parse:**

```bash
if [[ "${VIZ}" -eq 1 && "${GZ}" -eq 1 ]]; then
  echo "Error: --viz and --gz are mutually exclusive" >&2
  exit 2
fi
if [[ "${GZ_MODEL}" != "rc_cessna" && "${GZ}" -eq 0 ]]; then
  # default GZ_MODEL is rc_cessna; only error if user passed --model without --gz
  true
fi
```

Track `MODEL_SET=0` on `--model` and:

```bash
if [[ "${MODEL_SET}" -eq 1 && "${GZ}" -eq 0 ]]; then
  echo "Error: --model requires --gz" >&2
  exit 2
fi
```

- `mavlink_fanout_up`: also `grep -qx "${PX4_GZ_DOCKER_NAME:-px4-noble-gz-plane}-mavlink"`
- When `GZ=1`: `CONTAINER_NAME="${PX4_GZ_DOCKER_NAME:-px4-noble-gz-plane}"`
- `SIM_CMD="MAVLINK_FANOUT=${MAVLINK_FANOUT} bash ${PYTHON_ROOT}/scripts/runSimGzPlane.sh --mavlink-server --setup ${SETUP} --model ${GZ_MODEL}"`
- `IMG_CMD` already uses `MODE`; gz → `--mode gz`
- `CTL_CMD+=" --gz --spawn-gz-balloons"` when GZ=1 (keep `--no-sim`)
- Do not add `--viz` / `--spawn-fg-balloons` on this path

- [ ] **Step 5: Control**

In `run_balloon_control.py`:

- `--gz` store_true; `--spawn-gz-balloons` store_true; `--gz-container` default `px4-noble-gz-plane`
- If `args.gz`: `KILL_TARGET = "--gz"` (local variable, do not mutate module constant blindly — use `kill_target = "--gz" if args.gz else KILL_TARGET`) and `args.sim` default override: if user left default JSBSim script and `--gz`, set `args.sim = SCRIPTS_DIR / "runSimGzPlane.sh"`
- `kill_docker(target=kill_target)`
- `sim_extra = ["--viz"] if args.viz else []`; if gz: `sim_extra = ["--setup", str(args.setup)]` (and `--model` only if you add `--model` to control; race owns model via sim script — solo control can pass `--sim-arg` later; YAGNI: document `python3 run_balloon_control.py --gz` uses Cessna default via `runSimGzPlane.sh` defaults)
- After `connect`, if `args.gz`: `from fw_sitl.mavlink_io import wait_min_airspeed` and `wait_min_airspeed(master)` — on `RuntimeError`, print and return 1 **before** engage
- Engage: treat `args.gz` like `args.viz` (`accept_unhealthy=True`, `full_sim_restart=False`, 60 s arm, skip reboot if `args.gz and args.no_sim`)
- After rebase, if `args.spawn_gz_balloons or args.gz`: background thread `spawn_balloons_gz(race_balloons, container=args.gz_container)` named `gz-balloon-spawn`; catch Exception and print `GZ balloon spawn warning` / `world balloons are missing`

- [ ] **Step 6: Run tests**

Run: `cd /home/valentin/Projects/FixedWing/python && python3 -m unittest tests.test_gz_race_contracts tests.test_mavlink_fanout_contracts tests.test_gz_sim_contracts -v`

Expected: PASS. `test_viz_and_gz_exit_2` returns 2 without starting tmux.

- [ ] **Step 7: Commit** (skip unless asked)

---

### Task 7: Straight-flight runner + docs

**Files:**
- Create: `python/run_straight_flight_gz.py`
- Modify: `README.md`
- Modify: `UPDATES.md` (feature bump: current top is `0.17.6` → `0.18.0`)
- Modify: `Dockerfiles/README.md` only if Task 5 changed the image
- Test: `python/tests/test_gz_straight_flight.py` (help string / constants contract)

**Interfaces:**
- Consumes: `runSimGzPlane.sh`, `run_locked_line_hold` (same as YASim)
- Produces: `python3 python/run_straight_flight_gz.py` with `KILL_TARGET="--gz"`

- [ ] **Step 1: Write failing test**

```python
#!/usr/bin/env python3
from __future__ import annotations

import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
_RUN = _PYTHON_ROOT / "run_straight_flight_gz.py"


class TestGzStraightFlight(unittest.TestCase):
    def test_script_uses_gz_sim_and_kill(self) -> None:
        text = _RUN.read_text(encoding="utf-8")
        self.assertIn("runSimGzPlane.sh", text)
        self.assertIn('KILL_TARGET = "--gz"', text)
        self.assertIn("run_locked_line_hold", text)
        self.assertIn("accept_unhealthy=True", text)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run to verify fail**

Run: `cd /home/valentin/Projects/FixedWing/python && python3 -m unittest tests.test_gz_straight_flight -v`

Expected: FAIL (file missing).

- [ ] **Step 3: Implement runner**

Copy `python/run_straight_flight_yasim.py` to `python/run_straight_flight_gz.py` and change:

- Docstring / usage: Gazebo PX4 plane, GUI always
- `DEFAULT_SIM = SCRIPTS_DIR / "runSimGzPlane.sh"`
- `KILL_TARGET = "--gz"`
- `plot_title="Gazebo PX4 plane straight flight"`
- `arm_timeout_s=60.0`, `accept_unhealthy=True`, `full_sim_restart=False`, `max_attempts=1`
- Optional `--model` appended to `sim_extra_args` (`["--model", args.model]` if not default)

- [ ] **Step 4: Docs**

`README.md` Architecture: add Gazebo plant bullet — `runSimGzPlane.sh`, `./run_balloon_race.sh --gz`, camera=sensor not GUI grab, container `px4-noble-gz-plane`, `--model advanced_plane`.

`UPDATES.md` new top:

```markdown
## 0.18.0 - Gazebo PX4 plane plant (balloon race --gz)
- Additive plant: `runSimGzPlane.sh` / `run_straight_flight_gz.py` (Cessna default, `--model advanced_plane`).
- Race: `./run_balloon_race.sh --gz` — onboard `race_cam` → ZMQ, balloons in gz world, GUI for operator.
- In-air spawn pose `0,0,500,0,0,1.570796` + spawn velocity; `kill.sh --gz`.
```

- [ ] **Step 5: Run full unit suite**

Run: `cd /home/valentin/Projects/FixedWing/python && python3 -m unittest discover -s tests -v`

Expected: all PASS (including previous JSBSim/FG contracts).

- [ ] **Step 6: Manual smoke (not a unit gate)**

1. `python/scripts/runSimGzPlane.sh` — GUI, HEARTBEAT on 14550, airspeed > 15 m/s.
2. `./run_balloon_race.sh --gz` — three colored balloons, camera frames, OFFBOARD engage.

- [ ] **Step 7: Commit** (skip unless asked)

---

## Self-review (spec coverage)

| Spec item | Task |
|-----------|------|
| `runSimGzPlane.sh`, NVIDIA, X11, DISPLAY fail, pose north, overlay, fan-out | 4 |
| Overlay `race_cam` + initial V / PythonSystemLoader | 2, 4 |
| In-container ZMQ bridge, `--mode gz`, 30 s timeout | 5 |
| Balloon SDF RGB, NED→ENU, clear-then-place | 1, 3 |
| Race `--gz` / `--model` / `--setup`, mutex `--viz`, fan-out sidecar name | 6 |
| Control `--gz`, spawn after engage, viz-like engage, airspeed gate | 6 |
| `kill.sh --gz` / `--all` | 4 |
| `run_straight_flight_gz.py` | 7 |
| README / UPDATES 0.18.0 | 7 |
| python3-gz in image if missing | 5 |
| No host gz.transport, no GUI grab, no runway, no plant ABC | constraints |

No TBD/TODO left in tasks. `PythonSystemLoader` ECM API is the only runtime-fragile bit; contract tests lock XML + `velocity_from_env`; airspeed gate in Task 6 is the acceptance backstop.
