# Pure-Pursuit Guidance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace balloon-race on-target attitude chase with a pure-pursuit accel → attitude → thrust/speed pipeline, and move all plant calibration into commented JSONC files.

**Architecture:** Pluggable `PurePursuitAccel` produces \(\mathbf{a}_\mathrm{des}=k(\hat u-\hat v)\). `attitude_from_accel` (polar default + geometric) yields \(q_\mathrm{des}\). `thrust_energy` computes \(T\) from \(\dot V_a=a_\parallel\), \(D(\alpha)\), \(\gamma\), with a stall/thrust speed governor. `AttitudeChaseController` uses this when chasing; path-hold stays old. Plants load from `fw_sitl/plants/*.jsonc`.

**Tech Stack:** Python 3, numpy not required (stdlib + existing `fw_sitl.quat`), unittest, JSONC (comment-stripped JSON).

## Global Constraints

- Spec: `docs/superpowers/specs/2026-08-22-pure-pursuit-guidance-design.md` (locked decisions).
- Host tests: `cd python && python3 -m unittest discover -s tests`.
- Do **not** `git commit` unless the user explicitly asks.
- Do **not** replace path-hold or straight-flight in v1.
- Do **not** delete `q_des_from_los` / `chase_speed_mps` in the first pass (path/tests still import).
- PP thrust uses \(a_\parallel=\mathbf{a}_\mathrm{des}\cdot\hat v\), not body \(a_\mathrm{axial}\).
- Every plant JSONC key must have an in-file `//` or `/* */` comment.
- After meaningful code lands: bump `UPDATES.md` (feature → subver) and touch `README.md` only if architecture blurb needs the PP pipeline named.

---

## File map

| Path | Role |
|------|------|
| `python/fw_sitl/plants/*.jsonc` | Per-plant calibration (all old gains + aero/PP/governor) |
| `python/fw_sitl/plant_loader.py` | JSONC strip + validate → `PlantGains` |
| `python/fw_sitl/plant_gains.py` | `PlantGains` dataclass + `load_plant_gains` / flags façade |
| `python/fw_sitl/accel_laws.py` | Pure pursuit + \(a_\parallel\) / \(a_\perp\) split |
| `python/fw_sitl/attitude_from_accel.py` | Polar + geometric \(a\to q\) |
| `python/fw_sitl/thrust_energy.py` | \(D\), \(T\), speed governor |
| `python/fw_sitl/body_cmd_controllers.py` | Wire PP pipeline on chase branch |
| `python/tests/test_plant_loader.py` | JSONC + aero fields |
| `python/tests/test_accel_laws.py` | PP unit tests |
| `python/tests/test_attitude_from_accel.py` | Polar/geometric |
| `python/tests/test_thrust_energy.py` | Thrust + governor |
| `python/tests/test_body_cmd_controllers.py` | `last_law` starts with `pp` |
| `python/tests/test_plant_gains.py` | Still passes via JSONC load |
| `README.md` / `UPDATES.md` | Agent docs |

---

### Task 1: JSONC loader + `PlantGains` aero fields + one plant file

**Files:**
- Create: `python/fw_sitl/plant_loader.py`
- Create: `python/fw_sitl/plants/jsbsim_rascal.jsonc`
- Modify: `python/fw_sitl/plant_gains.py` (`PlantGains` fields + fingerprint)
- Create: `python/tests/test_plant_loader.py`

**Interfaces:**
- Consumes: none new
- Produces:
  - `strip_jsonc(text: str) -> str`
  - `load_plant_jsonc(path: Path) -> dict`
  - `plant_gains_from_dict(data: dict) -> PlantGains`
  - `PlantGains` gains new frozen fields: `mass_kg`, `wing_area_m2`, `cd0`, `k_induced`, `cl_alpha`, `rho_kg_m3`, `t_max_n`, `v_stall_mps`, `pp_gain`, `thrust_target_frac`, `v_min_mult`, `v_recover_mult`, `v_up_mps_s`, `attitude_from_accel` (`str`), `alpha_small_rad`

- [ ] **Step 1: Write failing tests** in `python/tests/test_plant_loader.py`:

```python
#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.plant_loader import plant_gains_from_dict, strip_jsonc


class TestStripJsonc(unittest.TestCase):
    def test_line_and_block_comments(self) -> None:
        raw = '// head\n{"a": 1, /* mid */ "b": 2}\n'
        self.assertEqual(json.loads(strip_jsonc(raw)), {"a": 1, "b": 2})

    def test_comment_inside_string_preserved(self) -> None:
        raw = '{"s": "not // a comment"}'
        self.assertEqual(json.loads(strip_jsonc(raw))["s"], "not // a comment")


class TestPlantGainsFromDict(unittest.TestCase):
    def _minimal(self) -> dict:
        return {
            "plant_id": "jsbsim_rascal",
            "pid_kp": 0.8,
            "pid_ki": 0.12,
            "pid_kd": 0.04,
            "bank_kp_heading": 1.0,
            "bank_kp_cross_track": 0.003,
            "bank_xt_lookahead_m": 180.0,
            "bank_max_roll_rad": 0.62,
            "bank_kp_alt": 0.028,
            "bank_max_pitch_rad": 0.12,
            "att_max_pitch_rad": 0.35,
            "att_los_max_pitch_rad": 0.70,
            "cruise_thrust": 0.62,
            "climb_thrust_per_m": 0.020,
            "min_thrust": 0.22,
            "max_thrust": 1.0,
            "speed_mps": 18.0,
            "approach_speed_mps": 15.0,
            "slow_range_m": 180.0,
            "speed_thrust_per_mps": 0.05,
            "lookahead_m": 500.0,
            "fw_airspd_min": 10.0,
            "fw_airspd_trim": 18.0,
            "fw_airspd_max": 40.0,
            "visual_lock_kp_alt": 0.020,
            "px4_inner": [["FW_THR_TRIM", 0.62]],
            "mass_kg": 13.0,
            "wing_area_m2": 1.0,
            "cd0": 0.05,
            "k_induced": 0.08,
            "cl_alpha": 5.0,
            "rho_kg_m3": 1.225,
            "t_max_n": 40.0,
            "v_stall_mps": 10.0,
            "pp_gain": 2.0,
            "thrust_target_frac": 0.8,
            "v_min_mult": 1.1,
            "v_recover_mult": 1.2,
            "v_up_mps_s": 0.5,
            "attitude_from_accel": "polar",
            "alpha_small_rad": 0.087,
        }

    def test_builds_plant(self) -> None:
        p = plant_gains_from_dict(self._minimal())
        self.assertEqual(p.plant_id, "jsbsim_rascal")
        self.assertAlmostEqual(p.mass_kg, 13.0)
        self.assertEqual(p.attitude_from_accel, "polar")
        self.assertEqual(p.px4_inner, (("FW_THR_TRIM", 0.62),))

    def test_missing_aero_fails(self) -> None:
        data = self._minimal()
        del data["mass_kg"]
        with self.assertRaises(KeyError):
            plant_gains_from_dict(data)

    def test_bad_attitude_mode_fails(self) -> None:
        data = self._minimal()
        data["attitude_from_accel"] = "euler"
        with self.assertRaises(ValueError):
            plant_gains_from_dict(data)
```

- [ ] **Step 2: Run — expect FAIL** (`ModuleNotFoundError: plant_loader`)

```bash
cd python && python3 -m unittest tests.test_plant_loader -v
```

- [ ] **Step 3: Implement `strip_jsonc` + `plant_gains_from_dict`**

In `plant_loader.py`, strip `//` and `/* */` outside of strings (character scanner; do not use naive regex that eats strings). Map dict → `PlantGains`; require all aero keys; `attitude_from_accel` must be `"polar"` or `"geometric"`; convert `px4_inner` list-of-pairs to `tuple[tuple[str, float], ...]`.

Extend `PlantGains` with the new fields and include them in `fingerprint()`.

Keep `_PLANTS` in `plant_gains.py` working for now by filling new fields with the same seed values as `jsbsim_rascal.jsonc` for every in-code plant (temporary), **or** only extend the dataclass and update the one in-code `jsbsim_rascal` entry first — prefer: add fields with seeds on **all** `_PLANTS` entries so existing `load_plant_gains` does not break mid-migration.

Create `plants/jsbsim_rascal.jsonc` with every key commented. Seed aero (tunable later):

```text
mass_kg=13, wing_area_m2=1.0, cd0=0.05, k_induced=0.08, cl_alpha=5.0,
rho_kg_m3=1.225, t_max_n=40, v_stall_mps=10, pp_gain=2.0,
thrust_target_frac=0.8, v_min_mult=1.1, v_recover_mult=1.2, v_up_mps_s=0.5,
attitude_from_accel="polar", alpha_small_rad=0.087 (~5°)
```

Plus all existing Rascal control values from `_PLANTS["jsbsim_rascal"]`.

- [ ] **Step 4: Run — PASS**

```bash
cd python && python3 -m unittest tests.test_plant_loader -v
```

- [ ] **Step 5: Commit only if user asks**

---

### Task 2: Migrate all plants to JSONC; thin `load_plant_gains`

**Files:**
- Create: `python/fw_sitl/plants/{jsbsim_rascal_viz,yasim_rascal,gz_rc_cessna,gz_advanced_plane,xplane_cessna172}.jsonc`
- Modify: `python/fw_sitl/plant_gains.py` — remove `_PLANTS` dict body; `load_plant_gains` reads JSONC
- Modify: `python/tests/test_plant_gains.py` — add assertions for aero fields on one plant; keep race snapshots
- Modify: `python/tests/test_plant_loader.py` — load real file from disk

**Interfaces:**
- Consumes: `plant_gains_from_dict`, `strip_jsonc`
- Produces: `load_plant_gains(plant_id) -> PlantGains` from `Path(__file__).parent / "plants" / f"{plant_id}.jsonc"`

- [ ] **Step 1: Failing test** — real file load + aero on registry:

```python
# in test_plant_loader.py
def test_load_jsbsim_rascal_jsonc_file(self) -> None:
    from fw_sitl.plant_gains import load_plant_gains
    p = load_plant_gains("jsbsim_rascal")
    self.assertAlmostEqual(p.v_stall_mps, 10.0)
    self.assertEqual(p.attitude_from_accel, "polar")

# in test_plant_gains.py TestPlantGainsRegistry
def test_every_plant_has_aero_and_pp(self) -> None:
    for pid in _PLANTS:
        p = load_plant_gains(pid)
        self.assertGreater(p.mass_kg, 0.0)
        self.assertGreater(p.t_max_n, 0.0)
        self.assertGreater(p.v_stall_mps, 0.0)
        self.assertGreater(p.pp_gain, 0.0)
        self.assertIn(p.attitude_from_accel, ("polar", "geometric"))
```

- [ ] **Step 2: Run — FAIL** until files exist / loader wired (or pass aero if Task 1 seeded in-code only — then Step 2 fails on missing files after `_PLANTS` removal)

- [ ] **Step 3: Write remaining JSONC files**

Copy control numbers from current `_PLANTS` entries. Aero seeds:

| plant | mass_kg | S | cd0 | t_max_n | v_stall |
|-------|---------|---|-----|---------|---------|
| jsbsim_rascal / viz | 13 | 1.0 | 0.05 | 40 | 10 |
| yasim_rascal | 13 | 1.0 | 0.05 | 45 | 10 |
| gz_rc_cessna | 10 | 0.8 | 0.06 | 30 | 8 |
| gz_advanced_plane | 12 | 0.9 | 0.05 | 35 | 10 |
| xplane_cessna172 | 1043 | 16.2 | 0.03 | 4000 | 25 |

Shared PP/governor defaults: `pp_gain=2.0`, `thrust_target_frac=0.8`, `v_min_mult=1.1`, `v_recover_mult=1.2`, `v_up_mps_s=0.5`, `attitude_from_accel="polar"`, `alpha_small_rad=0.087`, `cl_alpha=5.0`, `k_induced=0.08`, `rho_kg_m3=1.225`.

`load_plant_gains`:

```python
def load_plant_gains(plant_id: str) -> PlantGains:
    pid = str(plant_id)
    if pid not in KNOWN_PLANT_IDS:
        raise KeyError(f"unknown plant {pid!r}; expected one of {KNOWN_PLANT_IDS}")
    path = Path(__file__).resolve().parent / "plants" / f"{pid}.jsonc"
    if not path.is_file():
        raise FileNotFoundError(f"plant file missing: {path}")
    return plant_gains_from_dict(load_plant_jsonc(path))
```

Delete `_PLANTS` and `_JSB_INNER` etc. from `plant_gains.py` once JSONC holds them.

- [ ] **Step 4: Run plant tests — PASS**

```bash
cd python && python3 -m unittest tests.test_plant_gains tests.test_plant_loader -v
```

- [ ] **Step 5: Commit only if user asks**

---

### Task 3: Pure-pursuit accel law

**Files:**
- Create: `python/fw_sitl/accel_laws.py`
- Create: `python/tests/test_accel_laws.py`

**Interfaces:**
- Consumes: none
- Produces:
  - `Vec3 = tuple[float, float, float]`
  - `normalize(v: Vec3) -> Vec3`
  - `pure_pursuit_accel(u_hat: Vec3, v_hat: Vec3, *, gain: float) -> Vec3`
  - `split_parallel_perp(a: Vec3, v_hat: Vec3) -> tuple[float, Vec3]`  # (a_parallel, a_perp)

- [ ] **Step 1: Failing tests**

```python
def test_aligned_near_zero(self) -> None:
    a = pure_pursuit_accel((1, 0, 0), (1, 0, 0), gain=2.0)
    self.assertAlmostEqual(math.hypot(*a), 0.0, places=9)

def test_misaligned_has_lateral(self) -> None:
    a = pure_pursuit_accel((0, 1, 0), (1, 0, 0), gain=2.0)
    a_par, a_perp = split_parallel_perp(a, (1, 0, 0))
    self.assertAlmostEqual(a_par, -2.0)  # 2*((0,1,0)-(1,0,0))·(1,0,0) = 2*(-1)
    self.assertGreater(math.hypot(*a_perp), 1.0)

def test_gain_scales(self) -> None:
    a1 = pure_pursuit_accel((0, 1, 0), (1, 0, 0), gain=1.0)
    a2 = pure_pursuit_accel((0, 1, 0), (1, 0, 0), gain=3.0)
    self.assertAlmostEqual(a2[0], 3.0 * a1[0])
```

- [ ] **Step 2: Run — FAIL**

```bash
cd python && python3 -m unittest tests.test_accel_laws -v
```

- [ ] **Step 3: Implement**

```python
def pure_pursuit_accel(u_hat, v_hat, *, gain: float) -> Vec3:
    k = float(gain)
    return (k * (u_hat[0] - v_hat[0]), k * (u_hat[1] - v_hat[1]), k * (u_hat[2] - v_hat[2]))
```

Callers normalize \(\hat u,\hat v\) before calling (document that). Provide `normalize` that returns `(1,0,0)` if \(\|v\|<\epsilon\).

- [ ] **Step 4: Run — PASS**

- [ ] **Step 5: Commit only if user asks**

---

### Task 4: Attitude from accel (polar + geometric)

**Files:**
- Create: `python/fw_sitl/attitude_from_accel.py`
- Create: `python/tests/test_attitude_from_accel.py`

**Interfaces:**
- Consumes: `fw_sitl.quat.from_rpy`, `rpy_from_quat`
- Produces:
  - `AttitudeFromAccelResult` dataclass: `q_des: Quat`, `phi_c: float`, `theta_c: float`, `a_axial: float`
  - `polar_from_accel(a_des: Vec3, psi_c: float, *, g: float = 9.81) -> AttitudeFromAccelResult`
  - `geometric_from_accel(a_des: Vec3, psi_c: float, *, g: float = 9.81) -> AttitudeFromAccelResult`
  - `attitude_from_accel(a_des, psi_c, *, mode: str, g=9.81, max_roll=..., max_pitch=...) -> AttitudeFromAccelResult` — dispatches; clamps \(\phi,\theta\) then rebuilds `q_des` via `from_rpy`

NED gravity vector: `g_vec = (0, 0, +g)`.

Implement polar/geometric as in the approved sample (vehicle-1/2 polar; \(k^d\) geometric). On geometric near-zero \(\|\mathbf{a}_\mathrm{des}-\mathbf{g}\|\): fall back to `polar_from_accel`.

- [ ] **Step 1: Failing tests**

```python
def test_level_forward_small_roll(self) -> None:
    # a_des ≈ 0 → total force ≈ -g_vec wait: A = a_des - g_vec = (0,0,-g) in NED if a_des=0
    # Use mild right turn climb from sample
    a = (0.5, 2.0, -1.5)
    psi = math.radians(45.0)
    r1 = polar_from_accel(a, psi)
    r2 = geometric_from_accel(a, psi)
    self.assertGreater(abs(r1.q_des[0] * r2.q_des[0] + r1.q_des[1] * r2.q_des[1]
                           + r1.q_des[2] * r2.q_des[2] + r1.q_des[3] * r2.q_des[3]), 0.95)
    self.assertLess(abs(r1.a_axial - r2.a_axial), 0.5)

def test_dispatcher_clamps_roll(self) -> None:
    a = (0.0, 20.0, -1.0)  # huge lateral
    r = attitude_from_accel(a, 0.0, mode="polar", max_roll=0.4, max_pitch=0.5)
    self.assertLessEqual(abs(r.phi_c), 0.4 + 1e-9)
```

- [ ] **Step 2: Run — FAIL**

```bash
cd python && python3 -m unittest tests.test_attitude_from_accel -v
```

- [ ] **Step 3: Implement** (stdlib math only; no numpy dependency)

Port the sample algorithms to tuples. Use existing `from_rpy(phi, theta, psi)` for polar quaternion (matches 3-2-1). Geometric: build `R` columns, extract quaternion with trace/branch logic from the sample.

- [ ] **Step 4: Run — PASS**

- [ ] **Step 5: Commit only if user asks**

---

### Task 5: Thrust energy + speed governor

**Files:**
- Create: `python/fw_sitl/thrust_energy.py`
- Create: `python/tests/test_thrust_energy.py`

**Interfaces:**
- Consumes: plant aero fields (passed as scalars or a small `AeroParams` view)
- Produces:
  - `drag_n(*, rho, v, s, cd) -> float`
  - `cd_from_alpha(*, alpha, cd0, k_induced, cl_alpha, alpha_small, a_des, mass, rho, s) -> float`
  - `thrust_n(*, mass, a_parallel, drag, gamma, alpha) -> float`  # \((m a + D + m g \sin\gamma)/\cos\alpha\) with \(\alpha\) clamped
  - `SpeedGovernor` class holding `v_cmd: float` with method:

```python
def step(
    self,
    *,
    a_parallel: float,
    a_des: Vec3,
    v_meas: float,
    gamma: float,
    theta: float,
    dt: float,
    mass_kg: float,
    wing_area_m2: float,
    cd0: float,
    k_induced: float,
    cl_alpha: float,
    rho_kg_m3: float,
    t_max_n: float,
    v_stall_mps: float,
    thrust_target_frac: float,
    v_min_mult: float,
    v_recover_mult: float,
    v_up_mps_s: float,
    alpha_small_rad: float,
    v_cruise_mps: float,
    min_thrust: float,
    max_thrust: float,
) -> tuple[float, float]:  # (thrust_frac, v_cmd)
```

Governor order (exact):

1. If `v_cmd < v_min_mult * v_stall` (or `v_meas` if you initialize from meas): set `v_cmd = v_recover_mult * v_stall`.
2. Compute \(T(V_\mathrm{cmd})\). If \(T > t_\mathrm{max}\): binary-search / iterate \(V\) down until \(T \le \texttt{thrust_target_frac}\cdot t_\mathrm{max}\) or \(V = v_\mathrm{recover}\cdot V_s\).
3. Elif \(T < \texttt{thrust_target_frac}\cdot t_\mathrm{max}\): `v_cmd = min(v_cruise, v_cmd + v_up_mps_s * dt)`.
4. Recompute \(T\); `thrust_frac = clip(T/t_max_n, min_thrust, max_thrust)`.

\(\alpha=\theta-\gamma\); if \(|\alpha|<\alpha_\mathrm{small}\): \(\cos\alpha=1\), \(C_D=C_{D0}+k C_L^2\) with \(C_L = m\|\mathbf{a}_\mathrm{des}-\mathbf{g}\|/(q S)\) capped, else \(C_L=C_{L\alpha}\alpha\).

- [ ] **Step 1: Failing tests**

```python
def test_level_cruise_thrust_approx_drag(self) -> None:
    # a_par=0, gamma=0, alpha=0 → T ≈ D
    T = thrust_n(mass=13.0, a_parallel=0.0, drag=20.0, gamma=0.0, alpha=0.0)
    self.assertAlmostEqual(T, 20.0, places=6)

def test_climb_increases_thrust(self) -> None:
    T0 = thrust_n(mass=13.0, a_parallel=0.0, drag=20.0, gamma=0.0, alpha=0.0)
    T1 = thrust_n(mass=13.0, a_parallel=0.0, drag=20.0, gamma=0.2, alpha=0.0)
    self.assertGreater(T1, T0)

def test_overthrust_reduces_speed(self) -> None:
    gov = SpeedGovernor(v_cmd=30.0)
    # Use tiny t_max so T(V) saturates
    thr, v = gov.step(..., t_max_n=5.0, v_stall_mps=10.0, v_cruise_mps=30.0, ...)
    self.assertLess(v, 30.0)
    self.assertGreaterEqual(v, 1.2 * 10.0 - 1e-6)
    self.assertLessEqual(thr, 1.0)

def test_below_min_stall_recovers(self) -> None:
    gov = SpeedGovernor(v_cmd=10.0)  # < 1.1*10
    thr, v = gov.step(..., v_stall_mps=10.0, ...)
    self.assertAlmostEqual(v, 12.0, places=5)

def test_underthrust_ramps_speed(self) -> None:
    gov = SpeedGovernor(v_cmd=15.0)
    thr, v = gov.step(..., dt=1.0, v_up_mps_s=0.5, v_cruise_mps=18.0, t_max_n=1e6, ...)
    self.assertAlmostEqual(v, 15.5, places=5)
```

Fill `...` with consistent aero kwargs in the real test file.

- [ ] **Step 2: Run — FAIL**

```bash
cd python && python3 -m unittest tests.test_thrust_energy -v
```

- [ ] **Step 3: Implement `thrust_energy.py`**

- [ ] **Step 4: Run — PASS**

- [ ] **Step 5: Commit only if user asks**

---

### Task 6: Wire `AttitudeChaseController` chase branch

**Files:**
- Modify: `python/fw_sitl/body_cmd_controllers.py`
- Modify: `python/tests/test_body_cmd_controllers.py`

**Interfaces:**
- Consumes: `pure_pursuit_accel`, `normalize`, `split_parallel_perp`, `attitude_from_accel`, `SpeedGovernor`
- Produces: chase branch sets `last_law` to `"pp_polar"` or `"pp_geom"`; `last_speed_mps` = governor `v_cmd`; `last_thrust` = fraction

- [ ] **Step 1: Update / add failing tests**

Change assertions that expect `last_law == "los"` on in-view chase to:

```python
self.assertTrue(str(ctrl.last_law).startswith("pp"))
```

Update `test_close_and_fast_cuts_thrust_for_every_plant` (and any test that asserts `last_speed_mps ≈ approach_speed_mps`): instead assert `last_speed_mps >= plant.v_stall_mps * plant.v_recover_mult - 1e-6` and that thrust is finite in `[min_thrust, max_thrust]`. Keep path-branch tests (`last_law == "path"`) unchanged.

Add:

```python
def test_in_view_uses_pp_pipeline(self) -> None:
    plant = load_plant_gains("jsbsim_rascal")
    bridge = BodyCmdBridge(lookahead_m=500.0, speed_mps=plant.speed_mps)
    ctrl = AttitudeChaseController(bridge, speed_mps=plant.speed_mps, plant=plant)
    q_act = from_rpy(0.0, 0.0, 0.0)
    with patch("fw_sitl.body_cmd_controllers.send_attitude_target", create=True) as send:
        ctrl.send_chase_setpoint(
            MagicMock(),
            (0.0, 0.0, 0.0),
            (1.0, 0.1, 0.0),
            1,
            yaw_rad=0.0,
            q_act=q_act,
            dt=0.05,
            in_view=True,
            groundspeed=18.0,
            vx=18.0,
            vy=0.0,
            range_m=200.0,
        )
    self.assertTrue(str(ctrl.last_law).startswith("pp"))
    send.assert_called_once()
    thr = send.call_args[0][4]
    self.assertGreaterEqual(thr, plant.min_thrust)
    self.assertLessEqual(thr, plant.max_thrust)
```

- [ ] **Step 2: Run targeted tests — expect FAIL on `los` assertions**

```bash
cd python && python3 -m unittest tests.test_body_cmd_controllers -v
```

- [ ] **Step 3: Implement chase branch**

In `AttitudeChaseController.__init__`, add `self._speed_gov: SpeedGovernor | None = None` (lazy-init on first chase with `v_cmd=speed_mps`).

When `in_view` (same branch that today calls `q_des_from_los`):

```python
# û from dir_ned; v̂ from (vx,vy,vz) or (groundspeed along yaw) fallback
u_hat = normalize(dir_ned)
vel = (vx or 0.0, vy or 0.0, 0.0)  # if you have vz from caller later, use it
if math.hypot(vel[0], vel[1], vel[2]) < 0.5 and groundspeed:
    vel = (groundspeed * math.cos(yaw_act), groundspeed * math.sin(yaw_act), 0.0)
v_hat = normalize(vel)
V = max(math.hypot(*vel), 0.5)
gamma = math.asin(max(-1.0, min(1.0, -vel[2] / V)))
theta = rpy_from_quat(q_act)[1]
a_des = pure_pursuit_accel(u_hat, v_hat, gain=self._plant.pp_gain)
psi_c = math.atan2(dir_ned[1], dir_ned[0])
mode = self._plant.attitude_from_accel
res = attitude_from_accel(
    a_des, psi_c, mode=mode,
    max_roll=self._plant.bank_max_roll_rad,
    max_pitch=self._plant.att_los_max_pitch_rad,
)
q_des = res.q_des
a_par, _ = split_parallel_perp(a_des, v_hat)
if self._speed_gov is None:
    self._speed_gov = SpeedGovernor(v_cmd=self._speed_mps)
thrust, v_cmd = self._speed_gov.step(a_parallel=a_par, a_des=a_des, ..., plant fields...)
self.last_law = "pp_polar" if mode == "polar" else "pp_geom"
# then smooth roll/pitch as today, send_attitude_target
```

Pass `vz` if available from control later; for now if only `vx,vy` exist, \(\gamma\approx 0\) until extended — acceptable v1; prefer adding optional `vz` kwarg to `send_chase_setpoint` if control already has NED velocity (check `run_balloon_control.py` and thread it).

Do **not** call `chase_speed_mps` / `q_des_from_los` on this branch. Path branch unchanged.

- [ ] **Step 4: Run body_cmd + full discover — PASS**

```bash
cd python && python3 -m unittest tests.test_body_cmd_controllers tests.test_accel_laws tests.test_attitude_from_accel tests.test_thrust_energy tests.test_plant_gains tests.test_plant_loader -v
cd python && python3 -m unittest discover -s tests
```

- [ ] **Step 5: Commit only if user asks**

---

### Task 7: Docs (`UPDATES.md` / `README.md`)

**Files:**
- Modify: `UPDATES.md` — new top entry `0.37.0` (feature bump)
- Modify: `README.md` — mention PP accel→attitude→thrust chase and `fw_sitl/plants/*.jsonc` in Architecture

- [ ] **Step 1: Write UPDATES entry** (laconic bullets: JSONC plants, PP pipeline, polar/geometric, thrust governor)

- [ ] **Step 2: Patch README Architecture** one short clause on race attitude chase

- [ ] **Step 3: Commit only if user asks**

---

## Spec coverage checklist

| Spec item | Task |
|-----------|------|
| AccelLaw PP \(k(\hat u-\hat v)\) | 3 |
| Polar + geometric attitude | 4 |
| \(T=(ma_\parallel+D+mg\sin\gamma)/\cos\alpha\), quadratic \(D\) | 5 |
| AoA hybrid small-\(\alpha\) | 5 |
| Speed governor 0.8 / 1.1 / 1.2 | 5 |
| JSONC plants + comments | 1–2 |
| Wire `AttitudeChaseController` | 6 |
| Path-hold unchanged | 6 |
| Unit tests listed in spec | 3–6 |
| README/UPDATES | 7 |

## Placeholder / consistency self-review

- No TBD steps; interfaces name exact functions.
- `attitude_from_accel` string mode matches plant field and `last_law` suffix.
- Thrust always uses PP \(a_\parallel\), not `a_axial`.
- Commits gated on user request (project rule).
