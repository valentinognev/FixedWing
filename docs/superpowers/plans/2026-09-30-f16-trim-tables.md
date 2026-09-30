# F-16 Trim Tables Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Solve a wings-level `(vt, altitude)` trim grid for the Stevens and Morelli aero models, store it as JSON, and start every `run_f16.py` maneuver from the bilinear lookup of that grid.

**Architecture:** `python/f16/trim.py` searches one wings-level equilibrium (throttle, elevator, angle of attack) with the AeroBench orient-1 cost on the SI plant. `python/f16/trim_table.py` writes and loads `data/planes/f16/morelli.json` and `data/planes/f16/stevens.json`, and interpolates a query. `run_f16.py` looks up the maneuver speed at the setup spawn altitude, uses that state as `x0`, and copies it into `F16Llc.xequil` / `uequil`. The paper `K_lqr` is unchanged. AeroBench trajectory parity keeps today's benchmark initial state through `paper_x0` and does not read these files.

**Tech Stack:** Python 3, numpy, scipy (`scipy.optimize.minimize`, method `Nelder-Mead`), unittest (`cd python && python3 -m unittest <module> -v`). Host-only. No Docker, MAVLink, ZMQ, PX4, network, or `fgfs`.

**Spec:** This plan is the spec. Binding scope is the Approved design and the Global Constraints below. `docs/superpowers/plans/2026-09-29-f16-trim.md` is a different plan (one Morelli search, state-space printout, simulator left on the paper bias). Do not implement that plan and do not add its linearization or CLI.

## Approved design

Wings-level trim for both aero models lives in two JSON files, and every maneuver starts from that table.

Files: `data/planes/f16/morelli.json` and `data/planes/f16/stevens.json`. Units match the plant: m/s, metres, radians, throttle 0–1. Each file is a grid of solved points. A point stores `vt`, altitude, Mach, `alpha`, `theta`, power, throttle, and elevator. Sideslip, bank, flight-path angle, body rates, aileron, and rudder are zero. Cells that do not trim inside the model (Mach above 0.6, angle of attack outside −10° to +45°, or throttle or elevator on the stop) are left out.

Grid speeds: 120, 140, 153.0096, 164.592, 180, and 200 m/s. The two middle values are 502 ft/s and 540 ft/s. Grid altitudes: 304.8, 457.2, 1524, 3048, 6096, 9144, 12192, and 15240 m. 457.2 m is the shipped spawn.

`--aero` selects the file. The initial state, except north, east, and heading, is the bilinear interpolation of that table at the maneuver’s speed and the setup spawn altitude. Straight-and-level asks for 153.0096 m/s. Upright, inverted, and long GCAS ask for 164.592 m/s. All four start wings-level, so upright and inverted begin on the same state. The LQR equilibrium (`xequil`, `uequil`) is that same interpolated trim. The gain matrix stays the frozen paper gain. A request outside the table exits 2. AeroBench trajectory parity for the GCAS dives no longer applies to the runner. Tests check that a table point is an equilibrium of that aero model and that the sim starts from the interpolated row. Plant-parity tests keep the historical benchmark initial state.

## Global Constraints

- Wings-level only: β = 0, φ = 0, γ = 0, so θ = α. Body rates `p = q = r = 0`. Aileron and rudder are 0. Power at a solved node is `_tgear(throttle)`.
- Speed axis is true airspeed. Mach is `f16.model._adc(vt, h)[0]`, stored on the point, never a key.
- Grid, built with `f16.units.fts_to_ms` and `f16.units.ft_to_m`, not hand-rounded:
  - `vt_mps`: `fts_to_ms` of 120 / 0.3048, 140 / 0.3048, 502, 540, 180 / 0.3048, 200 / 0.3048. Store the resulting floats. The 502 ft/s and 540 ft/s entries are the maneuver speeds (`153.0096` and `164.592` when printed to 4 decimals).
  - `altitude_m`: `ft_to_m` of 1000, 1500, 5000, 10000, 20000, 30000, 40000, 50000.
- Omit a cell when Mach > 0.6, or α is outside `[deg_to_rad(-10), deg_to_rad(45)]`, or throttle is outside `[0, 1]`, or elevator is outside `[deg_to_rad(-25), deg_to_rad(25)]`, or the search still has cost > `1e-7` after both guesses. Endpoints of those intervals are kept.
- Search vector `s` is `(throttle, elevator_deg, alpha_rad)`. Guesses, in order: `[0.2, 0.0, 0.0]`, then `[0.8, -5.0, 0.15]`. Nelder-Mead `adaptive=False`, `xatol=1e-9`, `fatol=1e-9`, `maxiter=1000`. Repeat from the last `s` while cost > `1e-7`, at most 100 calls per guess. Cost weights are the MATLAB orient-1 weights. `vt_dot` is multiplied by `FT_PER_M` before squaring. If the weighted sum `r < 1`, the cost is `sqrt(r)`.
- `F16Llc()` with no arguments keeps the paper `_X_IMP` / `_U_IMP` bias and the paper `K_lqr`. Do not retune gains. The runner assigns `xequil` and `uequil` after the lookup.
- `Nz` stored in JSON is the plant return value (`0` at unaccelerated 1g), under the key `nz`.
- Off-node lookup interpolates `alpha_rad`, `throttle`, and `elevator_rad`, then sets `theta = alpha`, `power = _tgear(throttle)`, `vt` and `h` to the query, and the constrained states and surfaces to 0. On-node lookup returns that node the same way.
- A query outside a rectangle whose four corners exist raises `ValueError`. The runner prints `trim lookup failed: ...` on stderr and exits 2.
- Host tests only: `cd python && python3 -m unittest <module> -v`.
- Do not edit `python/f16/model.py` or `python/f16/units.py`. Import `_adc`, `_tgear`, and `subf16_derivative` from `f16.model`.
- Do not commit unless the user has asked for commits in that session. Leave each commit step unchecked until then.
- After the behavior change, `UPDATES.md` gets one new top entry `0.86.0`. README changes only the sentences named in Task 5.

## Parallelism

- Tasks 1–3 edit `python/f16/trim.py` then `python/f16/trim_table.py` in order. Do not dispatch them together.
- Task 4 waits for Task 3. It edits the runner, `compare.py`, and the GCAS tests.
- Task 5 (`README.md`, `UPDATES.md`) runs after Task 4.

## File map

| File | Task | Responsibility |
|------|------|----------------|
| `python/f16/trim.py` | 1–2 | Orient-1 cost and Nelder-Mead search for one `(vt, h, model)` |
| `python/tests/test_f16_trim.py` | 1–2 | Cost, Morelli equilibrium, Stevens equilibrium |
| `python/f16/trim_table.py` | 3 | Grid, JSON read/write, bilinear lookup |
| `data/planes/f16/morelli.json` | 3 | Solved Morelli grid |
| `data/planes/f16/stevens.json` | 3 | Solved Stevens grid |
| `python/tests/test_f16_trim_table.py` | 3 | Interpolation, omitted cells, on-disk equilibria |
| `python/run_f16.py` | 4 | Lookup is the flown state and the LQR bias |
| `python/f16/compare.py` | 4 | `scenario_x0` reads the table; `paper_x0` keeps the benchmark IC |
| `python/tests/test_f16_gcas.py` | 4 | Both GCAS cases start on the same wings-level row and stay in standby |
| `python/tests/test_f16_runner.py` | 4 | CSV row matches the file; outside the table exits 2 |
| `README.md`, `UPDATES.md` | 5 | Architecture sentence, known limit, `0.86.0` |

---

### Task 1: Orient-1 cost

**Files:**
- Create: `python/f16/trim.py`
- Test: `python/tests/test_f16_trim.py`

**Interfaces:**
- Consumes: `f16.model.subf16_derivative`, `f16.model._tgear`, `f16.units.DEG_TO_RAD`, `f16.units.FT_PER_M`, `f16.llc._X_IMP`, `f16.llc._U_IMP`
- Produces: `apply_wings_level(x, u) -> tuple[np.ndarray, np.ndarray]`; `wings_level_cost(s, vt_mps, height_m, model="morelli") -> float`. `s` has shape `(3,)`: throttle, elevator degrees, α radians. Returned arrays are copies. `model` is `"morelli"` or `"stevens"`.

- [ ] **Step 1: Write the failing test**

Create `python/tests/test_f16_trim.py`:

```python
import unittest

import numpy as np

from f16.llc import _U_IMP, _X_IMP
from f16.model import _tgear
from f16.units import DEG_TO_RAD, FT_PER_M, ft_to_m, fts_to_ms


class TestWingsLevelCost(unittest.TestCase):
    def test_constraints_set_beta_theta_rates_and_power(self) -> None:
        from f16.trim import apply_wings_level

        x = np.zeros(13)
        u = np.zeros(4)
        x[0] = 150.0
        x[1] = 0.04
        x[2] = 0.2
        x[3] = 0.2
        x[6] = 0.1
        x[11] = 300.0
        u[0] = 0.2
        u[1] = -0.01
        u[2] = 0.05
        u[3] = 0.05
        xc, uc = apply_wings_level(x, u)
        self.assertAlmostEqual(float(xc[4]), 0.04, places=12)
        self.assertEqual(float(xc[2]), 0.0)
        self.assertEqual(float(xc[3]), 0.0)
        self.assertEqual(float(xc[6]), 0.0)
        self.assertEqual(float(xc[7]), 0.0)
        self.assertEqual(float(xc[8]), 0.0)
        self.assertAlmostEqual(float(xc[12]), _tgear(0.2), places=12)
        self.assertAlmostEqual(float(xc[0]), 150.0, places=12)
        self.assertAlmostEqual(float(xc[11]), 300.0, places=12)
        self.assertAlmostEqual(float(uc[1]), -0.01, places=12)
        self.assertEqual(float(uc[2]), 0.0)
        self.assertEqual(float(uc[3]), 0.0)
        self.assertAlmostEqual(float(x[2]), 0.2, places=12)
        self.assertAlmostEqual(float(u[2]), 0.05, places=12)

    def test_paper_bias_is_not_a_morelli_root(self) -> None:
        from f16.trim import wings_level_cost

        s = np.array([_U_IMP[0], _U_IMP[1], _X_IMP[1]], dtype=float)
        cost = wings_level_cost(s, fts_to_ms(502.0), ft_to_m(1000.0), model="morelli")
        self.assertGreater(cost, 1.0)

    def test_cost_uses_feet_per_second_squared(self) -> None:
        from f16.model import subf16_derivative
        from f16.trim import apply_wings_level, wings_level_cost

        s = np.array([0.2, -1.0, 0.03], dtype=float)
        x = np.zeros(13)
        u = np.zeros(4)
        x[0] = fts_to_ms(502.0)
        x[11] = ft_to_m(1000.0)
        x[1] = s[2]
        u[0] = s[0]
        u[1] = s[1] * DEG_TO_RAD
        x, u = apply_wings_level(x, u)
        xd, _, _ = subf16_derivative(x, u, model="stevens")
        raw = 100.0 * (
            (xd[0] * FT_PER_M) ** 2 + xd[1] ** 2 + xd[2] ** 2 + xd[6] ** 2 + xd[7] ** 2 + xd[8] ** 2
        )
        expected = raw ** 0.5 if raw < 1.0 else raw
        self.assertAlmostEqual(
            wings_level_cost(s, x[0], x[11], model="stevens"), expected, places=9
        )

    def test_unknown_model_raises(self) -> None:
        from f16.trim import wings_level_cost

        with self.assertRaises(ValueError):
            wings_level_cost(np.array([0.2, 0.0, 0.03]), 150.0, 300.0, model="mixed")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd python && python3 -m unittest tests.test_f16_trim -v`

Expected: FAIL with `ModuleNotFoundError: No module named 'f16.trim'`.

- [ ] **Step 3: Write minimal implementation**

Create `python/f16/trim.py`:

```python
"""Wings-level F-16 trim (AeroBench trimmerFun orient 1) on the SI plant."""
from __future__ import annotations

import numpy as np

from f16.model import _tgear, subf16_derivative
from f16.units import DEG_TO_RAD, FT_PER_M

_MODELS = ("morelli", "stevens")


def apply_wings_level(x, u):
    """Orient 1. Returns copies. beta = phi = p = q = r = 0, theta = alpha, pow = tgear."""
    xc = np.asarray(x, dtype=float).copy()
    uc = np.asarray(u, dtype=float).copy()
    xc[2] = 0.0
    xc[3] = 0.0
    xc[4] = xc[1]
    xc[6] = 0.0
    xc[7] = 0.0
    xc[8] = 0.0
    uc[2] = 0.0
    uc[3] = 0.0
    xc[12] = _tgear(float(uc[0]))
    return xc, uc


def wings_level_cost(s, vt_mps: float, height_m: float, model: str = "morelli") -> float:
    """clf16 orient 1. s = [throttle, elevator_deg, alpha_rad]."""
    if model not in _MODELS:
        raise ValueError(f"model {model!r} is not implemented")
    s = np.asarray(s, dtype=float)
    x = np.zeros(13, dtype=float)
    u = np.zeros(4, dtype=float)
    x[0] = float(vt_mps)
    x[1] = float(s[2])
    x[11] = float(height_m)
    u[0] = float(s[0])
    u[1] = float(s[1]) * DEG_TO_RAD
    x, u = apply_wings_level(x, u)
    xd, _, _ = subf16_derivative(x, u, model=model)
    r = 100.0 * (
        (xd[0] * FT_PER_M) ** 2 + xd[1] ** 2 + xd[2] ** 2 + xd[6] ** 2 + xd[7] ** 2 + xd[8] ** 2
    )
    if r < 1.0:
        r = r ** 0.5
    return float(r)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd python && python3 -m unittest tests.test_f16_trim -v`

Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
git add python/f16/trim.py python/tests/test_f16_trim.py
git commit -m "feat(f16): wings-level trim cost for both aero models"
```

Do not commit unless the user has asked for commits in that session.

---

### Task 2: Search one flight condition

**Files:**
- Modify: `python/f16/trim.py`
- Test: `python/tests/test_f16_trim.py`

**Interfaces:**
- Consumes: `wings_level_cost`, `apply_wings_level` from Task 1
- Produces: `trim_wings_level(vt_mps, height_m, model="morelli", s0=None) -> tuple[np.ndarray, np.ndarray, float]`. The tuple is `(x_si (13,), u_si (4,), cost)`. Default start is `[0.2, 0.0, 0.0]`. One guess, the 100-iteration outer loop. The second guess belongs to the table builder in Task 3. Raises `ValueError` for a non-finite or non-positive `vt_mps`, a non-finite or negative `height_m`, or an unknown model. Raises `RuntimeError` when cost stays above `1e-7`.

- [ ] **Step 1: Write the failing test**

Append to `python/tests/test_f16_trim.py`:

```python
class TestTrimSearch(unittest.TestCase):
    def test_morelli_nominal_is_an_equilibrium(self) -> None:
        from f16.model import subf16_derivative
        from f16.trim import trim_wings_level
        from f16.units import rad_to_deg

        vt = fts_to_ms(502.0)
        height = ft_to_m(1000.0)
        x, u, cost = trim_wings_level(vt, height, model="morelli")
        self.assertLess(cost, 1e-7)
        self.assertAlmostEqual(float(x[0]), vt, places=6)
        self.assertAlmostEqual(float(x[11]), height, places=6)
        self.assertAlmostEqual(float(x[4]), float(x[1]), places=12)
        self.assertEqual(float(x[2]), 0.0)
        self.assertEqual(float(x[3]), 0.0)
        self.assertEqual(float(x[6]), 0.0)
        self.assertEqual(float(x[7]), 0.0)
        self.assertEqual(float(x[8]), 0.0)
        self.assertAlmostEqual(float(x[12]), _tgear(float(u[0])), places=9)
        self.assertEqual(float(u[2]), 0.0)
        self.assertEqual(float(u[3]), 0.0)
        self.assertGreater(float(u[0]), 0.05)
        self.assertLess(float(u[0]), 0.3)
        self.assertGreater(rad_to_deg(float(u[1])), -5.0)
        self.assertLess(rad_to_deg(float(u[1])), 0.0)
        self.assertGreater(float(x[1]), 0.01)
        self.assertLess(float(x[1]), 0.06)
        self.assertGreater(abs(float(x[1]) - 0.0389), 1e-3)
        xd, _, _ = subf16_derivative(x, u, model="morelli")
        self.assertLess(abs(float(xd[0])), 1e-6)
        self.assertLess(abs(float(xd[1])), 1e-6)
        self.assertLess(abs(float(xd[7])), 1e-6)

    def test_stevens_nominal_is_not_a_morelli_root(self) -> None:
        from f16.trim import trim_wings_level, wings_level_cost
        from f16.units import rad_to_deg

        vt = fts_to_ms(502.0)
        height = ft_to_m(1000.0)
        x, u, cost = trim_wings_level(vt, height, model="stevens")
        self.assertLess(cost, 1e-7)
        self.assertAlmostEqual(float(x[4]), float(x[1]), places=12)
        self.assertAlmostEqual(float(x[12]), _tgear(float(u[0])), places=9)
        cross = wings_level_cost(
            np.array([u[0], rad_to_deg(float(u[1])), x[1]]), vt, height, model="morelli"
        )
        self.assertGreater(cross, 1e-3)

    def test_rejects_bad_speed_height_or_model(self) -> None:
        from f16.trim import trim_wings_level

        with self.assertRaises(ValueError):
            trim_wings_level(0.0, 304.8)
        with self.assertRaises(ValueError):
            trim_wings_level(-10.0, 304.8)
        with self.assertRaises(ValueError):
            trim_wings_level(150.0, -1.0)
        with self.assertRaises(ValueError):
            trim_wings_level(150.0, 304.8, model="mixed")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd python && python3 -m unittest tests.test_f16_trim.TestTrimSearch -v`

Expected: FAIL with `ImportError` (`trim_wings_level`).

- [ ] **Step 3: Write minimal implementation**

Add to `python/f16/trim.py`. Keep a single import block at the top. Add `import math` and `from scipy.optimize import minimize`.

```python
_S0 = np.array([0.2, 0.0, 0.0], dtype=float)


def _check_condition(vt_mps: float, height_m: float, model: str) -> None:
    if model not in _MODELS:
        raise ValueError(f"model {model!r} is not implemented")
    if not math.isfinite(vt_mps) or vt_mps <= 0.0:
        raise ValueError(f"vt_mps must be positive and finite, got {vt_mps}")
    if not math.isfinite(height_m) or height_m < 0.0:
        raise ValueError(f"height_m must be non-negative and finite, got {height_m}")


def trim_wings_level(vt_mps: float, height_m: float, model: str = "morelli", s0=None):
    """Orient-1 trim at true airspeed and geometric height. Returns (x_si, u_si, cost)."""
    vt = float(vt_mps)
    height = float(height_m)
    _check_condition(vt, height, model)
    s = _S0.copy() if s0 is None else np.asarray(s0, dtype=float).copy()
    cost = wings_level_cost(s, vt, height, model=model)
    for _ in range(100):
        if cost <= 1e-7:
            break
        found = minimize(
            lambda z: wings_level_cost(z, vt, height, model=model),
            s,
            method="Nelder-Mead",
            options={"xatol": 1e-9, "fatol": 1e-9, "maxiter": 1000, "adaptive": False},
        )
        s = np.asarray(found.x, dtype=float)
        cost = float(found.fun)
    if cost > 1e-7:
        raise RuntimeError(f"trim did not reach 1e-7, cost {cost}")
    x = np.zeros(13, dtype=float)
    u = np.zeros(4, dtype=float)
    x[0] = vt
    x[1] = float(s[2])
    x[11] = height
    u[0] = float(s[0])
    u[1] = float(s[1]) * DEG_TO_RAD
    x, u = apply_wings_level(x, u)
    return x, u, float(cost)
```

The loop stops once `cost <= 1e-7`. The check after the loop raises when all 100 attempts stay above that threshold.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd python && python3 -m unittest tests.test_f16_trim -v`

Expected: PASS (7 tests). The nominal search is under a few seconds. If `test_stevens_nominal_is_not_a_morelli_root` fails because the cross cost is below `1e-3`, stop. Do not loosen the assertion. The two aero models must not share that root.

- [ ] **Step 5: Commit**

```bash
git add python/f16/trim.py python/tests/test_f16_trim.py
git commit -m "feat(f16): search wings-level trim at a true airspeed and height"
```

Do not commit unless the user has asked for commits in that session.

---

### Task 3: JSON grid and bilinear lookup

**Files:**
- Create: `python/f16/trim_table.py`
- Create: `data/planes/f16/morelli.json`
- Create: `data/planes/f16/stevens.json`
- Test: `python/tests/test_f16_trim_table.py`

**Interfaces:**
- Consumes: `trim_wings_level`, `wings_level_cost`, `f16.model._adc`, `f16.model._tgear`
- Produces:
  - `VT_MPS: tuple[float, ...]` and `ALTITUDE_M: tuple[float, ...]` as in Global Constraints
  - `TRIM_DIR: Path` = `Path(__file__).resolve().parents[2] / "data" / "planes" / "f16"`
  - `load_trim_table(model: str, root: Path | None = None) -> dict`
  - `lookup_table(table: dict, vt_mps: float, altitude_m: float) -> tuple[np.ndarray, np.ndarray]`
  - `lookup_trim(model: str, vt_mps: float, altitude_m: float, root: Path | None = None) -> tuple[np.ndarray, np.ndarray]`
  - `trimmed_initial(maneuver: str, aero: str = "morelli", height_m: float | None = None, root: Path | None = None) -> tuple[np.ndarray, np.ndarray]`
  - `write_trim_tables(root: Path | None = None) -> None`
  - `maneuver_vt_mps(maneuver: str) -> float` — `straight_level` returns `fts_to_ms(502)`; `gcas_upright`, `gcas_inverted`, and `gcas_long` return `fts_to_ms(540)`; anything else raises `ValueError`

`lookup_table` returns `x` shape `(13,)` and `u` shape `(4,)`. An on-node hit is a point with `abs(vt - vt_mps) <= 1e-6` and `abs(altitude - altitude_m) <= 1e-4`. Off-node, bracket on the file's `vt_mps` and `altitude_m` axes. All four corners must be present in `points`. Interpolate `alpha_rad`, `throttle`, and `elevator_rad`. Then `theta = alpha`, `power = _tgear(throttle)`, query `vt` and `h` are copied onto the state, and β, φ, rates, aileron, and rudder are 0. Missing rectangle or a query outside the axis min/max raises `ValueError`.

- [ ] **Step 1: Write the failing interpolation test**

Create `python/tests/test_f16_trim_table.py`:

```python
import unittest

import numpy as np

from f16.model import _tgear


def _table():
    return {
        "model": "morelli",
        "vt_mps": [100.0, 200.0],
        "altitude_m": [1000.0, 3000.0],
        "points": [
            {"vt_mps": 100.0, "altitude_m": 1000.0, "alpha_rad": 0.04, "throttle": 0.20, "elevator_rad": -0.02},
            {"vt_mps": 200.0, "altitude_m": 1000.0, "alpha_rad": 0.02, "throttle": 0.40, "elevator_rad": -0.01},
            {"vt_mps": 100.0, "altitude_m": 3000.0, "alpha_rad": 0.08, "throttle": 0.30, "elevator_rad": -0.04},
            {"vt_mps": 200.0, "altitude_m": 3000.0, "alpha_rad": 0.06, "throttle": 0.50, "elevator_rad": -0.03},
        ],
    }


class TestLookup(unittest.TestCase):
    def test_midpoint_is_the_mean_of_the_four_corners(self) -> None:
        from f16.trim_table import lookup_table

        x, u = lookup_table(_table(), 150.0, 2000.0)
        self.assertAlmostEqual(float(x[0]), 150.0, places=9)
        self.assertAlmostEqual(float(x[11]), 2000.0, places=9)
        self.assertAlmostEqual(float(x[1]), 0.05, places=9)
        self.assertAlmostEqual(float(x[4]), 0.05, places=9)
        self.assertAlmostEqual(float(u[0]), 0.35, places=9)
        self.assertAlmostEqual(float(u[1]), -0.025, places=9)
        self.assertAlmostEqual(float(x[12]), _tgear(0.35), places=9)
        self.assertEqual(float(x[2]), 0.0)
        self.assertEqual(float(x[3]), 0.0)
        self.assertEqual(float(u[2]), 0.0)
        self.assertEqual(float(u[3]), 0.0)

    def test_on_node_returns_that_corner(self) -> None:
        from f16.trim_table import lookup_table

        x, u = lookup_table(_table(), 200.0, 3000.0)
        self.assertAlmostEqual(float(x[1]), 0.06, places=9)
        self.assertAlmostEqual(float(u[0]), 0.50, places=9)
        self.assertAlmostEqual(float(u[1]), -0.03, places=9)

    def test_missing_corner_and_outside_axes_raise(self) -> None:
        from f16.trim_table import lookup_table

        table = _table()
        table["points"] = table["points"][:3]
        with self.assertRaises(ValueError):
            lookup_table(table, 150.0, 2000.0)
        with self.assertRaises(ValueError):
            lookup_table(_table(), 50.0, 2000.0)
        with self.assertRaises(ValueError):
            lookup_table(_table(), 150.0, 4000.0)

    def test_maneuver_speeds(self) -> None:
        from f16.trim_table import maneuver_vt_mps
        from f16.units import fts_to_ms

        self.assertAlmostEqual(maneuver_vt_mps("straight_level"), fts_to_ms(502.0), places=9)
        self.assertAlmostEqual(maneuver_vt_mps("gcas_upright"), fts_to_ms(540.0), places=9)
        self.assertAlmostEqual(maneuver_vt_mps("gcas_inverted"), fts_to_ms(540.0), places=9)
        self.assertAlmostEqual(maneuver_vt_mps("gcas_long"), fts_to_ms(540.0), places=9)
        with self.assertRaises(ValueError):
            maneuver_vt_mps("loop_the_loop")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd python && python3 -m unittest tests.test_f16_trim_table.TestLookup -v`

Expected: FAIL with `ModuleNotFoundError: No module named 'f16.trim_table'`.

- [ ] **Step 3: Implement lookup**

Create `python/f16/trim_table.py` with `lookup_table`, `maneuver_vt_mps`, the axis constants, and `TRIM_DIR`. `write_trim_tables` can wait for Step 6. Lookup implementation:

```python
"""Wings-level trim grids for the SI F-16."""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from f16.model import _adc, _tgear, subf16_derivative
from f16.trim import trim_wings_level
from f16.units import deg_to_rad, ft_to_m, fts_to_ms

VT_MPS = (
    fts_to_ms(120.0 / 0.3048),
    fts_to_ms(140.0 / 0.3048),
    fts_to_ms(502.0),
    fts_to_ms(540.0),
    fts_to_ms(180.0 / 0.3048),
    fts_to_ms(200.0 / 0.3048),
)
ALTITUDE_M = tuple(ft_to_m(ft) for ft in (1000, 1500, 5000, 10000, 20000, 30000, 40000, 50000))
TRIM_DIR = Path(__file__).resolve().parents[2] / "data" / "planes" / "f16"
_MODELS = ("morelli", "stevens")
_GUESSES = (
    np.array([0.2, 0.0, 0.0], dtype=float),
    np.array([0.8, -5.0, 0.15], dtype=float),
)
_MACH_MAX = 0.6
_ALPHA_MIN = deg_to_rad(-10.0)
_ALPHA_MAX = deg_to_rad(45.0)
_ELEV_MIN = deg_to_rad(-25.0)
_ELEV_MAX = deg_to_rad(25.0)
_GCAS = ("gcas_upright", "gcas_inverted", "gcas_long")


def maneuver_vt_mps(maneuver: str) -> float:
    if maneuver == "straight_level":
        return fts_to_ms(502.0)
    if maneuver in _GCAS:
        return fts_to_ms(540.0)
    raise ValueError(f"unknown maneuver {maneuver!r}")


def _corner(table: dict, vt: float, alt: float) -> dict | None:
    for point in table["points"]:
        if abs(float(point["vt_mps"]) - vt) <= 1e-6 and abs(float(point["altitude_m"]) - alt) <= 1e-4:
            return point
    return None


def _pack(alpha: float, throttle: float, elevator: float, vt: float, alt: float):
    x = np.zeros(13, dtype=float)
    u = np.zeros(4, dtype=float)
    x[0] = float(vt)
    x[1] = float(alpha)
    x[4] = float(alpha)
    x[11] = float(alt)
    x[12] = _tgear(float(throttle))
    u[0] = float(throttle)
    u[1] = float(elevator)
    return x, u


def lookup_table(table: dict, vt_mps: float, altitude_m: float):
    vt = float(vt_mps)
    alt = float(altitude_m)
    vts = [float(v) for v in table["vt_mps"]]
    alts = [float(v) for v in table["altitude_m"]]
    if vt < vts[0] - 1e-6 or vt > vts[-1] + 1e-6 or alt < alts[0] - 1e-4 or alt > alts[-1] + 1e-4:
        raise ValueError(f"trim lookup outside axes vt={vt} altitude={alt}")
    hit = _corner(table, vt, alt)
    if hit is not None:
        return _pack(hit["alpha_rad"], hit["throttle"], hit["elevator_rad"], vt, alt)
    i = max(i for i in range(len(vts) - 1) if vts[i] <= vt + 1e-6)
    j = max(j for j in range(len(alts) - 1) if alts[j] <= alt + 1e-4)
    corners = (
        _corner(table, vts[i], alts[j]),
        _corner(table, vts[i + 1], alts[j]),
        _corner(table, vts[i], alts[j + 1]),
        _corner(table, vts[i + 1], alts[j + 1]),
    )
    if any(point is None for point in corners):
        raise ValueError(f"trim lookup missing corner vt={vt} altitude={alt}")
    tx = 0.0 if vts[i + 1] == vts[i] else (vt - vts[i]) / (vts[i + 1] - vts[i])
    ty = 0.0 if alts[j + 1] == alts[j] else (alt - alts[j]) / (alts[j + 1] - alts[j])
    weights = ((1 - tx) * (1 - ty), tx * (1 - ty), (1 - tx) * ty, tx * ty)
    alpha = throttle = elevator = 0.0
    for weight, point in zip(weights, corners, strict=True):
        alpha += weight * float(point["alpha_rad"])
        throttle += weight * float(point["throttle"])
        elevator += weight * float(point["elevator_rad"])
    return _pack(alpha, throttle, elevator, vt, alt)
```

`_corner` uses the node values from the axes when the caller passes an axis station. `lookup_table`'s on-node path passes the query `vt` and `alt`, which match within the tolerances.

- [ ] **Step 4: Run the interpolation tests**

Run: `cd python && python3 -m unittest tests.test_f16_trim_table.TestLookup -v`

Expected: PASS (4 tests).

- [ ] **Step 5: Write the failing on-disk test**

Append to `python/tests/test_f16_trim_table.py`:

```python
class TestTrimFiles(unittest.TestCase):
    def test_both_files_are_equilibria_inside_the_envelope(self) -> None:
        from f16.model import _adc
        from f16.trim import wings_level_cost
        from f16.trim_table import ALTITUDE_M, VT_MPS, load_trim_table
        from f16.units import deg_to_rad, ft_to_m, fts_to_ms, rad_to_deg

        self.assertEqual(len(VT_MPS), 6)
        self.assertEqual(len(ALTITUDE_M), 8)
        for model in ("morelli", "stevens"):
            table = load_trim_table(model)
            self.assertEqual(table["model"], model)
            self.assertGreater(len(table["points"]), 0)
            seen = set()
            for point in table["points"]:
                vt = float(point["vt_mps"])
                alt = float(point["altitude_m"])
                seen.add((round(vt, 6), round(alt, 4)))
                mach, _ = _adc(vt, alt)
                self.assertLessEqual(mach, 0.6)
                self.assertAlmostEqual(float(point["mach"]), mach, places=6)
                self.assertGreaterEqual(float(point["alpha_rad"]), deg_to_rad(-10.0))
                self.assertLessEqual(float(point["alpha_rad"]), deg_to_rad(45.0))
                self.assertGreaterEqual(float(point["throttle"]), 0.0)
                self.assertLessEqual(float(point["throttle"]), 1.0)
                self.assertGreaterEqual(float(point["elevator_rad"]), deg_to_rad(-25.0))
                self.assertLessEqual(float(point["elevator_rad"]), deg_to_rad(25.0))
                self.assertAlmostEqual(float(point["theta_rad"]), float(point["alpha_rad"]), places=9)
                cost = wings_level_cost(
                    np.array([point["throttle"], rad_to_deg(float(point["elevator_rad"])), point["alpha_rad"]]),
                    vt,
                    alt,
                    model=model,
                )
                self.assertLess(cost, 1e-6)
            high = (round(VT_MPS[-1], 6), round(ALTITUDE_M[-1], 4))
            self.assertNotIn(high, seen)
            for vt in (fts_to_ms(502.0), fts_to_ms(540.0)):
                for alt in (ft_to_m(1000.0), ft_to_m(1500.0)):
                    self.assertIn((round(vt, 6), round(alt, 4)), seen)

    def test_nominal_file_matches_a_fresh_search(self) -> None:
        from f16.trim import trim_wings_level
        from f16.trim_table import lookup_trim
        from f16.units import ft_to_m, fts_to_ms

        vt = fts_to_ms(502.0)
        height = ft_to_m(1000.0)
        for model in ("morelli", "stevens"):
            fresh, u_fresh, cost = trim_wings_level(vt, height, model=model)
            self.assertLess(cost, 1e-7)
            x, u = lookup_trim(model, vt, height)
            self.assertAlmostEqual(float(x[1]), float(fresh[1]), delta=5e-6)
            self.assertAlmostEqual(float(u[0]), float(u_fresh[0]), delta=5e-5)
            self.assertAlmostEqual(float(u[1]), float(u_fresh[1]), delta=5e-5)

    def test_trimmed_initial_shares_the_gcas_row(self) -> None:
        from f16.trim_table import trimmed_initial
        from f16.units import ft_to_m

        upright, _ = trimmed_initial("gcas_upright", aero="morelli", height_m=ft_to_m(1000.0))
        inverted, _ = trimmed_initial("gcas_inverted", aero="morelli", height_m=ft_to_m(1000.0))
        np.testing.assert_allclose(upright, inverted, rtol=0, atol=0)
        self.assertEqual(float(upright[3]), 0.0)
        self.assertAlmostEqual(float(upright[4]), float(upright[1]), places=12)
```

- [ ] **Step 6: Run test to verify it fails**

Run: `cd python && python3 -m unittest tests.test_f16_trim_table.TestTrimFiles -v`

Expected: FAIL because `load_trim_table` is missing or the JSON files are absent.

- [ ] **Step 7: Write the grids**

Add `load_trim_table`, `lookup_trim`, `trimmed_initial`, and `write_trim_tables` to `python/f16/trim_table.py`:

```python
def load_trim_table(model: str, root: Path | None = None) -> dict:
    if model not in _MODELS:
        raise ValueError(f"model {model!r} is not implemented")
    directory = TRIM_DIR if root is None else Path(root)
    path = directory / f"{model}.json"
    return json.loads(path.read_text())


def lookup_trim(model: str, vt_mps: float, altitude_m: float, root: Path | None = None):
    return lookup_table(load_trim_table(model, root=root), vt_mps, altitude_m)


def trimmed_initial(maneuver: str, aero: str = "morelli", height_m: float | None = None, root: Path | None = None):
    if height_m is None:
        height_m = ft_to_m(1000.0)
    return lookup_trim(aero, maneuver_vt_mps(maneuver), float(height_m), root=root)


def _solve_point(model: str, vt: float, alt: float) -> dict | None:
    mach, _ = _adc(vt, alt)
    if mach > _MACH_MAX:
        return None
    for guess in _GUESSES:
        try:
            x, u, _ = trim_wings_level(vt, alt, model=model, s0=guess)
        except RuntimeError:
            continue
        alpha = float(x[1])
        throttle = float(u[0])
        elevator = float(u[1])
        if not (_ALPHA_MIN <= alpha <= _ALPHA_MAX and 0.0 <= throttle <= 1.0 and _ELEV_MIN <= elevator <= _ELEV_MAX):
            continue
        _, nz, _ = subf16_derivative(x, u, model=model)
        return {
            "vt_mps": vt,
            "altitude_m": alt,
            "mach": mach,
            "alpha_rad": alpha,
            "theta_rad": float(x[4]),
            "power": float(x[12]),
            "throttle": throttle,
            "elevator_rad": elevator,
            "nz": nz,
        }
    return None


def write_trim_tables(root: Path | None = None) -> None:
    directory = TRIM_DIR if root is None else Path(root)
    directory.mkdir(parents=True, exist_ok=True)
    for model in _MODELS:
        points = []
        for alt in ALTITUDE_M:
            for vt in VT_MPS:
                point = _solve_point(model, float(vt), float(alt))
                if point is not None:
                    points.append(point)
        payload = {
            "model": model,
            "constraints": {
                "beta_rad": 0.0,
                "phi_rad": 0.0,
                "gamma_rad": 0.0,
                "p_rad_s": 0.0,
                "q_rad_s": 0.0,
                "r_rad_s": 0.0,
                "aileron_rad": 0.0,
                "rudder_rad": 0.0,
            },
            "vt_mps": list(VT_MPS),
            "altitude_m": list(ALTITUDE_M),
            "points": points,
        }
        (directory / f"{model}.json").write_text(json.dumps(payload, indent=2) + "\n")
```

The Step 3 import already includes `subf16_derivative`. A station that fails both guesses, or trims outside the envelope, is omitted.

Generate the files:

Run: `cd python && python3 -c "from f16.trim_table import write_trim_tables; write_trim_tables()"`

Expected: `data/planes/f16/morelli.json` and `data/planes/f16/stevens.json` exist. This loop is two models times 48 stations times up to two searches. Allow several minutes. A station that fails both guesses is omitted, not fatal.

- [ ] **Step 8: Run test to verify it passes**

Run: `cd python && python3 -m unittest tests.test_f16_trim_table -v`

Expected: PASS. `test_both_files_are_equilibria_inside_the_envelope` confirms 200 m/s at 50000 ft is absent and every stored point has cost < `1e-6`.

- [ ] **Step 9: Commit**

```bash
git add python/f16/trim_table.py python/tests/test_f16_trim_table.py data/planes/f16/morelli.json data/planes/f16/stevens.json
git commit -m "feat(f16): write wings-level trim grids for both aero models"
```

Do not commit unless the user has asked for commits in that session.

---

### Task 4: Fly the lookup

**Files:**
- Modify: `python/run_f16.py`
- Modify: `python/f16/compare.py` (`scenario_x0`, `run_ours`, `run_python_reference`)
- Modify: `python/tests/test_f16_gcas.py`
- Modify: `python/tests/test_f16_runner.py`
- Test: `python/tests/test_f16_runner.py`, `python/tests/test_f16_gcas.py`, `python/tests/test_f16_compare.py`

**Interfaces:**
- Consumes: `trimmed_initial`, `lookup_trim`, `maneuver_vt_mps` from Task 3
- Produces: `paper_x0(scenario: str) -> np.ndarray` in `compare.py`, the current `scenario_x0` body moved unchanged. `scenario_x0(scenario, aero="morelli", height_m=None)` returns `trimmed_initial(...)[0]`. `run_ours` and `run_python_reference` call `paper_x0`, not `scenario_x0`. `run_f16._initial_state` returns `(x0, u0)` from `trimmed_initial(maneuver, aero=aero, height_m=h_m)` with `x0[9]`, `x0[10]` set from spawn and `x0[5] = 0`. `main` copies that `x0` into `llc.xequil` and `u0` into `llc.uequil` before building the autopilot. `K_lqr` is not assigned.

- [ ] **Step 1: Write the failing runner test**

Add `import numpy as np` to the imports in `python/tests/test_f16_runner.py`. Append to `TestRunner`:

```python
    def test_straight_level_starts_at_the_morelli_trim_row(self) -> None:
        from f16.trim_table import lookup_trim
        from f16.units import ft_to_m, fts_to_ms

        vt = fts_to_ms(502.0)
        height = ft_to_m(1000.0)
        x, _ = lookup_trim("morelli", vt, height)
        setup = {
            "maneuver": "straight_level",
            "duration_s": 1.0,
            "spawn": {"n_m": 10.0, "e_m": -4.0, "d_m": -height},
            "gcas_floor_m": 304.8,
        }
        with tempfile.TemporaryDirectory() as td:
            sp = Path(td) / "setup.json"
            cp = Path(td) / "out.csv"
            sp.write_text(json.dumps(setup))
            r = subprocess.run(
                [sys.executable, "run_f16.py", "--setup", str(sp), "--csv", str(cp), "--aero", "morelli"],
                cwd=".",
                capture_output=True,
                text=True,
            )
            self.assertEqual(r.returncode, 0, r.stderr)
            row = next(csv.DictReader(cp.read_text().splitlines()))
            self.assertAlmostEqual(float(row["vt_mps"]), vt, places=4)
            self.assertAlmostEqual(float(row["d_m"]), -height, places=4)
            self.assertAlmostEqual(float(row["n_m"]), 10.0, places=4)
            self.assertAlmostEqual(float(row["alpha"]), float(x[1]), places=6)
            self.assertAlmostEqual(float(row["theta"]), float(x[1]), places=6)
            self.assertAlmostEqual(float(row["phi"]), 0.0, places=6)
            self.assertAlmostEqual(float(row["psi"]), 0.0, places=6)

    def test_stevens_alpha_differs_from_morelli(self) -> None:
        from f16.trim_table import lookup_trim
        from f16.units import ft_to_m, fts_to_ms

        height = ft_to_m(1500.0)
        morelli, _ = lookup_trim("morelli", fts_to_ms(540.0), height)
        stevens, _ = lookup_trim("stevens", fts_to_ms(540.0), height)
        setup = {
            "maneuver": "gcas_upright",
            "duration_s": 0.1,
            "spawn": {"n_m": 0.0, "e_m": 0.0, "d_m": -height},
            "gcas_floor_m": 304.8,
        }
        alphas = []
        with tempfile.TemporaryDirectory() as td:
            sp = Path(td) / "setup.json"
            sp.write_text(json.dumps(setup))
            for aero in ("morelli", "stevens"):
                cp = Path(td) / f"{aero}.csv"
                r = subprocess.run(
                    [sys.executable, "run_f16.py", "--setup", str(sp), "--csv", str(cp), "--aero", aero],
                    cwd=".",
                    capture_output=True,
                    text=True,
                )
                self.assertEqual(r.returncode, 0, r.stderr)
                row = next(csv.DictReader(cp.read_text().splitlines()))
                alphas.append(float(row["alpha"]))
        self.assertAlmostEqual(alphas[0], float(morelli[1]), places=6)
        self.assertAlmostEqual(alphas[1], float(stevens[1]), places=6)
        self.assertFalse(np.allclose(morelli, stevens, atol=1e-4))

    def test_spawn_outside_the_table_exits_2(self) -> None:
        setup = {
            "maneuver": "straight_level",
            "duration_s": 1.0,
            "spawn": {"n_m": 0.0, "e_m": 0.0, "d_m": -100.0},
            "gcas_floor_m": 304.8,
        }
        with tempfile.TemporaryDirectory() as td:
            sp = Path(td) / "setup.json"
            sp.write_text(json.dumps(setup))
            r = subprocess.run(
                [sys.executable, "run_f16.py", "--setup", str(sp)],
                cwd=".",
                capture_output=True,
                text=True,
            )
            self.assertEqual(r.returncode, 2)
            self.assertIn("trim lookup failed", r.stderr)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd python && python3 -m unittest tests.test_f16_runner.TestRunner.test_straight_level_starts_at_the_morelli_trim_row tests.test_f16_runner.TestRunner.test_spawn_outside_the_table_exits_2 -v`

Expected: FAIL. The current runner writes the paper α (about 0.0389 rad) or does not print `trim lookup failed`.

- [ ] **Step 3: Wire the runner and split the benchmark initial state**

In `python/f16/compare.py`, rename the current `scenario_x0` body to `paper_x0`. The `gcas_long` branch keeps calling `paper_x0("gcas_upright")`. Replace `scenario_x0` with:

```python
def scenario_x0(scenario: str, aero: str = "morelli", height_m: float | None = None) -> np.ndarray:
    """Flown SI state from the trim table. Parity uses paper_x0 instead."""
    from f16.trim_table import trimmed_initial

    if scenario not in _SCENARIOS:
        raise ValueError(f"unknown scenario {scenario!r}")
    x, _ = trimmed_initial(scenario, aero=aero, height_m=height_m)
    return x
```

In `run_ours` and `run_python_reference`, replace `scenario_x0(scenario)` with `paper_x0(scenario)`. `run_ours` still constructs `F16Llc()` with the paper bias. That is the plant-parity path.

In `python/run_f16.py`, change `_initial_state` to:

```python
def _initial_state(maneuver: str, aero: str, n_m: float, e_m: float, d_m: float):
    from f16.trim_table import trimmed_initial

    pn_m, pe_m, h_m = ned_m_to_f16_m(n_m, e_m, d_m)
    if not all(math.isfinite(v) for v in (pn_m, pe_m, h_m)) or h_m <= 0.0:
        _die("bad setup: unphysical spawn")
    try:
        x0, u0 = trimmed_initial(maneuver, aero=aero, height_m=h_m)
    except ValueError as exc:
        _die(f"trim lookup failed: {exc}")
    x0 = x0.copy()
    x0[5] = 0.0
    x0[9] = pn_m
    x0[10] = pe_m
    return x0, u0.copy()
```

In `main`, after `llc = F16Llc()`:

```python
    x0, u0 = _initial_state(maneuver, args.aero, n_m, e_m, d_m)
    llc.xequil = x0.copy()
    llc.uequil = u0
    autopilot = _autopilot(maneuver, x0, llc, floor_m)
```

Remove the `scenario_x0` import from `run_f16.py` if nothing else in that file uses it.

In `python/tests/test_f16_gcas.py`, replace `_assert_spec_state` and `_fly` so both GCAS scenarios start from `trimmed_initial` at `H_GCAS_M`, install that trim on the LLC, and expect a wings-level state. Replace the two mode assertions with standby-only:

```python
    def _fly(self, scenario: str) -> dict:
        from f16.trim_table import trimmed_initial

        x0, u0 = trimmed_initial(scenario, aero="morelli", height_m=H_GCAS_M)
        self.assertAlmostEqual(float(x0[0]), VT_GCAS_MPS, places=6)
        self.assertEqual(float(x0[3]), 0.0)
        self.assertAlmostEqual(float(x0[4]), float(x0[1]), places=12)
        self.assertAlmostEqual(float(x0[11]), H_GCAS_M, places=6)
        llc = F16Llc()
        llc.xequil = x0.copy()
        llc.uequil = u0.copy()
        ap = GcasAutopilot(init_mode="standby", llc=llc)
        return run_sim(ap, x0, t_end=SCENARIO_HORIZONS[scenario], step=1 / 30)

    def test_mode_sequence_upright(self) -> None:
        out = self._fly("gcas_upright")
        self.assertEqual(_collapsed(out["modes"]), ["standby"])
        self.assertGreater(out["min_h_m"], 0.0)

    def test_mode_sequence_inverted(self) -> None:
        from f16.trim_table import trimmed_initial

        out = self._fly("gcas_inverted")
        self.assertEqual(_collapsed(out["modes"]), ["standby"])
        upright, _ = trimmed_initial("gcas_upright", aero="morelli", height_m=H_GCAS_M)
        inverted, _ = trimmed_initial("gcas_inverted", aero="morelli", height_m=H_GCAS_M)
        np.testing.assert_allclose(upright, inverted, rtol=0, atol=0)
```

Drop the unused `deg_to_rad` import from `python/tests/test_f16_gcas.py` if nothing else in the file uses it. Delete `_assert_spec_state`; `_fly` replaces it.

`python/tests/test_f16_scenarios.py` stays as it is: `long_x0("gcas_long")` follows `scenario_x0("gcas_upright")`, and both are now the table row.

`python/tests/test_f16_llc_si.py` stays as it is. `F16Llc()` is still the paper bias.

- [ ] **Step 4: Run tests to verify they pass**

Run:

```bash
cd python && python3 -m unittest tests.test_f16_runner tests.test_f16_gcas tests.test_f16_scenarios tests.test_f16_llc_si tests.test_f16_compare tests.test_f16_trim tests.test_f16_trim_table -v
```

Expected: PASS. `test_ours_matches_python_reference` still compares the paper initial state. `test_mode_sequence_upright` and `test_mode_sequence_inverted` stay in `standby`. `test_spawn_outside_the_table_exits_2` exits 2.

- [ ] **Step 5: Commit**

```bash
git add python/run_f16.py python/f16/compare.py python/tests/test_f16_gcas.py python/tests/test_f16_runner.py
git commit -m "feat(f16): start each maneuver from the trim-table lookup"
```

Do not commit unless the user has asked for commits in that session.

---

### Task 5: Document the tables

**Files:**
- Modify: `README.md`
- Modify: `UPDATES.md`

**Interfaces:**
- Consumes: the runner behavior from Task 4
- Produces: `UPDATES.md` section `0.86.0` at the top, and the two README edits below. No other markdown files.

- [ ] **Step 1: Update the docs**

Insert at the top of `UPDATES.md`, above `0.85.0`:

```markdown
## 0.86.0 - F-16 trim tables
- Wings-level `(vt, altitude)` trim for `stevens` and `morelli` is solved into `data/planes/f16/{stevens,morelli}.json`.
- `run_f16.py` starts every maneuver from the bilinear lookup at the maneuver speed (straight-level 502 ft/s, GCAS 540 ft/s) and the setup spawn altitude, and copies that trim into `xequil` / `uequil`. `K_lqr` stays the paper gain.
- Cells with Mach > 0.6, angle of attack outside −10° to +45°, or throttle or elevator outside its stop are omitted. A lookup outside a complete rectangle exits 2.
```

In the `python/f16/` row of the Architecture table, after the sentence that ends `run_f16.py --aero` (default `morelli`)., add:

`Wings-level trim for a run is the bilinear lookup in data/planes/f16/{morelli,stevens}.json at the maneuver speed (straight-level 153.0096 m/s, GCAS 164.592 m/s) and the setup spawn altitude. That state is the initial condition and the LQR equilibrium. The gain matrix stays the paper K_lqr.`

Replace the first sentence of the first Known limits bullet (the sentence that begins `Upright and inverted use the AeroBench`) with:

`The runner starts upright, inverted, and long GCAS from the same wings-level trim row (540 ft/s at the spawn altitude), so those runs do not use the AeroBench dive attitudes. AeroBench trajectory parity still integrates paper_x0, the historical benchmark state.`

- [ ] **Step 2: Check the version landed**

Run: `head -n 8 UPDATES.md`

Expected: the first heading is `## 0.86.0 - F-16 trim tables`.

- [ ] **Step 3: Commit**

```bash
git add README.md UPDATES.md
git commit -m "docs: describe the F-16 trim tables"
```

Do not commit unless the user has asked for commits in that session.
