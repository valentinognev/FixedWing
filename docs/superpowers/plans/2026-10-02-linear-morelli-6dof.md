# Linear Morelli Host 6DOF Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A host-only 13-state airplane whose aerodynamic forces and moments are the existing Morelli polynomial, with a synthetic airplane whose nonlinear polynomial slots are zero.

**Architecture:** `morelli_coefficients` in `python/f16/aero_morelli.py` is not rewritten. A new `python/plane/` package loads `data/planes/<id>/morelli.json`, evaluates that polynomial, and integrates the same rigid-body equations the F-16 uses, with mass, inertia, reference lengths, and a first-order thrust lag taken from the JSON. Zero slots remove the nonlinear terms. The factor `(1 - beta**2)` on `Cz` stays, because it is code in the polynomial, not a stored coefficient.

**Tech Stack:** Python 3, numpy, scipy RK45, unittest (`cd python && python3 -m unittest <module> -v`). Host-only. No Docker, MAVLink, ZMQ, PX4, network, or `fgfs`.

**Spec:** This plan is the spec. There is no separate design document. The approved design is the Approved design section below.

## Approved design

The F-16 plant stays. Do not change `python/f16/aero_morelli.py`, `python/f16/aero_data.py`, `python/f16/aero_stevens.py`, `python/f16/model.py`, `python/f16/sim.py`, or `data/planes/f16/`.

A new airplane calls `morelli_coefficients` with `root` set to the directory that contains that airplane's `morelli.json`. Alpha and beta pass through `f16.units.aerobench_poly_rad` before that call, the same way `subf16_derivative` does. Controls are radians. `xcg` and `xcg_ref` are fractions of the mean chord, the same dimensionless quantities the polynomial already takes.

Morelli group lengths, in the order `aero_data._MORELLI_GROUPS` already uses:

| Group | Length | Indexes that stay linear |
|-------|--------|--------------------------|
| `cx` | 7 | 0 constant, 1 `alpha`, 3 `de` |
| `cxq` | 5 | 0 |
| `cy` | 3 | 0 `beta`, 1 `da`, 2 `dr` |
| `cyp` | 4 | 0 |
| `cyr` | 4 | 0 |
| `cz` | 6 | 0 constant, 1 `alpha`, 5 `de` |
| `czq` | 5 | 0 |
| `cl` | 8 | 0 `beta` |
| `clp` | 4 | 0 |
| `clr` | 5 | 0 |
| `clda` | 7 | 0 |
| `cldr` | 7 | 0 |
| `cm` | 8 | 0 constant, 1 `alpha`, 2 `de` |
| `cmq` | 6 | 0 |
| `cn` | 7 | 0 `beta` |
| `cnp` | 5 | 0 |
| `cnr` | 3 | 0 |
| `cnda` | 10 | 0 |
| `cndr` | 6 | 0 |

Every other index is a power or a product. The synthetic airplane stores `0.0` there. `cy` has no nonlinear index.

`Cz` is still `(cz[0] + cz[1] * alpha + higher powers) * (1 - beta**2) + cz[5] * de`. With the higher powers at zero, a nonzero `beta` still multiplies the constant and the `alpha` term by `(1 - beta**2)`. That residual stays. Do not edit the polynomial to remove it.

State `x` is 13 SI values, altitude positive up, matching `python/f16/model.py`: `vt`, `alpha`, `beta`, `phi`, `theta`, `psi`, `p`, `q`, `r`, `pn`, `pe`, `alt`, `power`. Control `u` is `throttle`, `elevator`, `aileron`, `rudder`. Throttle and `power` are dimensionless. Surfaces are radians.

Inertia constants use the same arrangement as `_subf16`:

```text
gamma = Ixx * Izz - Ixz**2
c1 = ((Iyy - Izz) * Izz - Ixz**2) / gamma
c2 = (Ixx - Iyy + Izz) * Ixz / gamma
c3 = Izz / gamma
c4 = Ixz / gamma
c5 = (Izz - Ixx) / Iyy
c6 = Ixz / Iyy
c7 = 1 / Iyy
c8 = (Ixx * (Ixx - Iyy) + Ixz**2) / gamma
c9 = Ixx / gamma
```

`he` is engine angular momentum in kg·m². The synthetic airplane sets it to 0. Thrust along body +x is `power * t_max_n * max(0, 1 - vt / v_ref_mps)`. `power` obeys `(throttle - power) / tau_s`. Gravity is `f16.units.G_MPS2`. Dynamic pressure uses the same troposphere and stratosphere split as `model._adc`, via `f16.units` constants, not a second atmosphere model.

The file `data/planes/linear/morelli.json` is a synthetic linear airplane so the integrator has a file to load. It is not a Cessna, and it is not an Aircraft Intuitive Design import. Those numbers are a later plan.

No autopilot, trim survey, AID client, or PX4 glue in this plan.

## Global Constraints

- Host tests only: `cd python && python3 -m unittest <module> -v`. TDD: write the test, run it, see it fail, then implement. Record RED and GREEN command output in the task report.
- Every implementer and every reviewer uses model `grok-4.7-high`. If that slug is rejected by the tool, use `cursor-grok-4.6-high`. Fast models, Composer, Claude, GPT, and `inherit` are forbidden.
- Implementers do not launch subagents or reviewers. The parent dispatches the reviewer after the implementer reports.
- Do not push. Do not update git config. Do not use `--no-verify` or `--amend`. Commit with a HEREDOC only when the user has approved execution of this plan. Until that approval, stop after the plan file.
- Wave tasks that the Parallelism section marks parallel run in separate git worktrees branched from that wave's base commit. They do not share a working tree. Reviewers are read-only on the task branch.
- Do not start the next wave until every reviewer in the current wave has accepted and the parent has merged those branches.
- A merge conflict means the file split failed. Stop. Do not auto-resolve.
- `UPDATES.md` and `README.md` are edited only in Task 7. Current top version is `0.88.0`. The new entry is `0.89.0`.
- Do not add a controller, a trim solver, or an AID importer.

## Parallelism

| Wave | Parallel implementers | Base commit | Then parallel reviewers | Merge order |
|------|----------------------|-------------|-------------------------|-------------|
| 1 | Task 1, Task 2, Task 3 | `HEAD` when execution starts | Reviewer 1, 2, 3 | 1, then 2, then 3 |
| 2 | Task 4, Task 5 | merge commit of wave 1 | Reviewer 4, 5 | 4, then 5 |
| 3 | Task 6 | merge commit of wave 2 | Reviewer 6 | 6 |
| 4 | Task 7 | merge commit of wave 3 | Reviewer 7 | 7 |
| 5 | none | merge commit of wave 4 | one whole-branch reviewer | none |

Tasks in one wave do not import each other's new modules. Task 4 and Task 5 may import wave 1 modules. Task 6 may import Task 4. Task 7 may import Task 5 and Task 6.

Each reviewer reads only that task's brief, that task's diff, and the tests named in the task. Accept when the checks at the bottom of the task pass and the GREEN command is in the report. Reject when `python/f16/` or `data/planes/f16/` changed, when a test asserts a value the production function just returned without an independent formula, or when the recorded RED step did not fail.

The whole-branch reviewer reads the merged diff from the pre-wave-1 `HEAD`. Accept when the new unittest modules pass, `tests.test_f16_aero_coefficients` and `tests.test_f16_aero` still pass, and `python/f16/` is untouched.

## File map

| File | Task | Responsibility |
|------|------|----------------|
| `python/plane/__init__.py` | 1 | Empty package |
| `python/plane/groups.py` | 1 | Morelli lengths and which indexes are linear |
| `python/tests/test_plane_groups.py` | 1 | Zeros drop nonlinear terms; the `Cz` beta factor remains |
| `python/plane/aircraft.py` | 2 | Load mass, inertia, geometry, thrust; compute `c1`–`c9` |
| `python/tests/test_plane_aircraft.py` | 2 | Schema and inertia |
| `python/plane/atmosphere.py` | 3 | Mach, dynamic pressure, density |
| `python/tests/test_plane_atmosphere.py` | 3 | Sea level and stratosphere |
| `python/plane/dynamics.py` | 4 | 13-state derivative |
| `python/tests/test_plane_dynamics.py` | 4 | Analytic force and moment cases |
| `data/planes/linear/morelli.json` | 5 | Synthetic linear airplane |
| `python/tests/test_plane_linear_json.py` | 5 | Nonlinear slots are 0; linear slots match this plan |
| `python/plane/sim.py` | 6 | Constant-control RK45 |
| `python/tests/test_plane_sim.py` | 6 | Open-loop alpha grows with zero lift |
| `python/run_plane.py` | 7 | CLI |
| `python/tests/test_plane_runner.py` | 7 | CSV and exit 2 |
| `README.md` | 7 | One table row |
| `UPDATES.md` | 7 | `0.89.0` |

---

### Task 1: Morelli linear index

**Files:**
- Create: `python/plane/__init__.py`
- Create: `python/plane/groups.py`
- Create: `python/tests/test_plane_groups.py`

**Interfaces:**
- Consumes: `morelli_coefficients` and `load_aero_coefficients` as they exist today.
- Produces: `MORELLI_LENGTHS: dict[str, int]`, `LINEAR_INDEX: dict[str, tuple[int, ...]]`, `zero_coefficients() -> dict[str, list[float]]`, `nonlinear_index(name: str) -> tuple[int, ...]`.

- [ ] **Step 1: Write the failing test**

Create `python/tests/test_plane_groups.py`:

```python
import json
import tempfile
import unittest
from pathlib import Path

from f16.aero_data import _MORELLI_GROUPS
from f16.aero_morelli import morelli_coefficients


class TestPlaneGroups(unittest.TestCase):
    def test_lengths_cover_the_f16_groups(self) -> None:
        from plane.groups import MORELLI_LENGTHS, LINEAR_INDEX, zero_coefficients

        self.assertEqual(tuple(MORELLI_LENGTHS), _MORELLI_GROUPS)
        coeff = zero_coefficients()
        for name, length in MORELLI_LENGTHS.items():
            self.assertEqual(len(coeff[name]), length)
            self.assertTrue(set(LINEAR_INDEX[name]).issubset(range(length)))

    def test_all_zero_coefficients_are_zero_when_the_state_is_not(self) -> None:
        from plane.groups import zero_coefficients

        root = _write(zero_coefficients())
        got = _eval(root, alpha=0.4, beta=0.3, de=0.2, da=-0.1, dr=0.15, p=0.5, q=-0.4, r=0.3)
        self.assertEqual(got, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    def test_elevator_square_is_absent_until_its_slot_is_set(self) -> None:
        from plane.groups import zero_coefficients

        coeff = zero_coefficients()
        coeff["cx"][0] = -0.2
        coeff["cx"][3] = 0.5
        root = _write(coeff)
        base = _eval(root, de=0.0)[0]
        moved = _eval(root, de=0.2)[0]
        self.assertAlmostEqual(moved - base, 0.5 * 0.2, delta=1e-12)
        coeff["cx"][2] = 4.0
        root = _write(coeff)
        squared = _eval(root, de=0.2)[0]
        self.assertAlmostEqual(squared - base, 0.5 * 0.2 + 4.0 * 0.2 ** 2, delta=1e-12)

    def test_cz_keeps_the_beta_square_factor_when_higher_slots_are_zero(self) -> None:
        from plane.groups import zero_coefficients

        coeff = zero_coefficients()
        coeff["cz"][0] = -1.0
        root = _write(coeff)
        cz = _eval(root, beta=0.2)[2]
        self.assertAlmostEqual(cz, -1.0 * (1.0 - 0.2 ** 2), delta=1e-12)

    def test_roll_keeps_only_the_beta_term(self) -> None:
        from plane.groups import zero_coefficients, nonlinear_index

        self.assertIn(1, nonlinear_index("cl"))
        coeff = zero_coefficients()
        coeff["cl"][0] = 0.5
        root = _write(coeff)
        cl = _eval(root, alpha=0.3, beta=0.2)[3]
        self.assertAlmostEqual(cl, 0.5 * 0.2, delta=1e-12)


def _write(coefficients: dict) -> Path:
    directory = Path(tempfile.mkdtemp())
    payload = {"model": "morelli", "coefficients": coefficients}
    (directory / "morelli.json").write_text(json.dumps(payload))
    return directory


def _eval(root: Path, **kw) -> tuple[float, ...]:
    args = dict(alpha=0.0, beta=0.0, de=0.0, da=0.0, dr=0.0, p=0.0, q=0.0, r=0.0)
    args.update(kw)
    return tuple(
        float(v)
        for v in morelli_coefficients(
            args["alpha"], args["beta"], args["de"], args["da"], args["dr"],
            args["p"], args["q"], args["r"], 1.0, 1.0, 10.0, 0.25, 0.25, root=root,
        )
    )
```

- [ ] **Step 2: Run the test and see it fail**

Run: `cd python && python3 -m unittest tests.test_plane_groups -v`

Expected: FAIL with `ModuleNotFoundError: No module named 'plane'`.

- [ ] **Step 3: Implement**

`python/plane/__init__.py` is an empty file.

`python/plane/groups.py`:

```python
"""Which Morelli polynomial indexes are linear in one variable."""
from __future__ import annotations

MORELLI_LENGTHS: dict[str, int] = {
    "cx": 7, "cxq": 5, "cy": 3, "cyp": 4, "cyr": 4,
    "cz": 6, "czq": 5, "cl": 8, "clp": 4, "clr": 5,
    "clda": 7, "cldr": 7, "cm": 8, "cmq": 6, "cn": 7,
    "cnp": 5, "cnr": 3, "cnda": 10, "cndr": 6,
}

LINEAR_INDEX: dict[str, tuple[int, ...]] = {
    "cx": (0, 1, 3),
    "cxq": (0,),
    "cy": (0, 1, 2),
    "cyp": (0,),
    "cyr": (0,),
    "cz": (0, 1, 5),
    "czq": (0,),
    "cl": (0,),
    "clp": (0,),
    "clr": (0,),
    "clda": (0,),
    "cldr": (0,),
    "cm": (0, 1, 2),
    "cmq": (0,),
    "cn": (0,),
    "cnp": (0,),
    "cnr": (0,),
    "cnda": (0,),
    "cndr": (0,),
}


def zero_coefficients() -> dict[str, list[float]]:
    return {name: [0.0] * length for name, length in MORELLI_LENGTHS.items()}


def nonlinear_index(name: str) -> tuple[int, ...]:
    linear = set(LINEAR_INDEX[name])
    return tuple(i for i in range(MORELLI_LENGTHS[name]) if i not in linear)
```

- [ ] **Step 4: Run the test and see it pass**

Run: `cd python && python3 -m unittest tests.test_plane_groups -v`

Expected: PASS, 5 tests.

- [ ] **Step 5: Commit**

```bash
git add python/plane/__init__.py python/plane/groups.py python/tests/test_plane_groups.py
git commit -m "$(cat <<'EOF'
Add the Morelli index of linear polynomial slots.

EOF
)"
```

**Reviewer 1:** `grok-4.7-high`. Accept only if all five tests pass, `nonlinear_index("cy")` is empty, and `python/f16/aero_morelli.py` is not in the diff. Reject if the `Cz` test expected `-1.0` at `beta=0.2` instead of `-0.96`.

---

### Task 2: Aircraft JSON

**Files:**
- Create: `python/plane/aircraft.py`
- Create: `python/tests/test_plane_aircraft.py`

**Interfaces:**
- Consumes: `load_aero_coefficients("morelli", root=directory)` to reject a bad `coefficients` object at load time. Does not import `plane.groups`.
- Produces: frozen dataclass `Aircraft` with fields `mass_kg`, `s_m2`, `b_m`, `cbar_m`, `xcg`, `xcg_ref`, `ixx`, `iyy`, `izz`, `ixz`, `he`, `t_max_n`, `tau_s`, `v_ref_mps`, `c1`, `c2`, `c3`, `c4`, `c5`, `c6`, `c7`, `c8`, `c9`, `coeff_dir: Path`. Functions `load_aircraft(plane: str, root: Path | None = None) -> Aircraft`, `thrust_n(power: float, vt: float, aircraft: Aircraft) -> float`, `state_vector(payload_initial: dict) -> np.ndarray`, `control_vector(payload_controls: dict) -> np.ndarray`. `load_aircraft` also returns the parsed `initial` and `controls` dicts on the aircraft as `initial: dict` and `controls: dict`.

- [ ] **Step 1: Write the failing test**

Create `python/tests/test_plane_aircraft.py`:

```python
import json
import tempfile
import unittest
from pathlib import Path

LENGTHS = {
    "cx": 7, "cxq": 5, "cy": 3, "cyp": 4, "cyr": 4,
    "cz": 6, "czq": 5, "cl": 8, "clp": 4, "clr": 5,
    "clda": 7, "cldr": 7, "cm": 8, "cmq": 6, "cn": 7,
    "cnp": 5, "cnr": 3, "cnda": 10, "cndr": 6,
}
INITIAL = {
    "vt_mps": 40.0, "alpha_rad": 0.02, "beta_rad": 0.0, "phi_rad": 0.0,
    "theta_rad": 0.02, "psi_rad": 0.0, "p_rad_s": 0.0, "q_rad_s": 0.0,
    "r_rad_s": 0.0, "pn_m": 0.0, "pe_m": 0.0, "alt_m": 500.0, "power": 0.4,
}
CONTROLS = {
    "throttle": 0.4, "elevator_rad": 0.0, "aileron_rad": 0.0, "rudder_rad": 0.0,
}


def _aircraft(**overrides) -> dict:
    body = {
        "mass_kg": 2.0, "s_m2": 1.0, "b_m": 2.0, "cbar_m": 1.0,
        "xcg": 0.25, "xcg_ref": 0.25, "ixx": 2.0, "iyy": 3.0, "izz": 4.0,
        "ixz": 0.0, "he": 0.0, "t_max_n": 100.0, "tau_s": 2.0, "v_ref_mps": 50.0,
    }
    body.update(overrides)
    return body


def _write(aircraft: dict, *, include_aircraft: bool = True) -> Path:
    directory = Path(tempfile.mkdtemp())
    payload = {
        "model": "morelli",
        "initial": INITIAL,
        "controls": CONTROLS,
        "coefficients": {name: [0.0] * length for name, length in LENGTHS.items()},
    }
    if include_aircraft:
        payload["aircraft"] = aircraft
    (directory / "morelli.json").write_text(json.dumps(payload))
    return directory


class TestPlaneAircraft(unittest.TestCase):
    def test_inertia_with_zero_product(self) -> None:
        from plane.aircraft import load_aircraft

        directory = _write(_aircraft())
        aircraft = load_aircraft(directory.name, root=directory.parent)
        self.assertAlmostEqual(aircraft.c1, -0.5, delta=1e-12)
        self.assertAlmostEqual(aircraft.c2, 0.0, delta=1e-12)
        self.assertAlmostEqual(aircraft.c3, 0.5, delta=1e-12)
        self.assertAlmostEqual(aircraft.c4, 0.0, delta=1e-12)
        self.assertAlmostEqual(aircraft.c5, 2.0 / 3.0, delta=1e-12)
        self.assertAlmostEqual(aircraft.c6, 0.0, delta=1e-12)
        self.assertAlmostEqual(aircraft.c7, 1.0 / 3.0, delta=1e-12)
        self.assertAlmostEqual(aircraft.c8, -0.25, delta=1e-12)
        self.assertAlmostEqual(aircraft.c9, 0.25, delta=1e-12)

    def test_thrust_falls_with_speed(self) -> None:
        from plane.aircraft import load_aircraft, thrust_n

        directory = _write(_aircraft())
        aircraft = load_aircraft(directory.name, root=directory.parent)
        self.assertAlmostEqual(thrust_n(0.5, 10.0, aircraft), 40.0, delta=1e-12)
        self.assertAlmostEqual(thrust_n(0.5, 50.0, aircraft), 0.0, delta=1e-12)
        self.assertAlmostEqual(thrust_n(0.5, 80.0, aircraft), 0.0, delta=1e-12)

    def test_missing_aircraft_raises(self) -> None:
        from plane.aircraft import load_aircraft

        directory = _write(_aircraft(), include_aircraft=False)
        with self.assertRaises(ValueError) as caught:
            load_aircraft(directory.name, root=directory.parent)
        self.assertIn("aircraft", str(caught.exception))

    def test_nonpositive_inertia_raises(self) -> None:
        from plane.aircraft import load_aircraft

        directory = _write(_aircraft(ixx=0.0))
        with self.assertRaises(ValueError) as caught:
            load_aircraft(directory.name, root=directory.parent)
        self.assertIn("ixx", str(caught.exception))

    def test_singular_gamma_raises(self) -> None:
        from plane.aircraft import load_aircraft

        directory = _write(_aircraft(ixx=1.0, izz=1.0, ixz=1.0))
        with self.assertRaises(ValueError) as caught:
            load_aircraft(directory.name, root=directory.parent)
        self.assertIn("gamma", str(caught.exception))

    def test_vectors(self) -> None:
        from plane.aircraft import control_vector, load_aircraft, state_vector

        directory = _write(_aircraft())
        aircraft = load_aircraft(directory.name, root=directory.parent)
        state = state_vector(aircraft.initial)
        controls = control_vector(aircraft.controls)
        self.assertEqual(state.shape, (13,))
        self.assertEqual(float(state[0]), 40.0)
        self.assertEqual(controls.shape, (4,))
        self.assertEqual(float(controls[0]), 0.4)
```

`load_aircraft(plane, root)` reads `root / plane / "morelli.json"`. `_write` returns that plane directory, so each test passes `directory.name` and `directory.parent`.

- [ ] **Step 2: Run the test and see it fail**

Run: `cd python && python3 -m unittest tests.test_plane_aircraft -v`

Expected: FAIL with `No module named 'plane.aircraft'`.

- [ ] **Step 3: Implement**

`load_aircraft` resolves `root / plane / "morelli.json"` when `root` is set, otherwise `<repo>/data/planes/<plane>/morelli.json`, where the repo is `Path(__file__).resolve().parents[2]`. Read JSON. Require object `aircraft` with the keys listed above. Require `mass_kg`, `s_m2`, `b_m`, `cbar_m`, `ixx`, `iyy`, `izz`, `tau_s`, and `t_max_n` to be `> 0`. Require `gamma > 0`. Require `initial` and `controls` to contain the keys in Task 5. Call `load_aero_coefficients("morelli", root=the directory)` and let its `ValueError` propagate. Store `coeff_dir` as that directory.

```python
def thrust_n(power: float, vt: float, aircraft: Aircraft) -> float:
    scale = max(0.0, 1.0 - vt / aircraft.v_ref_mps)
    return float(power) * aircraft.t_max_n * scale
```

`state_vector` order: `vt_mps`, `alpha_rad`, `beta_rad`, `phi_rad`, `theta_rad`, `psi_rad`, `p_rad_s`, `q_rad_s`, `r_rad_s`, `pn_m`, `pe_m`, `alt_m`, `power`.

`control_vector` order: `throttle`, `elevator_rad`, `aileron_rad`, `rudder_rad`.

- [ ] **Step 4: Run the test and see it pass**

Run: `cd python && python3 -m unittest tests.test_plane_aircraft -v`

Expected: PASS, 6 tests.

- [ ] **Step 5: Commit**

```bash
git add python/plane/aircraft.py python/tests/test_plane_aircraft.py
git commit -m "$(cat <<'EOF'
Load a host airplane's mass, inertia, and thrust from JSON.

EOF
)"
```

**Reviewer 2:** `grok-4.7-high`. Accept only if the six tests pass and `c1` is computed from the formula in the Approved design, not hard-coded as `-0.5` for every aircraft. Reject if this task imported `plane.groups` or edited `python/f16/`.

---

### Task 3: Atmosphere

**Files:**
- Create: `python/plane/atmosphere.py`
- Create: `python/tests/test_plane_atmosphere.py`

**Interfaces:**
- Consumes: `RHO0_KG_M3`, `LAPSE_PER_M`, `T0_K`, `T_STRAT_K`, `H_STRAT_M`, `R_AIR` from `f16.units`.
- Produces: `air(vt_mps: float, alt_m: float) -> tuple[float, float, float]` returning `(mach, qbar_pa, rho)`.

- [ ] **Step 1: Write the failing test**

```python
import math
import unittest

from f16.units import H_STRAT_M, LAPSE_PER_M, R_AIR, RHO0_KG_M3, T0_K, T_STRAT_K


class TestPlaneAtmosphere(unittest.TestCase):
    def test_sea_level(self) -> None:
        from plane.atmosphere import air

        mach, qbar, rho = air(10.0, 0.0)
        self.assertAlmostEqual(rho, RHO0_KG_M3, delta=0)
        self.assertAlmostEqual(qbar, 0.5 * RHO0_KG_M3 * 100.0, delta=1e-9)
        speed = math.sqrt(1.4 * R_AIR * T0_K)
        self.assertAlmostEqual(mach, 10.0 / speed, delta=1e-12)

    def test_stratosphere_uses_the_f16_temperature(self) -> None:
        from plane.atmosphere import air

        mach, _, rho = air(10.0, H_STRAT_M)
        tfac = 1.0 - LAPSE_PER_M * H_STRAT_M
        self.assertAlmostEqual(rho, RHO0_KG_M3 * tfac ** 4.14, delta=1e-9)
        speed = math.sqrt(1.4 * R_AIR * T_STRAT_K)
        self.assertAlmostEqual(mach, 10.0 / speed, delta=1e-12)

    def test_nonpositive_speed_or_negative_altitude_raises(self) -> None:
        from plane.atmosphere import air

        with self.assertRaises(ValueError):
            air(0.0, 0.0)
        with self.assertRaises(ValueError):
            air(10.0, -1.0)
```

- [ ] **Step 2: Run the test and see it fail**

Run: `cd python && python3 -m unittest tests.test_plane_atmosphere -v`

Expected: FAIL with `No module named 'plane.atmosphere'`.

- [ ] **Step 3: Implement**

Copy the branch structure of `model._adc`: `tfac = 1 - LAPSE_PER_M * alt`; temperature is `T_STRAT_K` when `alt >= H_STRAT_M`, otherwise `T0_K * tfac`; `rho = RHO0_KG_M3 * tfac ** 4.14`; `mach = vt / sqrt(1.4 * R_AIR * t)`; `qbar = 0.5 * rho * vt * vt`. Raise `ValueError` if `vt <= 0` or `alt < 0`. Do not import `f16.model`.

- [ ] **Step 4: Run the test and see it pass**

Run: `cd python && python3 -m unittest tests.test_plane_atmosphere -v`

Expected: PASS, 3 tests.

- [ ] **Step 5: Commit**

```bash
git add python/plane/atmosphere.py python/tests/test_plane_atmosphere.py
git commit -m "$(cat <<'EOF'
Add the host airplane atmosphere using the F-16 ISA constants.

EOF
)"
```

**Reviewer 3:** `grok-4.7-high`. Accept only if the stratosphere test uses `T_STRAT_K` for the speed of sound and `tfac ** 4.14` for density. Reject if `python/f16/model.py` is in the diff.

---

### Task 4: Thirteen-state derivative

**Files:**
- Create: `python/plane/dynamics.py`
- Create: `python/tests/test_plane_dynamics.py`

**Interfaces:**
- Consumes: `Aircraft`, `thrust_n`, `load_aircraft` from Task 2; `air` from Task 3; `morelli_coefficients`; `aerobench_poly_rad`; `G_MPS2`. The temp JSON uses `zero_coefficients` from Task 1 only inside the test, to build the file.
- Produces: `plane_derivative(x: np.ndarray, u: np.ndarray, aircraft: Aircraft) -> np.ndarray` with shape `(13,)`.

- [ ] **Step 1: Write the failing test**

Build a temp plane with Task 2's inertia (`Ixx=2`, `Iyy=3`, `Izz=4`, `Ixz=0`, `he=0`, `mass_kg=2`, `s_m2=1`, `b_m=2`, `cbar_m=1`, `xcg=0.25`, `xcg_ref=0.25`, `tau_s=2`, `v_ref_mps=50`) and all Morelli slots zero unless a test sets one. State helper: 13 zeros, then set the fields the test names. `u = [0, 0, 0, 0]` unless the test sets throttle.

1. `test_zero_aero_alpha_rate_is_gravity_over_speed` — `vt=10`, `alt=0`, `power=0`, `t_max_n=0`. `xd[1] == G_MPS2 / 10` within `1e-12`. `xd[0] == 0`, `xd[2] == 0`, `xd[11] == 0`, `xd[12] == 0`.
2. `test_thrust_sets_speed_rate` — `t_max_n=100`, `mass_kg=2`, `power=0.5`, `vt=10`, `alt=0`, angles and rates 0. Thrust is `0.5 * 100 * (1 - 10/50) = 40`. `xd[0] == 40 / 2` within `1e-9`. `xd[12] == (0 - 0.5) / 2`.
3. `test_roll_acceleration_from_clp` — set `clp[0] = -0.5`, `p=0.2`, `vt=10`, `alt=0`, other rates 0, `power=0`, `t_max_n=0`. `phat = 0.2 * 2 / (2 * 10)`. `Cl = -0.5 * phat`. `qbar` from `air(10, 0)`. `xd[6] == qbar * 1 * 2 * (1/2) * Cl` within `1e-9`, because `c3 = 1/Ixx = 0.5` and `q = r = he = 0`.
4. `test_pitch_acceleration_from_cm_alpha` — set `cm[1] = -0.8`, `alpha=0.1`, `vt=10`, `alt=0`, `q=0`, `p=0`, `r=0`, `power=0`, `t_max_n=0`. Expected `Cm = -0.8 * aerobench_poly_rad(0.1)`. With `s=1`, `cbar=1`, `c7=1/3`, `xd[7] == qbar * 1 * 1 * (1/3) * Cm` within `1e-9`, where `qbar` is `air(10, 0)[1]`.
5. `test_bad_shape_raises` — `x` of length 12 raises `ValueError`.

Write `xd[6]` and `xd[7]` from those formulas in the test. Do not call `plane_derivative` to build the expected number.

- [ ] **Step 2: Run the test and see it fail**

Run: `cd python && python3 -m unittest tests.test_plane_dynamics -v`

Expected: FAIL with `No module named 'plane.dynamics'`.

- [ ] **Step 3: Implement**

`plane_derivative` checks shapes `(13,)` and `(4,)`. Raise `ValueError` if `vt <= 0`. Then:

```python
vt, alpha, beta, phi, theta, psi, p, q, r = (float(v) for v in x[:9])
alt = float(x[11])
power = float(x[12])
throttle, de, da, dr = (float(v) for v in u)
_, qbar, _ = air(vt, alt)
cx, cy, cz, cl, cm, cn = morelli_coefficients(
    aerobench_poly_rad(alpha), aerobench_poly_rad(beta),
    de, da, dr, p, q, r,
    aircraft.cbar_m, aircraft.b_m, vt, aircraft.xcg, aircraft.xcg_ref,
    root=aircraft.coeff_dir,
)
thrust = thrust_n(power, vt, aircraft)
cb = math.cos(beta)
ub = vt * math.cos(alpha) * cb
vb = vt * math.sin(beta)
wb = vt * math.sin(alpha) * cb
sth, cth = math.sin(theta), math.cos(theta)
sph, cph = math.sin(phi), math.cos(phi)
spsi, cpsi = math.sin(psi), math.cos(psi)
qs = qbar * aircraft.s_m2
qsb = qs * aircraft.b_m
inv_m = 1.0 / aircraft.mass_kg
ax = inv_m * (qs * cx + thrust)
ay = inv_m * qs * cy
az = inv_m * qs * cz
udot = r * vb - q * wb - G_MPS2 * sth + ax
vdot = p * wb - r * ub + G_MPS2 * cth * sph + ay
wdot = q * ub - p * vb + G_MPS2 * cth * cph + az
dum = ub * ub + wb * wb
xd = np.zeros(13)
xd[0] = (ub * udot + vb * vdot + wb * wdot) / vt
xd[1] = (ub * wdot - wb * udot) / dum
xd[2] = (vt * vdot - vb * xd[0]) * cb / dum
xd[3] = p + (sth / cth) * (q * sph + r * cph)
xd[4] = q * cph - r * sph
xd[5] = (q * sph + r * cph) / cth
he = aircraft.he
xd[6] = (aircraft.c2 * p + aircraft.c1 * r + aircraft.c4 * he) * q + qsb * (aircraft.c3 * cl + aircraft.c4 * cn)
xd[7] = (aircraft.c5 * p - aircraft.c7 * he) * r + aircraft.c6 * (r * r - p * p) + qs * aircraft.cbar_m * aircraft.c7 * cm
xd[8] = (aircraft.c8 * p - aircraft.c2 * r + aircraft.c9 * he) * q + qsb * (aircraft.c4 * cl + aircraft.c9 * cn)
t1 = sph * cpsi
t2 = cph * sth
t3 = sph * spsi
s1 = cth * cpsi
s2 = cth * spsi
s3 = t1 * sth - cph * spsi
s4 = t3 * sth + cph * cpsi
s5 = sph * cth
s6 = t2 * cpsi + t3
s7 = t2 * spsi - t1
s8 = cph * cth
xd[9] = ub * s1 + vb * s3 + wb * s6
xd[10] = ub * s2 + vb * s4 + wb * s7
xd[11] = ub * sth - vb * s5 - wb * s8
xd[12] = (throttle - power) / aircraft.tau_s
return xd
```

This is the F-16 kinematic and moment block with `1/mass` in place of `RM_PER_KG` and the JSON inertia constants in place of the F-16 `c1`–`c9` literals. Do not return `Nz` or `Ny`.

- [ ] **Step 4: Run the test and see it pass**

Run: `cd python && python3 -m unittest tests.test_plane_dynamics -v`

Expected: PASS, 5 tests.

- [ ] **Step 5: Commit**

```bash
git add python/plane/dynamics.py python/tests/test_plane_dynamics.py
git commit -m "$(cat <<'EOF'
Integrate Morelli coefficients in a 13-state host derivative.

EOF
)"
```

**Reviewer 4:** `grok-4.7-high`. Accept only if the five tests pass and the moment lines match the block above. Reject if `python/f16/model.py` was edited or if `Nz` was added.

---

### Task 5: Synthetic linear airplane file

**Files:**
- Create: `data/planes/linear/morelli.json`
- Create: `python/tests/test_plane_linear_json.py`

**Interfaces:**
- Consumes: `MORELLI_LENGTHS`, `LINEAR_INDEX`, `nonlinear_index`, `zero_coefficients` from Task 1. `load_aircraft("linear")` from Task 2.
- Produces: the shipped JSON. Linear slots:

```text
cx[0] = -0.05
cy[0] = -0.6
cy[2] = 0.15
cyp[0] = -0.3
cyr[0] = 0.2
cz[0] = -0.2
cz[1] = -4.5
cz[5] = -0.4
czq[0] = -2.0
cl[0] = -0.1
clp[0] = -0.4
clr[0] = 0.1
clda[0] = -0.15
cldr[0] = 0.01
cm[0] = 0.05
cm[1] = -0.8
cm[2] = -1.0
cmq[0] = -12.0
cn[0] = 0.08
cnp[0] = -0.03
cnr[0] = -0.12
cnda[0] = 0.01
cndr[0] = -0.07
```

Every index not in that list is `0.0`, including the linear indexes that are omitted (`cx[1]`, `cx[3]`, `cxq[0]`, `cy[1]`).

`aircraft`: `mass_kg` 1000, `s_m2` 16, `b_m` 10, `cbar_m` 1.5, `xcg` 0.25, `xcg_ref` 0.25, `ixx` 1200, `iyy` 2000, `izz` 3000, `ixz` 0, `he` 0, `t_max_n` 2500, `tau_s` 1, `v_ref_mps` 70.

`initial`: `vt_mps` 40, `alpha_rad` 0.02, `beta_rad` 0, `phi_rad` 0, `theta_rad` 0.02, `psi_rad` 0, `p_rad_s` 0, `q_rad_s` 0, `r_rad_s` 0, `pn_m` 0, `pe_m` 0, `alt_m` 500, `power` 0.4.

`controls`: `throttle` 0.4, `elevator_rad` 0, `aileron_rad` 0, `rudder_rad` 0.

`model` is `"morelli"`.

- [ ] **Step 1: Write the failing test**

`test_nonlinear_slots_are_zero` loads the shipped file from the repo (`load_aircraft("linear")` and the `coefficients` object via `load_aero_coefficients("morelli", root=aircraft.coeff_dir)`). For every group, each index in `nonlinear_index(name)` equals `0.0`. Each linear value in the list above equals that number within `0`. `test_aircraft_block` checks `mass_kg == 1000`, `he == 0`, `initial["vt_mps"] == 40`, `controls["throttle"] == 0.4`.

- [ ] **Step 2: Run the test and see it fail**

Run: `cd python && python3 -m unittest tests.test_plane_linear_json -v`

Expected: FAIL with `ValueError` or `FileNotFoundError` because `data/planes/linear/morelli.json` is absent.

- [ ] **Step 3: Write the JSON**

Start from `zero_coefficients()`, set the listed indexes, and write `data/planes/linear/morelli.json` with `aircraft`, `initial`, `controls`, and `coefficients`. Do not copy `data/planes/f16/morelli.json`.

- [ ] **Step 4: Run the test and see it pass**

Run: `cd python && python3 -m unittest tests.test_plane_linear_json -v`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add data/planes/linear/morelli.json python/tests/test_plane_linear_json.py
git commit -m "$(cat <<'EOF'
Add a synthetic linear Morelli airplane with nonlinear slots at zero.

EOF
)"
```

**Reviewer 5:** `grok-4.7-high`. Accept only if every nonlinear index is `0.0` and the file's comment-free JSON does not claim the numbers are a Cessna or an AID result. Reject if `data/planes/f16/morelli.json` is in the diff.

---

### Task 6: Open-loop integration

**Files:**
- Create: `python/plane/sim.py`
- Create: `python/tests/test_plane_sim.py`

**Interfaces:**
- Consumes: `plane_derivative` from Task 4. `load_aircraft` from Task 2. A temp airplane, not the shipped file.
- Produces: `integrate(x0: np.ndarray, u: np.ndarray, aircraft: Aircraft, t_end: float, step: float = 0.05) -> dict` with `times` a list of floats and `states` an `(n, 13)` array. Includes the initial sample at `t = 0`.

- [ ] **Step 1: Write the failing test**

Temp airplane: all coefficients zero, `t_max_n=0`, `mass_kg=2`, `s_m2=1`, `b_m=2`, `cbar_m=1`, `xcg=0.25`, `xcg_ref=0.25`, `ixx=2`, `iyy=3`, `izz=4`, `ixz=0`, `he=0`, `tau_s=2`, `v_ref_mps=50`, plus the `initial` and `controls` objects written in Task 2's test (`vt_mps` 40 is only the JSON initial; the integrated `x0` below replaces it). `x0` has `vt=10`, `alt=100`, everything else 0. `u` is four zeros. `t_end=0.2`, `step=0.05`.

1. `test_alpha_grows_when_lift_is_zero` — `states[-1, 1] > states[0, 1]`. `times[0] == 0`. `times[-1]` is within one step of `0.2`. Every state entry is finite.
2. `test_nonpositive_duration_raises` — `t_end=0` raises `ValueError`.

- [ ] **Step 2: Run the test and see it fail**

Run: `cd python && python3 -m unittest tests.test_plane_sim -v`

Expected: FAIL with `No module named 'plane.sim'`.

- [ ] **Step 3: Implement**

Use `scipy.integrate.RK45` with `rtol=1e-7` and `atol=1e-7`, the same tolerances as `f16.sim.run_sim`. Hold `u` constant. Append a sample every `step` seconds until `t_end`. Raise `ValueError` if `t_end <= 0` or `step <= 0`. If a derivative is non-finite, stop and keep the samples already stored; set `rejected_t` on the returned dict to that time, otherwise `None`.

- [ ] **Step 4: Run the test and see it pass**

Run: `cd python && python3 -m unittest tests.test_plane_sim -v`

Expected: PASS, 2 tests.

- [ ] **Step 5: Commit**

```bash
git add python/plane/sim.py python/tests/test_plane_sim.py
git commit -m "$(cat <<'EOF'
Integrate the host airplane at constant controls.

EOF
)"
```

**Reviewer 6:** `grok-4.7-high`. Accept only if the zero-lift run ends at a larger alpha and the integrator is RK45. Reject if an autopilot or a trim loop was added.

---

### Task 7: Runner and docs

**Files:**
- Create: `python/run_plane.py`
- Create: `python/tests/test_plane_runner.py`
- Modify: `README.md` (package-layout table only)
- Modify: `UPDATES.md` (new top entry)

**Interfaces:**
- Consumes: `load_aircraft`, `state_vector`, `control_vector`, `integrate`.
- Produces: `python/run_plane.py`. `main(argv: list[str] | None = None) -> int`.

- [ ] **Step 1: Write the failing test**

`test_linear_csv` runs `main(["--plane", "linear", "--duration", "0.1", "--csv", tmp])` and expects return code 0. The CSV header is `t,vt,alpha,beta,phi,theta,psi,p,q,r,pn,pe,alt,power`. The first data row has `t=0` and `vt=40`. At least two data rows. All parsed values are finite.

`test_unknown_plane_exits_2` runs `main(["--plane", "no-such-plane", "--duration", "0.1", "--csv", tmp])` and expects return code 2.

`test_help_exits_0` runs `main(["--help"])` and expects return code 0.

- [ ] **Step 2: Run the test and see it fail**

Run: `cd python && python3 -m unittest tests.test_plane_runner -v`

Expected: FAIL with `No module named 'run_plane'` or an import error of `main`.

- [ ] **Step 3: Implement**

argparse: `--plane` default `linear`, `--duration` required and positive, `--csv` required path. Load the airplane, build `x0` and `u` from `initial` and `controls`, call `integrate` with `step=0.05`. Write the header and one row per sample, `t` then the 13 states, `%.10g` formatting is fine. `ValueError` and missing file print the error to stderr and return 2. `--help` returns 0 and does not integrate.

In the package-layout table in `README.md`, add this row immediately after the `python/f16/` row:

```markdown
| `python/plane/` | Host-only 13-state airplane. Aero is `morelli_coefficients` and `data/planes/<id>/morelli.json`. `data/planes/linear/morelli.json` is synthetic: nonlinear Morelli slots are 0. `cd python && python3 run_plane.py --plane linear --duration 1 --csv /tmp/linear.csv`. No PX4. |
```

Insert at the top of `UPDATES.md`:

```markdown
## 0.89.0 - Linear Morelli host 6DOF
- `python/plane/` integrates a 13-state airplane with `morelli_coefficients`. Nonlinear slots in `data/planes/linear/morelli.json` are zero.
- `python/run_plane.py --plane linear` writes a host-only CSV. No PX4, MAVLink, Docker, or FlightGear.
```

- [ ] **Step 4: Run the tests and see them pass**

Run: `cd python && python3 -m unittest tests.test_plane_runner tests.test_plane_groups tests.test_plane_aircraft tests.test_plane_atmosphere tests.test_plane_dynamics tests.test_plane_linear_json tests.test_plane_sim tests.test_f16_aero_coefficients tests.test_f16_aero -v`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add python/run_plane.py python/tests/test_plane_runner.py README.md UPDATES.md
git commit -m "$(cat <<'EOF'
Add the host runner for the linear Morelli airplane.

EOF
)"
```

**Reviewer 7:** `grok-4.7-high`. Accept only if the CSV test passes, unknown plane returns 2, and `UPDATES.md` starts at `0.89.0`. Reject if the README calls the synthetic file a Cessna.

---

### Whole-branch review

After Task 7 is merged, dispatch one reviewer on `grok-4.7-high`.

Scope: diff from the pre-wave-1 commit to the Task 7 merge. Run the unittest command in Task 7 Step 4.

Accept when that command passes, `python/f16/` and `data/planes/f16/` have no diff, the synthetic file's nonlinear slots are zero, and no task added a controller or an AID client.

Reject and send one fix task if any of those fail. The fix task gets its own reviewer. Then stop.
