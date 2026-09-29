# F-16 Independent Aero Models Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the F-16 plant evaluate either a Stevens aero model or a Morelli aero model, each complete by itself, with no Stevens table added on top of the Morelli polynomials.

**Architecture:** Atmosphere, engine, mass, inertia, and the rigid-body equations stay in `python/f16/model.py`. Each aero module returns the final six coefficients `Cx, Cy, Cz, Cl, Cm, Cn`. Stevens builds those from the Appendix A.8 lookup tables, then adds the Stevens `DAMP` table and the one CG shift. Morelli returns the polynomial model only; those polynomials already contain `Cxq, Cyp, Cyr, Czq, Clp, Clr, Cmq, Cnp, Cnr` and one CG shift. `model.py` does not add damping or a second CG shift after the call. `subf16_derivative(..., model="morelli"|"stevens")` selects. The runner flag is `--aero`, default `morelli`.

**Tech Stack:** Python 3, numpy, unittest (`cd python && python3 -m unittest <module> -v`). Host-only. No Docker, MAVLink, ZMQ, PX4, network, or `fgfs`.

**Spec:** This plan is the spec. Binding scope is the Global Constraints below. The Stevens tables are Appendix A.8 in `refs/F16stevens.md`. The machine-readable copy to port is `../F16/AeroBenchVVPython/code/aerobench/lowlevel/{cx,cy,cz,cl,cm,cn,dlda,dldr,dnda,dndr,dampp}.py`. Do not retype arrays from the markdown; it is line-broken. The Morelli polynomials are the body of `_morellif16` in `python/f16/model.py`, matching `morellif16.py` in that AeroBench tree.

## Global Constraints

- Two aero models only: `stevens` and `morelli`. Do not keep a third "mixed" or "aerobench" mode. Do not add damping from one model onto the other.
- Stevens includes its own damping table (`CXq, CYr, CYp, CZq, Clr, Clp, Cmq, Cnr, Cnp` versus angle of attack). That table is part of Stevens, not a shared overlay. Morelli does not call it. Morelli's rate derivatives are the polynomial terms already inside `_morellif16`.
- `model.py` after the split calls one `coefficients(...)` function and then the existing force and moment equations. Delete the block that does `d = _dampp(...)` and the following six lines that add `cq * d`, `b2v * d`, and `czt * (xcgr - xcg)` / `cyt * (xcgr - xcg)`. Those additions move into `aero_stevens.coefficients` only.
- Both CG locations stay `0.35`. At that value the CG shift is zero, so removing the second add does not move today's trim residual. The shift is still applied once, inside the model that owns it: Stevens adds it because the tables do not; Morelli already adds it inside the polynomial and must not add it again.
- Public state and surface commands stay SI. Stevens table indexes use degrees. Convert with `aerobench_deg` (`57.29578`) at the Stevens boundary only. Do not use `180/pi` for those indexes. Morelli polynomial angles and surfaces stay radians, using the existing `aerobench_poly_rad` on α and β only.
- `subf16_derivative` default model remains `"morelli"`, meaning the pure polynomial model. Unknown model strings raise `ValueError`.
- Do not edit `../F16/AeroBenchVV`, `../F16/AeroBenchVVPython`, or `../F16/f16-flight-dynamics`. Do not edit `python/f16/llc.py` or the LQR gains. Do not retune GCAS floors.
- AeroBench `subf16_model(..., "morelli")` and `f16dynamics` still add the Stevens damping table. They are the reference for Stevens (`model="stevens"`) and for Morelli only at `p = q = r = 0`, where that table contributes nothing. They are not the reference for pure Morelli at a nonzero rate.
- GCAS trajectory parity against AeroBench's mixed Morelli plant is retired. Straight-and-level trajectory parity stays, because that run holds near-zero rates. Derivative tests below replace the GCAS lock.
- Engine, atmosphere, and inertia stay shared. The idle-thrust entry −710 lbf is already in `_thrust` and is not part of this split.
- Host tests only: `cd python && python3 -m unittest <module> -v`.
- Each task is one implementer, then one reviewer. Commit commands are in the tasks. Do not commit unless the user has asked for commits in that session; the controller may defer them.
- After the behavior change, UPDATES gets one new top entry `0.85.0`. README changes only the sentences named in Task 5.

## Parallelism

- Task 1 (`aero_morelli.py`, `model.py` Morelli path, `test_f16_aero.py`, `test_f16_model_si.py`) finishes before Task 2. Task 2 adds Stevens into the same `model.py` dispatcher.
- Task 3 threads `--aero` through `sim.py` and `run_f16.py` after Task 2.
- Task 4 retargets the mixed-Morelli parity tests after Task 3.
- Task 5 (`README.md`, `UPDATES.md`) runs last.

## File map

| File | Task | Responsibility |
|------|------|----------------|
| `python/f16/aero_morelli.py` | 1 | Polynomial coefficients only |
| `python/f16/model.py` | 1–2 | Dispatcher; no post-hoc damping |
| `python/tests/test_f16_aero.py` | 1–2 | Pure Morelli, pure Stevens, they differ at nonzero `q` |
| `python/tests/test_f16_model_si.py` | 1 | Zero-rate Morelli match; off-trim no longer expects the mix |
| `python/f16/aero_stevens.py` | 2 | Appendix A.8 tables plus Stevens damping and one CG shift |
| `python/f16/sim.py`, `python/run_f16.py` | 3 | `--aero morelli\|stevens` |
| `python/tests/test_f16_runner.py` | 3 | Bad `--aero` exits 2 |
| `python/tests/test_f16_model_llc.py`, `test_f16_cpp_plant.py`, `test_f16_compare.py` | 4 | Drop mixed-Morelli off-trim and GCAS trajectory locks |
| `README.md`, `UPDATES.md` | 5 | `0.85.0` |

---

### Task 1: Pure Morelli

**Files:**
- Create: `python/f16/aero_morelli.py`
- Modify: `python/f16/model.py`
- Create: `python/tests/test_f16_aero.py`
- Modify: `python/tests/test_f16_model_si.py`

**Interfaces:**
- Consumes: the body of `_morellif16` in `model.py`; `f16.units.aerobench_poly_rad`
- Produces: `morelli_coefficients(alpha_rad, beta_rad, de_rad, da_rad, dr_rad, p, q, r, cbar_m, b_m, vt_mps, xcg, xcgref) -> tuple` of six floats `(Cx, Cy, Cz, Cl, Cm, Cn)`. `subf16_derivative(..., model="morelli")` uses that tuple and does not call `_dampp`.

- [ ] **Step 1: Write the failing test**

Create `python/tests/test_f16_aero.py`:

```python
import sys
import unittest

import numpy as np

REF = "/home/valentin/Projects/FlightSimulation/F16/AeroBenchVVPython/code"
sys.path.insert(0, REF)

from aerobench.lowlevel.morellif16 import Morellif16  # noqa: E402
from aerobench.lowlevel.subf16_model import subf16_model as ref_subf16  # noqa: E402

from f16.llc import _U_IMP, _X_IMP  # noqa: E402
from f16.model import subf16_derivative  # noqa: E402
from f16.units import (  # noqa: E402
    aerobench_poly_rad,
    state_imp_to_si,
    state_si_to_imp,
    u_imp_to_si,
)


class TestPureMorelli(unittest.TestCase):
    def test_coefficients_match_polynomial_at_nonzero_pitch_rate(self) -> None:
        from f16.aero_morelli import morelli_coefficients
        from f16.units import B_M, CBAR_M

        alpha = 0.04
        beta = 0.01
        de = -0.02
        p, q, r = 0.05, 0.2, -0.03
        vt = 150.0
        ours = morelli_coefficients(
            aerobench_poly_rad(alpha), aerobench_poly_rad(beta), de, 0.01, -0.01,
            p, q, r, CBAR_M, B_M, vt, 0.35, 0.35,
        )
        ref = Morellif16(
            aerobench_poly_rad(alpha), aerobench_poly_rad(beta), de, 0.01, -0.01,
            p, q, r, CBAR_M, B_M, vt, 0.35, 0.35,
        )
        np.testing.assert_allclose(ours, ref, rtol=0, atol=1e-12)

    def test_zero_rate_derivative_matches_aerobench_morelli(self) -> None:
        x_imp = _X_IMP.copy()
        u_imp = _U_IMP.copy()
        xd, nz, ny = subf16_derivative(state_imp_to_si(x_imp), u_imp_to_si(u_imp), model="morelli")
        rxd, rnz, rny, _, _ = ref_subf16(x_imp, u_imp, "morelli")
        np.testing.assert_allclose(state_si_to_imp(xd), rxd, rtol=1e-8, atol=1e-8)
        self.assertAlmostEqual(nz, float(rnz), places=6)
        self.assertAlmostEqual(ny, float(rny), places=6)

    def test_nonzero_pitch_rate_is_not_the_mixed_model(self) -> None:
        x_imp = _X_IMP.copy()
        x_imp[7] = 0.2
        u_imp = _U_IMP.copy()
        xd, _, _ = subf16_derivative(state_imp_to_si(x_imp), u_imp_to_si(u_imp), model="morelli")
        rxd, _, _, _, _ = ref_subf16(x_imp, u_imp, "morelli")
        self.assertGreater(abs(float(state_si_to_imp(xd)[7]) - float(rxd[7])), 1e-3)
```

In `python/tests/test_f16_model_si.py`, `test_off_trim_matches_aerobench_after_conversion` currently adds a roll rate and expects the mixed AeroBench Morelli derivative. Replace that method with a zero-rate speed/alpha/elevator perturbation that still matches, and delete any expectation that a nonzero rate matches `ref_subf16(..., "morelli")`:

```python
    def test_off_trim_zero_rate_matches_aerobench_morelli(self) -> None:
        x = _X_IMP.copy()
        x[0] += 20.0
        x[1] += 0.02
        u = _U_IMP.copy()
        u[1] += 2.0
        self._match(x, u)
```

Leave `test_stevens_raises` in place until Task 2. Leave `test_dampp_ten_aerobench_degrees_from_radians` importing `f16.model._dampp` until Task 2 moves the table.

- [ ] **Step 2: Run test to verify it fails**

Run: `cd python && python3 -m unittest tests.test_f16_aero.TestPureMorelli -v`

Expected: FAIL with `ModuleNotFoundError: No module named 'f16.aero_morelli'`.

- [ ] **Step 3: Write minimal implementation**

Create `python/f16/aero_morelli.py` by moving the body of `_morellif16` unchanged. Public name `morelli_coefficients`. Do not import `_dampp`. Do not add `cq * d[...]`.

In `python/f16/model.py`:

- Import `morelli_coefficients`.
- Replace the `_morellif16` call and the following damping / CG block (`tvt` through the `cnt = cnt + b2v * ...` line) with:

```python
    cxt, cyt, czt, clt, cmt, cnt = morelli_coefficients(
        aerobench_poly_rad(x[1]), aerobench_poly_rad(x[2]), el, ail, rdr,
        p, q, r, cbar, b, vt, xcg, xcgr)
```

- Pass `model` into `_subf16_morelli`. When `model != "morelli"`, raise `ValueError` with the same message `subf16_derivative` uses today (`model {model!r} is not implemented`). `subf16_derivative` keeps that check so `"stevens"` still raises in this task.
- Delete `_morellif16` from `model.py`. Keep `_dampp` until Task 2.

`aerobench_poly_rad` stays on α and β only. `el`, `ail`, and `rdr` are already radians.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd python && python3 -m unittest tests.test_f16_aero tests.test_f16_model_si -v`

Expected: PASS. `test_stevens_raises` still passes.

- [ ] **Step 5: Commit**

```bash
git add python/f16/aero_morelli.py python/f16/model.py python/tests/test_f16_aero.py python/tests/test_f16_model_si.py
git commit -m "feat(f16): evaluate Morelli aero without the Stevens damping table"
```

Do not commit unless the user has asked for commits in that session.

---

### Task 2: Pure Stevens

**Files:**
- Create: `python/f16/aero_stevens.py`
- Modify: `python/f16/model.py`
- Modify: `python/tests/test_f16_aero.py`
- Modify: `python/tests/test_f16_model_si.py`

**Interfaces:**
- Consumes: AeroBench `cx, cy, cz, cl, cm, cn, dlda, dldr, dnda, dndr, dampp` (read-only); `f16.units.aerobench_deg`
- Produces: `stevens_coefficients(alpha_rad, beta_rad, de_rad, da_rad, dr_rad, p, q, r, cbar_m, b_m, vt_mps, xcg, xcgref) -> tuple` of six floats. The tuple already includes the damping-table increments and one CG shift. `subf16_derivative(..., model="stevens")` returns the SI derivative. `model="other"` still raises `ValueError`.

- [ ] **Step 1: Write the failing test**

Append to `python/tests/test_f16_aero.py`:

```python
class TestPureStevens(unittest.TestCase):
    def test_sideforce_matches_appendix_linear_law(self) -> None:
        from f16.aero_stevens import stevens_coefficients
        from f16.units import B_M, CBAR_M, aerobench_deg

        beta = 0.1
        ail = 0.05
        rdr = -0.02
        beta_deg = aerobench_deg(beta)
        ail_deg = aerobench_deg(ail)
        rdr_deg = aerobench_deg(rdr)
        expected_cy = -0.02 * beta_deg + 0.021 * (ail_deg / 20.0) + 0.086 * (rdr_deg / 30.0)
        _, cy, _, _, _, _ = stevens_coefficients(
            0.0, beta, 0.0, ail, rdr, 0.0, 0.0, 0.0, CBAR_M, B_M, 150.0, 0.35, 0.35,
        )
        self.assertAlmostEqual(cy, expected_cy, places=10)

    def test_derivative_matches_aerobench_stevens_with_pitch_rate(self) -> None:
        x_imp = _X_IMP.copy()
        x_imp[7] = 0.2
        u_imp = _U_IMP.copy()
        xd, nz, ny = subf16_derivative(state_imp_to_si(x_imp), u_imp_to_si(u_imp), model="stevens")
        rxd, rnz, rny, _, _ = ref_subf16(x_imp, u_imp, "stevens")
        np.testing.assert_allclose(state_si_to_imp(xd), rxd, rtol=1e-8, atol=1e-8)
        self.assertAlmostEqual(nz, float(rnz), places=6)
        self.assertAlmostEqual(ny, float(rny), places=6)

    def test_models_differ_at_nonzero_pitch_rate(self) -> None:
        x = state_imp_to_si(_X_IMP)
        u = u_imp_to_si(_U_IMP)
        x = x.copy()
        x[7] = 0.2
        xd_m, _, _ = subf16_derivative(x, u, model="morelli")
        xd_s, _, _ = subf16_derivative(x, u, model="stevens")
        self.assertGreater(abs(float(xd_m[7] - xd_s[7])), 1e-4)

    def test_unknown_model_raises(self) -> None:
        with self.assertRaises(ValueError):
            subf16_derivative(state_imp_to_si(_X_IMP), u_imp_to_si(_U_IMP), model="mixed")
```

In `test_f16_model_si.py`, replace `test_stevens_raises` with the unknown-model case above if it still expects `ValueError` for `"stevens"`. Move `test_dampp_ten_aerobench_degrees_from_radians` to import `damp_table` from `f16.aero_stevens` (the function Task 2 names below). Expected row for 10 AeroBench degrees stays:

```python
[2.08, 0.962, 0.258, -31.2, 0.208, -0.383, -6.11, -0.370, -0.013]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd python && python3 -m unittest tests.test_f16_aero.TestPureStevens -v`

Expected: FAIL with `ModuleNotFoundError: No module named 'f16.aero_stevens'`, or `ValueError` because `"stevens"` is not implemented yet.

- [ ] **Step 3: Write minimal implementation**

Create `python/f16/aero_stevens.py`.

Port these functions by copying each array and interpolation body from the matching AeroBench file, unchanged: `cx`, `cy`, `cz`, `cl`, `cm`, `cn`, `dlda`, `dldr`, `dnda`, `dndr`. Their arguments stay degrees, as in AeroBench. Move `_dampp` from `model.py` to `damp_table(alpha_rad)` in this module. Keep the `0.2 * aerobench_deg(alpha_rad)` index. Delete `_dampp` from `model.py`.

`stevens_coefficients` converts radians to AeroBench degrees, calls the tables, then applies damping and one CG shift:

```python
def stevens_coefficients(alpha_rad, beta_rad, de_rad, da_rad, dr_rad, p, q, r, cbar, b, vt, xcg, xcgref):
    alpha = aerobench_deg(alpha_rad)
    beta = aerobench_deg(beta_rad)
    de = aerobench_deg(de_rad)
    da = aerobench_deg(da_rad)
    dr = aerobench_deg(dr_rad)
    dail = da / 20.0
    drdr = dr / 30.0
    cx = cx_table(alpha, de)
    cy = cy_table(beta, da, dr)
    cz = cz_table(alpha, beta, de)
    cl = cl_table(alpha, beta) + dlda_table(alpha, beta) * dail + dldr_table(alpha, beta) * drdr
    cm = cm_table(alpha, de)
    cn = cn_table(alpha, beta) + dnda_table(alpha, beta) * dail + dndr_table(alpha, beta) * drdr
    tvt = 0.5 / vt
    b2v = b * tvt
    cq = cbar * q * tvt
    d = damp_table(alpha_rad)
    cx = cx + cq * d[0]
    cy = cy + b2v * (d[1] * r + d[2] * p)
    cz = cz + cq * d[3]
    cl = cl + b2v * (d[4] * r + d[5] * p)
    cm = cm + cq * d[6] + cz * (xcgref - xcg)
    cn = cn + b2v * (d[7] * r + d[8] * p) - cy * (xcgref - xcg) * cbar / b
    return cx, cy, cz, cl, cm, cn
```

Use the real function names from the port. `dail` and `drdr` match AeroBench `subf16_model`: aileron degrees over 20, rudder degrees over 30.

In `model.py`, select coefficients and do not add damping afterward:

```python
def _coefficients(model, alpha, beta, de, da, dr, p, q, r, cbar, b, vt, xcg, xcgref):
    if model == "morelli":
        return morelli_coefficients(
            aerobench_poly_rad(alpha), aerobench_poly_rad(beta), de, da, dr,
            p, q, r, cbar, b, vt, xcg, xcgref)
    if model == "stevens":
        return stevens_coefficients(
            alpha, beta, de, da, dr, p, q, r, cbar, b, vt, xcg, xcgref)
    raise ValueError(f"model {model!r} is not implemented")
```

`_subf16_morelli` takes `model` and calls `_coefficients`. Rename it to `_subf16` only if every caller in `model.py` is updated in this same step. `subf16_derivative` passes `model` through and deletes its own duplicate `"morelli"` check so the single `ValueError` comes from `_coefficients`.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd python && python3 -m unittest tests.test_f16_aero tests.test_f16_model_si -v`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add python/f16/aero_stevens.py python/f16/aero_morelli.py python/f16/model.py python/tests/test_f16_aero.py python/tests/test_f16_model_si.py
git commit -m "feat(f16): add the Stevens aero tables as their own model"
```

Do not commit unless the user has asked for commits in that session.

---

### Task 3: Select the model from the runner

**Files:**
- Modify: `python/f16/sim.py`
- Modify: `python/run_f16.py`
- Modify: `python/tests/test_f16_runner.py`

**Interfaces:**
- Consumes: `subf16_derivative(..., model=)`
- Produces: `run_sim(..., aero="morelli")` forwards `aero` into every derivative evaluation. `run_f16.py --aero {morelli,stevens}`, default `morelli`. Any other value exits 2 with one stderr line before integrating.

- [ ] **Step 1: Write the failing test**

Append to `python/tests/test_f16_runner.py`:

```python
    def test_bad_aero_exits_2(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            sp = Path(td) / "setup.json"
            sp.write_text(json.dumps({
                "maneuver": "straight_level",
                "duration_s": 1.0,
                "spawn": {"n_m": 0.0, "e_m": 0.0, "d_m": -304.8},
                "gcas_floor_m": 304.8,
            }))
            r = subprocess.run(
                [sys.executable, "run_f16.py", "--setup", str(sp), "--aero", "mixed"],
                cwd=".",
                capture_output=True,
                text=True,
            )
            self.assertEqual(r.returncode, 2)
            self.assertIn("aero", r.stderr)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd python && python3 -m unittest tests.test_f16_runner.TestRunner.test_bad_aero_exits_2 -v`

Expected: FAIL because `--aero` is unrecognized (exit code is not 2, or stderr does not contain `aero`).

- [ ] **Step 3: Write minimal implementation**

`run_sim` gains `aero: str = "morelli"` and passes it into `controlled_derivative`, which passes it to `subf16_derivative`. Default keeps every current caller on Morelli.

`run_f16.py` adds `--aero` with default `morelli`. If the value is not `morelli` or `stevens`, call the existing `_die` with `unknown aero {value}`. Pass the choice into `run_sim`.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd python && python3 -m unittest tests.test_f16_runner tests.test_f16_aero -v`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add python/f16/sim.py python/run_f16.py python/tests/test_f16_runner.py
git commit -m "feat(f16): select Stevens or Morelli aero from the runner"
```

Do not commit unless the user has asked for commits in that session.

---

### Task 4: Retire mixed-Morelli parity locks

**Files:**
- Modify: `python/tests/test_f16_cpp_plant.py`
- Modify: `python/tests/test_f16_compare.py`
- Modify: `python/tests/test_f16_model_llc.py` only if an off-trim derivative still expects AeroBench `morelli` or `f16dynamics` at nonzero `p`, `q`, or `r`

**Interfaces:**
- Consumes: pure `morelli` and `stevens` from Tasks 1–2
- Produces: zero-rate C++ and Python derivative checks unchanged; nonzero-rate checks against the mixed plant removed; GCAS trajectory equality against AeroBench removed; straight-and-level trajectory equality kept

- [ ] **Step 1: Update the tests**

In `test_f16_cpp_plant.py`, delete `test_morelli_derivative_matches_cpp_off_trim`. Keep `test_morelli_derivative_matches_cpp_at_trim` and `test_cpp_trim_derivatives_and_u`: those states have `p = q = r = 0`, so pure Morelli and the mixed C++ plant agree.

In `test_f16_compare.py`, change `test_ours_matches_python_reference` so the loop runs only `"straight_level"`. Delete `test_gcas_long_matches_python_reference`. Keep `test_ours_matches_cpp_plant_reference` (straight-and-level, 3 s) and `test_python_reference_short_horizon`.

If `test_f16_model_llc.py` has a derivative comparison at nonzero rate against `ref_subf16(..., "morelli")` or `f16dynamics`, delete that comparison. Keep the trim-state comparison.

- [ ] **Step 2: Run the affected tests**

Run: `cd python && python3 -m unittest tests.test_f16_aero tests.test_f16_model_si tests.test_f16_model_llc tests.test_f16_cpp_plant tests.test_f16_compare tests.test_f16_sim_sl tests.test_f16_gcas -v`

Expected: PASS. `f16dynamics` tests skip when that module is unbuilt. GCAS floor and mode tests still run on pure Morelli. If a GCAS floor or mode assertion fails, stop. Do not retune floors or gains in this plan.

- [ ] **Step 3: Commit**

```bash
git add python/tests/test_f16_cpp_plant.py python/tests/test_f16_compare.py python/tests/test_f16_model_llc.py
git commit -m "test(f16): stop requiring the mixed Morelli plant"
```

Do not commit unless the user has asked for commits in that session.

---

### Task 5: Docs

**Files:**
- Modify: `README.md`
- Modify: `UPDATES.md`

**Interfaces:**
- Consumes: `--aero` from Task 3
- Produces: UPDATES `0.85.0` on top; one architecture sentence; one run example

- [ ] **Step 1: Update the docs**

Insert at the top of `UPDATES.md`, under `# Updates`:

```markdown
## 0.85.0 - Separate Stevens and Morelli aero
- `subf16_derivative(..., model="stevens"|"morelli")` and `run_f16.py --aero`. Stevens is the Appendix A.8 tables plus the Stevens damping table and one CG shift. Morelli is the polynomial model only; its own rate derivatives stay, and the Stevens damping table is not added. Default `--aero` is `morelli`. GCAS trajectory parity against AeroBench's mixed Morelli plant is retired. Straight-and-level parity stays.
```

On the architecture row for `python/f16/`, append:

```text
Aero is `stevens` or `morelli`, selected by `run_f16.py --aero` (default `morelli`). Stevens damping is not added to the Morelli polynomials.
```

In the Run section, after the `run_f16.py` line, add:

```text
cd python && python3 run_f16.py --aero stevens   # same runner; Appendix A.8 aero instead of Morelli polynomials
```

- [ ] **Step 2: Confirm the suite that guards the split**

Run: `cd python && python3 -m unittest tests.test_f16_aero tests.test_f16_runner.TestRunner.test_bad_aero_exits_2 -v`

Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add README.md UPDATES.md
git commit -m "docs: note the separate Stevens and Morelli aero models"
```

Do not commit unless the user has asked for commits in that session.
