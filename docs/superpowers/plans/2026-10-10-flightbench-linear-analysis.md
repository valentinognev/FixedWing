# Flightbench Linear Analysis Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task, dispatching each wave's implementers together and each wave's reviewers together. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A `linear`-law run returns the task's controller diagram, the open-loop state-space card and minimal Laplace transfer function, and one root locus plus one Nyquist plot per gain; the page draws them under the time histories.

**Architecture:** Four pure modules (`siso`, `locus`, `nyquist`, `diagram`) do the algebra on arrays they are given. `cards` and `loops` apply them to a task's linearized plant and the controller `build_task` already flies. `run_linear` attaches the JSON to `RunResult`; the React page only renders it. LQR and NDI leave the field unset.

**Tech Stack:** Python 3 (`python3`), numpy, `scipy.signal` (`ss2tf`, `tf2zpk`), unittest. Existing Vite/React/TypeScript page, Vitest, the current `Plot` mock pattern. No `python-control`. No new package.

**Spec:** `docs/superpowers/specs/2026-10-10-flightbench-linear-analysis-design.md` — read it before your task. Channel map, diagram table, sweep counts, Nyquist formula, and the JSON shape live there. This plan does not repeat them.

## Global Constraints

- **Do not commit, push, amend, or stash.** Leave the task's files in the working tree. The tree already has unrelated uncommitted edits (`README.md`, `UPDATES.md`, `kill.sh`, `start.sh`); do not revert them. Only Task 9 edits `README.md` and `UPDATES.md`, and it only adds its own entry.
- Subagents and reviewers: model `grok-4.7-high`, passed explicitly on every launch. No other model, no fast variant.
- TDD every task: write the test, run it and record RED (failing for the missing behavior, not an import typo), implement, record GREEN. The report includes both outputs.
- Python tests are unittest modules under `python/tests/`. Run from `python/` as `python3 -m unittest tests.<module> -v`.
- Units stay SI and radians in Python and in the JSON. The diagram's clip label is the text `±30 deg`. Gain text uses `format(value, ".6g")` (6 significant figures).
- The page does not form a transfer function, a locus, or a Nyquist curve. It renders `linear_analysis`.
- Do not edit `python/x31/`, plant packages, `data/planes/**`, or acceptance thresholds. Do not add a dependency.
- `closed_loop_matrix` stays the Jacobian of the flying controller. Analysis calls it; it does not reimplement the loops.
- A `linear_analysis` value of `None` is omitted from the JSON. LQR and NDI keep the default `None`.

## Review Focus

1. **Negative ordinary gain.** The gain form accepts `K < 0`. The locus must contain the negative log ray, `0`, and `K`, or the current marker falls off the curve — Task 2 `test_negative_gain_adds_the_negative_ray`.
2. **A lead grid point where zero equals pole.** That point removes the lead state and splits the branch count — Task 6 `test_lead_grid_drops_the_point_where_zero_equals_pole`.
3. **Nyquist sample on a pole.** `|L| > 1e6` or a non-finite sample is dropped; the other frequencies stay — Task 3 `test_huge_and_nonfinite_samples_are_dropped`.
4. **`kp_psi` closed inside the yaw-orientation card.** The card would then be the full loop and the `kp_psi` locus would count that gain twice — Task 5 `test_yaw_card_ignores_kp_psi_and_keeps_the_bank_state`.
5. **LQR JSON grows `linear_analysis`.** The LQR page would render an empty analysis block — Task 7 `test_lqr_response_omits_linear_analysis`.

## Parallelism

Dispatch every implementer in a wave in one batch. As each implementer reports RED and GREEN, dispatch that task's reviewer without waiting for the rest of the wave. Reviewers in a wave run together. Start the next wave only after every reviewer in the current wave has accepted. One whole-branch reviewer after Task 9.

| Wave | Tasks (dispatch together) | Waits for |
|------|---------------------------|-----------|
| 1 | 1, 2, 3, 4 | — |
| 2 | 5, 6 | 1 (Task 5), 2 and 3 (Task 6) |
| 3 | 7, 8 | 4, 5, 6 (Task 7). Task 8 waits for nothing in code; it uses the JSON in this plan |
| 4 | 9 | 7, 8 |

A reviewer reads the spec sections named in its task, the diff of that task's files, and the RED/GREEN output, then reruns that task's GREEN command. Model `grok-4.7-high`. Accept only when the new tests failed for the missing behavior and now pass. The whole-branch reviewer runs the Final verification.

## File map

| File | Task | Responsibility |
|------|------|----------------|
| `python/flightbench/analysis/__init__.py` | 1 | Empty package marker (a docstring). Later tasks add modules beside it |
| `python/flightbench/analysis/siso.py`, `python/tests/test_flightbench_analysis_siso.py` | 1 | Minimal SISO card |
| `python/flightbench/analysis/locus.py`, `python/tests/test_flightbench_analysis_locus.py` | 2 | Grids, branch joining, affine locus |
| `python/flightbench/analysis/nyquist.py`, `python/tests/test_flightbench_analysis_nyquist.py` | 3 | `L(jω)` and the cap |
| `python/flightbench/analysis/diagram.py`, `python/tests/test_flightbench_analysis_diagram.py` | 4 | Eleven controller diagrams |
| `python/flightbench/analysis/cards.py`, `python/tests/test_flightbench_analysis_cards.py` | 5 | Per-task open-loop cards, including yaw orientation |
| `python/flightbench/analysis/loops.py`, `python/tests/test_flightbench_analysis_loops.py` | 6 | Per-gain locus and Nyquist on the flying controller |
| `python/flightbench/common.py`, `python/flightbench/laws/linear.py`, `python/flightbench/api.py`, `python/tests/test_flightbench_api.py` | 7 | `RunResult.linear_analysis` and JSON |
| `web/flightbench/src/types.ts`, `src/components/{ControllerDiagram,OpenLoopCard,ComplexPlot,CoefficientCharts}.tsx`, `src/App.tsx`, `src/__tests__/analysis.test.tsx` | 8 | Render the block |
| `README.md`, `UPDATES.md` | 9 | `0.103.0` entry and one architecture sentence |

---

### Task 1: Minimal SISO card

**Files:**
- Create: `python/flightbench/analysis/__init__.py`, `python/flightbench/analysis/siso.py`
- Test: `python/tests/test_flightbench_analysis_siso.py`

**Interfaces:**
- Produces: `siso_card(A: np.ndarray, b: np.ndarray, c: np.ndarray, d: float, *, input: str, output: str, states: tuple[str, ...]) -> dict`
- Dict keys: `input`, `output`, `states` (list of str), `A` (n×n), `B` (n×1), `C` (1×n), `D` ([[d]]), `numerator`, `denominator` (descending powers, denominator monic), `poles`, `zeros` (list of `{real, imag}`).
- Poles and zeros are ordered by descending real part, then descending imaginary part.
- Build the polynomial with `scipy.signal.ss2tf` then `tf2zpk`. Cancel a pole/zero pair when `|p − z| ≤ 1e-6 * max(1, |p|)`. Rebuild with `numpy.poly`: numerator is `k * poly(remaining zeros)` (`[k]` when no zeros remain), denominator is `poly(remaining poles)`. The returned `A` is the caller's matrix, not the cancelled realization.

- [x] **Step 1: Write the failing test**

```python
class TestSisoCard(unittest.TestCase):
    def test_cancelled_pair_leaves_A_and_drops_out_of_G(self):
        # (s+1)/(s^2+3s+2) cancels to 1/(s+2); A still has both eigenvalues.
        A = np.array([[-3.0, -2.0], [1.0, 0.0]])
        card = siso_card(A, np.array([1.0, 0.0]), np.array([1.0, 1.0]), 0.0,
                         input="pitch", output="theta", states=("x1", "x2"))
        np.testing.assert_allclose(card["numerator"], [1.0])
        np.testing.assert_allclose(card["denominator"], [1.0, 2.0])
        self.assertEqual(card["poles"], [{"real": -2.0, "imag": 0.0}])
        self.assertEqual(card["zeros"], [])
        self.assertTrue(np.any(np.isclose(np.linalg.eigvals(card["A"]), -1.0)))
        self.assertEqual(card["B"], [[1.0], [0.0]])
        self.assertEqual(card["C"], [[1.0, 1.0]])
        self.assertEqual(card["D"], [[0.0]])
        self.assertEqual(card["states"], ["x1", "x2"])
```

- [x] **Step 2: RED** — `cd python && python3 -m unittest tests.test_flightbench_analysis_siso -v` → `ModuleNotFoundError` or `ImportError` for `siso_card`.
- [x] **Step 3: Implement** `siso_card` as specified.
- [x] **Step 4: GREEN** — same command.

### Task 2: Locus grids and branches

**Files:**
- Create: `python/flightbench/analysis/locus.py`
- Test: `python/tests/test_flightbench_analysis_locus.py`

**Interfaces:**
- Produces:
  - `affine_grid(k: float) -> np.ndarray` — unique ascending union of `0`, `k`, and `numpy.logspace(-3, 3, 80) * S` with `S = max(abs(k), 1)`. When `k < 0`, also the negation of that log grid.
  - `parameter_grid(k: float, *, skip: float | None = None) -> np.ndarray` — `k > 0`. Unique ascending union of `k` and 80 log-spaced values from `0.05*k` to `20*k`. Drop a value `v` when `skip` is not `None` and `v == skip`.
  - `join_branches(eigs: np.ndarray) -> list[list[dict]]` — `eigs` shape `(n_values, n_modes)`, complex. At column 0, order modes by increasing real part, then increasing imaginary part. Each later column: for each branch in that order, take the unmatched eigenvalue closest in the complex plane; a tie takes the smaller real part, then the smaller imaginary part.
  - `locus_from_eigs(values: np.ndarray, eigs: np.ndarray) -> dict` — drop a column that contains a non-finite entry, then `{values: list[float], branches: join_branches(...)}`. Each branch has the same length as `values`.
  - `affine_locus(m0: np.ndarray, m1: np.ndarray, k: float) -> dict` — eigenvalues of `m0 + v * (m1 - m0)` on `affine_grid(k)`, passed through `locus_from_eigs`.

- [x] **Step 1: Write the failing tests**

```python
class TestLocus(unittest.TestCase):
    def test_affine_locus_hits_the_current_eigenvalues(self):
        # M(K) = diag(-1-K, -2). At K=3 the eigenvalues are -4 and -2.
        m0 = np.diag([-1.0, -2.0])
        m1 = np.diag([-2.0, -2.0])
        locus = affine_locus(m0, m1, 3.0)
        index = locus["values"].index(3.0)
        got = sorted((b[index]["real"], b[index]["imag"]) for b in locus["branches"])
        self.assertEqual(got, [(-4.0, 0.0), (-2.0, 0.0)])

    def test_negative_gain_adds_the_negative_ray(self):
        grid = affine_grid(-2.0)
        self.assertIn(0.0, grid.tolist())
        self.assertIn(-2.0, grid.tolist())
        self.assertTrue(np.any(grid < 0))
        self.assertTrue(np.any(grid > 0))

    def test_parameter_grid_spans_the_spec_factors(self):
        grid = parameter_grid(2.0)
        self.assertAlmostEqual(grid[0], 0.1)
        self.assertAlmostEqual(grid[-1], 40.0)
        self.assertIn(2.0, grid.tolist())

    def test_skip_drops_an_equal_endpoint(self):
        # 0.05*2 = 0.1 is the first log endpoint. An equal skip removes it.
        grid = parameter_grid(2.0, skip=0.1)
        self.assertNotIn(0.1, grid.tolist())
        self.assertIn(2.0, grid.tolist())

    def test_nonfinite_column_is_dropped(self):
        # Row 1 has a NaN, so that grid value and its column leave together.
        locus = locus_from_eigs(
            np.array([0.0, 1.0]),
            np.array([[-1+0j, -2+0j], [np.nan, -3+0j]]),
        )
        self.assertEqual(locus["values"], [0.0])
        self.assertEqual(len(locus["branches"][0]), 1)
```

- [x] **Step 2: RED** — `cd python && python3 -m unittest tests.test_flightbench_analysis_locus -v`.
- [x] **Step 3: Implement** the five functions. `affine_locus` calls `numpy.linalg.eigvals`.
- [x] **Step 4: GREEN.**

### Task 3: Nyquist samples

**Files:**
- Create: `python/flightbench/analysis/nyquist.py`
- Test: `python/tests/test_flightbench_analysis_nyquist.py`

**Interfaces:**
- Produces: `NYQUIST_OMEGA` (`numpy.logspace(-2, 2, 400)`), `NYQUIST_CAP = 1e6`, and `nyquist(A: np.ndarray, b: np.ndarray, c: np.ndarray, d: float, gain_c: float, omega: np.ndarray | None = None) -> dict`.
- `L(jω) = -gain_c * (c (jωI − A)^{-1} b + d)`. Default `omega` is `NYQUIST_OMEGA`. Drop a sample that is non-finite or whose modulus exceeds `NYQUIST_CAP`. Return `{omega, real, imag}` lists of equal length. `b` and `c` are shape `(n,)`.

- [x] **Step 1: Write the failing tests**

```python
class TestNyquist(unittest.TestCase):
    def test_one_frequency_matches_the_solve(self):
        # L = -2/(s+1). At ω=1, L = -1 + j.
        out = nyquist(np.array([[-1.0]]), np.array([1.0]), np.array([1.0]), 0.0,
                      2.0, omega=np.array([1.0]))
        self.assertAlmostEqual(out["real"][0], -1.0, places=6)
        self.assertAlmostEqual(out["imag"][0], 1.0, places=6)

    def test_huge_and_nonfinite_samples_are_dropped(self):
        # |L| at ω=1e-2 is about 1e11. The ω=1 sample (modulus 1) stays.
        out = nyquist(np.array([[-1.0]]), np.array([1.0]), np.array([1.0]), 0.0,
                      1e9, omega=np.array([1e-2, 1.0, np.inf]))
        self.assertEqual(out["omega"], [1.0])
```

- [x] **Step 2: RED** — `cd python && python3 -m unittest tests.test_flightbench_analysis_nyquist -v`.
- [x] **Step 3: Implement** `nyquist` with one `numpy.linalg.solve` per finite frequency.
- [x] **Step 4: GREEN.**

### Task 4: Controller diagrams

**Files:**
- Create: `python/flightbench/analysis/diagram.py`
- Test: `python/tests/test_flightbench_analysis_diagram.py`

**Interfaces:**
- Produces: `format_gain(value: float) -> str` (`format(value, ".6g")`) and `controller_diagram(task: str, gains: dict[str, float], *, pitch_ff: float | None = None, throttle_ff: float | None = None, vt0: float | None = None) -> list[dict]`.
- Each loop dict: `input`, `output`, `feedback`, `nodes`, `wires`. Node: `id`, `kind` (`input` | `sum` | `block` | `plant`), `label`, `text`. Wire: `from`, `to`, `sign` (`"+"`, `"-"`, or `""`).
- `input`, `output`, and `feedback` use series/channel names (`theta`, `q`, `nz`, `vt`, `gamma`, `r`, `phi`, `psi`, `pitch`, `throttle`, `roll`, `yaw`). Yaw orientation's input is `r_cmd`.
- Every loop includes a node `{id: "plant", kind: "plant", label: "airframe"}`. PI text is `f"{format_gain(kp)} + {format_gain(ki)}/s"`. Lead text is `f"({format_gain(pole)}/{format_gain(zero)})*(s+{format_gain(zero)})/(s+{format_gain(pole)})"`. Washout text is `f"s/(s+1/{format_gain(tau)})"`. A plain gain's text is `format_gain` of that gain. Disturbance blocks: `+1 deg`, `doublet ±2 deg`, `+2 deg pulse`. Feed-forward labels `Δpitch` and `Δthrottle`, text `format_gain` of the argument. Clip label `clip`, text `±30 deg`.
- Topologies are the spec's diagram table. Omit a block the task's gain list does not contain. `feedback` is the primary measured signal; rate and cross-feed paths are wires with sign `"-"`.
- Unknown task id raises `flightbench.common.FlightbenchError`.

- [x] **Step 1: Write the failing tests**

```python
_PITCH = {"kp_theta": 2.0, "ki_theta": 0.5, "kq": 0.5}
_LEAD = {**_PITCH, "lead_zero": 1.0, "lead_pole": 4.0}

class TestDiagram(unittest.TestCase):
    def test_pitch_disturbance_has_pi_kq_and_no_lead(self):
        loops = controller_diagram("pitch_disturbance", _PITCH)
        texts = [n["text"] for loop in loops for n in loop["nodes"]]
        self.assertIn("2 + 0.5/s", texts)
        self.assertIn("0.5", texts)
        self.assertIn("+1 deg", texts)
        self.assertFalse(any("lead" in n["label"] for loop in loops for n in loop["nodes"]))
        self.assertTrue(any(n["id"] == "plant" for loop in loops for n in loop["nodes"]))
        self.assertTrue(any(w["sign"] == "-" for loop in loops for w in loop["wires"]))

    def test_each_task_names_its_own_gains(self):
        # Gains below are the registry seeds. Every seed appears as text; a gain
        # the task does not close does not.
        cases = {
            "short_period_phugoid": {"kq": 0.5},
            "lead_pitch": _LEAD,
            "trim_cruise": {**_PITCH, "kp_v": 0.05, "ki_v": 0.005},
            "airspeed": {**_PITCH, "kp_v": 0.05, "ki_v": 0.005},
            "acceleration": {"kp_nz": 0.05, "ki_nz": 0.2, "kq": 0.5, "kp_v": 0.05, "ki_v": 0.005},
            "steady_descent": {"kp_gamma": 1.0, "ki_gamma": 0.2, **_PITCH, "kp_v": 0.05, "ki_v": 0.005},
            "dutch_roll": {"kr": 0.5, "tau_w": 1.0},
            "turn_coordination": {"kp_phi": 2.0, "ki_phi": 0.1, "kp_p": 0.5, "kr": 0.5, "tau_w": 1.0, "k_beta": 1.0, "k_ari": 0.25},
            "sideslip_turn": {"kp_phi": 2.0, "ki_phi": 0.1, "kp_p": 0.5, "kr": 0.5, "tau_w": 1.0, "k_beta": 1.0, "ki_beta": 0.2, "k_betadot": 0.5},
            "yaw_orientation": {"kp_phi": 2.0, "ki_phi": 0.1, "kp_p": 0.5, "k_beta": 1.0, "k_betadot": 0.5, "kp_psi": 0.5, "kr_track": 1.0},
        }
        for task, gains in cases.items():
            texts = " ".join(n["text"] for loop in controller_diagram(task, gains, pitch_ff=0.01, throttle_ff=-0.02, vt0=50.0) for n in loop["nodes"])
            for value in gains.values():
                self.assertIn(format_gain(value), texts, task)
        cruise = controller_diagram("trim_cruise", cases["trim_cruise"], pitch_ff=0.01, throttle_ff=-0.02)
        cruise_text = " ".join(n["text"] for loop in cruise for n in loop["nodes"])
        self.assertIn(format_gain(0.01), cruise_text)
        self.assertIn(format_gain(-0.02), cruise_text)
        yaw = controller_diagram("yaw_orientation", cases["yaw_orientation"], vt0=50.0)
        self.assertIn("±30 deg", " ".join(n["text"] for loop in yaw for n in loop["nodes"]))
        dutch = controller_diagram("dutch_roll", cases["dutch_roll"])
        self.assertIn("s/(s+1/1)", " ".join(n["text"] for loop in dutch for n in loop["nodes"]))
        short = controller_diagram("short_period_phugoid", cases["short_period_phugoid"])
        self.assertNotIn("0.5/s", " ".join(n["text"] for loop in short for n in loop["nodes"]))
```

- [x] **Step 2: RED** — `cd python && python3 -m unittest tests.test_flightbench_analysis_diagram -v`.
- [x] **Step 3: Implement** `format_gain` and `controller_diagram` for the eleven task ids. Read the spec diagram table and `tasks/longitudinal.py` / `tasks/lateral.py` for which loops exist. Do not import the plant.
- [x] **Step 4: GREEN.**

### Task 5: Open-loop cards for the eleven tasks

**Files:**
- Create: `python/flightbench/analysis/cards.py`
- Test: `python/tests/test_flightbench_analysis_cards.py`

**Interfaces:**
- Consumes: `siso_card` (Task 1); `linearize.longitudinal`, `linearize.lateral`; `measure.linear_measurement` definitions (`gamma = theta − alpha`, `nz = (vt0/g)·(q − (A dx)[alpha])`, both `D = 0`); `tasks.build_task`.
- Produces: `open_loop_cards(task: str, model: LinearModel, trim: TrimPoint, gains: dict[str, float]) -> list[dict]`. One dict per spec channel row, each a `siso_card` result. `trim_cruise` returns pitch→theta then throttle→vt, and the two `A` arrays are equal. Other tasks return one card.
- State outputs use the unit row of the family subsystem. `gamma` and `nz` use the measurement rows and `d = 0`.
- Yaw orientation: state names are the lateral states plus `phi_integral`. `kp_psi` is not applied. `phi_ref = (trim.vt_mps / G) * r_cmd` with `G = f16.units.G_MPS2` (the clip is not binding at zero deviation). Bank, `k_beta`, `k_betadot`, and `kr_track` stay at `gains`. Build `A`, `b`, `c` by central difference with step `1e-6`, same pattern as `engine.closed_loop_matrix`, on the lateral state plus the bank integrator. Output is `psi`.

- [x] **Step 1: Write the failing tests**

```python
def _long_model():
    states = ("vt", "alpha", "theta", "q", "altitude")
    A = -0.1 * np.eye(5)
    B = np.zeros((5, 2))
    B[states.index("q"), 0] = 1.0       # pitch
    B[states.index("vt"), 1] = 1.0      # throttle
    return LinearModel(A=A, B=B, states=states, inputs=("pitch", "throttle"))

class TestCards(unittest.TestCase):
    def test_pitch_card_is_theta_over_pitch(self):
        card = open_loop_cards("lead_pitch", _long_model(), _trim(), _LEAD)[0]
        self.assertEqual(card["input"], "pitch")
        self.assertEqual(card["output"], "theta")
        self.assertEqual(card["C"][0][card["states"].index("theta")], 1.0)

    def test_gamma_row_is_theta_minus_alpha(self):
        card = open_loop_cards("steady_descent", _long_model(), _trim(), _DESCENT)[0]
        states = card["states"]
        self.assertEqual(card["C"][0][states.index("theta")], 1.0)
        self.assertEqual(card["C"][0][states.index("alpha")], -1.0)
        self.assertEqual(card["D"], [[0.0]])

    def test_trim_cruise_shares_A(self):
        cards = open_loop_cards("trim_cruise", _long_model(), _trim(), _CRUISE)
        self.assertEqual([c["input"] for c in cards], ["pitch", "throttle"])
        self.assertEqual([c["output"] for c in cards], ["theta", "vt"])
        self.assertEqual(cards[0]["A"], cards[1]["A"])

    def test_yaw_card_ignores_kp_psi_and_keeps_the_bank_state(self):
        low = open_loop_cards("yaw_orientation", _lat_model(), _trim(), {**_YAW, "kp_psi": 0.5})[0]
        high = open_loop_cards("yaw_orientation", _lat_model(), _trim(), {**_YAW, "kp_psi": 5.0})[0]
        self.assertEqual(low["states"][-1], "phi_integral")
        self.assertEqual(low["A"], high["A"])
        moved = open_loop_cards("yaw_orientation", _lat_model(), _trim(), {**_YAW, "kp_phi": 9.0})[0]
        self.assertNotEqual(low["A"], moved["A"])
        self.assertEqual(len(low["states"]), 6)

    def test_remaining_channels(self):
        expect = {
            "pitch_disturbance": ("pitch", "theta"),
            "short_period_phugoid": ("pitch", "q"),
            "acceleration": ("pitch", "nz"),
            "airspeed": ("throttle", "vt"),
            "dutch_roll": ("yaw", "r"),
            "turn_coordination": ("roll", "phi"),
            "sideslip_turn": ("roll", "phi"),
        }
        for task, (src, dst) in expect.items():
            model = _long_model() if dst in ("theta", "q", "nz", "vt") else _lat_model()
            card = open_loop_cards(task, model, _trim(), _GAINS[task])[0]
            self.assertEqual((card["input"], card["output"]), (src, dst), task)
            self.assertEqual(card["D"], [[0.0]], task)
```

`_GAINS` maps each task id above to a complete seed dict (Task 4's values; `k_ari` is `0.25`).

`_trim()` is a `TrimPoint` with `vt_mps=100`, `alpha=theta=0`, `u` four zeros, `x` 13 zeros. `_lat_model()` is the lateral `LinearModel`: states `(beta, phi, psi, p, r)`, `A = -0.1 I`, `B` shape `(5, 2)` with `B[p, roll] = 1` and `B[r, yaw] = 1`, inputs `(roll, yaw)`. Gain dicts are the seeds from Task 4. `_LEAD`, `_DESCENT`, `_CRUISE`, `_YAW` are those seed dicts.

- [x] **Step 2: RED** — `cd python && python3 -m unittest tests.test_flightbench_analysis_cards -v`.
- [x] **Step 3: Implement** `open_loop_cards`. Always slice with `linearize.longitudinal` or `linearize.lateral` before `siso_card`, including when `model` is already that subsystem. The yaw card is the central-difference realization described above.
- [x] **Step 4: GREEN.**

### Task 6: Per-gain locus and Nyquist

**Files:**
- Create: `python/flightbench/analysis/loops.py`
- Test: `python/tests/test_flightbench_analysis_loops.py`

**Interfaces:**
- Consumes: `affine_locus`, `parameter_grid`, `locus_from_eigs` (Task 2); `nyquist` (Task 3); `engine.closed_loop_matrix`; `tasks.build_task`.
- Produces: `coefficient_charts(task: str, model: LinearModel, trim: TrimPoint, gains: dict[str, float], ctx: TaskContext) -> list[dict]`. One entry per `get_task(task).gains`, in that order: `{name, locus: {values, branches}, nyquist: {omega, real, imag}}`.
- Ordinary gain (`lead_zero`, `lead_pole`, `tau_w` excluded): `m0 = closed_loop_matrix` with that gain set to `0`, `m1` with it set to `1`, others unchanged. Locus is `affine_locus(m0, m1, gains[name])`.
- `lead_zero`, `lead_pole`, `tau_w`: grid is `parameter_grid`. `lead_zero` passes `skip=gains["lead_pole"]`; `lead_pole` passes `skip=gains["lead_zero"]`; `tau_w` passes no skip. At each remaining grid value, rebuild the controller and take `numpy.linalg.eigvals(closed_loop_matrix(...))`. Then `locus_from_eigs`.
- Nyquist injection: zero that ordinary gain and add an external scalar `v` in its place. `gain_c` and the signal `σ` are the spec's table (`c(K)` is `gain_c`). Linearize `A`, `b`, `c_row`, `d` at the zero state with step `1e-6`. Call `nyquist`. `v` replaces the term; the gain is not also applied.
- `lead_zero` and `lead_pole` copy the `kp_theta` Nyquist arrays. `tau_w` copies the `kr` arrays. Compute the source gain first.
- Channel directions: `kp_*` / `ki_*` add `v` on that loop's channel (pitch, throttle, roll, or, for `kp_psi`, into `r_cmd`). `kq` adds `v` on pitch with `gain_c = -K` and `σ = q`. `kp_p` adds `v` on roll, `σ = p`, `gain_c = -K`. `kr` adds `v` on yaw, `σ = washout output`, `gain_c = -K`. `k_beta`, `k_betadot`, `k_ari`, `kr_track` add `v` on yaw with `gain_c = +K` and the spec's `σ`.

- [x] **Step 1: Write the failing tests**

```python
class TestLoops(unittest.TestCase):
    def test_kq_nyquist_sign_on_a_diagonal_plant(self):
        # q_dot = -0.1 q + v once kq is removed. c=-K, K=2 → L(0) = 20.
        charts = coefficient_charts("short_period_phugoid", _pitch_model(), _trim(),
                                    {"kq": 2.0}, _ctx())
        ny = charts[0]["nyquist"]
        index = int(np.argmin(np.abs(np.array(ny["omega"]) - 0.01)))
        # L(jω) = 2 / (jω + 0.1). Check ω closest to 0.01 against that value.
        omega = ny["omega"][index]
        expect = 2.0 / (1j * omega + 0.1)
        self.assertAlmostEqual(ny["real"][index], expect.real, places=4)
        self.assertAlmostEqual(ny["imag"][index], expect.imag, places=4)

    def test_locus_at_current_kq_matches_the_direct_matrix(self):
        charts = coefficient_charts("short_period_phugoid", _pitch_model(), _trim(),
                                    {"kq": 2.0}, _ctx())
        direct = np.linalg.eigvals(closed_loop_matrix(_pitch_model(), _trim(), _controller()))
        index = charts[0]["locus"]["values"].index(2.0)
        got = np.array([complex(b[index]["real"], b[index]["imag"]) for b in charts[0]["locus"]["branches"]])
        for lam in direct:
            dist = np.min(np.abs(got - lam))
            self.assertLessEqual(dist, 1e-6 * max(1.0, abs(lam)))

    def test_lead_zero_branches_stay_put_and_lead_pole_tracks(self):
        charts = {c["name"]: c for c in coefficient_charts(
            "lead_pitch", _pitch_model(), _trim(), _LEAD, _ctx())}
        for branch in charts["lead_zero"]["locus"]["branches"]:
            reals = [p["real"] for p in branch]
            self.assertAlmostEqual(max(reals), min(reals), places=6)
        for value, point in zip(charts["lead_pole"]["locus"]["values"],
                                _column(charts["lead_pole"]["locus"]["branches"])):
            self.assertTrue(any(abs(p["real"] + value) < 1e-6 * max(1.0, abs(value))
                                and abs(p["imag"]) < 1e-6 for p in point))
        self.assertEqual(charts["lead_zero"]["nyquist"], charts["kp_theta"]["nyquist"])
        self.assertEqual(charts["lead_pole"]["nyquist"], charts["kp_theta"]["nyquist"])

    def test_tau_w_nyquist_matches_kr(self):
        charts = {c["name"]: c for c in coefficient_charts(
            "dutch_roll", _lat_model(), _trim(), {"kr": 0.5, "tau_w": 1.0}, _ctx())}
        self.assertEqual(charts["tau_w"]["nyquist"], charts["kr"]["nyquist"])

    def test_lead_grid_drops_the_point_where_zero_equals_pole(self):
        # 0.05*2 = 0.1 is the first parameter_grid endpoint. Skipping it removes it.
        gains = {**_LEAD, "lead_zero": 2.0, "lead_pole": 0.1}
        charts = {c["name"]: c for c in coefficient_charts(
            "lead_pitch", _pitch_model(), _trim(), gains, _ctx())}
        self.assertNotIn(0.1, charts["lead_zero"]["locus"]["values"])
        self.assertIn(2.0, charts["lead_zero"]["locus"]["values"])

    def test_kp_and_ki_nyquist_differ(self):
        charts = {c["name"]: c for c in coefficient_charts(
            "pitch_disturbance", _pitch_model(), _trim(), _PITCH, _ctx())}
        self.assertNotEqual(charts["kp_theta"]["nyquist"], charts["ki_theta"]["nyquist"])
```

`_pitch_model` is the longitudinal model from Task 5. `_controller()` is `build_task("short_period_phugoid", {"kq": 2.0}, ctx).controller`. `_column` zips branches at one grid index into a list of `{real, imag}`. `_ctx()` is a `TaskContext` whose adapter is an object with `extras=()` and whose trim is `_trim()`. `build_task` requires every gain of the task; the dicts above are complete.

- [x] **Step 2: RED** — `cd python && python3 -m unittest tests.test_flightbench_analysis_loops -v`.
- [x] **Step 3: Implement** `coefficient_charts`. For the broken loop, central-difference the controller with that gain at 0 and `v` added on the channel; `σ` is the spec's signal read from `linear_measurement` or from the controller state (integrator, washout output).
- [x] **Step 4: GREEN.**

### Task 7: Attach the analysis to the linear run

**Files:**
- Modify: `python/flightbench/common.py` (`RunResult`), `python/flightbench/laws/linear.py` (`run_linear`), `python/flightbench/api.py` (`to_json`), `python/tests/test_flightbench_api.py`
- Test: `python/tests/test_flightbench_api.py`

**Interfaces:**
- Consumes: `open_loop_cards` (Task 5), `coefficient_charts` (Task 6), `controller_diagram` (Task 4). `run_linear` already has `model`, `point`, `second`, `values`, and `setup.controller`.
- Produces: `RunResult.linear_analysis: dict | None = None`. `run_linear` sets it to `{open_loop, diagram, coefficients}` with `open_loop` from `open_loop_cards`, `coefficients` from `coefficient_charts`, and `diagram` from `controller_diagram`. Pass `pitch_ff` / `throttle_ff` only for `trim_cruise`, as `second.u - point.u` on the pitch and throttle channels. Pass `vt0=point.vt_mps`. `to_json` includes the key only when the value is not `None`. `run_lqr` and `run_ndi` stay unchanged.

- [x] **Step 1: Write the failing tests** in `test_flightbench_api.py`

```python
def test_linear_response_passes_linear_analysis_through(self):
    result = _result()
    result.linear_analysis = {"open_loop": [], "diagram": [], "coefficients": []}
    with mock.patch("flightbench.api.run", return_value=result):
        body = self.client.post("/api/runs", json=_RUN).json()
    self.assertEqual(set(body["linear_analysis"]), {"open_loop", "diagram", "coefficients"})

def test_lqr_response_omits_linear_analysis(self):
    result = _result()
    result.law = "lqr"
    result.linear_analysis = None
    with mock.patch("flightbench.api.run", return_value=result):
        body = self.client.post(
            "/api/runs", json={**_RUN, "plane": "f16", "law": "lqr"}
        ).json()
    self.assertNotIn("linear_analysis", body)

def test_real_cessna_dutch_roll_includes_the_analysis(self):
    # Extend the existing real dutch-roll test:
    self.assertEqual([c["name"] for c in body["linear_analysis"]["coefficients"]], ["kr", "tau_w"])
    card = body["linear_analysis"]["open_loop"][0]
    self.assertEqual(card["input"], "yaw")
    self.assertEqual(card["output"], "r")
    self.assertTrue(body["linear_analysis"]["diagram"])
```

Keep the existing shape test's key set as it is: a `RunResult` with `linear_analysis is None` still omits the key.

- [x] **Step 2: RED** — `cd python && python3 -m unittest tests.test_flightbench_api.TestFlightbenchApi.test_linear_response_passes_linear_analysis_through tests.test_flightbench_api.TestFlightbenchApi.test_lqr_response_omits_linear_analysis tests.test_flightbench_api.TestFlightbenchApi.test_real_cessna_dutch_roll_returns_both_runs -v`. The two new tests fail. The real dutch-roll test fails on the new assertions.
- [x] **Step 3: Implement** the field, the `run_linear` assignment, and the `to_json` omit. Do not compute analysis for a `None` field inside `to_json`.
- [x] **Step 4: GREEN** — `cd python && python3 -m unittest tests.test_flightbench_api -v`.

### Task 8: Draw the analysis

**Files:**
- Create: `web/flightbench/src/components/ControllerDiagram.tsx`, `web/flightbench/src/components/OpenLoopCard.tsx`, `web/flightbench/src/components/ComplexPlot.tsx`, `web/flightbench/src/components/CoefficientCharts.tsx`, `web/flightbench/src/__tests__/analysis.test.tsx`
- Modify: `web/flightbench/src/types.ts`, `web/flightbench/src/App.tsx`

**Interfaces:**
- Consumes: the `linear_analysis` JSON from Task 7. `App` already holds the gain `values` sent with the run.
- Produces: `LinearAnalysis` and the nested types on `RunResponse.linear_analysis?: LinearAnalysis`. `ControllerDiagram` draws one SVG, deduping node id `plant`. `OpenLoopCard` prints state names as matrix headings and the Laplace ratio with each coefficient passed through `format(value, ".6g")` equivalent (`toPrecision(6)`). `ComplexPlot` is a scatter whose axes are titled `real` and `imag` (not time). `CoefficientCharts` draws one locus and one Nyquist per coefficient. The locus marks the sample at the gain in `values`. The Nyquist marks `(-1, 0)`. An empty `omega` or an empty `values` renders the coefficient name and no `ComplexPlot`.
- `App` order under a shown linear response: `ModesTable`, `RunPlots`, `ControllerDiagram`, the open-loop cards, `CoefficientCharts`. A response without `linear_analysis` renders none of the three.

- [x] **Step 1: Write the failing test** `analysis.test.tsx`. Mock `ComplexPlot` the way `plots.test.tsx` mocks `Plot`, recording props. Fixture: pitch-disturbance diagram with a PI node text `2 + 0.5/s`, a `kq` node text `0.5`, two loops that both contain `id: "plant"`, one open-loop card whose `states` are `["theta", "q"]` and whose numerator/denominator are `[1]` and `[1, 2]`, and two coefficients whose locus `values` include the gain passed to the charts. Assert the PI text, the `kq` text, a single plant label in the document, the Laplace text containing `1` and `s`, matrix headings `theta` and `q`, and two complex plots. The locus plot's marks include the sample at that gain. The Nyquist plot's marks include `(-1, 0)`. A second render with `nyquist.omega = []` asserts that coefficient's plot is absent. A response with no `linear_analysis` asserts the PI text is absent.

- [x] **Step 2: RED** — `cd web/flightbench && npx vitest run src/__tests__/analysis.test.tsx`.
- [x] **Step 3: Implement** the components and the `App` placement. Reuse the page's dark panel classes. Do not import `plotly.js` outside `ComplexPlot` (same rule as `Plot`).
- [x] **Step 4: GREEN** — `cd web/flightbench && npx vitest run`.

### Task 9: Docs

**Files:**
- Modify: `README.md` (the `python/flightbench/` row only), `UPDATES.md` (new top entry)

**Interfaces:**
- Consumes: the finished behavior of Tasks 7 and 8.
- Produces: `UPDATES.md` section `## 0.103.0 - Flightbench linear analysis` stating that a linear-law run returns the controller diagram, the open-loop matrices and minimal Laplace form, and a root locus and Nyquist plot per gain, and that LQR and NDI omit it. One sentence added to the `python/flightbench/` row in `README.md` with the same fact. Do not edit any other row. Do not revert the uncommitted edits already in those files.

- [x] **Step 1: Write the failing check** — the docs are the deliverable. Before editing, confirm the current top `UPDATES.md` heading is `0.102.0`. If it is not, stop and report the heading; do not pick a different number.
- [x] **Step 2: Add** the `0.103.0` entry at the top and the one README sentence.
- [x] **Step 3: Check** — `head -n 8 UPDATES.md` shows `0.103.0` first, and `rg -n "linear analysis" README.md` hits the flightbench row.

## Final verification

Whole-branch reviewer, after Task 9 is accepted:

```bash
cd python && python3 -m unittest tests.test_flightbench_analysis_siso tests.test_flightbench_analysis_locus tests.test_flightbench_analysis_nyquist tests.test_flightbench_analysis_diagram tests.test_flightbench_analysis_cards tests.test_flightbench_analysis_loops tests.test_flightbench_api -v
cd web/flightbench && npx vitest run
```

Accept when those pass, `python/x31/` is untouched, and `RunResult.linear_analysis` defaults to `None`.
