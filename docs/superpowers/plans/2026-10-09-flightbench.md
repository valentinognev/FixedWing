# Flightbench Control-Law Calibration Bench Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task, dispatching each wave's tasks together. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A local calibration bench: pick a plane (`f16`, `x31`, `cessna172`), a law (`linear` everywhere, `lqr` on F-16, `ndi` on X-31) and one of eleven Dickinson tasks, edit gains, run it on the nonlinear plant (and, for `linear`, on the linearized plant too) and read Plotly charts in a React page.

**Architecture:** `python/flightbench/` is the simulator: plane adapters behind one common state/channel convention, generic trim and linearization, measurements, a fixed-step engine, task definitions with classical loops, three laws, a tuner, and a FastAPI app. `web/flightbench/` is a Vite + React + TypeScript + Tailwind + Plotly client of that API. Existing plants are called, never copied; `python/x31/` is never edited.

**Tech Stack:** Python 3.14 (`python3` = `/home/valentin/anaconda/bin/python3`: numpy 2.5, scipy 1.18, fastapi 0.141, uvicorn 0.40, httpx 0.28), unittest. Node 22.18 / npm 10.9, Vite, React 18, TypeScript, Tailwind v4 (`@tailwindcss/vite`), `react-plotly.js` + `plotly.js-dist-min`, Vitest + Testing Library + jsdom.

**Spec:** `docs/superpowers/specs/2026-10-09-flightbench-design.md` — read it before your task. It holds the state/channel convention, the plane table, the loop formulas, the task table with acceptance rows, the LQR/NDI command tables and the API shapes. This plan does not repeat them.

## Global Constraints

- **Do not commit, push, amend or stash.** Leave changes in the working tree. The tree already carries unrelated uncommitted edits (`README.md`, `UPDATES.md`, `python/run_x31.py`, `python/tests/test_x31_runner.py`, `run_x31_view.sh`); do not revert, reformat or include them in your work.
- **Never edit `python/x31/`** (vendored GPLv2, SHA-256 pinned by `tests/test_x31_vendored_port.py`). Reading its private helpers is allowed. Never edit `f16/llc.py`, `f16/model.py`, `plane/dynamics.py`, `x31_plant.py`, or any `data/planes/**` file.
- Subagents and reviewers: model `grok-4.7-high`, passed explicitly on every launch. No other model, no fast variant.
- TDD every task: write the tests, run them and record RED (failing for the missing behavior, not an import typo), implement, record GREEN. Reports include both command outputs.
- Python tests are unittest modules `python/tests/test_flightbench_<topic>.py`; run from `python/` as `python3 -m unittest tests.test_flightbench_<topic> -v`. Imports are `from flightbench... import ...` (tests run with `python/` as cwd, as the rest of the tree).
- Units: SI and radians everywhere in Python and in the API; the UI converts angles to degrees for display only. `G = f16.units.G_MPS2`.
- Common state indices, channel order and series names are defined once in `flightbench/common.py` (Task 1). Never index with bare integers elsewhere.
- Acceptance thresholds are the spec's task table. If a plane/task misses one, report it with the numbers — never loosen a threshold or a test.
- `UPDATES.md` new top entry `0.101.0 - Flightbench control-calibration bench` (current top is 0.100.2). Only Task 25 edits `README.md` / `UPDATES.md`.
- The web app never computes flight dynamics; it renders API JSON.

## Review Focus

1. **Channel sign per plane.** A wrong pitch/roll/yaw sign makes every default gain destabilizing on that plane — Tasks 2–4 assert `B[q, pitch] > 0`, `B[p, roll] > 0`, `B[r, yaw] > 0` on a finite difference of the adapter at trim.
2. **Saturation and trim near a limit.** The X-31 trims at canard −27.5° of a −90…+30° range and the F-16 elevator is ±25°; clipping must apply to `trim + deviation` identically in linear and nonlinear runs — Task 10 test with a controller that demands past the limit.
3. **A plant stop mid-run.** X-31 85° alpha, altitude ≤ 0, non-finite derivative: samples before the stop survive, `stopped_at` is set, the API returns 200 — Tasks 10 and 21 tests.
4. **Concurrent NDI runs with different gains.** Module-attribute override must not leak between threads or survive an exception — Task 18 test runs two overrides from two threads and one that raises.
5. **Plane switch with a stale law/gain set in the UI.** Switching `f16`+`lqr` → `cessna172` must reset the law to `linear` and refetch gains — Task 12 test.

## Parallelism

| Wave | Tasks (dispatch together) | Waits for |
|------|---------------------------|-----------|
| 1 | 1 | — |
| 2 | 2, 3, 4, 5, 6, 7 | 1 (7 needs nothing) |
| 3 | 8, 9, 10, 11, 12 | 2–5 (12 needs 7) |
| 4 | 13, 14 | 8 (14 needs 12) |
| 5 | 15, 16, 17, 18 | 9–11, 13 (17 needs 2, 8; 18 needs 4, 6, 8) |
| 6 | 19, 20 | 15, 16 |
| 7 | 21, 22, 23, 24 | 17–20 |
| 8 | 25 | all |

Parallel tasks share one working tree and touch disjoint files (File map). A per-task reviewer (`grok-4.7-high`) gates each task: it reads only that task's files, its RED/GREEN output, and reruns its test command. After Task 25 one whole-branch reviewer runs the Final verification.

## File map

| File | Task | Responsibility |
|------|------|----------------|
| `python/flightbench/__init__.py`, `common.py`, `adapters/__init__.py`, `adapters/base.py`, `tests/test_flightbench_core.py` | 1 | Indices, names, dataclasses, errors, adapter protocol, plane registry |
| `adapters/f16.py`, `tests/test_flightbench_adapter_f16.py` | 2 | F-16 adapter |
| `adapters/cessna172.py`, `tests/test_flightbench_adapter_cessna.py` | 3 | Cessna adapter |
| `adapters/x31.py`, `tests/test_flightbench_adapter_x31.py` | 4 | X-31 adapter (common ↔ native) |
| `blocks.py`, `tests/test_flightbench_blocks.py` | 5 | PI, Lead, Washout state-space blocks |
| `python/x31_guidance.py` (modify), `tests/test_x31_guidance_commanded.py` | 6 | `run_commanded` extraction |
| `web/flightbench/**` scaffold, `src/api.ts`, `src/types.ts`, `.gitignore` | 7 | Vite/React/Tailwind/Vitest scaffold + typed client |
| `trim.py`, `tests/test_flightbench_trim.py` | 8 | `trim_level` |
| `measure.py`, `tests/test_flightbench_measure.py` | 9 | gamma, nz, beta-dot estimate, series rows |
| `engine.py`, `tests/test_flightbench_engine.py` | 10 | RK4 engine, linear/nonlinear runs, closed-loop matrix |
| `tasks/__init__.py`, `tasks/base.py`, `tasks/defaults.py`, `tests/test_flightbench_tasks_base.py` | 11 | Task metadata, gain specs, profiles, defaults loader, builder dispatch |
| `web/flightbench/src/components/{PlanePicker,LawPicker,AeroPicker,TrimPanel,TaskList,GainForm}.tsx` + tests | 12 | Selection and gain UI |
| `linearize.py`, `tests/test_flightbench_linearize.py` | 13 | Linearization, subsystems, modes |
| `web/flightbench/src/components/{RunPlots,ModesTable,RunBanner}.tsx`, `src/App.tsx` + tests | 14 | Plots, modes, run flow |
| `tasks/longitudinal.py`, `tests/test_flightbench_tasks_longitudinal.py` | 15 | Seven longitudinal tasks |
| `tasks/lateral.py`, `tests/test_flightbench_tasks_lateral.py` | 16 | Four lateral tasks |
| `laws/__init__.py` (stub only), `laws/lqr.py`, `tests/test_flightbench_law_lqr.py` | 17 | F-16 LQR law |
| `laws/ndi.py`, `tests/test_flightbench_law_ndi.py` | 18 | X-31 NDI law |
| `laws/linear.py`, `laws/__init__.py` (dispatch), `tests/test_flightbench_law_linear.py` | 19 | Linear law + law dispatch + gain specs |
| `tune.py`, `tests/test_flightbench_tune.py` | 20 | Tuner CLI |
| `api.py`, `tests/test_flightbench_api.py`, `python/requirements.txt` | 21 | FastAPI app |
| `data/flightbench/f16.json`, `tests/test_flightbench_accept_f16.py` | 22 | F-16 defaults + acceptance |
| `data/flightbench/cessna172.json`, `tests/test_flightbench_accept_cessna.py` | 23 | Cessna defaults + acceptance |
| `data/flightbench/x31.json`, `tests/test_flightbench_accept_x31.py` | 24 | X-31 defaults + acceptance |
| `run_flightbench.sh`, `README.md`, `UPDATES.md`, `tests/test_flightbench_e2e.py` | 25 | Launcher, docs, end-to-end smoke |

All `flightbench/...` paths are under `python/`; all `tests/...` paths are `python/tests/...`.

---

### Task 1: Core types and plane registry

**Files:**
- Create: `python/flightbench/__init__.py` (empty docstring), `python/flightbench/common.py`, `python/flightbench/adapters/__init__.py`, `python/flightbench/adapters/base.py`
- Test: `python/tests/test_flightbench_core.py`

**Interfaces:**
- Produces (`flightbench.common`):
  - `VT, ALPHA, BETA, PHI, THETA, PSI, P, Q, R, NORTH, EAST, ALT = range(12)`; `STATE_NAMES = ("vt","alpha","beta","phi","theta","psi","p","q","r","north","east","altitude")`
  - `CHANNELS = ("throttle","pitch","roll","yaw")`; `THROTTLE, PITCH, ROLL, YAW = range(4)`
  - `SERIES = ("time","vt","alpha","beta","phi","theta","psi","p","q","r","altitude","gamma","nz","throttle","pitch","roll","yaw")`
  - `RK4_DT = 0.01`, `SAMPLE_DT = 0.02`
  - `class FlightbenchError(ValueError)` (→ HTTP 400), `class TrimError(FlightbenchError)` (→ 422), `class PlantStop(Exception)` with `.reason: str`
  - `@dataclass(frozen=True) TrimPoint(x: np.ndarray, u: np.ndarray, vt_mps: float, altitude_m: float)` — `x` is the common state (12 + extras), `u` the four channels
  - `@dataclass(frozen=True) LinearModel(A: np.ndarray, B: np.ndarray, states: tuple[str, ...], inputs: tuple[str, ...])`
  - `@dataclass(frozen=True) Mode(name: str, wn: float, zeta: float, real: float, imag: float)`
  - `@dataclass Trace(series: dict[str, np.ndarray], stopped_at: float | None = None, stop_reason: str | None = None)` — keys exactly `SERIES`
  - `@dataclass RunResult(plane: str, law: str, task: str, aero: str, trim: TrimPoint, runs: dict[str, Trace], reference: tuple[str, np.ndarray, np.ndarray] | None, metrics: dict)` — `reference = (signal, time, values)`
- Produces (`flightbench.adapters.base`): `class PlaneAdapter(Protocol)` with attributes `id: str`, `label: str`, `aero: str`, `aero_models: tuple[str, ...]`, `extras: tuple[str, ...]`, `channel_labels: dict[str, str]`, `limits: np.ndarray` (shape (4, 2), absolute channel lo/hi), `default_trim: tuple[float, float]` (vt_mps, altitude_m); methods `derivative(x: np.ndarray, u: np.ndarray) -> np.ndarray` (common state derivative; raises `PlantStop`), `extras_equilibrium(throttle: float) -> np.ndarray`. Helper `clip_channels(adapter, u) -> np.ndarray`.
- Produces (`flightbench.adapters`): `PLANES: dict[str, tuple[str, str, tuple[str, ...]]]` = `{"f16": ("flightbench.adapters.f16", "F16Adapter", ("linear", "lqr")), "x31": ("flightbench.adapters.x31", "X31Adapter", ("linear", "ndi")), "cessna172": ("flightbench.adapters.cessna172", "Cessna172Adapter", ("linear",))}`; `plane_ids() -> tuple[str, ...]`; `laws_for(plane: str) -> tuple[str, ...]`; `get_adapter(plane: str, aero: str | None = None) -> PlaneAdapter` (lazy `importlib`; class takes `aero` kwarg; `None` → that adapter's default). Unknown plane → `FlightbenchError` whose message lists `plane_ids()`.

- [ ] **Step 1: Write the failing tests**

```python
class TestCore(unittest.TestCase):
    def test_state_and_channel_names_line_up_with_indices(self):
        self.assertEqual(common.STATE_NAMES[common.ALT], "altitude")
        self.assertEqual(common.CHANNELS[common.YAW], "yaw")
        self.assertEqual(common.SERIES[0], "time")
    def test_laws_per_plane(self):
        self.assertEqual(adapters.laws_for("f16"), ("linear", "lqr"))
        self.assertEqual(adapters.laws_for("x31"), ("linear", "ndi"))
        self.assertEqual(adapters.laws_for("cessna172"), ("linear",))
    def test_unknown_plane_names_the_legal_set(self):
        with self.assertRaises(common.FlightbenchError) as ctx:
            adapters.get_adapter("b747")
        for plane in ("f16", "x31", "cessna172"):
            self.assertIn(plane, str(ctx.exception))
    def test_trim_error_is_a_flightbench_error(self):
        self.assertTrue(issubclass(common.TrimError, common.FlightbenchError))
    def test_clip_channels_uses_absolute_limits(self):
        class A: limits = np.array([[0, 1], [-0.4, 0.4], [-0.3, 0.3], [-0.5, 0.5]])
        np.testing.assert_allclose(clip_channels(A(), np.array([2, -1, 0.1, 9])), [1, -0.4, 0.1, 0.5])
```

- [ ] **Step 2: Run to verify RED** — `cd python && python3 -m unittest tests.test_flightbench_core -v` → `ModuleNotFoundError: flightbench`.
- [ ] **Step 3: Implement** the files above as specified.
- [ ] **Step 4: Run to verify GREEN** — same command, all pass.

---

### Task 2: F-16 adapter

**Files:** Create `python/flightbench/adapters/f16.py`; Test `python/tests/test_flightbench_adapter_f16.py`

**Interfaces:**
- Consumes: Task 1 protocol; `f16.model.subf16_derivative(x13, u4, model) -> (xd, Nz, Ny)`, `f16.model._tgear`, `f16.llc.CtrlLimits`, `f16.trim.trim_wings_level`.
- Produces: `class F16Adapter(aero: str = "morelli")` — `id "f16"`, `label "F-16"`, `aero_models ("morelli", "stevens")`, `extras ("power",)`, `channel_labels {"throttle": "throttle", "pitch": "elevator", "roll": "aileron", "yaw": "rudder"}`, `default_trim (153.0096, 457.2)`; `SIGNS: np.ndarray` (4,) native = SIGNS · channel; `to_native_controls(u) -> np.ndarray`, `channels_from_native(u4) -> np.ndarray` (inverse; used by Task 17). `derivative` raises `PlantStop` for non-finite output or `x[ALT] <= 0`.

- [ ] **Step 1: Write the failing tests**

```python
def _trim():
    x, u, _ = trim_wings_level(153.0096, 457.2, "morelli")
    return x, u
class TestF16Adapter(unittest.TestCase):
    def test_derivative_is_the_native_plant(self):
        a = F16Adapter(); x, u4 = _trim()
        u = a.channels_from_native(u4)
        np.testing.assert_array_equal(a.derivative(x, u), subf16_derivative(x, u4, "morelli")[0])
    def test_channel_signs_give_positive_rate_response(self):
        a = F16Adapter(); x, u4 = _trim(); u = a.channels_from_native(u4)
        for ch, rate in ((PITCH, Q), (ROLL, P), (YAW, R)):
            du = np.zeros(4); du[ch] = 1e-4
            self.assertGreater((a.derivative(x, u + du)[rate] - a.derivative(x, u - du)[rate]), 0.0)
    def test_limits_are_ctrl_limits_in_channel_space(self):
        lim = F16Adapter().limits
        np.testing.assert_allclose(sorted(abs(lim[PITCH])), [np.radians(25)] * 2)
        np.testing.assert_allclose(lim[THROTTLE], [0, 1])
    def test_extras_equilibrium_is_tgear(self):
        self.assertAlmostEqual(F16Adapter().extras_equilibrium(0.5)[0], 64.94 * 0.5)
    def test_unknown_aero_raises(self):
        with self.assertRaises(FlightbenchError): F16Adapter(aero="vlm")
    def test_ground_contact_stops(self):
        a = F16Adapter(); x, u4 = _trim(); x = x.copy(); x[ALT] = 0.0
        with self.assertRaises(PlantStop): a.derivative(x, a.channels_from_native(u4))
    def test_registry_builds_it(self):
        self.assertEqual(get_adapter("f16", "stevens").aero, "stevens")
```

- [ ] **Step 2: RED** — `python3 -m unittest tests.test_flightbench_adapter_f16 -v`.
- [ ] **Step 3: Implement.** Choose `SIGNS` so the sign test passes (expected `[1, -1, ±1, ±1]`; decide aileron/rudder signs from the test, not from memory).
- [ ] **Step 4: GREEN.**

---

### Task 3: Cessna 172 adapter

**Files:** Create `python/flightbench/adapters/cessna172.py`; Test `python/tests/test_flightbench_adapter_cessna.py`

**Interfaces:**
- Consumes: `plane.aircraft.load_aircraft("cessna172", model="tornado")`, `state_vector`, `control_vector`, `plane.dynamics.plane_derivative`.
- Produces: `class Cessna172Adapter(aero: str = "tornado")` — `id "cessna172"`, `label "Cessna 172"`, `aero_models ("tornado",)`, `extras ("power",)`, labels as F-16, limits throttle 0–1, elevator ±25°, aileron ±20°, rudder ±24° (channel space), `default_trim (23.114578472541158, 100.0)`, `extras_equilibrium(throttle) = [throttle]`, `SIGNS`, `to_native_controls`, `channels_from_native`, `file_trim() -> tuple[np.ndarray, np.ndarray]` (the file's `initial` state and `controls` converted to channels — the oracle Task 8 uses).

- [ ] **Step 1: Failing tests** — same five behaviours as Task 2 (native parity at `file_trim`, B-sign test, limits, extras equilibrium, registry `get_adapter("cessna172")`) plus:

```python
    def test_file_trim_is_a_trim(self):
        a = Cessna172Adapter(); x, u = a.file_trim()
        xd = a.derivative(x, u)
        for i in (VT, ALPHA, BETA, P, Q, R, ALT): self.assertLess(abs(xd[i]), 1e-9)
```
- [ ] **Step 2: RED** — `python3 -m unittest tests.test_flightbench_adapter_cessna -v`.
- [ ] **Step 3: Implement** (load the aircraft once per adapter instance).
- [ ] **Step 4: GREEN.**

---

### Task 4: X-31 adapter

**Files:** Create `python/flightbench/adapters/x31.py`; Test `python/tests/test_flightbench_adapter_x31.py`

**Interfaces:**
- Consumes: `x31_plant.derivative(t, PlantState, SurfaceCommand)` (raises `x31.dynamics.AngleLimitError`), `x31.quaternion.body_321_to_q`, `q_to_body_321`, `rotate_body_to_earth`, `x31.types.PlantState`, `SurfaceCommand`.
- Produces: `class X31Adapter(aero: str = "most31")` — `id "x31"`, `label "X-31"`, `aero_models ("most31",)`, `extras ()`, `channel_labels {"throttle": "thrust", "pitch": "canard", "roll": "aileron", "yaw": "rudder"}`, `default_trim (50.0, 457.2)`, `THRUST_MAX_KN = 146.0`, limits from the spec (aileron ±30°, canard −90…+30°, rudder ±30° mapped through `SIGNS`; throttle 0–1); `to_native(x) -> PlantState`, `to_common(state: PlantState) -> np.ndarray`, `surface_command(u) -> SurfaceCommand` (degrees; flap/thrust_pitch/thrust_yaw = 0; thrust = 146·throttle). `derivative` = directional central difference of `to_common` along the native derivative, step `h = 1e-6` s, with the psi difference wrapped to (−π, π]; `AngleLimitError` → `PlantStop(str(exc))`.

- [ ] **Step 1: Failing tests**

```python
STATE = np.array([60.0, 0.12, 0.03, 0.2, 0.15, 1.0, 0.05, -0.02, 0.04, 100.0, -50.0, 800.0])
class TestX31Adapter(unittest.TestCase):
    def test_round_trip(self):
        a = X31Adapter(); np.testing.assert_allclose(a.to_common(a.to_native(STATE)), STATE, atol=1e-12)
    def test_euler_and_position_rates_match_kinematics(self):
        a = X31Adapter(); xd = a.derivative(STATE, np.array([0.2, 0.0, 0.0, 0.0]))
        phi, th, p, q, r = STATE[PHI], STATE[THETA], STATE[P], STATE[Q], STATE[R]
        self.assertAlmostEqual(xd[PHI], p + np.tan(th) * (q * np.sin(phi) + r * np.cos(phi)), places=6)
        self.assertAlmostEqual(xd[THETA], q * np.cos(phi) - r * np.sin(phi), places=6)
        vel = a.to_native(STATE).vel
        self.assertAlmostEqual(xd[ALT], -vel[2], places=6)
    def test_vt_rate_matches_native_acceleration(self):
        a = X31Adapter(); u = np.array([0.2, 0.0, 0.0, 0.0]); s = a.to_native(STATE)
        nat = x31_plant.derivative(0.0, s, a.surface_command(u))
        self.assertAlmostEqual(a.derivative(STATE, u)[VT], float(np.dot(s.vel, nat.vel)) / STATE[VT], places=6)
    def test_channel_signs(self): ...  # B-sign test as Task 2 at STATE with throttle 0.2
    def test_angle_limit_is_a_plant_stop(self):
        x = STATE.copy(); x[ALPHA] = np.radians(86)
        with self.assertRaises(PlantStop): X31Adapter().derivative(x, np.zeros(4))
    def test_surface_command_units(self):
        cmd = X31Adapter().surface_command(np.array([0.5, 0.0, np.radians(10), 0.0]))
        self.assertAlmostEqual(cmd.thrust, 73.0); self.assertAlmostEqual(abs(cmd.aileron), 10.0)
        self.assertEqual((cmd.flap, cmd.thrust_pitch, cmd.thrust_yaw), (0.0, 0.0, 0.0))
```
- [ ] **Step 2: RED** — `python3 -m unittest tests.test_flightbench_adapter_x31 -v`.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: GREEN**, then also run `python3 -m unittest tests.test_x31_vendored_port -v` (still passes).

---

### Task 5: Linear blocks

**Files:** Create `python/flightbench/blocks.py`; Test `python/tests/test_flightbench_blocks.py`

**Interfaces:**
- Produces: a block protocol `n_states: int`, `derivative(xb: np.ndarray, e: float) -> np.ndarray`, `output(xb: np.ndarray, e: float) -> float`, `ss() -> tuple[A, B, C, D]`; classes `PI(kp: float, ki: float)` (1 state = ∫e), `Lead(zero: float, pole: float)` (unit DC gain `(pole/zero)(s+zero)/(s+pole)`; `zero == pole` is identity with 0 states; non-positive zero/pole → `FlightbenchError`), `Washout(tau: float)` (`s/(s+1/tau)`; `tau <= 0` → `FlightbenchError`).

- [ ] **Step 1: Failing tests** — for each block, integrate a unit step with RK4 at `dt = 1e-3` for 5 s and compare to the analytic response at t = 0.5, 1, 5 within 1e-6:
  - PI: `kp + ki·t`; Lead(1, 4): `1 + 3·exp(−4t)`; Washout(2): `exp(−t/2)`.
  - `ss()` reproduces `output` and `derivative` at a random state/input (`C·x + D·e`, `A·x + B·e`).
  - `Lead(1, 1).n_states == 0` and output equals input; `Lead(0, 1)` and `Washout(0)` raise `FlightbenchError`.
- [ ] **Step 2: RED** — `python3 -m unittest tests.test_flightbench_blocks -v`.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: GREEN.**

---

### Task 6: X-31 commanded runner

**Files:** Modify `python/x31_guidance.py` (`run_guided` at lines 374–514); Create `python/tests/data/x31_guided_ndi_2s.npz` (recorded before the edit); Test `python/tests/test_x31_guidance_commanded.py`

**Interfaces:**
- Produces: `run_commanded(controller: str, initial: tuple[pos, vel, q, w], duration: float, command_fn: Callable[[float, dict, float], dict], *, step: float = 1/30, hold_s: float = HOLD_S, label_fn: Callable[[], str] | None = None, surface_offset: Callable[[float], dict[str, float]] | None = None, actuator_outputs: dict[str, float] | None = None) -> dict`. `command_fn(t, sensed, dt)` returns `{"V", "Chi", "Gamma"}`; `surface_offset(t)` returns degrees (kN for thrust) added to the controller's surface command per port field name before the actuator derivative and in the logged command; `actuator_outputs` initializes each actuator so its output equals the given value (state = output / transfer gain, read from `x31.actuators`' model table; channels not named keep `actuators.initial_state()`). Output = `run_guided`'s dict plus `"p"`, `"q"`, `"r"` arrays [rad/s].
- `run_guided(controller, name, duration=None, step=1/30)` keeps its signature and returns exactly the same keys and values (drops `p`, `q`, `r`), implemented as a call to `run_commanded` with the mission's guidance driving `command_fn`/`label_fn`.

- [ ] **Step 1: Failing tests**

```python
class TestRunCommanded(unittest.TestCase):
    def test_run_guided_output_unchanged(self):
        # record BEFORE editing x31_guidance.py: python3 -c "..." dumps run_guided('ndi','gcas_upright',duration=2.0,step=0.5) to tests/data/x31_guided_ndi_2s.npz (keys array + every column; save it beside the test)
        ref = np.load(DATA / "x31_guided_ndi_2s.npz", allow_pickle=True)
        out = run_guided("ndi", "gcas_upright", duration=2.0, step=0.5)
        self.assertEqual(set(out), set(ref["keys"]))
        for k in ("t", "vt_mps", "alpha", "theta", "canard_cmd_deg"):
            np.testing.assert_array_equal(out[k], ref[k])
    def test_constant_command_logs_rates(self):
        out = run_commanded("ndi", _initial("gcas_upright"), 1.0, lambda t, s, dt: {"V": 50.0, "Chi": 0.0, "Gamma": 0.0}, step=0.1, hold_s=0.1)
        for k in ("p", "q", "r"): self.assertEqual(out[k].shape, out["t"].shape)
    def test_surface_offset_reaches_the_command(self):
        off = lambda t: {"rudder": 2.0} if t >= 0.5 else {}
        a = run_commanded("ndi", _initial("gcas_upright"), 1.0, CONST, step=0.1, hold_s=0.1)
        b = run_commanded("ndi", _initial("gcas_upright"), 1.0, CONST, step=0.1, hold_s=0.1, surface_offset=off)
        i = np.searchsorted(a["t"], 0.6)
        self.assertAlmostEqual(b["rudder_cmd_deg"][i] - a["rudder_cmd_deg"][i], 2.0, delta=0.5)
```
(The 0.5° delta allows for the feedback the offset itself provokes within 0.1 s; the tightness is in the next test.)
```python
    def test_offset_before_onset_changes_nothing(self):
        # same two runs: every column identical for t < 0.5
    def test_actuator_outputs_initialize_the_surfaces(self):
        out = run_commanded("ndi", _initial("gcas_upright"), 0.1, CONST, step=0.1, actuator_outputs={"canard": -10.0, "thrust": 40.0})
        self.assertAlmostEqual(out["canard_deg"][0], -10.0, places=9); self.assertAlmostEqual(out["thrust_kn"][0], 40.0, places=9)
```
- [ ] **Step 2: Record the reference npz first**, then RED — `python3 -m unittest tests.test_x31_guidance_commanded -v` (`ImportError: run_commanded`).
- [ ] **Step 3: Implement** the extraction.
- [ ] **Step 4: GREEN**, and `python3 -m unittest tests.test_x31_guidance tests.test_x31_most31_wiring tests.test_x31_vendored_port -v` unchanged.

---

### Task 7: Web scaffold and typed client

**Files:** Create `web/flightbench/` (`package.json`, `vite.config.ts`, `tsconfig.json`, `index.html`, `src/main.tsx`, `src/index.css`, `src/types.ts`, `src/api.ts`, `src/test/setup.ts`, `src/__tests__/api.test.ts`); Modify root `.gitignore` (add `node_modules/`).

**Interfaces:**
- Produces (`src/types.ts`): TypeScript types mirroring the spec's API exactly — `PlaneInfo {id, label, laws: Law[], aero_models: string[], default_trim: {vt_mps, altitude_m}, channel_labels: Record<Channel, string>}`, `Law = "linear" | "lqr" | "ndi"`, `Channel`, `TaskInfo {id, label, family: "longitudinal" | "lateral", lesson, duration_s, description}`, `GainSpec {name, label, value, unit}`, `RunRequest`, `Series` (all `number[]`, keys per spec), `Mode`, `Trim`, `RunResponse`.
- Produces (`src/api.ts`): `fetchPlanes(): Promise<PlaneInfo[]>`, `fetchTasks(): Promise<TaskInfo[]>`, `fetchGains(plane, law, task): Promise<GainSpec[]>`, `postRun(req: RunRequest): Promise<RunResponse>`; non-2xx → throws `ApiError {status, detail}` with the response's `detail`.
- Scripts: `dev` (vite, port 5173, proxy `/api` → `http://127.0.0.1:8765`), `build`, `test` (`vitest run`). Vitest environment jsdom, setup imports `@testing-library/jest-dom/vitest`. Plotly is imported only through `src/components/Plot.tsx` (Task 14) so tests can `vi.mock` it.

- [ ] **Step 1: Scaffold** with `npm create vite@latest flightbench -- --template react-ts` inside `web/`, add deps `tailwindcss @tailwindcss/vite react-plotly.js plotly.js-dist-min` and dev deps `vitest jsdom @testing-library/react @testing-library/jest-dom @testing-library/user-event @types/react-plotly.js`. Remove the template demo content.
- [ ] **Step 2: Failing tests** (`src/__tests__/api.test.ts`, `fetch` stubbed with `vi.stubGlobal`):
  - `postRun` posts JSON to `/api/runs` with `Content-Type: application/json` and returns the parsed body.
  - a 400 `{detail: "law 'lqr' is not available on cessna172; available: linear"}` rejects with `ApiError` whose `status === 400` and `detail` is that string.
  - `fetchGains("f16","linear","lead_pitch")` requests `/api/gains?plane=f16&law=linear&task=lead_pitch`.
- [ ] **Step 3: RED** — `cd web/flightbench && npx vitest run`.
- [ ] **Step 4: Implement** `api.ts`, `types.ts`.
- [ ] **Step 5: GREEN** and `npm run build` succeeds.

---

### Task 8: Level trim

**Files:** Create `python/flightbench/trim.py`; Test `python/tests/test_flightbench_trim.py`

**Interfaces:**
- Consumes: adapters (Tasks 2–4).
- Produces: `trim_level(adapter: PlaneAdapter, vt_mps: float, altitude_m: float) -> TrimPoint` per spec (unknowns alpha, pitch, throttle; theta = alpha; extras from `extras_equilibrium`; `least_squares`; `TrimError` if residual norm > 1e-8 or a channel outside `adapter.limits`); `vt_mps <= 0` or `altitude_m < 0` → `FlightbenchError`. Initial guess: alpha 0.05, pitch at the middle of its limits clipped to 0 if 0 is inside, throttle 0.3.

- [ ] **Step 1: Failing tests**

```python
    def test_f16_matches_trim_wings_level(self):
        t = trim_level(F16Adapter(), 153.0096, 457.2)
        x, u, _ = trim_wings_level(153.0096, 457.2, "morelli")
        a = F16Adapter(); un = a.to_native_controls(t.u)
        self.assertAlmostEqual(t.x[ALPHA], x[1], delta=1e-5); self.assertAlmostEqual(un[1], u[1], delta=1e-5); self.assertAlmostEqual(un[0], u[0], delta=1e-5)
    def test_cessna_matches_its_file_trim(self):
        a = Cessna172Adapter(); x0, u0 = a.file_trim()
        t = trim_level(a, x0[VT], x0[ALT]); np.testing.assert_allclose(t.u, u0, atol=1e-5); self.assertAlmostEqual(t.x[ALPHA], x0[ALPHA], delta=1e-5)
    def test_x31_trims_at_50(self):
        t = trim_level(X31Adapter(), 50.0, 457.2)
        self.assertAlmostEqual(np.degrees(t.x[ALPHA]), 17.32, delta=0.05)   # probe 2026-10-09: 17.319°, canard −27.54°, thrust 21.32 kN
    def test_trim_is_steady(self):  # every plane: |xd[VT, ALPHA, BETA, P, Q, R, ALT]| < 1e-7 and extras derivative < 1e-7
    def test_impossible_speed_raises_trim_error(self):
        with self.assertRaises(TrimError): trim_level(Cessna172Adapter(), 5.0, 100.0)
    def test_bad_condition_is_a_flightbench_error(self):
        with self.assertRaises(FlightbenchError): trim_level(F16Adapter(), -1.0, 100.0)
```
- [ ] **Step 2: RED** — `python3 -m unittest tests.test_flightbench_trim -v`.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: GREEN.**

---

### Task 9: Measurements

**Files:** Create `python/flightbench/measure.py`; Test `python/tests/test_flightbench_measure.py`

**Interfaces:**
- Consumes: Task 1 types; a `LinearModel` whose states follow Task 13's `LINEAR_STATES` order (tests build one by hand).
- Produces:
  - `MEASUREMENTS = ("vt","alpha","beta","phi","theta","psi","p","q","r","altitude","gamma","nz","beta_dot_est")` — all deviations from trim.
  - `flight_path_angle(x: np.ndarray) -> float` (spec formula on the absolute common state).
  - `nonlinear_measurement(x: np.ndarray, adapter, trim: TrimPoint, needs_nz: bool) -> dict[str, float]` — `nz` uses `adapter.derivative(x, trim.u)[ALPHA]`; when `needs_nz` is false `nz` is `0.0` and no extra derivative is evaluated.
  - `linear_measurement(dx: np.ndarray, model: LinearModel, trim: TrimPoint) -> dict[str, float]` — `gamma = θ − α`, `nz` from `(A @ dx)[alpha]`.
  - `series_row(y: dict, trim: TrimPoint, u_abs: np.ndarray, t: float) -> dict[str, float]` → one `SERIES` row (absolute states, `gamma` absolute, `nz` increment, channels absolute).

- [ ] **Step 1: Failing tests**
  - level wings at trim: `flight_path_angle(trim.x) == 0` within 1e-12 (Cessna file trim); a state with `theta = alpha + 0.1`, wings level, β = 0 gives 0.1 within 1e-12.
  - at trim both measurement functions return all zeros (Cessna file trim, a hand linear model with `A = 0`).
  - `beta_dot_est` for `p = 0.1, r = 0.05, phi = 0.2` equals `0.1·sin α0 − 0.05·cos α0 + (G/vt0)·cos θ0·sin 0.2` within 1e-12, identically in both functions.
  - linear `nz` with `A[alpha, vt] = 2.0`, `dx[vt] = 0.5`, `dx[q] = 0.1` equals `(vt0/G)·(0.1 − 1.0)`.
  - `needs_nz=False` evaluates the adapter derivative zero times (count with a stub adapter).
- [ ] **Step 2: RED** — `python3 -m unittest tests.test_flightbench_measure -v`.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: GREEN.**

---

### Task 10: Engine

**Files:** Create `python/flightbench/engine.py`; Test `python/tests/test_flightbench_engine.py`

**Interfaces:**
- Consumes: Tasks 1, 9; a `LinearModel` (hand-built in tests).
- Produces:
  - `class Controller(Protocol)`: `n_states: int`, `needs_nz: bool`, `derivative(t, xc, y) -> np.ndarray`, `output(t, xc, y) -> np.ndarray` (4 channel deviations before trim add/clip; includes any feed-forward).
  - `simulate_nonlinear(adapter, trim, controller, duration, disturbance=None) -> Trace`
  - `simulate_linear(model, adapter, trim, controller, duration, disturbance=None) -> Trace` (deviation state in `model.states` order; `adapter` supplies limits)
  - `closed_loop_matrix(model, trim, controller) -> np.ndarray` (finite-difference Jacobian of the unclipped linear closed loop at zero, refs at their t = 0 values)
  - `disturbance(t) -> np.ndarray` (4 channel deviations) added after the controller and before clipping.
- Fixed-step RK4 at `RK4_DT`, rows every `SAMPLE_DT`, first row at t = 0, last at `duration`. Channels: `u = clip(trim.u + controller.output + disturbance)`, held over each RK4 step (evaluated at the step start, so steps/pulses on the 0.01 s grid switch exactly). A `PlantStop` or non-finite state ends the run: earlier rows kept, `stopped_at` = time of the last good row, `stop_reason` = the stop's reason.

- [ ] **Step 1: Failing tests**

```python
class Zero:  n_states = 0; needs_nz = False
    def derivative(self, t, xc, y): return np.zeros(0)
    def output(self, t, xc, y): return np.zeros(4)
class TestEngine(unittest.TestCase):
    def test_trim_stays_put(self):           # Cessna file trim, Zero, 5 s: max|vt − vt0| < 1e-6, |theta − θ0| < 1e-8
    def test_rows_on_the_grid(self):         # len == 251 for 5 s; time[1] == 0.02; keys == SERIES
    def test_linear_matches_analytic_first_order(self):
        # 1-state model A=[[-2]], B=[[0,0,0,1]] with a constant yaw output 0.1 → x(t) = 0.05(1 − e^{−2t}) within 1e-8
    def test_clipping_is_identical_in_both_runs(self):
        # controller demanding pitch +1 rad: both traces' pitch == adapter.limits[PITCH][1] at every row
    def test_plant_stop_keeps_earlier_rows(self):
        # stub adapter raising PlantStop("85 degree angle limit") once t > 1.0: stopped_at == 1.0 (±SAMPLE_DT), rows end there, reason kept
    def test_disturbance_onset_is_exact(self):
        # pulse at 1.0: pitch row at t=0.98 equals trim, row at t=1.0 equals trim + pulse
    def test_closed_loop_matrix_of_a_known_loop(self):
        # A=[[0,1],[0,0]] (states theta, q), B pitch column [0,1], controller pitch = −2θ − 3q → eig = {−1, −2}
```
- [ ] **Step 2: RED** — `python3 -m unittest tests.test_flightbench_engine -v`.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: GREEN.**

---

### Task 11: Task registry, profiles, defaults loader

**Files:** Create `python/flightbench/tasks/__init__.py`, `tasks/base.py`, `tasks/defaults.py`; Test `python/tests/test_flightbench_tasks_base.py`

**Interfaces:**
- Produces (`tasks.base`):
  - `@dataclass(frozen=True) GainSpec(name, label, unit, seed: float, log: bool = False)`; `SEEDS: dict[str, float]` = the spec's seed list verbatim; `LOG_GAINS = {"tau_w", "lead_zero", "lead_pole"}`.
  - `@dataclass(frozen=True) TaskInfo(id, label, family, lesson, duration_s: float, description, gains: tuple[GainSpec, ...], reference_signal: str | None, second_trim_factor: float | None = None)`.
  - `TASKS: dict[str, TaskInfo]` — the eleven ids in spec order with the spec's family, lesson folder name, duration and gain names:
    - `pitch_disturbance`: `kp_theta, ki_theta, kq`; ref `theta`
    - `short_period_phugoid`: `kq`; ref `None`
    - `lead_pitch`: `kp_theta, ki_theta, kq, lead_zero, lead_pole`; ref `theta`
    - `trim_cruise` (`second_trim_factor = 1.2`), `airspeed`: `kp_theta, ki_theta, kq, kp_v, ki_v`; ref `vt`
    - `acceleration`: `kp_nz, ki_nz, kq, kp_v, ki_v`; ref `nz`
    - `steady_descent`: `kp_gamma, ki_gamma, kp_theta, ki_theta, kq, kp_v, ki_v`; ref `gamma`
    - `dutch_roll`: `kr, tau_w`; ref `None`
    - `turn_coordination`: `kp_phi, ki_phi, kp_p, kr, tau_w, k_beta, k_ari`; ref `phi`
    - `sideslip_turn`: `kp_phi, ki_phi, kp_p, kr, tau_w, k_beta, ki_beta, k_betadot`; ref `phi`
    - `yaw_orientation`: `kp_phi, ki_phi, kp_p, k_beta, k_betadot, kp_psi, kr_track`; ref `psi`
  - `task_ids()`, `get_task(id) -> TaskInfo` (unknown → `FlightbenchError` listing ids).
  - Profiles: `step(t, t0, amp)`, `pulse(t, t0, t1, amp)` (amp on `[t0, t1)`), `doublet(t, t0, width, amp)` (+amp on `[t0, t0+w)`, −amp on `[t0+w, t0+2w)`).
  - `@dataclass(frozen=True) TaskContext(adapter, trim: TrimPoint, trim2: TrimPoint | None)`; `@dataclass TaskSetup(controller: Controller, disturbance: Callable[[float], np.ndarray] | None, reference: Callable[[float], float] | None)` — `reference` returns the **absolute** value of `reference_signal` (`nz`: increment).
- Produces (`tasks.defaults`): `DEFAULTS_DIR = <repo>/data/flightbench`; `load_defaults(plane) -> dict` (missing file → `{}`); `default_gains(plane: str, task: str) -> dict[str, float]` (file value per gain, else seed); `write_defaults(plane, aero, trim: tuple[float, float], table: dict[str, dict[str, float]], generated_by: str) -> Path` (spec JSON shape, sorted keys, 2-space indent).
- Produces (`tasks/__init__`): `build_task(task_id, gains: dict[str, float], ctx: TaskContext) -> TaskSetup` and `acceptance(task_id, trace: Trace, ctx: TaskContext, closed_loop_modes: list[Mode]) -> list[str]` dispatching lazily to `tasks.longitudinal` / `tasks.lateral` (`BUILDERS: dict[str, Callable]`, `ACCEPT: dict[str, Callable]` in each module; gains are validated against `TASKS[task_id].gains` **before** the lazy import, so this task's tests run before those modules exist); unknown gain names → `FlightbenchError` naming the task's gains; non-finite gain → `FlightbenchError`; missing gains → `default_gains` is NOT applied here (callers pass a complete dict).

- [ ] **Step 1: Failing tests** — eleven ids in order; families split 7 longitudinal / 4 lateral; every gain name has a seed; every `lesson` is an existing directory under `/home/valentin/Books/FixedPlane/BenDickenson/Ref/0{3,4}_*/` (skip this one test with `unittest.skipUnless` when the books path is absent); profile edge values (`pulse` at `t1` is 0, `doublet` at `t0 + w` is −amp); `default_gains` with a temp `DEFAULTS_DIR` (patch the module attribute) returns file values and falls back to seeds; `write_defaults` round-trips through `load_defaults`; `build_task("lead_pitch", {"bogus": 1.0}, ctx)` raises naming `kp_theta`.
- [ ] **Step 2: RED** — `python3 -m unittest tests.test_flightbench_tasks_base -v`.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: GREEN.**

---

### Task 12: Web selection and gain UI

**Files:** Create `web/flightbench/src/components/{PlanePicker,LawPicker,AeroPicker,TrimPanel,TaskList,GainForm}.tsx`, `src/state.ts`, tests `src/__tests__/{selection,gainform}.test.tsx`

**Interfaces:**
- Consumes: Task 7 types and api.
- Produces: `src/state.ts` — `type Selection = {plane: string, law: Law, aero: string, task: string, trim: {vt_mps: number, altitude_m: number}}` and pure reducer `selectionReducer(state, action)` with actions `planeChanged(plane: PlaneInfo)` (law → `plane.laws[0]` if the current law is not in `plane.laws`, aero → `plane.aero_models[0]`, trim → `plane.default_trim`), `lawChanged`, `aeroChanged`, `taskChanged`, `trimChanged`. Components are controlled (`value`, `onChange`): `LawPicker` lists only `plane.laws`; `AeroPicker` renders nothing for one model; `TaskList` groups by family under headings "Longitudinal" / "Lateral" and shows `lesson` as secondary text; `GainForm({specs, values, onChange, onReset})` renders one `<input type="number">` per spec with `aria-label={spec.label}`, shows the unit, and exposes `isValid(values)` (every value a finite number).

- [ ] **Step 1: Failing tests**
  - reducer: `{plane:"f16", law:"lqr"}` + `planeChanged(cessna)` → `law === "linear"`, trim = cessna default; `f16` + `linear` + `planeChanged(x31)` keeps `linear`.
  - `LawPicker` for x31 renders options `linear`, `ndi` only.
  - `TaskList` with two tasks of each family shows both headings and calls `onChange("dutch_roll")` on click.
  - `GainForm`: typing `abc` into "kq" makes `isValid` false; "Reset to defaults" calls `onReset`.
- [ ] **Step 2: RED** — `npx vitest run`.
- [ ] **Step 3: Implement** (Tailwind classes only; no CSS files beyond `index.css`'s `@import "tailwindcss"`).
- [ ] **Step 4: GREEN.**

---

### Task 13: Linearization and modes

**Files:** Create `python/flightbench/linearize.py`; Test `python/tests/test_flightbench_linearize.py`

**Interfaces:**
- Consumes: adapters, `trim_level` (Task 8).
- Produces: `linear_states(adapter) -> tuple[str, ...]` = `("vt","alpha","beta","phi","theta","psi","p","q","r","altitude", *adapter.extras)`; `LONG_STATES = ("vt","alpha","theta","q","altitude")` (+ extras appended by `subsystem`), `LONG_INPUTS = ("pitch","throttle")`, `LAT_STATES = ("beta","phi","psi","p","r")`, `LAT_INPUTS = ("roll","yaw")`; `to_linear(x_common, adapter) -> np.ndarray`, `from_linear(dx, trim, adapter) -> np.ndarray` (absolute common state; north/east = trim); `linearize(adapter, trim) -> LinearModel` (central difference, relative step `1e-6·max(1, |x|)`); `subsystem(model, states, inputs) -> LinearModel`; `longitudinal(model)`, `lateral(model)`; `modes(model, family: str) -> list[Mode]` per spec classification; `open_loop_modes(model) -> list[Mode]` (longitudinal then lateral).

- [ ] **Step 1: Failing tests**
  - F-16 Morelli at 153.0096 m/s / 457.2 m: `short_period.wn > phugoid.wn`, both present, `0 < short_period.zeta < 1`.
  - Cessna at file trim: `dutch_roll`, `roll`, `spiral` all present; `roll.real < 0`.
  - Lateral/longitudinal decoupling at wings-level trim: `max|A[long, lat]| < 1e-6·max|A|`.
  - Linear vs nonlinear for a small input (every plane): pitch +0.2° step for 1 s through `simulate_linear` vs `simulate_nonlinear` with an open-loop controller: `max|Δq|` agrees to within 2% of the nonlinear peak.
  - `modes` on a hand matrix `diag-block [[-1, 2], [-2, -1]]` reports `wn = √5`, `zeta = 1/√5`.
  - A longitudinal matrix whose short period is two real poles: `short_period` absent, no exception.
- [ ] **Step 2: RED** — `python3 -m unittest tests.test_flightbench_linearize -v`.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: GREEN.**

---

### Task 14: Web plots, modes and run flow

**Files:** Create `web/flightbench/src/components/{Plot,RunPlots,ModesTable,RunBanner}.tsx`, `src/App.tsx` (replace), `src/plotting.ts`, tests `src/__tests__/{plots,app}.test.tsx`

**Interfaces:**
- Consumes: Tasks 7, 12.
- Produces: `src/plotting.ts` — `LONG_SIGNALS = ["theta","q","alpha","vt","gamma","nz","altitude","pitch","throttle"]`, `LAT_SIGNALS = ["phi","p","beta","r","psi","roll","yaw"]`, `ANGLE_SIGNALS` (set of radian signals incl. `pitch`, `roll`, `yaw`), `toDisplay(signal, values) -> number[]` (rad → deg for angles), `tracesFor(signal, response) -> PlotTrace[]` (one per run in `response.runs`, named `linear` / `nonlinear`; plus a dashed `reference` trace when `response.reference.signal === signal`). `Plot.tsx` is the only importer of `react-plotly.js`. `RunPlots({response, family})` renders one `Plot` per signal with a shared x range and, when `stopped_at` is set, a vertical line shape at it. `RunBanner` shows `stop_reason` and any API error `detail`. `ModesTable` shows open-loop and closed-loop modes (`wn` 3 decimals, `zeta` 3 decimals). `App` wires selection → `fetchGains` on plane/law/task change → Run (disabled while running or while `GainForm` is invalid) → `postRun` → plots.

- [ ] **Step 1: Failing tests** (`vi.mock("../components/Plot")` to a stub recording props):
  - linear-law response → each `Plot` receives 2 traces named `linear`, `nonlinear`; lqr response → 1.
  - `theta` values `[0.1]` display as `[5.729...]`; `vt` unchanged.
  - reference on `theta` adds a third trace with `line.dash === "dash"` only on the theta chart.
  - `stopped_at: 3.2` → the stub receives a shape at x = 3.2 and the banner text contains the stop reason.
  - `App`: changing the plane to cessna triggers `fetchGains("cessna172","linear", task)`; a 422 from `postRun` shows its `detail`; the Run button is disabled during an in-flight request.
- [ ] **Step 2: RED** — `npx vitest run`.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: GREEN** and `npm run build`.

---

### Task 15: Longitudinal tasks

**Files:** Create `python/flightbench/tasks/longitudinal.py`; Test `python/tests/test_flightbench_tasks_longitudinal.py`

**Interfaces:**
- Consumes: Tasks 5 (blocks), 9 (measurement keys), 10 (`Controller`, engine), 11 (`TaskContext`, `TaskSetup`, profiles, `SEEDS`), 13 (linearize for tests).
- Produces: `BUILDERS` and `ACCEPT` for `pitch_disturbance`, `short_period_phugoid`, `lead_pitch`, `trim_cruise`, `acceleration`, `airspeed`, `steady_descent`, implementing the spec's loop formulas, command/disturbance column and acceptance column exactly (degrees in the spec → radians here). Each builder returns a controller whose `n_states` is the sum of its blocks' states and whose `needs_nz` is true only for `acceleration`. `trim_cruise` requires `ctx.trim2` (else `FlightbenchError`) and adds `Δu = trim2.u − trim.u` on pitch and throttle from 1 s. `ACCEPT[...]` returns human-readable failure strings with the measured number and the threshold.

- [ ] **Step 1: Failing tests** (Cessna at file trim, seeds — fast and well-conditioned; Tasks 22–24 own the per-plane default acceptance):
  - each builder's reference profile: `lead_pitch` reference at 0.99 s is θ0, at 1.0 s θ0 + 5°; `acceleration` reference is 0.5 on [1, 4) and 0 at 4.0; `steady_descent` reference at T is −3°.
  - `pitch_disturbance` disturbance pitch component is `+1°` from 1.0 s, zero before; other channels zero.
  - `short_period_phugoid` with `kq = 0` → `closed_loop_matrix` eigenvalues equal the open-loop longitudinal eigenvalues (as sets, 1e-6) plus no controller states.
  - Sign sanity on the linear model: `lead_pitch` with seeds, θ(5 s) − θ0 > 0; `airspeed` V(T) − vt0 > 0; `steady_descent` γ(T) < 0.
  - `ACCEPT["lead_pitch"]` on a synthetic trace ending at θ0 + 4.5° returns one failure mentioning `0.25`; ending at θ0 + 5.0° returns `[]`.
  - `trim_cruise` without `trim2` raises `FlightbenchError`.
- [ ] **Step 2: RED** — `python3 -m unittest tests.test_flightbench_tasks_longitudinal -v`.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: GREEN.**

---

### Task 16: Lateral tasks

**Files:** Create `python/flightbench/tasks/lateral.py`; Test `python/tests/test_flightbench_tasks_lateral.py`

**Interfaces:**
- Consumes: as Task 15.
- Produces: `BUILDERS` and `ACCEPT` for `dutch_roll`, `turn_coordination`, `sideslip_turn`, `yaw_orientation` per the spec (yaw damper `−kr·Washout(tau_w)[r]`; bank PI with `−kp_p·p`; sideslip `+k_beta·β (+ki_beta·∫β) (+k_betadot·beta_dot_est)`; ARI `+k_ari·roll`; yaw tracking `r_cmd = kp_psi·e_ψ`, `φ_ref = clip((vt0/G)·r_cmd, ±30°)`, `yaw += kr_track·(r_cmd − r)`).

- [ ] **Step 1: Failing tests** (Cessna file trim, seeds):
  - `dutch_roll` with `kr = 0` → closed-loop eigenvalues equal open-loop lateral eigenvalues plus the washout pole `−1/tau_w`.
  - `dutch_roll` with seeds: closed-loop Dutch-roll ζ greater than open-loop ζ.
  - `turn_coordination` linear run: φ(T) − φ0 > 0.
  - `yaw_orientation`: the controller never commands φ_ref beyond 30° (probe `output` with ψ error 3 rad and read the bank loop's reference through a debug attribute `last_phi_ref`).
  - `sideslip_turn` uses `beta_dot_est` (a measurement dict with only `beta_dot_est` non-zero produces a non-zero yaw output when `k_betadot ≠ 0`, zero when it is 0).
  - `ACCEPT["turn_coordination"]` reports `max|β|` failures with the measured degrees.
- [ ] **Step 2: RED** — `python3 -m unittest tests.test_flightbench_tasks_lateral -v`.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: GREEN.**

---

### Task 17: LQR law (F-16)

**Files:** Create `python/flightbench/laws/__init__.py` (empty package docstring; Task 19 fills it), `python/flightbench/laws/lqr.py`; Test `python/tests/test_flightbench_law_lqr.py`

**Interfaces:**
- Consumes: `F16Adapter` (`channels_from_native`, `to_native_controls`, `SIGNS`), `trim_level`, `f16.sim.run_sim`, `f16.autopilot.F16Autopilot`, `f16.llc.F16Llc`, `f16.units.K_GAMMA_PER_RAD`, `K_VT_PER_MPS`, `G_MPS2`, task profiles (Task 11), `measure.flight_path_angle`, `measure.nonlinear_measurement`, `linearize.open_loop_modes` (Task 13).
- Produces: `lqr_gain_specs() -> tuple[GainSpec, ...]` — `K_long_0..2`, `K_lat_0_0..K_lat_1_4` (SI, defaults = `F16Llc().K_lqr` blocks), `k_theta` 15.0, `k_gamma` `K_GAMMA_PER_RAD`, `k_vt` `K_VT_PER_MPS`, `k_phi` 1.0, `k_psi` 0.5; `run_lqr(task: str, gains: dict[str, float] | None = None, aero: str | None = None, trim: tuple[float, float] | None = None) -> RunResult` per the spec's LQR table. Internals: `TaskAutopilot(F16Autopilot)` (stores `t` on the llc in `get_u_ref`, clips Nz to [−1, 6]) and `DisturbedLlc(F16Llc)` (adds the task disturbance in native units after `super().get_u`, then re-clips to `ctrlLimits`). `run_sim(..., step=SAMPLE_DT, aero=...)`; `rejected_t` or ground contact → `stopped_at`/`stop_reason`. Series built with `series_row` from each 16-state sample (channels = `channels_from_native(u)`); `metrics = {"open_loop_modes": ...}`; `runs = {"nonlinear": trace}`.

- [ ] **Step 1: Failing tests**
  - default gains reproduce `F16Llc().K_lqr` exactly.
  - every task completes without a stop on Morelli (one `subTest` per task id; mark `@unittest.skipUnless(os.environ.get("FLIGHTBENCH_SLOW"), ...)` only if the eleven together exceed 120 s — measure first and record the time in the report).
  - direction: `lead_pitch` θ(T) > θ0; `airspeed` V(T) > vt0; `steady_descent` γ(T) < 0; `turn_coordination` φ(T) > 0.1 rad; `yaw_orientation` ψ(T) > ψ0.
  - an edited `K_long_0` (×0.5) changes the `lead_pitch` θ trace (`max|Δθ| > 1e-4`).
  - `pitch_disturbance` pitch series steps by +1° (channel space) at 1.0 s relative to the 0.98 s row, within the LQR's own response (`> 0.5°`).
  - `stevens` aero runs `pitch_disturbance` to completion.
- [ ] **Step 2: RED** — `python3 -m unittest tests.test_flightbench_law_lqr -v`.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: GREEN**; also `python3 -m unittest tests.test_f16_model_llc -v` unchanged.

---

### Task 18: NDI law (X-31)

**Files:** Create `python/flightbench/laws/ndi.py`; Test `python/tests/test_flightbench_law_ndi.py`

**Interfaces:**
- Consumes: `x31_guidance.run_commanded` (Task 6), `X31Adapter` (`to_native`, `surface_command`, `SIGNS`), `trim_level`, profiles, `measure`, `linearize.open_loop_modes`; `x31.ndi`, `x31.maneuver`.
- Produces: `ndi_gain_specs() -> tuple[GainSpec, ...]` (spec names and defaults: `omega, omega_i, omega_a, omega_ai, mu_dot_I, mu_dot_ff, vel_P, vel_I, chi_P, chi_I, gamma_P, gamma_I, mu_P`); `expand_gains(values) -> tuple[dict, dict]` (the dicts `x31.ndi.gains()` and `x31.maneuver.gains()` return, built by the same formulas — defaults must equal those functions' output exactly); `ndi_gains_override(values)` context manager (module-level `threading.Lock`; replaces `x31.ndi.gains` and `x31.ndi.maneuver_gains` with closures returning copies; restores in `finally`); `run_ndi(task, gains=None, trim=None) -> RunResult` per the spec's NDI table (`hold_s = 0.1`, `step = SAMPLE_DT`, `actuator_outputs` = trim surfaces from `surface_command(trim.u)`, disturbances via `surface_offset` in degrees through `SIGNS`). Series: states from the logged columns (`d_m` → altitude = −d), `p, q, r` from Task 6 columns, channels from the flown surfaces (`thrust_kn / 146`, angles through `SIGNS`).

- [ ] **Step 1: Failing tests**
  - `expand_gains(defaults)` equals `(x31.ndi.gains(), x31.maneuver.gains())`.
  - inside `ndi_gains_override({"omega": 5.0, ...})`, `x31.ndi.gains()["p"]["P_gain"] == 5.0 + 1.0`; after exit, the original; after an exception inside the block, the original.
  - two threads overriding different `omega` values each observe their own value inside the block (the lock serializes them; assert observed values, not timing).
  - after a modified-gain `run_ndi("airspeed", {"vel_P": 0.4}, ...)`, `tests.test_x31_vendored_port` digests still match (call its digest helper or rerun the module via `unittest` in-process).
  - every task completes without a stop (subTests; same slow-flag rule as Task 17).
  - direction: `airspeed` V(T) > V0; `steady_descent` γ(T) < 0; `yaw_orientation` ψ(T) > ψ0.
- [ ] **Step 2: RED** — `python3 -m unittest tests.test_flightbench_law_ndi -v`.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: GREEN**; also `python3 -m unittest tests.test_x31_vendored_port tests.test_x31_guidance -v`.

---

### Task 19: Linear law and law dispatch

**Files:** Create `python/flightbench/laws/linear.py`; Modify `python/flightbench/laws/__init__.py`; Test `python/tests/test_flightbench_law_linear.py`

**Interfaces:**
- Consumes: everything above.
- Produces (`laws.linear`): `run_linear(plane: str, task: str, gains: dict | None = None, aero: str | None = None, trim: tuple[float, float] | None = None) -> RunResult` — adapter → `trim_level` (and `trim2` at `second_trim_factor·vt` when set) → `linearize` → `build_task` with `default_gains(plane, task)` overlaid by `gains` → `simulate_linear` and `simulate_nonlinear` → `metrics = {"open_loop_modes", "closed_loop_modes" (from `closed_loop_matrix` classified with the task family's `modes`), "trims" (trim_cruise only)}`; `reference = (signal, time, values)` sampled on the nonlinear grid; top-level stop = the nonlinear trace's.
- Produces (`laws/__init__`): `run(plane, law, task, gains=None, aero=None, trim=None) -> RunResult` (law not in `laws_for(plane)` → `FlightbenchError` "law 'lqr' is not available on cessna172; available: linear"); `gain_specs(plane, law, task) -> list[dict]` (`[{name, label, value, unit}]`: linear → task gains with `default_gains`; lqr → `lqr_gain_specs()`; ndi → `ndi_gain_specs()`).

- [ ] **Step 1: Failing tests** (Cessna, file-trim speed):
  - `run_linear("cessna172", "lead_pitch")` has `runs` keys `{"linear", "nonlinear"}`, equal-length time grids ending at 10.0, both finite.
  - with seeds the linear and nonlinear θ agree at T within 0.5° (small-step regime).
  - `metrics["closed_loop_modes"]` non-empty; `trim_cruise` returns two trims, the second at `1.2·vt0`.
  - `run("cessna172", "lqr", "lead_pitch")` raises with the message above; `run("f16", "lqr", ...)` dispatches to `run_lqr` (patch it).
  - `gain_specs("x31", "ndi", "airspeed")` names `omega`; `gain_specs("f16", "linear", "dutch_roll")` returns exactly `kr`, `tau_w`.
  - unknown gain → `FlightbenchError`; `float("nan")` gain → `FlightbenchError`.
- [ ] **Step 2: RED** — `python3 -m unittest tests.test_flightbench_law_linear -v`.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: GREEN.**

---

### Task 20: Tuner

**Files:** Create `python/flightbench/tune.py`; Test `python/tests/test_flightbench_tune.py`

**Interfaces:**
- Consumes: Tasks 8, 10, 11, 13, 15, 16.
- Produces: `objective(plane_ctx, task, gains) -> float` = `ITAE + 100·max(0, σ_max + 0.05)` per spec, ITAE on `reference − measured` of the task's `reference_signal` over the linear run (tasks with `reference_signal = None`: ITAE on θ deviation for `short_period_phugoid`, on β for `dutch_roll`); a linear run that leaves the finite domain scores `1e9`. `tune_task(plane, task, aero=None, maxiter=300) -> dict[str, float]` (Nelder–Mead from `default_gains` (seeds if no file), log-space for `LOG_GAINS`). `main(argv) -> int` — `python -m flightbench.tune --plane f16 [--task lead_pitch] [--aero morelli] [--maxiter N]`; merges results into the existing file via `write_defaults(..., generated_by="python -m flightbench.tune --plane <id>")`; prints one line per task: `task  J_seed -> J_tuned  sigma_max`.

- [ ] **Step 1: Failing tests**
  - `objective` with a destabilizing gain (Cessna `dutch_roll`, `kr = -50`) exceeds the seed objective by > 100.
  - `tune_task("cessna172", "dutch_roll", maxiter=40)` returns a dict with exactly `kr`, `tau_w`, objective ≤ seed objective, `tau_w > 0`.
  - `main(["--plane", "cessna172", "--task", "dutch_roll", "--maxiter", "5"])` with `DEFAULTS_DIR` patched to a temp dir writes a file whose `linear.dutch_roll` has both gains and `generated_by` names the plane; returns 0. Unknown plane → returns 2 and prints the legal set.
- [ ] **Step 2: RED** — `python3 -m unittest tests.test_flightbench_tune -v`.
- [ ] **Step 3: Implement** (`python/flightbench/__main__` not needed; `tune.py` has `if __name__ == "__main__": raise SystemExit(main(sys.argv[1:]))`).
- [ ] **Step 4: GREEN.**

---

### Task 21: API

**Files:** Create `python/flightbench/api.py`; Modify `python/requirements.txt` (add `fastapi>=0.110`, `uvicorn>=0.29`, `httpx>=0.27` under a `# flightbench` comment); Test `python/tests/test_flightbench_api.py`

**Interfaces:**
- Consumes: `adapters`, `tasks.base`, `laws.run`, `laws.gain_specs`, Task 13 modes.
- Produces: `app = FastAPI(title="flightbench")` with the spec's four routes; Pydantic request model `RunRequest`; `to_json(result: RunResult) -> dict` (numpy → lists, the spec's `RunResponse` shape exactly); exception handlers: `TrimError` → 422, `FlightbenchError` → 400 (both `{"detail": str(exc)}`); non-finite gains and out-of-range trim (spec bounds) → 400 before running. If `web/flightbench/dist` exists, it is mounted with `StaticFiles(html=True)` at `/` after the API routes. `main()` runs `uvicorn.run(app, host="127.0.0.1", port=8765)`.

- [ ] **Step 1: Failing tests** (`TestClient(app)`; patch `flightbench.api.run` with a stub returning a tiny `RunResult` for shape tests):
  - `GET /api/planes` lists the three ids with their laws and `channel_labels`; x31 pitch label `canard`.
  - `GET /api/tasks` returns eleven items; `GET /api/gains?plane=f16&law=linear&task=dutch_roll` returns `kr`, `tau_w`.
  - `POST /api/runs` with law `lqr` on `cessna172` → 400, detail names `linear`; unknown task → 400; `gains: {"kq": "NaN"}`-equivalent (send `1e400` which parses to inf) → 400; trim `vt_mps: 0` → 400; stub raising `TrimError` → 422.
  - stub result with a stopped nonlinear trace → 200 with `stopped_at` set.
  - shape: response keys exactly `plane, law, task, aero, trim, runs, reference, metrics, stopped_at, stop_reason`; each series has every `SERIES` key with equal lengths.
  - one real (unpatched) `POST /api/runs` `{plane: "cessna172", law: "linear", task: "dutch_roll"}` → 200 with `linear` and `nonlinear` runs.
- [ ] **Step 2: RED** — `python3 -m unittest tests.test_flightbench_api -v`.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: GREEN.**

---

### Tasks 22, 23, 24: Per-plane default gains and acceptance

One task per plane (22 `f16` Morelli, 23 `cessna172`, 24 `x31`); identical structure, disjoint files.

**Files:** Create `data/flightbench/<plane>.json` (generated), `python/tests/test_flightbench_accept_<f16|cessna|x31>.py`

**Interfaces:**
- Consumes: Tasks 19, 20; Task 17 (22 only), Task 18 (24 only).

- [ ] **Step 1: Write the acceptance test** (`subTest` per task id): for the plane's default trim and `default_gains`, `run_linear(plane, task)` → nonlinear trace not stopped and `T` reached; `closed_loop_matrix` spectral abscissa < 0; `acceptance(task, trace, ctx, closed_loop_modes) == []` (the spec's acceptance column). Task 22 additionally asserts every `run_lqr` task completes; Task 24 every `run_ndi` task completes.
- [ ] **Step 2: RED** — run it before any defaults file exists: `python3 -m unittest tests.test_flightbench_accept_<plane> -v` (seeds; record which rows fail).
- [ ] **Step 3: Tune** — `cd python && python3 -m flightbench.tune --plane <plane>` (run in the background; record the printed table). Re-run tasks that still fail with `--task <id> --maxiter 1000`. Seed changes are allowed only in the generated JSON, never in `SEEDS`.
- [ ] **Step 4: GREEN** — the acceptance test passes. **If a row cannot be met**, leave it failing, mark that `subTest` with `self.skipTest("UNMET: <task> <measured> vs <threshold>")` only with a one-line reason the parent agent will surface to the user, and list it in the report. Never edit a threshold.

---

### Task 25: Launcher, docs, end-to-end smoke

**Files:** Create `run_flightbench.sh` (repo root, executable), `python/tests/test_flightbench_e2e.py`; Modify `README.md`, `UPDATES.md`

**Interfaces:**
- Consumes: Tasks 14, 21.
- Produces: `./run_flightbench.sh` — default: build `web/flightbench/dist` if absent (`npm ci && npm run build`), then `cd python && python3 -m uvicorn flightbench.api:app --host 127.0.0.1 --port 8765` and print `flightbench: http://127.0.0.1:8765`; `--dev`: uvicorn with `--reload` plus `npm run dev` in `web/flightbench`, both killed on Ctrl-C (`trap`); `--port N` overrides 8765; unknown flag → exit 2 with usage.

- [ ] **Step 1: Failing e2e test** — skips unless `web/flightbench/dist/index.html` exists: `TestClient(app)` `GET /` returns 200 HTML containing `<div id="root">`; `POST /api/runs` for `f16`/`lqr`/`pitch_disturbance`, `x31`/`ndi`/`dutch_roll`, `cessna172`/`linear`/`turn_coordination` each return 200 with finite series.
- [ ] **Step 2: RED** (before `npm run build`: skipped is not RED — build first, then run to see the mount-missing failure if `api.py` lacks it).
- [ ] **Step 3: Implement** the script; add `README.md` rows (`python/flightbench/`, `web/flightbench/`, `run_flightbench.sh`, a Run line) and the `UPDATES.md` `0.101.0` entry (one line per deliverable, the acceptance outcome per plane incl. any UNMET rows, the test commands).
- [ ] **Step 4: GREEN** — `python3 -m unittest tests.test_flightbench_e2e -v`; `bash -n run_flightbench.sh`; `./run_flightbench.sh --bogus` exits 2.

---

## Final verification (whole-branch reviewer)

```bash
cd python && python3 -m unittest discover -s tests -p 'test_flightbench*.py' -v
cd python && python3 -m unittest tests.test_x31_guidance tests.test_x31_guidance_commanded tests.test_x31_vendored_port tests.test_x31_most31_wiring tests.test_f16_model_llc -v
cd web/flightbench && npx vitest run && npm run build
git status --short   # only plan-listed files changed besides the pre-existing edits; nothing under python/x31/ or data/planes/
```

Then start `./run_flightbench.sh`, open `http://127.0.0.1:8765`, run `cessna172`/`linear`/`lead_pitch` and confirm two traces per chart, and `f16`/`lqr`/`turn_coordination` for one. Report UNMET acceptance rows verbatim.
