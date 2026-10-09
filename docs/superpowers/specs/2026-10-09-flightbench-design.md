# Flightbench — Control-Law Calibration Bench Design

Date: 2026-10-09
Status: design approved in chat; this file is the written spec for review before the implementation plan executes.

## Goal

A local tool for calibrating an aircraft control law. The user picks a plane, a control law and a calibration task from Ben Dickinson's longitudinal and lateral flight-control lessons, edits the law's gains, runs the task on the plane's nonlinear plant, and reads the time histories. When the law is the classical `linear` law, the same loop also flies the plane's linearized model about trim and both traces are drawn on one chart, so model error and control error are visible together.

## Decisions (locked)

- **Planes:** `f16`, `x31`, `cessna172`. More are added later, each as one adapter module and one registry entry. Tasks, laws and UI never import a plane package.
- **Laws per plane:** `linear` on every plane; `lqr` on `f16` only; `ndi` on `x31` only. A law the plane does not have is not offered.
- **Calibration:** the user edits the selected law's gains. With `linear`, every chart overlays `linear` (linearized plant) and `nonlinear` traces. With `lqr` / `ndi`, the chart shows the nonlinear plant only.
- **`linear` is one classical loop per task family** (Dickinson topology), not one state-feedback gain.
- **Tasks:** every flight task in `/home/valentin/Books/FixedPlane/BenDickenson/Ref/03_Longitudinal_Flight_Control` and `04_Lateral_Flight_Control` except landing guidance and flare (later spec). Duplicate Patreon posts are one task; simulator-release posts ("all scripts", "how to add a controller", version 1.8/1.9 drops) are not tasks.
- **Stack:** Python backend (`python/flightbench/`, FastAPI + uvicorn on `127.0.0.1`), React + TypeScript + Vite + Tailwind + Plotly frontend (`web/flightbench/`). The browser never computes a derivative.
- **Host-only:** no PX4, Docker, FlightGear, MAVLink, ZMQ.
- **No edit to `python/x31/`** (vendored, SHA-256 pinned). No change to the F-16 paper `K_lqr` default or to `run_f16.py` / `run_x31.py` behaviour.

## Plants

Every plane hides behind a `PlaneAdapter`. The bench works in one **common state**:

```text
index: 0 vt [m/s], 1 alpha, 2 beta, 3 phi, 4 theta, 5 psi [rad], 6 p, 7 q, 8 r [rad/s],
       9 north, 10 east [m], 11 altitude [m, up], 12.. plane extras (engine state)
```

and four **channels** `u = (throttle, pitch, roll, yaw)`: throttle is a fraction in [0, 1]; pitch, roll, yaw are surface deflections in radians, sign-normalized by the adapter so that **positive pitch gives positive q̇, positive roll gives positive ṗ, positive yaw gives positive ṙ** at trim.

| Plane | Native derivative | Aero model(s) | Extras | Channel → surface | Limits (absolute) | Default trim |
|---|---|---|---|---|---|---|
| `f16` | `f16.model.subf16_derivative(x13, u4, model)` | `morelli` (default), `stevens` | `power` (eq. `f16.model._tgear(throttle)`) | pitch → −elevator, roll → aileron, yaw → rudder (signs fixed by the B-sign test) | `f16.llc.CtrlLimits`: throttle 0–1, elevator ±25°, aileron ±21.5°, rudder ±30° | 153.0096 m/s, 457.2 m |
| `cessna172` | `plane.dynamics.plane_derivative(x13, u4, load_aircraft("cessna172", "tornado"))` | `tornado` | `power` (eq. = throttle) | as F-16 | throttle 0–1, elevator ±25°, aileron ±20°, rudder ±24° | the file's `initial`: 23.114578 m/s, 100 m |
| `x31` | `x31_plant.derivative(t, PlantState, SurfaceCommand)` | `most31` | none | pitch → canard, roll → aileron, yaw → rudder, throttle → `thrust = 146·throttle`; flap, thrust_pitch, thrust_yaw held 0 (signs fixed by the B-sign test) | `x31.actuators` Saturate limits: aileron ±30°, canard −90…+30°, rudder ±30°, thrust 0–146 | 50 m/s, 457.2 m |

- F-16 and Cessna native states already are the common layout (13 states, `x[12]` power).
- X-31 converts common ↔ native `(pos NED, vel earth, q, w)`: `q = body_321_to_q(phi, theta, psi)`, body velocity `vt·(cosα cosβ, sinβ, sinα cosβ)` rotated to earth, `pos = (north, east, −altitude)`. Its common derivative is the directional central difference of `to_common` along the native derivative (step 1e-6 s). X-31 surfaces are degrees in the port; the adapter converts.
- **No actuator dynamics in the `linear` law on any plane**: channels drive the surfaces directly (the F-16 and Cessna plants have none; the X-31 port actuators are bypassed). The `ndi` law flies the port's actuators, as `run_x31.py` does.
- Plant stops (`AngleLimitError`, non-finite state or derivative, altitude ≤ 0) end the run; samples up to the stop are kept and `stopped_at` / `stop_reason` are set.

## Trim and linear model

- **Trim** (`trim_level`): wings-level, `gamma = 0`, `beta = phi = p = q = r = 0`, `theta = alpha`; unknowns `(alpha, pitch, throttle)`, extras at their equilibrium for that throttle; residuals `(vt_dot, alpha_dot, q_dot)` solved by `scipy.optimize.least_squares`. Fails (`TrimError`) if `‖residual‖ > 1e-8` or a channel is outside its limit. Oracle: F-16 `f16.trim.trim_wings_level(153.0096, 457.2, "morelli")` (alpha, elevator, throttle within 1e-5) and the Cessna file's `initial`/`controls` (within 1e-5).
- **Linearization**: central difference of the common derivative at trim, relative step `1e-6·max(1, |x|)`, over states `(vt, alpha, beta, phi, theta, psi, p, q, r, altitude, extras…)` (north/east dropped) and the four channels. Subsystems: longitudinal `(vt, alpha, theta, q, altitude, extras)` × `(pitch, throttle)`; lateral `(beta, phi, psi, p, r)` × `(roll, yaw)`. The `linear` run integrates the **full** linear model, so any trim coupling is kept.
- **Modes**: longitudinal complex pairs — the higher `wn` is `short_period`, the lower `phugoid`; lateral — the complex pair is `dutch_roll`, the most negative real eigenvalue `roll`, the real eigenvalue nearest zero `spiral`. A mode reports `wn` [rad/s], `zeta`, `eigenvalue`. A missing mode (e.g. a real-split short period) is reported as absent, never invented.

## Measurements

Shared by both runs so the loop sees the same signals:

- `gamma`: nonlinear `asin((ub sinθ − vb sinφ cosθ − wb cosφ cosθ)/vt)`; linear `theta − alpha` (deviation).
- `nz` (load-factor deviation, g): `(vt0/g)·(q − alpha_dot_free)`, where `alpha_dot_free` is the plant's alpha derivative at the current state **with the channels held at trim** (nonlinear: adapter derivative at `u_trim`; linear: `(A·x)[alpha]`). This avoids an algebraic loop and is the same definition in both runs.
- `beta_dot_est` (Dickinson 1.3 estimator): `p·sin(alpha0) − r·cos(alpha0) + (g/vt0)·cos(theta0)·sin(phi)`.
- `g = f16.units.G_MPS2`.

## Runs

`linear`-law runs integrate plant + controller states with fixed-step RK4, `dt = 0.01 s`, sampled every `0.02 s` (steps and pulses land on the grid). Fixed steps make stops exact and runs deterministic. `lqr` and `ndi` keep their existing integrators (`RK45` / port `ode45`) resampled to the same 0.02 s grid.

## The `linear` law: loops

Blocks are linear SISO state-space: `PI(kp, ki)`, `Lead(zero, pole)` with unit DC gain `(pole/zero)·(s+zero)/(s+pole)`, `Washout(tau)` = `s/(s + 1/tau)`. All errors are `ref − measured` on deviations from trim. The same controller object drives the linear and the nonlinear run. Channel commands are trim + deviation, clipped to the plane's limits in both runs.

Loop building blocks (gain names are the API names):

- **pitch:** `pitch = kp_theta·e_θ + ki_theta·∫e_θ − kq·q`, `e_θ = Lead(lead_zero, lead_pole)[θ_ref] − θ` (Dickinson 1.x: `u = −KI∫e − KP e − Kq q`, lead in series on the command).
- **speed:** `throttle = kp_v·e_V + ki_v·∫e_V`.
- **path:** `θ_ref = kp_gamma·e_γ + ki_gamma·∫e_γ` into the pitch loop.
- **nz:** `pitch = kp_nz·e_nz + ki_nz·∫e_nz − kq·q`.
- **yaw damper:** `yaw = −kr·Washout(tau_w)[r]`.
- **bank:** `roll = kp_phi·e_φ + ki_phi·∫e_φ − kp_p·p`.
- **sideslip:** `yaw += k_beta·β + ki_beta·∫β + k_betadot·beta_dot_est` (positive yaw reduces positive β).
- **ARI:** `yaw += k_ari·roll`.

## Tasks

Each task: id, label, family, Dickinson lesson folder, duration, reference profile(s), disturbance profile, the loops it closes (its gain names), metrics, acceptance. Step/pulse times are seconds; angles are degrees here and radians in code.

| id | family | lesson folder | T [s] | Command / disturbance | Loops (gains) | Acceptance (linear law, defaults, nonlinear run) |
|---|---|---|---|---|---|---|
| `pitch_disturbance` | longitudinal | `2023-06-05_lead-compensation…` | 10 | pitch-channel disturbance +1° step at 1 s; θ_ref 0 | pitch | `|θ(T) − θ0| ≤ 0.1°` |
| `short_period_phugoid` | longitudinal | `2023-06-05_lead-compensation…` Part A (open-loop short-period / phugoid) | 60 | pitch-channel doublet +2° on [1, 2), −2° on [2, 3) | pitch-rate damper `pitch = −kq·q` (gain `kq`; `kq = 0` is the bare airframe) | closed-loop short-period ζ ≥ 0.5 (linear model) |
| `lead_pitch` | longitudinal | `2023-06-19_lead-compensated…` | 10 | θ_ref +5° step at 1 s | pitch | `|θ(T) − θ0 − 5°| ≤ 0.25°` |
| `trim_cruise` | longitudinal | `2023-04-23_trim…`, `2023-04-24_trim-to-cruise…` | 40 | second trim at `1.2·vt0`, same altitude; at 1 s: θ_ref = θ2 − θ1, V_ref = V2 − V1, feed-forward Δu = u2 − u1 on pitch and throttle | pitch + speed | `|V(T) − V2| ≤ 5%·(V2 − V1)`, `|θ(T) − θ2| ≤ 0.25°` |
| `acceleration` | longitudinal | `2023-02-13_acceleration-control…` | 10 | nz_ref +0.5 g pulse on [1, 4) | nz + speed (holds vt0) | mean nz on [2.5, 4) within ±0.1 g of 0.5 |
| `airspeed` | longitudinal | `2023-07-10_airspeed-control…` | 40 | V_ref +10%·vt0 step at 1 s; θ_ref 0 | speed + pitch | `|V(T) − V_ref| ≤ 5%` of the step |
| `steady_descent` | longitudinal | `2023-09-09_full-lesson…` (Section 1.6.4 airspeed + glide-slope control) | 30 | γ_ref −3° step at 1 s | path → pitch, speed (holds vt0) | `|γ(T) + 3°| ≤ 0.3°`, `|V(T) − vt0| ≤ 2%·vt0` |
| `dutch_roll` | lateral | `2024-12-18_dutch-roll-control…` | 15 | yaw-channel pulse +2° on [1, 1.5) | yaw damper (`kr`, `tau_w`) | closed-loop Dutch-roll ζ ≥ 0.3 (linear model); `|β(T)| ≤ 0.2°` |
| `turn_coordination` | lateral | `2025-03-01_…automatic-turn-coordination…` | 20 | φ_ref +20° step at 1 s | bank + yaw damper + sideslip (`k_beta` only) + ARI | `|φ(T) − 20°| ≤ 1°`, `max|β| ≤ 2°` |
| `sideslip_turn` | lateral | `2025-05-09_…sideslip-sideslip-rate…` | 20 | φ_ref +20° step at 1 s | bank + yaw damper + sideslip (`k_beta`, `ki_beta`, `k_betadot`) | `|φ(T) − 20°| ≤ 1°`, `max|β| ≤ 1°` |
| `yaw_orientation` | lateral | `2025-12-21_…yaw-orientation-control…` | 30 | ψ_ref +30° step at 1 s; `r_cmd = kp_psi·e_ψ`; `φ_ref = clip((vt0/g)·r_cmd, ±30°)`; `yaw += kr_track·(r_cmd − r)` | bank + sideslip (`k_beta`, `k_betadot`) + yaw tracking (`kp_psi`, `kr_track`) | `|ψ(T) − 30°| ≤ 1.5°`, `max|β| ≤ 2°` |

Every task, every plane, linear law with defaults: the nonlinear run reaches `T` without a stop and the linear closed-loop matrix has spectral abscissa < 0. If a plane/task cannot meet its row, that is reported, never loosened.

Metrics per run: open-loop modes (longitudinal + lateral) at trim; for `linear`, closed-loop modes of the task's loop on the linear model; `trim_cruise` also returns both trim points.

### Default gains

Seeds (channel-normalized, SI): `kp_theta 2.0, ki_theta 0.5, kq 0.5, lead_zero 1.0, lead_pole 1.0, kp_v 0.05, ki_v 0.005, kp_gamma 1.0, ki_gamma 0.2, kp_nz 0.05, ki_nz 0.2, kr 0.5, tau_w 1.0, kp_phi 2.0, ki_phi 0.1, kp_p 0.5, k_beta 1.0, ki_beta 0.2, k_betadot 0.5, k_ari 0.0, kp_psi 0.5, kr_track 1.0`.

`python -m flightbench.tune --plane <id> [--task <id>]` tunes each task's gains on that plane's **linear** model by Nelder–Mead from the seeds, minimizing `ITAE(tracked error) + 100·max(0, σ_max + 0.05)` (σ_max = closed-loop spectral abscissa); `tau_w`, `lead_zero`, `lead_pole` are optimized in log space. It writes `data/flightbench/<plane>.json`:

```json
{"plane": "f16", "aero": "morelli", "trim": {"vt_mps": 153.0096, "altitude_m": 457.2},
 "generated_by": "python -m flightbench.tune --plane f16", "linear": {"<task>": {"<gain>": 0.0}}}
```

Missing file or task → seeds. `lqr` and `ndi` defaults come from code, not this file.

## The `lqr` law (F-16)

Flies `F16Llc` with an editable `K_lqr` through `f16.sim.controlled_derivative` semantics (16 states: 13 plant + 3 LQR integrators). Task commands map to the LQR reference `(Nz, ps, Ny_r, throttle_delta)`; `throttle_delta` is relative to `F16Llc.uequil[0]`. Channel disturbances are added to the LQR's `u` after `get_u`.

Editable gains: `K_long` (1×3), `K_lat` (2×5) in SI (defaults: `f16/llc.py` paper values × `DEG_TO_RAD`), and outer gains `k_theta` (θ → Nz, default 15.0), `k_gamma` (γ → Nz, default `K_GAMMA_PER_RAD` = 15.0), `k_vt` (V → throttle, default `K_VT_PER_MPS`), `k_phi` (φ → ps, default 1.0), `k_psi` (ψ → φ_ref via `vt0/g`, default 0.5).

| task | `Nz` | `ps` | throttle | disturbance |
|---|---|---|---|---|
| `pitch_disturbance` | `k_theta·(θ0 − θ)` | 0 | trim | the +1° pitch-channel step at 1 s, mapped to elevator by the F-16 adapter's channel sign |
| `short_period_phugoid` | 0 | 0 | trim | pitch doublet as linear |
| `lead_pitch` | `k_theta·(θ_ref − θ)` | 0 | trim + `k_vt·(vt0 − V)` | — |
| `trim_cruise` | `k_theta·(θ2 − θ)` | 0 | trim2 + `k_vt·(V2 − V)` | — |
| `acceleration` | nz pulse | 0 | trim + `k_vt·(vt0 − V)` | — |
| `airspeed` | `k_theta·(θ0 − θ)` | 0 | trim + `k_vt·(V_ref − V)` | — |
| `steady_descent` | `k_gamma·(γ_ref − γ)` | 0 | trim + `k_vt·(vt0 − V)` | — |
| `dutch_roll` | `k_gamma·(0 − γ)` | 0 | trim | rudder pulse as linear |
| `turn_coordination`, `sideslip_turn` | `k_gamma·(0 − γ) + (1/cos φ − 1)` | `k_phi·(φ_ref − φ)` | trim + `k_vt·(vt0 − V)` | — |
| `yaw_orientation` | as turn | `k_phi·(φ_ref − φ)`, `φ_ref = clip((vt0/g)·k_psi·(ψ_ref − ψ), ±30°)` | as turn | — |

`Nz` is clipped to `[NzMin, NzMax]` = [−1, 6]. Acceptance: every task completes without a stop and the commanded signal moves in the commanded direction.

## The `ndi` law (X-31)

Flies the port's NDI with its actuators through a commanded runner extracted from `x31_guidance.run_guided` (`run_commanded`), which `run_guided` then delegates to with its output keys and values unchanged. `run_commanded` additionally logs body rates `p`, `q`, `r` [rad/s]; `run_guided` drops them. Commands are `{V, Chi, Gamma}` (m/s, deg, deg), refreshed every `hold_s = 0.1` s. A surface offset (degrees, per port surface) carries channel disturbances. The run starts from the bench trim with each actuator initialized to its trim output and the NDI integrators at zero (port behaviour).

| task | command |
|---|---|
| `pitch_disturbance` | hold `V0, Chi0, 0`; canard offset +1° at 1 s |
| `short_period_phugoid` | hold; canard doublet as linear |
| `lead_pitch` | `Gamma` +5° step at 1 s (NDI has no θ command) |
| `trim_cruise` | `V` → `1.2·V0` at 1 s |
| `acceleration` | `Gamma(t) = ∫ g·nz_ref/V0 dt` over the pulse |
| `airspeed` | `V` +10% step at 1 s |
| `steady_descent` | `Gamma` −3° step at 1 s |
| `dutch_roll` | hold; rudder offset pulse as linear |
| `turn_coordination`, `sideslip_turn` | `Chi(t) = Chi0 + (g·tan 20°/V0)·(t − 1)` for t ≥ 1 |
| `yaw_orientation` | `Chi` +30° step at 1 s |

Editable gains: the four bandwidths `omega` 10.0, `omega_i` 1.0, `omega_a` 2.0, `omega_ai` 0.4 (expanded into `x31.ndi.gains()`' dict by the same formulas), `mu_dot_I` 0.3, `mu_dot_ff` −1.0, and outer `vel_P` 0.2, `vel_I` 0.012, `chi_P` 0.5, `chi_I` 0.075, `gamma_P` 0.5, `gamma_I` 0.075, `mu_P` 1.5 (`x31.maneuver.gains()`). A run replaces `x31.ndi.gains` and `x31.ndi.maneuver_gains` (module attributes) for its duration under one process-wide lock and restores them in `finally`; the vendored files are not edited. Acceptance: every task completes without a stop; vendored SHA-256 test still passes after a modified-gain run.

## API

FastAPI app `flightbench.api:app`, `127.0.0.1:8765`. Synchronous handlers (thread pool).

```text
GET  /api/planes                         → [{id, label, laws, aero_models, default_trim{vt_mps, altitude_m}, channel_labels{throttle, pitch, roll, yaw}}]
GET  /api/tasks                          → [{id, label, family, lesson, duration_s, description}]
GET  /api/gains?plane=&law=&task=        → [{name, label, value, unit}]   (defaults for that plane/law/task)
POST /api/runs                           → RunResponse
```

Request: `{plane, law, task, aero?, trim?: {vt_mps, altitude_m}, gains?: {name: value}}` — omitted gains take defaults.

```text
RunResponse = {
  plane, law, task, aero,
  trim: Trim,
  runs: {nonlinear: Series, linear?: Series},          # linear only for law == "linear"
  reference: {signal, time, values} | null,            # the commanded trace, absolute
  metrics: {open_loop_modes: [Mode], closed_loop_modes?: [Mode], trims?: [Trim]},
  stopped_at: float | null, stop_reason: str | null    # nonlinear run's stop
}
Trim   = {vt_mps, altitude_m, alpha_rad, theta_rad, controls: {throttle, pitch, roll, yaw}}
Series = {time, vt, alpha, beta, phi, theta, psi, p, q, r, altitude, gamma, nz, throttle, pitch, roll, yaw}
         (SI, radians, absolute values except nz, which is the load-factor increment in g; sampled every 0.02 s)
Mode   = {name, wn, zeta, real, imag}
```

Errors: unknown plane/law/task, a law the plane lacks, an unknown gain name, a non-finite gain, or a trim outside `0 < vt_mps ≤ 400`, `0 ≤ altitude_m ≤ 15000` → **400** with `detail` naming the legal set or value. `TrimError` → **422**. A plant stop is not an error: 200 with `stopped_at`.

`./run_flightbench.sh` builds `web/flightbench/dist` if missing and serves it from the same FastAPI app at `/`; `--dev` runs uvicorn plus `npm run dev` (Vite proxies `/api` to 8765).

## UI

One page (`web/flightbench/`), Tailwind, dark-neutral:

1. **Plane** select → **Law** select (only that plane's laws; resets when the plane changes) → **Aero** select (only if the plane lists more than one) → **Trim** fields (vt, altitude; default from the plane).
2. **Task list** in two groups, Longitudinal and Lateral; each item shows label and lesson.
3. **Gain form**: one numeric field per gain from `/api/gains`; "Reset to defaults"; a non-number blocks Run.
4. **Run** button (disabled while running); errors show the API `detail`.
5. **Plots** (Plotly, one chart per signal, shared time axis): longitudinal tasks `theta, q, alpha, vt, gamma, nz, altitude, pitch, throttle`; lateral `phi, p, beta, r, psi, roll, yaw`. Angles shown in degrees. `linear` law → two traces `linear` / `nonlinear` per chart; otherwise one. The reference trace is dashed on its chart. `stopped_at` draws a vertical marker and a banner.
6. **Modes table**: open-loop and (linear law) closed-loop modes, `wn`, `zeta`.

## Tests (TDD, each task test-first)

- Python: `cd python && python3 -m unittest discover -s tests -p 'test_flightbench*.py'` (unittest style, as the tree). API with `fastapi.testclient.TestClient`.
- Web: `cd web/flightbench && npx vitest run` with Testing Library + jsdom; Plotly is mocked.
- Required coverage: adapter B-sign test and native-derivative parity per plane; trim oracles; linearization vs finite nonlinear response for a small input; mode classification; block step responses; engine stop handling; each task's acceptance row on each plane; LQR and NDI completion on every task; vendored X-31 hashes after an NDI gain override; `run_guided` output unchanged after the `run_commanded` extraction; API error codes; UI law list, gain form, trace count.

## Out of scope

Landing guidance and flare; Cessna aero other than `tornado`; editing the linear model's derivatives; PX4 SID changes; persistence of user gain sets (export/import is a later spec).
