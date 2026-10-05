# Cessna 172 Aerodynamic Data From USAF_DATCOM

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract the aerodynamic data from the USAF_DATCOM Cessna 172 analysis into `data/planes/cessna172/`, as four aero models in the schema the existing linear Morelli host airplane already uses, so `run_plane.py --plane cessna172 --model tornado` flies it.

**Architecture:** A new `python/aero_convert/` package owns the conversion — units, moment-reference shift, sign normalisation, and the Morelli slot mapper — deliberately *not* reusing AID's internal unit heuristics. A solver module drives the four solvers that live in the sibling repo `../USAF_DATCOM/AircraftIntuitiveDesign/Python/src/aid/` and returns one normalised derivative set per model. `python/scripts/build_cessna172_aero.py` writes the five `.jsonc` files. `f16/aero_data.py`, `f16/aero_morelli.py`, `plane/aircraft.py`, `plane/dynamics.py` and `run_plane.py` gain a `--model` parameter so a file named `tornado.jsonc` can be opened; every default keeps its current behaviour.

**Tech Stack:** Python 3, numpy, scipy (only via the sibling `aid` package). unittest (`cd python && python3 -m unittest <module> -v`). Host-only. No Docker, MAVLink, ZMQ, PX4, FlightGear, or network.

**Spec:** This plan is the spec. There is no separate design document.

**Source data:** `/home/valentin/Projects/FlightSimulation/USAF_DATCOM/AircraftIntuitiveDesign/Analyses/Cessna172.jsonc` — a DATCOM geometry-phase dump in feet (`"unit": "ft"`). All of `Analyses/` is an external directory: read it, never write it. All solver scratch goes to a `TemporaryDirectory`.

## Approved design

### The five files

```
data/planes/cessna172/
  geometry.jsonc    SI reference geometry + every solver's raw reported derivatives, with provenance
  tornado.jsonc     Tornado derivative set   <- the flight model
  datcom.jsonc      DATCOM handbook derivative set
  avl.jsonc         AVL 3.52 derivative set
  flow5.jsonc       flow5 derivative set (static + control only)
```

Each of the four model files is self-contained and schema-identical to `data/planes/linear/morelli.json`: top-level `model`, `aircraft`, `initial`, `controls`, `coefficients`, plus a `provenance` block. `geometry.jsonc` carries the SI reference geometry and the unconverted per-degree solver output.

### Head-of-file comments

Every one of the five files opens with real `//` line comments, before the opening brace. These are the remarks a future agent needs in order to revisit and re-derive any number. Each file's comment block states, at minimum:

1. Which solver produced the coefficients, at which mesh and flight condition.
2. That the source planform is a **Cessna 172 at roughly 1/3 linear scale**, with the scale ratios (span 12 ft vs 36.1 ft, chord 2 ft vs 4.89 ft, area 24 ft² vs 174 ft²) and the resulting wing loading (6.70 lbf/ft² vs a real 13.2).
3. That the source file's own mass, `AERO.WT = 5` slugs = 72.574 kg, is used, and that this is *not* a geometrically similar 1/3-scale C172 (that would be ~53.5 kg), so no scaling of published full-scale figures is self-consistent.
4. Every unit conversion applied, and that AID's `_per_deg` / `_per_rad` helpers were **not** trusted — see "Known defects in the source" below.
5. The moment-reference shift that was applied, with the reference point and the CG used.
6. The sign convention (`cz` is the body +z, DOWN force coefficient, so `cz = -CL` and `cz[1]` is negative), and that `data/planes/linear/morelli.json` is the sign-convention authority where it disagrees with the F-16 set - it WINS on `cm[1]` (`-0.8` vs `+0.0466`) and `cnp[0]` (`-0.03` vs `+0.0268`) and LOSES on `cnda[0]` (`+0.01` vs `-0.0335`). Its `cz[1] = -4.5` AGREES with `cz = -CL` and is correct; do not describe it as a defect. — see "Sign convention".
7. A per-slot provenance note naming, for each of the 19 coefficient arrays, which slots are real solver output and which are zeroed.
8. The exact command that regenerates the file.

### Known defects in the source (verified before writing this plan)

These were measured, not assumed. The converter owns the corrections itself and does not inherit them from AID.

- **`aid.handbook_pass._per_rad` is a no-op below 1.0** (`handbook_pass.py:131`). DATCOM stores lift slope per degree (`HT.a = 0.0756`/deg, i.e. 4.330/rad). `_per_rad(0.0756)` returns `0.0756`, which `longitudinal_dynamic.py` then uses where a per-radian value is required. Consequence: DATCOM `CLq` comes out **57x too small** — `0.103` instead of `5.880`. Any DATCOM rate derivative taken from that path must be re-derived with the per-degree to per-radian factor applied by this plan.
- **Tornado reports moments about `ref_point`, not the CG, and its x axis is AFT-positive.** Measured on this aircraft: `geo["ref_point"] = [0, 0, 0]` and `geo["CG"] = [2.94, 0, 0]` ft, while the source planform runs `AERO.XW = 2.2`, `AERO.XCG = 2.94`, `AERO.XH = 8.75` — x increasing aft, the usual DATCOM convention. Raw Tornado `Cm_a = -7.72`/rad is a 177 % MAC static margin, which is impossible. Two corrections are needed, and they are easy to get wrong in a way that still lands on the right answer: the source offset is **aft**-positive, so `shift_moments_to_cg` must be called with `dx = -0.896112 m`, not `+0.896112`; and Tornado's `CL` is lift while `plane/dynamics.py`'s `cz` is the body **+z, down** force coefficient, so the call must pass `cz = -4.363`, not `+4.363`. The two sign errors cancel, so `(+4.363, +0.896112)` also returns `-1.31`/rad — which is exactly why the wrong pairing is hard to notice. But `cz = -4.363` with `dx = +0.896112` returns `-14.1336`/rad, a 323.9 % MAC static margin. **Task 3 must use `cz = -4.363, dx = -0.896112`.**
- **Tornado's roll and yaw RATE columns are component-sense inverted; the axis mapping explains the rest.** An earlier draft of this bullet blamed `np.cross(colloc - cg, rot_rates)` at `tornado/boundado/boundary.py:33` for `r x omega` instead of `omega x r`. **That diagnosis is wrong**: `r x omega` is the CORRECT form for the quantity being perturbed, since `_bc_column` builds `n.(wind + r x omega)` and `wind1` is the relative wind (minus the body velocity), so the two negations cancel. The real defect is narrower: `rot_rates = np.array([state["P"], state["Q"], state["R"]])` drops the standard-aero rates straight into the Tornado (aft, right, up) COMPONENT slots without converting their sense. Roll-right is `omega_x_aft = -P`, nose-up is `omega_y_right = +Q`, nose-right is `omega_z_up = -R`, so the **P and R columns are sign-inverted**; alpha and beta are untouched. Measured confirmation on this aircraft: `CY_b = -0.408` (correct, textbook `CY_beta < 0`), `CY_P = +0.160` (wrong, needs `< 0`), `CY_R = -0.387` (wrong, needs `> 0`), `Cm_Q = -19.21` (correct) - exactly the P/R-vs-Q/beta pattern.
  The OBSERVED inversions remain real: `CL_Q = +9.59` where `CL_q < 0`; `Cl_b = +0.054` where `Cl_beta < 0`; `Cn_b = -0.247` where `Cn_beta > 0`; `Cn_P = +0.069`. They are removed by the axis mapping alone, BEFORE any table normalisation. **Materiality: none.** The reviewer checked all 25 written coefficients under (a) the shipped mapping, (b) the P/R-corrected hypothesis, and (c) no mapping at all, and they are IDENTICAL - because `normalise_sign` is the final authority and the shift term's sign cancels with its force coefficient's. So no blanket flip is to be applied, and this dispute does not affect the deliverable.
- **The plan's own quoted Tornado figures were a 0-2 deg SECANT, not `coeff_create` derivatives.** Measured: the secant gives `CL_a = 4.3591` and `Cm_a = -7.7122`; `coeff_create` gives `CL_a = 5.148013` and `Cm_a = -8.786140`, the latter confirmed to 1e-12 against the sibling's MATLAB gold at `Results/matlab/Cessna 172/tornado.json`. A full re-run rebuilds and re-tilts the wake (`lattice.py::_wakesetup2`), so the secant is a different, nonlinear, wake-varying quantity. **Use `coeff_create`'s central difference (delta = 1e-4 rad) on the fixed wake.** Consequence: the static margin is **0.2367 cbar**, not the 0.2994 the secant would give.
- **The plan's 1 % `CL_a` cross-check gate is not attainable across two full runs.** Measured: the coarse 0-2 deg secant reads **15.32 %** below `coeff_create`, because the wake is re-tilted between runs. The workable substitute is a frozen-lattice secant: `+-0.05 deg` reads 0.3044 % low. That form catches a changed `delta`, a swapped perturbation column, a wrong `b2w` row, a mesh change or a state mix-up, but it is NOT independent of `coeff_create` (both read the same function) and is blind to a wake-setup drift BETWEEN runs. Record the coarse value too, and gate it loosely; do not present the 1 % figure as met.
- **The CG shift takes the BODY-Z force slope, not the lift slope.** `shift_moments_to_cg`'s `cz` is the body +z coefficient (`units.py`), i.e. `coeff_create["CZ_a"]` / `["CZ_Q"]` - not `CL_a` / `CL_Q`. The two differ because Tornado also carries `NP[0]`. Measured effect: `cm[1] = -1.2186` with `CL_a` versus `-1.1927` with `CZ_a` (2.2 %), `cm_q = -19.2498` versus `-19.2147` (0.18 %). Both margins stay in band, so nothing fails - but the shift is documented as exact for a coefficient linear in alpha, and this IS the residual linearisation error.
- **`CNb` / `Cn_da` have no DATCOM source.** `aid/lateral.py:616` returns a hardcoded `0.1` placeholder for `Clda` and `Cnda`, and `aid/handbook_controls.py` omits aileron yaw and aileron side force entirely. Tornado and flow5 do produce both, and Tornado is the flight model, so this gap does not reach `tornado.jsonc`. For `datcom.jsonc` it is zeroed and declared.
- **Tornado's rate derivatives are quoted at the SOURCE flight condition, not the trim speed — a factor of 7.43.** The flight condition comes from the source `AERO`: Mach 0.03, so Tornado's `state["AS"] = 10.2073 ft/s = 3.1112 m/s`. `coeff_create` forms every rate derivative as `(dC/dq) / (c_mac/(2·AS))`, i.e. per `p_hat` at **10.2073 m/s**. But `plane/dynamics.py` forms `phat = p·b/(2V)` at the trimmed `V = 23.1146 m/s`. The model's rate-damping terms are therefore **7.43x weaker** than the solver implies at the speed the model actually flies. This is a FLIGHT-CONDITION mismatch, not a unit error: `atm["a"] = 1116.29` is ft/s and `b_ref` is in feet, so Tornado is internally consistent in ft and ft/s. (An earlier draft of this plan called it a unit mismatch; that was wrong.) The written coefficients are not rescaled — the plan fixes the flight condition to the source's — but the factor is recorded in both data files, and **Task 4's adapters must hit the same thing and record it too.** A rescale, if wanted, is a one-line multiply by `AS_source / V_trim` on the nine slots that are genuinely per-p-hat or per-q-hat (`cyp[0]`, `cyr[0]`, `czq[0]`, `cmq[0]`, `clp[0]`, `clr[0]`, `cnp[0]`, `cnr[0]`, `cxq[0]`) and on NOTHING else. `cy[1]` and `cy[2]` are `cy_da` and `cy_dr` - aileron and rudder side force per radian of CONTROL DEFLECTION - and must never be rescaled. Whether to rescale at all is a decision for the user, not a bug fix.
- **Morelli's `cx` array cannot represent drag rise with alpha.** `cx` has slots for a constant, alpha, and `qhat`, but no `CD * alpha`. `CD_alpha` is not representable in this schema and is zeroed in every file, declared in the head-of-file comments.

### Sign convention

Derived from the force and moment algebra in `python/plane/dynamics.py`, **not** from the F-16 Morelli coefficient file.

`plane/dynamics.py` establishes, in order:

- `xd[11] = ub*sin(theta) - vb*cos(theta)*sin(phi) - wb*cos(phi)*cos(theta)`. That is the NED altitude rate `h_dot = u sin(theta) + v cos(theta) sin(phi) - w cos(theta) cos(phi)`, so **positive `w` is downward** and the body **z axis points down**.
- `az = (1/m) * qbar * S * cz` therefore acts **downward** for positive `cz`. **`cz` is the body-z force coefficient with down positive, so `cz = -CL`.** This is the load-bearing fact. Note that a downward force produces a nose-up *moment* (via `wdot`, hence `alpha_dot`), which is easy to mistake for "positive `cz` is lift up" — it is not; it is a downward force.
- `xd[7]` (which is `q`) receives `qbar * S * cbar * c7 * cm` with `c7 = 1/iyy > 0`, so **positive `cm` is nose up** and `cm[1]` (Cm_alpha) is **negative** for a statically stable aircraft.
- `xd[6]` (which is `p`) receives `qbar * S * b * c3 * cl` with `c3 = izz/gamma > 0`, so **positive `cl` is roll right**.
- `xd[8]` (which is `r`) receives `qbar * S * b * c9 * cn` with `c9 = ixx/gamma > 0`, so **positive `cn` is nose right**.
- `xd[0]` (which is `vt`) receives `inv_m * (qbar * S * cx + thrust)`, so **negative `cx` is drag**.
- The three control axes get their meaning from `cm[2]`, which is negative in both in-tree reference files, so **positive `de` is a nose-down elevator command** (elevator trailing edge UP). From there:
  - **positive `da` is right-aileron trailing edge UP** - the right wing's lift falls, so roll is left, so `clda[0]` is negative. Adverse yaw then takes the nose left, so `cnda[0]` is negative. The net side force is to the right, so `cy[1]` is positive.
  - **positive `dr` is rudder trailing edge LEFT** - the tail is pushed right, the nose swings left, so `cndr[0]` is negative; the side force is to the right, so `cy[2]` is positive; the roll is to the right, so `cldr[0]` is positive.

  These are conventions rather than physics, but they are pinned by the sign of `cm[2]` in the two in-tree reference files, which agree on it. That is why `cndr[0] < 0` is derived and not merely transcribed.
- `cm[2] = Cm_de` is **negative** in both in-tree reference files, so positive `de` is a **nose-down** elevator command. `cz[5] = CL_de` is negative in both, meaning positive `de` (nose down) produces positive lift — physically right for a tail-down manoeuvre. This fixes the meaning of `de`.

Independent confirmation that `cz = -CL`: the in-tree F-16 set has `cm[0] = -0.0203`, `cm[1] = +0.0466`, giving `alpha_trim = -cm0/cm1 = +0.4354 rad`, and `cz[0] + cz[1]*alpha_trim = -1.97`. Only `cz = -CL` turns that into a plausible lift coefficient (`CL = +1.97`); `cz = +CL` yields negative lift, which is unphysical. `run_f16.py` trims that file successfully.

Consequently, for a stable aircraft: **`cz[1]` is negative** (CL_alpha positive, since `CL_alpha = -cz[1]`) and **`cm[1]` is negative**.

Note for the record, and the file that governs where the two in-tree sets disagree: **`data/planes/linear/morelli.json` is the reference for sign conventions**, because the user named it as the flight model for this work. It agrees with the F-16 set on 17 of the 20 invariant rows. It disagrees on exactly two, and in both cases `linear/morelli.json` is the physically correct one:
- `cm[1]`: `linear` has `-0.8`, the F-16 set has `+0.0466`. A positive Cm_alpha is statically unstable; `linear` is right.
- `cnp[0]`: `linear` has `-0.03`, the F-16 set has `+0.0268`. `linear` is right.

One further disagreement, this time `linear/morelli.json` being the odd one out: its `cnda[0] = +0.01` opposes the F-16 set's `-0.0335` for the same physical effect (adverse yaw accompanies an aileron roll). The F-16 sign is used. Fixing `linear/morelli.json` is out of scope for this plan; it is recorded here and in `UPDATES.md`.

### Physics invariants

Every model file must satisfy these, and `python/tests/test_plane_cessna172_json.py` pins them. They follow from `plane/dynamics.py` plus conventional aerodynamics, and are the check that catches a unit slip, a reference slip, or a sign slip.

| Slot | Quantity | Required sign | Plausible magnitude |
|------|----------|---------------|--------------------|
| `cz[0]` | CL0 (`= -cz[0]`) | < 0 | -0.6 … -0.05 |
| `cz[1]` | CL_alpha (`= -cz[1]`) | < 0 | -6.5 … -4.0 /rad |
| `cm[1]` | Cm_alpha | < 0 | −1.5 … −0.3 /rad |
| `czq[0]` | CL_q | < 0 | magnitude 3 … 25 |
| `cmq[0]` | Cm_q | < 0 | magnitude 5 … 60 |
| `cl[0]` | Cl_beta | < 0 | −0.35 … −0.03 |
| `cy[0]` | CY_beta | < 0 | −1.2 … −0.15 |
| `cn[0]` | Cn_beta | > 0 | 0.02 … 0.40 |
| `clp[0]` | Cl_p | < 0 | −0.9 … −0.15 |
| `clr[0]` | Cl_r | > 0 | 0.005 … 0.25 |
| `cyp[0]` | CY_p | < 0 | −1.2 … −0.02 |
| `cyr[0]` | CY_r | > 0 | 0.05 … 1.2 |
| `cnp[0]` | Cn_p | < 0 | −0.35 … −0.005 |
| `cnr[0]` | Cn_r | < 0 | −0.9 … −0.05 |
| `cx[0]` | CD0 | < 0 | −0.15 … −0.005 |
| `cxq[0]` | CD_q | > 0 | 0.0 … 2.0 |
| `clda[0]` | Cl_da | < 0 | magnitude 0.02 … 0.6 |
| `cnda[0]` | Cn_da | < 0 | magnitude 0.005 … 0.25 |
| `cy[1]` | CY_da | > 0 | 0.005 … 0.30 |
| `cldr[0]` | Cl_dr | > 0 | 0.002 … 0.20 |
| `cndr[0]` | Cn_dr | < 0 | magnitude 0.02 … 0.90 |
| `cy[2]` | CY_dr | > 0 | 0.02 … 0.60 |
| `cz[5]` | CL_de | < 0 | -2.0 … -0.05 |
| `cm[2]` | Cm_de | < 0 | −4.0 … −0.30 |

Derived: static margin in cbar is **`cm[1] / cz[1]`** — because `CL_alpha = -cz[1]`, the textbook `-Cm_alpha / CL_alpha` reduces to `cm[1] / cz[1]`, and both slots are negative so the ratio is positive. It must land in **0.05 … 0.45** cbar (5–45 % MAC).

`cm[0]` (Cm0) is deliberately **absent from this table and its sign is never normalised.** The two in-tree reference files disagree — `linear/morelli.json` has `+0.05`, the F-16 set has `-0.0203` — and Cm0's sign depends on tail rigging, not on a convention this plan can derive. It is validated instead by the trim test in Task 5. `cz[0]` (CL0) *is* in the table because `linear/morelli.json` and the F-16 set agree on it.

Sign normalisation is **mechanical**: the converter multiplies by −1 when a reported value's sign disagrees with the table, and records the flip in `provenance`. It never hand-edits a number. Where a solver's sign cannot be established from an invariant, the slot is zeroed and declared rather than guessed.

### Mass, inertia, thrust, and the initial state

Made self-consistent **with the scaled planform actually present**, not with a full-scale C172.

- **`mass_kg` = 72.574** — the source file's own `AERO.WT` of 5 slugs × `g` = 32.174 ft/s², converted to SI. Sourced, not invented.
- **`ixx`, `iyy`, `izz`** are derived from this mass and this span by standard radius-of-gyration rules of thumb (`k_xx = 0.35 b`, `k_yy = 0.25 b`, `k_zz = 0.30 b`, `I = m k²`), computed in the converter, with `ixz = 0` and `he = 0`. The rule used for each axis is named in the file's head-of-file comments. Scaling published full-scale inertia was rejected: because inertia scales as mass × length², it compounds any error in the mass and length scales, and those two are already mutually inconsistent here.
- **`t_max_n`, `v_ref_mps`, `power`** and the `initial` block come from a **trim solve the converter performs**, not from a published engine number. The solver picks a target cruise CL, solves alpha from `CL0 + CL_alpha * alpha`, solves the throttle that zeroes the pitching moment, then solves `t_max_n` from `thrust_n(power, v, aircraft)` so that the resulting thrust equals the drag at that condition. `plane.aircraft.thrust_n` is `power * t_max_n * max(0, 1 - v / v_ref_mps)`, and `v_ref_mps` is set slightly above the cruise speed so the factor stays positive. This makes mass, thrust and the initial state mutually consistent by construction, which is the only way they can be.
- **Defensive fallback:** if no target CL yields a positive `t_max_n` and a positive throttle, the converter falls back to `t_max_n = weight`, `v_ref_mps = 1.5 * v`, `power = 0.5`, and records the fallback in `provenance`. It must not fail silently and it must not emit a negative or zero `t_max_n`, because `plane/aircraft.py` validates `_POSITIVE_KEYS`.

### Fail-closed

If a solver cannot produce a derivative set — binary missing, non-finite output, unusable sign — the converter writes the model file with the obtainable slots filled, the rest zeroed, and a populated `provenance.missing` list naming each absent slot and why. It never fabricates a value, and it never substitutes the F-16's numbers. A missing binary is recorded, not raised.

## Global Constraints

- **SI in, per-radian in.** Every length is metres, every mass kilograms, every inertia kg·m², every speed m/s, every angle in **radians**, every moment coefficient per **radian**. The source is feet and per degree. The per-degree to per-radian factor is `57.29577951308232`. A per-degree number must never reach a Morelli array.
- **Do not use AID's `_per_deg`, `_per_rad`, or `_per_rad` helpers** in `aid/stability.py`, `aid/handbook_controls.py`, or `aid/handbook_pass.py`. They are heuristic (`if a > 1: convert else pass`) and were measured to be wrong for this aircraft. Where an AID helper is unavoidable, re-apply the conversion explicitly afterwards and assert it.
- **Every moment is about the CG.** Tornado is not; shift it. DATCOM, AVL and flow5 report at the CG; verify, do not assume.
- **Do not edit** `python/f16/aero_morelli.py`'s polynomial structure, `plane/groups.py`'s `MORELLI_LENGTHS` / `LINEAR_INDEX` / `nonlinear_index`, `plane/dynamics.py`'s force and moment algebra, or `plane/aircraft.py`'s `_AIRCRAFT_KEYS` / `_POSITIVE_KEYS` / `_STATE_KEYS` / `_CONTROL_KEYS` / `thrust_n` / `_inertia_coefficients`. These are the model structure.
- **No `nan` or `inf` may enter `DerivativeSet` or reach a coefficient array.** A solver returning a non-finite derivative is treated as not producing that derivative: the field becomes `None`, the slot is zeroed, and the slot is named in `provenance.missing` with the reason. The finiteness check belongs at the solver boundary, not in `normalise_sign`, so `normalise_sign` never has to decide the sign of `nan`.
- Three distinct provenance states for a slot, and all three must be recorded: **missing** (field `None`), **flipped** (sign normalised against its invariant), and **not-normalised** (field present but carrying no invariant - today only `cm0`). A slot in the third state must be recorded as such, not silently omitted.
- Every nonlinear slot — `nonlinear_index(name)` from `plane/groups.py` — is exactly `0.0`. Same rule as `data/planes/linear/morelli.json`. Only the slots listed as linear in `LINEAR_INDEX` may be non-zero.
- `cd[0]`-style absolute magnitudes: no coefficient array element may be non-finite. Every value written is a finite float.
- Defaults must not change: `load_aero_coefficients(model, root=None)` with `model="morelli"` and `root=None` still resolves `data/planes/f16/morelli.json`; `load_aircraft("linear")` still reads `data/planes/linear/morelli.json`; `run_plane.py --plane linear` still works with no `--model`.
- `stevens` keeps its own `_STEVENS_GROUPS` validation. The new Morelli-schema model names are checked against `_MORELLI_GROUPS`. An unknown model still raises `ValueError` containing `not implemented`.
- The converter reads `../USAF_DATCOM/AircraftIntuitiveDesign/Analyses/Cessna172.jsonc` and imports from `../USAF_DATCOM/AircraftIntuitiveDesign/Python/src/aid/`. **Never write into either.** All scratch in a `TemporaryDirectory`. Import the sibling `aid` package by inserting its `src` on `sys.path`; do not add it to a requirements file or vendor it.
- Commit each task on branch `cessna172-aero-data`. Do not push. Do not update git config. Do not use `--no-verify`. Message via HEREDOC. **Never** stage the unrelated `Dockerfiles` submodule pointer change or the untracked `docs/superpowers/plans/2026-10-02-linear-morelli-6dof.md`.
- Host tests only: `cd python && python3 -m unittest <module> -v`. TDD: write the test, run it, watch it fail for the right reason, then implement. Record RED and GREEN command output in the report.
- No `.md` file other than `README.md` and `UPDATES.md` is created or edited by this plan.

## Parallelism

Tasks 1–6 are sequential: 2 depends on nothing but its own maths, 3 depends on 2, 4 depends on 2 and 3, 5 depends on all data files, 6 depends on 5. **Do not dispatch any two together.**

## Rulings

Recorded here so a later reader can revisit them.

- **`Ruling: the CG offset is `dx = -0.896112 m`, not `-0.895512 m`. The earlier figure was a digit transposition by the controller.** The source is `AERO.XCG = 2.94` ft and `2.94 * 0.3048 = 0.896112` m exactly. The plan propagated `0.895512` into a Task 2 test constant, a docstring, and a pinned `Cm_a`, so the pinned value `-1.3106842913385819` was wrong; the correct value is `-1.3063899999999986`, a 29.9 % MAC static margin. Caught by the round-3 re-reviewer. — Costs: one more fix round on Task 2. Nothing downstream used the bad number: Task 3 has not been dispatched.
- **`Ruling: the controller's Finding A against Task 2 was a FALSE POSITIVE and is withdrawn.** I reported two sign errors in `shift_moments_to_cg` (`Cm` and `Cn`) on the strength of a verification script of my own that wrongly assumed the function's `cd` argument was the raw `cx` coefficient rather than the positive drag magnitude. The implementer refused the change and proved the existing code against `numpy.cross`. Re-checked independently: `M_CG = M_O - (CG-O) x F` is reproduced to 0.0 across four offset cases including non-zero `dy` and `dz`. The implementer was right and I was wrong. — Costs: none to the code. The parameter rename (`cl` -> `cz`, `cd` -> `cx`) landed anyway and is the real remedy: the old names were a genuine trap that invited precisely the error I made.

- **SUPERSEDED - do not follow. Retained only so a reader who greps for it finds the correction.** An earlier draft ruled that CL_alpha is positive (`cz[1] > 0`), overriding `linear/morelli.json`'s `-4.5`. That was wrong. See the ruling titled "`cz[1]` (CL_alpha) is NEGATIVE" below, which replaces it.
- **`Ruling: inertia is derived from radius-of-gyration rules, not scaled from published values.** — The source's own mass (5 slugs = 72.574 kg) is not a geometrically similar 1/3-scale C172 (that is ~53.5 kg), so mass and length scale are mutually inconsistent; inertia scaling as `m * l²` would compound that error rather than cancel it. — Costs: the inertia numbers are rules-of-thumb rather than measured, so roll/pitch/yaw mode frequencies are indicative rather than authoritative.
- **`Ruling: thrust and the initial state come from a trim solve, not a published engine number.** — The planform is 1/3 scale, so a full-scale O-320 thrust figure is meaningless against it; a solved value is consistent with the geometry by construction. — Costs: `t_max_n` is not an engine datasheet number and must not be quoted as one.
- **`Ruling: sign normalisation is mechanical from the invariant table, not hand-authored per slot.** — Guarantees no number is silently inverted and every flip is logged. — Costs: if the invariant table is itself wrong for some slot, the converter will faithfully produce a wrong sign; the table is therefore part of the reviewed spec.
- **`Ruling: `cz[1]` (CL_alpha) is NEGATIVE, and the static margin is `cm[1] / cz[1]`. This reverses the first draft of this plan, which asserted `cz[1] > 0`.** — `plane/dynamics.py` puts `az = +qbar S cz / m` into `wdot` in a **z-down** body frame (`xd[11]` is the NED altitude rate, so `+w` is down). A downward force does raise `alpha_dot`, which is what the first draft mistaken for "positive `cz` is lift up" — but it is a downward force, not lift. Confirmed independently against the in-tree F-16 set, which `run_f16.py` trims successfully: `alpha_trim = -cm0/cm1 = +0.4354 rad`, and `cz0 + cz1*alpha = -1.97`, which is a plausible lift coefficient only under `cz = -CL`. — Costs: this reverses a statement the user was shown when the design was approved, so the head-of-file comments must state the `cz = -CL` identity explicitly rather than leave it implicit.
- **`Ruling: Tornado's x axis is AFT-positive, so the CG offset passed to `shift_moments_to_cg` is NEGATIVE.** The source planform runs x aft (`AERO.XW = 2.2` to `AERO.XCG = 2.94` to `AERO.XH = 8.75` ft) and Tornado inherits that, but `shift_moments_to_cg` takes `r = CG - O` in the `plane/dynamics.py` frame where x is FORWARD. — Costs: `dy` and `dz` are 0 for this aircraft and the `cz` sign flip cancels the `dx` sign flip in the pinned `Cm_a`, so the error is invisible in the single case the plan quotes. Task 3 must pair `cz = -4.363` with `dx = -0.896112`, and the Task 2 test pins that pairing rather than either sign alone.
- **`Ruling: use `coeff_create`'s derivatives, not a multi-run secant, for `cl_alpha` and `cm_alpha`.** The plan's quoted `-7.720` / `4.363` pair is a 0-2 deg secant; the true fixed-wake derivatives are `-8.786140` / `5.148013`, confirmed against MATLAB gold. A re-run re-tilts the wake, so the secant measures a different quantity. — Costs: the static margin is 0.2367 cbar, not 0.2994, and every downstream `cm[1]`-based figure moves.
- **`Ruling: `mass_kg` is 72.9695, not 72.574, but 72.574 ships.** 5 slugs x `SLUG_TO_KG` (14.59390294, the plan's own constant) = 72.96951 kg; 72.574 corresponds to g = 32.0 ft/s^2, and the plan's own sentence quoted 32.174. So the plan's figure is 0.54 % low. Nothing else in the repo settles it (`linear/morelli.json` is a generic 1000 kg aeroplane). The implementer shipped the plan's mandated literal, which plan test 6 pins verbatim in the header, and recorded both figures with the reason in `provenance.mass`. — Costs: weight, trim qbar, V and `t_max_n` are all 0.54 % off, and the header must state that g = 32.0 ft/s^2 appears nowhere in the source.
- **`Ruling: three derived radii of gyration, not four`** (roll, pitch, yaw). The plan said four; `he = 0` so there is no fourth.
- **`Ruling: `linear/morelli.json` is the sign-convention authority where it and the F-16 set disagree.`** — The user named it as the flight model for this work. It wins on `cm[1]` (`-0.8` vs `+0.0466`) and `cnp[0]` (`-0.03` vs `+0.0268`), where a positive value would be statically unstable. It loses on `cnda[0]` (`+0.01` vs `-0.0335`), where adverse yaw fixes the sign. — Costs: the Cessna files will disagree with `data/planes/f16/morelli.json` on `cm[1]` and `cnp[0]`; a reader must not assume the two sets share a convention.

## File map

| File | Task | Responsibility |
|------|------|----------------|
| `f16/aero_data.py` | 1 | Accept the new model names; resolve `.jsonc` then `.json` |
| `f16/aero_morelli.py` | 1 | `model` keyword passed to the loader |
| `plane/aircraft.py` | 1 | `load_aircraft(plane, model=)`; JSONC reader |
| `plane/dynamics.py` | 1 | Pass the aircraft's model through |
| `run_plane.py` | 1 | `--model` flag |
| `python/tests/test_plane_model_select.py` | 1 | Model selection and `.jsonc` |
| `python/aero_convert/__init__.py` | 2 | Package |
| `python/aero_convert/units.py` | 2 | ft to SI, per-degree to per-radian, CG moment shift, sign normalisation |
| `python/aero_convert/morelli.py` | 2 | Derivative set to the 19 Morelli arrays |
| `python/tests/test_aero_convert_units.py` | 2 | Conversion maths, no solver |
| `python/tests/test_aero_convert_morelli.py` | 2 | Slot mapping, no solver |
| `python/aero_convert/solvers.py` | 3, 4 | The four solver adapters |
| `python/scripts/build_cessna172_aero.py` | 3, 4 | CLI entry; writes the five files |
| `data/planes/cessna172/geometry.jsonc` | 3 | SI reference geometry and raw solver output |
| `data/planes/cessna172/tornado.jsonc` | 3 | The flight model |
| `data/planes/cessna172/{datcom,avl,flow5}.jsonc` | 4 | The other three model sets |
| `python/tests/test_aero_convert_solvers.py` | 3, 4 | Adapters return all 19 arrays |
| `python/tests/test_plane_cessna172_json.py` | 5 | Physics invariants, trim, modes, cross-model |
| `README.md` | 6 | One row, one run line |
| `UPDATES.md` | 6 | `0.90.0` entry |

### Task 1: `--model` selection and `.jsonc` reading

**Deliverable:** `run_plane.py --plane cessna172 --model tornado` opens `data/planes/cessna172/tornado.jsonc`, and every existing caller is unaffected.

**Files:** `f16/aero_data.py`, `f16/aero_morelli.py`, `plane/aircraft.py`, `plane/dynamics.py`, `run_plane.py`, new `python/tests/test_plane_model_select.py`.

**RED tests to write first**, in `python/tests/test_plane_model_select.py`:

1. `load_aero_coefficients("tornado", root=<tmpdir>)` reads `<tmpdir>/tornado.json` when it is the only file present. Fails today with `ValueError: model 'tornado' not implemented`.
2. `load_aero_coefficients("tornado", root=<tmpdir>)` reads `<tmpdir>/tornado.jsonc` when a `.jsonc` exists. Proves the `.jsonc` fallback and that `//` comments are stripped.
3. `load_aero_coefficients("stevens", ...)` still requires the Stevens group set and still rejects a Morelli-only file.
4. `load_aero_coefficients("nonesuch", ...)` still raises `ValueError` containing `not implemented`.
5. `load_aircraft("linear")` with no `model` still loads and still reports the same `mass_kg`.
6. `load_aircraft(<tmp plane dir>, model="tornado")` where only `tornado.jsonc` exists loads, and `aircraft.model == "tornado"`.
7. `morelli_coefficients(..., model="tornado", root=<tmpdir>)` reads the Tornado file, and the default `model="morelli"` still reads the Morelli file.

**Implementation:**

- `f16/aero_data.py`: extend `_MODELS` to `("morelli", "stevens", "tornado", "datcom", "avl", "flow5")`. Group selection becomes a dict — `morelli` and the four new names validate against `_MORELLI_GROUPS`, `stevens` against `_STEVENS_GROUPS`. Resolution tries `directory / f"{model}.jsonc"` first, then `directory / f"{model}.json"`; a missing file still raises `ValueError(f"missing {model}.json")`. The cache key stays the resolved path. Strip `//` line comments before `json.loads`, honouring quoted strings so a `//` inside a string is not stripped — `aid/jsonc.py:5` is the reference implementation and the same semantics.
- `f16/aero_morelli.py`: add keyword-only `model: str = "morelli"` alongside the existing `root`, and pass it to `load_aero_coefficients`. The polynomial expressions stay byte-identical.
- `plane/aircraft.py`: `load_aircraft(plane: str, model: str = "morelli", root: Path | None = None)`. Resolve `<model>.jsonc` then `<model>.json` in the plane directory. Reuse a shared comment-stripping helper rather than a second copy — put it in `f16/aero_data.py` and import it, so there is exactly one implementation. Add `model` to the frozen `Aircraft` dataclass.
- `plane/dynamics.py`: pass `model=aircraft.model` into `morelli_coefficients`.
- `run_plane.py`: add `--model` with default `"morelli"`; pass it to `load_aircraft`. Unknown plane or model must exit 2 with a message on stderr, matching the existing behaviour.

**GREEN:** all seven tests pass, and `cd python && python3 -m unittest tests.test_plane_model_select tests.test_plane_linear_json tests.test_plane_aircraft tests.test_plane_dynamics tests.test_plane_runner tests.test_f16_aero_coefficients tests.test_f16_aero -v` is all green with no behaviour change for the existing planes.

### Task 2: Conversion core

**Deliverable:** `aero_convert.units` and `aero_convert.morelli` convert a normalised derivative set into the 19 Morelli arrays, with no solver involved and no file written. Fast feedback on the hard maths.

**Files:** new `python/aero_convert/__init__.py`, `python/aero_convert/units.py`, `python/aero_convert/morelli.py`, new `python/tests/test_aero_convert_units.py`, new `python/tests/test_aero_convert_morelli.py`.

**`units.py` exports:**

- `DEG_TO_RAD = 57.29577951308232`, `FT_TO_M = 0.3048`, `SLUG_TO_KG = 14.59390294`, `G_FT_S2 = 32.174`.
- `per_degree_to_per_radian(value: float) -> float` — `value * DEG_TO_RAD`. Named for what it does; no heuristics.
- `ft(value: float) -> float`
- `shift_moments_to_cg(cl, cd, cy, cl_m, cm, cn, *, dx, dy, dz, s_ref, b_ref, c_ref) -> tuple` — moves the moment reference from `(dx, dy, dz)` relative to the CG to the CG, in the SI frame with `z` down. `Cm` gains `+dx * CL / c_ref`, `Cl` gains `(dx * CZ - dz * CY) / b_ref`, `Cn` gains `(dy * CZ - dx * CY) / b_ref`. Implement the sign of each term from `plane/dynamics.py`'s algebra and prove it in a test against a hand-worked case, not against the formula itself.
- `normalise_sign(name: str, value: float) -> tuple[float, bool]` — returns `(value, flipped)`, flipping when the sign disagrees with the invariant table for that slot. The table lives in `units.py` as `SIGN_INVARIANTS` mapping a slot key to `+1` or `-1`. Every call site passes the slot key.

**`morelli.py` exports:**

- `DerivativeSet` — a dataclass holding the normalised per-radian derivatives as named optional fields: `cl0, cl_alpha, cl_q, cl_beta, cl_p, cl_r, cl_da, cl_dr, cl_de, cd0, cd_q, cy_beta, cy_p, cy_r, cy_da, cy_dr, cm0, cm_alpha, cm_q, cm_de, cn_beta, cn_p, cn_r, cn_da, cn_dr`, all `float | None`. `None` means the solver does not produce it and the slot is zeroed and declared.
- `to_morelli(derivatives: DerivativeSet) -> tuple[dict[str, list[float]], list[str]]` — returns the `coefficients` object and the list of slot keys that were zeroed because their field was `None`. Every array has exactly `MORELLI_LENGTHS[name]` entries from `plane/groups.py`. Import that mapping rather than retyping the lengths.
- The slot mapping, written once, explicitly:

  | Array | Slot | Source field |
  |-------|------|--------------|
  | `cx` | `[0]` | `cd0` |
  | `cxq` | `[0]` | `cd_q` |
  | `cy` | `[0]`, `[1]`, `[2]` | `cy_beta`, `cy_da`, `cy_dr` |
  | `cyp` | `[0]` | `cy_p` |
  | `cyr` | `[0]` | `cy_r` |
  | `cz` | `[0]`, `[1]`, `[5]` | `cl0`, `cl_alpha`, `cl_de` |
  | `czq` | `[0]` | `cl_q` |
  | `cl` | `[0]` | `cl_beta` |
  | `clp` | `[0]` | `cl_p` |
  | `clr` | `[0]` | `cl_r` |
  | `clda` | `[0]` | `cl_da` |
  | `cldr` | `[0]` | `cl_dr` |
  | `cm` | `[0]`, `[1]`, `[2]` | `cm0`, `cm_alpha`, `cm_de` |
  | `cmq` | `[0]` | `cm_q` |
  | `cn` | `[0]` | `cn_beta` |
  | `cnp` | `[0]` | `cn_p` |
  | `cnr` | `[0]` | `cn_r` |
  | `cnda` | `[0]` | `cn_da` |
  | `cndr` | `[0]` | `cn_dr` |

**RED tests to write first:**

- `per_degree_to_per_radian(0.0756)` equals `0.0756 * 57.29577951308232` to 1e-12, and the value `0.0756` (HT.a, per degree) becomes `4.330` per radian.
- `ft(12.0)` is `3.6576` exactly.
- `shift_moments_to_cg` reproduces a hand-worked three-element case to 1e-12, including the `dz` term in the `Cl` result and the `dy` term in the `Cn` result.
- The measured Tornado case is reproduced: `Cm_a` about `ref_point` is `-7.720`, `c_ref = 0.6096 m`, Tornado's lift slope maps to `cz = -4.363`, and the source offset is aft-positive so `dx = -0.896112 m`. The shifted `Cm_a` is `-7.720 + (-0.896112) * (-4.363) / 0.6096 = -1.3063899999999986`, asserted to 1e-9. This is a 29.9 % MAC static margin. This pins the sign of the `Cm` shift term AND the aft-positive frame convention. **The test must discriminate**: flipping either `dx` or `cz` alone must turn it red — flipping `dx` alone gives `-14.133610000000001`, a 323.9 % MAC static margin; flipping `cz` alone gives the same. Never assert a pairing in which the two sign errors cancel.
- `normalise_sign("cz_alpha", -4.0)` returns `(-4.0, False)` — a negative CL_alpha is already correct, so nothing flips. `normalise_sign("cz_alpha", 4.0)` returns `(-4.0, True)`. `normalise_sign("cy_beta", -0.6)` returns `(-0.6, False)`. `normalise_sign("cn_beta", -0.2)` returns `(+0.2, True)`.
- `normalise_sign("cm0", -0.05)` returns `(-0.05, False)` — `cm0` carries no invariant, so its sign is never touched.
- `normalise_sign("cz_alpha", 0.0)` and `normalise_sign("cd0", 0.0)` both return `(0.0, False)` and do not raise, and the result is not negative zero. Use a `+1`-pinned key such as `"cd_q"` as well, because only a `+1` key can distinguish the zero short-circuit. `normalise_sign("cz", 0.0)` must **raise** `KeyError` — `cz` is a coefficient-array name, not a slot key. A name that is neither in the table nor in `NO_INVARIANT` must raise `KeyError` naming the key and listing the valid names, so an adapter typo fails loudly instead of silently shipping an inverted sign.
- `to_morelli` with an all-`None` `DerivativeSet` returns all-zero arrays of the right lengths and names every slot as missing.
- `to_morelli` returns `MORELLI_LENGTHS[name]` entries for all 19 arrays.
- Every slot in `nonlinear_index(name)` is `0.0` for a `DerivativeSet` with every field populated.
- A `DerivativeSet` carrying the invariant-table values yields a `coefficients` object matching the table row by row.

**GREEN:** both new test modules pass; `cd python && python3 -m unittest discover -s tests` shows no new failures.

### Task 3: Tornado adapter and the flight model

**Deliverable:** `data/planes/cessna172/tornado.jsonc` and `geometry.jsonc` exist, are physics-invariant-clean, and regenerate from the source file. `run_plane.py --plane cessna172 --model tornado --duration 5` writes a finite CSV.

**Files:** new `python/aero_convert/solvers.py`, new `python/scripts/build_cessna172_aero.py`, `data/planes/cessna172/geometry.jsonc`, `data/planes/cessna172/tornado.jsonc`, new `python/tests/test_aero_convert_solvers.py`.

**Solver driver:** `solvers.py` imports the sibling package by inserting `../USAF_DATCOM/AircraftIntuitiveDesign/Python/src` on `sys.path` (resolve it relative to the repo root, overridable by the `AID_SRC` environment variable so a checkout elsewhere works). Use `aid.aircraft.load_jsonc`, `aid.tornado_io.tornado_io`, `aid.tornado.lattice.lattice_setup`, `aid.tornado.boundary.set_boundary`, `aid.tornado.solver.solve`, `aid.tornado.coeff_create`, and for control derivatives `aid.tornado.control_deriv.tornado_controls`.

**Tornado run:** mesh `("10", "5")` — the AID batch default for Tornado. Flight condition from the source `AERO`: Mach 0.03, altitude 0, `betha` 0. **`NP[0]` is kept in the lattice** — it is the `AR 33.3, S 3 ft²` panel, and it contributes 9.62 % of the aircraft CL (measured; an earlier draft of this plan said ~18 %). Excluding it is explicitly out of scope.

**Mapping from `coeff_create` output to `DerivativeSet`** (Tornado names → field):

| Tornado | Field | Conversion |
|---------|-------|------------|
| `CL` | `cl0` | Tornado's `CL_a` slope form; compute `cl0` from two alpha samples rather than trusting `CL` at one alpha |
| `CL_a` | `cl_alpha` | already per radian (`state["alpha"]` is in radians) |
| `CL_Q`, `CL_R`, `CL_P` | `cl_q`, `cl_r`, `cl_p` | already divided by `fac` in `coeff_create`; **sign-normalise** |
| `CC_b`, `CY_b` | `cy_beta` | `CY_b` is body-axis; **sign-normalise** |
| `CY_P`, `CY_R` | `cy_p`, `cy_r` | **sign-normalise** |
| `Cl_b` | `cl_beta` | **sign-normalise** |
| `Cl_P`, `Cl_R` | `cl_p`, `cl_r` | **sign-normalise** |
| `Cm_a`, `Cm_Q` | `cm_alpha`, `cm_q` | **shift to the CG first**, then sign-normalise |
| `Cn_b`, `Cn_P`, `Cn_R` | `cn_beta`, `cn_p`, `cn_r` | **shift to the CG first**, then sign-normalise |
| `CX_Q` | `cd_q` | sign-normalise |
| `CD` at a reference alpha | `cd0` | parasite drag, `CD` minus `K * CL²` |
| `tornado_controls` aileron row | `cl_da`, `cy_da`, `cn_da` | Tornado's `blank_row` gives `Cl`, `CY`, `Cn` per **degree**; multiply by `DEG_TO_RAD`, then sign-normalise |
| `tornado_controls` rudder row | `cl_dr`, `cy_dr`, `cn_dr` | same |
| `tornado_controls` elevator row | `cl_de`, `cm_de` | `CL` and `Cm` per degree; multiply by `DEG_TO_RAD`, sign-normalise, shift `cm_de` to the CG |

`cl0` is the one value `coeff_create` does not hand over cleanly. Solve it from the measured `CL_a`: run at `alpha = 0` and at `alpha = 2 deg` in the sibling package, then `cl0 = CL(alpha=0)`. Use that same two-point difference as a cross-check on `coeff_create`'s own `CL_a` and fail loudly if they disagree by more than 1 % — that disagreement is how a mesh or convention drift shows up.

**Geometry block** (`geometry.jsonc`): SI `s_ref`, `b_ref`, `c_ref`, `x_cg`, `x_cg_ref`, the wing/HT/VT planform essentials, the source flight condition, the four derived radii of gyration with the rule used for each, and each solver's **raw unconverted** reported derivatives so a future reader can re-derive without re-running anything.

**Mass / inertia / thrust / initial:** as specified under "Mass, inertia, thrust, and the initial state". The trim solve uses the Tornado `cl0`, `cl_alpha`, `cm0`, `cm_alpha`, `cm_de` and the model `cx[0]`.

**RED tests to write first** in `python/tests/test_aero_convert_solvers.py`:

1. `tornado_derivatives(<source>)` returns a `DerivativeSet` with no `None` in `cl_alpha`, `cm_alpha`, `cl_q`, `cm_q`, `cl_beta`, `cy_beta`, `cn_beta`, `cl_p`, `cl_r`, `cn_p`, `cn_r`, `cl_da`, `cy_da`, `cn_da`, `cl_dr`, `cy_dr`, `cn_dr`, `cl_de`, `cm_de`.
2. Every one of those satisfies its invariant row's sign.
3. `cn_da` is non-zero — this is the specific slot DATCOM cannot produce, so it is the specific test that proves the file is complete.
4. `to_morelli` on the Tornado set gives every array the right length, every nonlinear slot exactly `0.0`, and every element finite.
5. The static margin of the Tornado set, computed as `cm[1] / cz[1]` in cbar, is in 0.05 … 0.45.
6. The file exists after a build, starts with a `//` comment line before the `{`, and the comment block contains each of the eight required items from "Head-of-file comments" — test for the presence of the substrings `Tornado`, `1/3`, `72.574`, `57.29577951308232`, `ref_point`, `Cd0`, `CD_alpha`, `build_cessna172_aero.py`.

**GREEN:** all six pass; `python3 run_plane.py --plane cessna172 --model tornado --duration 5 --csv /tmp/c172.csv` exits 0 and every CSV field parses as a finite float.

### Task 4: DATCOM, AVL and flow5 adapters

**Deliverable:** `datcom.jsonc`, `avl.jsonc`, `flow5.jsonc` exist with the same schema, the same `.jsonc` head-comment standard, and honest coverage.

Dispatched as one subagent: these are the same shape as Task 3 and each needs no independent judgment.

**Files:** `python/aero_convert/solvers.py`, `python/scripts/build_cessna172_aero.py`, the three `.jsonc` files, `python/tests/test_aero_convert_solvers.py` (append).

- **DATCOM:** run the real Fortran via `aid.datcom_run.run_datcom` on the source file (binary at `Matlab/fsroot/code/DATCOM/datcom`, resolved by `aid.paths.datcom_wrapper`). Do **not** take rate derivatives from `aid.longitudinal_dynamic` — the measured `_per_rad` defect makes them 57x wrong. Instead: take `CLa`, `Cma`, `Cm0`, `CL0`, `CD0`, `CYb`, `Cnb`, `Clb` from `aid.stability` / `aid.lateral` / `aid.drag`; take `CLde`, `Cmde`, `CYdr`, `Cl_dr`, `Cn_dr` from `aid.handbook_controls`; take `Clda` from `aid.handbook_controls`' aileron row; and compute `CLq`, `Cmq`, `CYp`, `CYr`, `Clp`, `Clr`, `Cnp`, `Cnr` from the DATCOM Section 7 formulae with the per-degree lift slope converted by this plan, **not** via `aid.longitudinal_dynamic` / `aid.lateral.lateral_dynamic`. `Cn_da` and `CY_da` are `None` — zeroed and declared, with `aid/lateral.py:616`'s `0.1` placeholder explicitly **not** used.
- **AVL:** `aid.avl_io.run_avl_full`, mesh `("10", "10")`. Stability derivatives from the `.st` files, surface-deflection derivatives from the `.sb` files via `aid.avl_parse.rows_from_sb`. AVL's `.sb` carries aileron and rudder deflection derivatives, so `Cn_da` and `CY_da` should be available; if a row is absent, it is `None` and declared, not guessed.
- **flow5:** `aid.flow5_io.write_flow5_deck` + `run_flow5_native`, mesh `("10", "10")`. Static derivatives from the deck's own alpha rows. flow5 is a steady panel code with **no rate rows**, so `cl_q`, `cm_q`, `cy_p`, `cy_r`, `cl_p`, `cl_r`, `cn_p`, `cn_r`, `cd_q` are all `None` — zeroed and declared. `aid.flow5_controls.flow5_controls` keeps only `Cl`/`Cn` for aileron and `CY`/`Cl`/`Cn` for rudder, so `cn_da` and `cy_da` are available and `cd_de` is not representable.

**RED tests to write first**, appended to `python/tests/test_aero_convert_solvers.py`:

1. `datcom_derivatives`, `avl_derivatives`, `flow5_derivatives` each return a `DerivativeSet` with `cl_alpha`, `cm_alpha`, `cl_beta`, `cy_beta`, `cn_beta`, `cl_da`, `cl_de`, `cm_de` all non-`None`.
2. Each satisfies every invariant row it has a value for.
3. `flow5_derivatives` has `cl_q is None` and `cm_q is None`.
4. `datcom_derivatives` has `cn_da is None` and `cy_da is None`.
5. Each model's static margin, computed as `cm[1] / cz[1]` in cbar, is in 0.05 … 0.45.
6. All three files exist, open with a `//` comment before the `{`, and carry the eight required head-comment items.
7. Cross-model: every model's `cl_alpha` lies within -6.5 … -4.0 /rad (i.e. `cz[1]` negative and in magnitude 4.0 … 6.5), so a unit or reference slip in any one adapter is visible.

**GREEN:** all pass; each file opens through `load_aircraft("cessna172", model=<name>)`.

### Task 5: Physics-invariant suite

**Deliverable:** `python/tests/test_plane_cessna172_json.py` pins the shipped files against the invariants table, the trim state, the mode poles, and each other. This is the acceptance gate.

**Files:** new `python/tests/test_plane_cessna172_json.py`.

**Tests**, all reading the committed `.jsonc` files through `load_aircraft` and `load_aero_coefficients`:

1. **Schema** — all four model files load; `model` matches the filename; every one of the 19 arrays is present with `MORELLI_LENGTHS[name]` entries; every element is a finite float; every nonlinear slot is exactly `0.0`.
2. **Invariants** — every row of the "Physics invariants" table holds for `tornado.jsonc`, magnitudes included. For the other three, every row they have a value for. `cm[0]` is explicitly excluded: its sign is never normalised.
3. **Static margin** — `cm[1] / cz[1]` in 0.05 … 0.45 cbar for all four (remember `CL_alpha = -cz[1]`).
4. **Trim** — for `tornado.jsonc`, `cm[0] + cm[1] * alpha = 0` gives `alpha_trim` in −2° … +8°, and `cz[0] + cz[1] * alpha_trim` is a plausible cruise CL in 0.2 … 1.2.
5. **Modes** — build the longitudinal 2×2 short-period pair and the lateral-directional roll/yaw pair from the coefficients and assert the short-period poles have negative real part and the phugoid poles too. Print every pole so a regression is visible in the test output rather than only in a failure message.
6. **Cross-model agreement** — `tornado`, `datcom`, `avl` and `flow5` agree on `cl_alpha` within a factor of 1.6 of each other, and all four agree on the **sign** of every invariant slot they share.
7. **Declared gaps** — for each model, every slot that is zero appears in that file's `provenance.missing`, and nothing appears in `provenance.missing` that is non-zero.
8. **Runner smoke** — `run_plane.py --plane cessna172 --model tornado --duration 5` exits 0 and the CSV is all finite.

**GREEN:** `cd python && MPLBACKEND=Agg python3 -m unittest tests.test_plane_cessna172_json -v` is all green, and `cd python && python3 -m unittest discover -s tests` introduces no new failures.

### Task 6: Docs

**Deliverable:** the change is discoverable.

**Files:** `README.md`, `UPDATES.md`. No other `.md`.

**`UPDATES.md`:** a new top entry, version `0.90.0` (feature), titled something like "Cessna 172 aerodynamic data from USAF_DATCOM". One line each for: the five files and their schema; that `tornado.jsonc` is the flight model and the other three are the cross-checks; the `--model` flag and `.jsonc` reading; that every number is SI and per-radian; that the source planform is 1/3 scale and the mass/inertia/thrust are self-consistent with it rather than full-scale; the `--model`/`run_plane.py` run line; and **the sign-convention note: `cz` is the body-z force coefficient in a z-down frame, so `cz = -CL` and `cz[1]` is negative; `data/planes/linear/morelli.json` is the convention authority but its `cnda[0] = +0.01` opposes the F-16 set and is left unchanged as out of scope.**

**`README.md`:** one row in the architecture table for `python/aero_convert/`, one sentence extending the `python/plane/` row to name the Cessna plane and `--model`, and one line in the `Run` block:

```bash
cd python && python3 run_plane.py --plane cessna172 --model tornado --duration 5 --csv /tmp/c172.csv
```

Do not restate anything already in `UPDATES.md`. Update `README.md`'s idea or architecture only insofar as this changes them; otherwise this is one row and one line.

**Verification:** `cd python && python3 -m unittest discover -s tests` is green, then stop. Do not commit.