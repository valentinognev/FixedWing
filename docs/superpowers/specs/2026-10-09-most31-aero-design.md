# MOST31 Generalized Aero Model Design

Date: 2026-10-09
Status: approved in chat (model, schema, translation, integration, proof); pending file review before implementation plan.

## Goal

One aerodynamic coefficient model, MOST31, that is an exact superset of the three aero models in the tree — X-31 Figure 2.2, F-16 Morelli, F-16 Stevens — so each becomes a MOST31 coefficient file in which the terms it does not use are zero. All angles are radians. MOST31 is the aero evaluator for both the F-16 plant and the X-31 plant.

## Non-goals

- `python/plane/` (Cessna 172, linear airplane) stays on its own Morelli path.
- X-31 controllers (gain schedule, NDI, maneuver generator) keep the port's internal Figure 2.2 model.
- No refit, no new aerodynamic data, no Mach dependence, no flap aero (the port has none).
- No edit to the vendored `python/x31/` port.

## Decisions (locked)

- **Exact superset.** A translated file reproduces its source evaluator to floating-point precision (tolerances under Proof), not a fit.
- **Fixed dense schema.** Every MOST31 file has the same named arrays with the same shapes; unused entries are literally `0.0`.
- **Old models stay as source and oracle.** `data/planes/f16/{morelli,stevens}.json`, `x31.params.fig22()`, `morelli_coefficients`, `stevens_coefficients`, `x31.aero.body_force_moment` are unchanged. A committed converter generates the MOST31 files; runtime F-16/X-31 never calls the old evaluators.
- **X-31 plant step is FixedWing-owned.** New `python/x31_plant.py` mirrors the port's plant call order and swaps only the aero; the port stays byte-identical (SHA-256 pin test unchanged).
- **AeroBench angle factor is baked into the F-16 data.** The repo feeds Morelli `alpha * 57.29578 * pi/180` and Stevens `angle_rad * 57.29578` degrees; the translated files absorb that factor so MOST31 takes plain radians and existing F-16 parity tests stay green.

## Model

### Inputs (SI, radians)

`alpha, beta` [rad]; `de, da, dr, dc` (elevator, aileron, rudder, canard) [rad]; `p, q, r` [rad/s]; `V` [m/s] > 0; `b, cbar` [m]; `xcg, xcgref` [fraction of cbar].

`phat = p*b/(2V)`, `qhat = q*cbar/(2V)`, `rhat = r*b/(2V)`.

### Outputs and multipliers

Nine outputs, in this order: body `Cx, Cy, Cz, Cl, Cm, Cn`, wind `CL, CD, CY`.

Seven multipliers, in this order: `1, phat, qhat, rhat, da, dr, dc`.

For output `o`:

```text
C_o = sum over multiplier m of  [ P_o,m(alpha, beta, de) + T_o,m(alpha, beta, de) ] * m
```

### Polynomial part

```text
P_o,m = sum_{s,n,j,k} poly[o, m, s, n, j, k] * sgn(alpha)^s * |alpha|^n * beta^j * de^k
s in {0,1}, n in 0..12, j in 0..3, k in 0..3, sgn(0) = 0, x^0 = 1 (including 0^0).
```

- Signed `alpha^n` is slot `s = n mod 2`.
- X-31 even polynomial `c6 + sum_i c_i |a|^i` → `s = 0`; odd (flag 1) `c6 + sgn(a) sum_i c_i |a|^i` → constant at `s = 0, n = 0`, powers at `s = 1`. This equals the port's `-P + 2*c6` for negative angles.
- `n` reaches 12 because the X-31 canard terms `C_dc(alpha) * (dc - offset(alpha))` expand into a `dc`-multiplier slot plus the α-only product `-C_dc(alpha) * offset(alpha)` (two degree-6 polynomials). For a product of two sign-carrying factors, `sgn^2` maps to `s = 0` (exact for α ≠ 0; at α = 0 every such term has `|alpha|^n`, n ≥ 1, and is 0).

### Table part

Explicit strictly increasing radian breakpoints per axis. Interpolation is linear between breakpoints and **linear extrapolation from the end segment** outside the grid (this is what the AeroBench `fix`/clamp index logic computes). 2-D tables interpolate along α first, then the second axis (bilinear).

```text
T_o,m = sum_j table_a[o, m, j](alpha) * beta^j            j in 0..3
      + table_ae[o, m](alpha, de)
      + table_ab[o, m](alpha, beta)
      + table_aabsb[o, m](alpha, |beta|) * sgn(beta)
```

### Assembly

1. Evaluate the nine outputs.
2. CG shift on the **body set only** (both F-16 sources do this with their totals including rate terms):
   `Cm += Cz * (xcgref - xcg)`; `Cn -= Cy * (xcgref - xcg) * cbar / b`.
3. Wind set to body, the port's exact rule (`x31/aero.py`):
   ```text
   CX_w = CL*sin(a) - CY*sin(b) - CD*cos(a)*cos(b)
   CY_w =             CY*cos(b) - CD*cos(a)*sin(b)
   CZ_w = -CL*cos(a)            - CD*sin(a)
   ```
   (drag term is `rotate_body_to_earth(body_321_to_q(0, -a, b), [-CD, 0, 0])`, checked to 1e-16.)
4. Return body totals `(CX, CY, CZ, Cl, Cm, Cn) = (Cx + CX_w, Cy + CY_w, Cz + CZ_w, Cl, Cm, Cn)`.

Plants scale as today: force `qbar*S*[CX, CY, CZ]`, moment `qbar*S*[b*Cl, cbar*Cm, b*Cn]`.

## Schema (version 1)

JSON object:

| Key | Content |
|-----|---------|
| `schema` | `"MOST31"` |
| `version` | `1` |
| `source` | provenance string (source file/function, factors applied) |
| `outputs` | the nine output names above, in order |
| `multipliers` | the seven multiplier names above, in order |
| `grids` | `alpha_rad` (12), `elevator_rad` (5), `beta_rad` (7), `abs_beta_rad` (7) |
| `poly` | shape `(9, 7, 2, 13, 4, 4)` |
| `table_a` | `(9, 7, 4, 12)` |
| `table_ae` | `(9, 7, 12, 5)` |
| `table_ab` | `(9, 7, 12, 7)` |
| `table_aabsb` | `(9, 7, 12, 7)` |

Grid counts are part of version 1. All three committed files use the Stevens grids: α −10°..45° step 5°, elevator −24°..24° step 12°, β −30°..30° step 10°, |β| 0°..30° step 5°, each converted as `deg / 57.29578`.

Serialization is deterministic (fixed key order, Python `repr` floats) so regeneration can be compared byte-for-byte.

The loader raises `ValueError` naming the key on: wrong `schema`/`version`, missing key, wrong `outputs`/`multipliers` lists, wrong shape, non-finite value, non-increasing or wrong-length grid. `evaluate` raises `ValueError` for `V <= 0`.

## Translation

Converter in `python/most31/`; `cd python && python3 -m most31.build` writes:

- `data/planes/f16/most31_morelli.json` from `data/planes/f16/morelli.json`
- `data/planes/f16/most31_stevens.json` from `data/planes/f16/stevens.json`
- `data/planes/x31/most31.json` from `x31.params.fig22()` (the v0.3.1 table the runner flies)

With `r = 57.29578 * pi / 180` (AeroBench factor) and `k = 180 / pi`:

**Morelli → MOST31.** Each Morelli monomial goes to its slot under its multiplier (`Cx0, Cy0, Cz0, Cl0, Cm0, Cn0` → `1`; `Cxq, Czq, Cmq` → `qhat`; `Cyp, Clp, Cnp` → `phat`; `Cyr, Clr, Cnr` → `rhat`; `Clda, Cnda` and `Cy0`'s `da` term → `da`; `Cldr, Cndr` and `Cy0`'s `dr` term → `dr`). A monomial `alpha^n beta^j de^k` gets coefficient `c * r^(n+j)`. `Cz0`'s `(1 - beta^2)` factor becomes `+c` at `j = 0` and `-c * r^2` at `j = 2`.

**Stevens → MOST31.** `cx`, `cm` → `table_ae` under `1`. `cz` → `table_a` under `1` at `j = 0` and `-(57.29578/beta_norm_deg)^2 * cz` at `j = 2`; `cz_elevator * 57.29578 / elevator_norm_deg` → `poly` `(Cz, 1, k=1)`. `cl`, `cn` → `table_aabsb` under `1`. `dlda`, `dnda` → `table_ab` under `da` times `57.29578 / aileron_norm_deg`; `dldr`, `dndr` → `table_ab` under `dr` times `57.29578 / rudder_norm_deg`. `cy_beta * 57.29578` → `poly (Cy, 1, j=1)`; `cy_aileron * 57.29578 / aileron_norm_deg` → `(Cy, da)`; `cy_rudder * 57.29578 / rudder_norm_deg` → `(Cy, dr)`. Damping columns 0..8 → `table_a` `j = 0`: `(Cx, qhat), (Cy, rhat), (Cy, phat), (Cz, qhat), (Cl, rhat), (Cl, phat), (Cm, qhat), (Cn, rhat), (Cn, phat)`.

**X-31 Figure 2.2 → MOST31.** A polynomial coefficient on `|alpha_deg|^n` becomes `c * k^n` on `|alpha|^n`. Wind: `CL(alpha)` → `(CL, 1)`; `CD(alpha)` → `(CD, 1)`; `CY(alpha)*beta_rad` → `(CY, 1, j=1)`; `CYdr(alpha)` → `(CY, dr)` times `k`. Body moments: `Clb, Cnb` → `j = 1` under `1`; `Clp, Cnp` → `phat`; `Clr, Cnr` → `rhat`; `Clda, Cnda` → `da` times `k`; `Cldr, Cndr` → `dr` times `k`; `Cm0` → `(Cm, 1)`; `Cmq` → `(Cm, qhat)`. Canard: `CLdc(alpha) * k` → `(CL, dc)`, `-CLdc(alpha) * lift_offset_deg(alpha)` added to `(CL, 1)`; same for `Cmdc` / `moment_offset` into `Cm`. `CYda` is 0 in the port and stays 0.

## Integration

**F-16.** `f16/model._coefficients(model, ...)` evaluates MOST31 from `data/planes/f16/most31_{model}.json` (loaded once, cached), with `dc = 0`, `xcg = xcgref = 0.35`. `--aero morelli|stevens` is unchanged on the CLI; any other value still raises `ValueError` (runner exits 2). `model.py` no longer imports `morelli_coefficients` / `stevens_coefficients`.

**X-31.** New `python/x31_plant.py` (FixedWing code, not port code):

- `rates(t, state, surface)` — the port's `dynamics.rates` sequence with `body_force_moment` replaced by MOST31 from `data/planes/x31/most31.json`; reuses the port's `_air_data`, `_thrust_body`, `_inertia`, `physical`, quaternion functions, `_RHO`, `_G`. Surfaces in degrees are converted to radians; `de = 0`; `xcg = xcgref`.
- `derivative(t, state, surface)` — `rates` plus the 85° α / 80° β limit, raising the port's `dynamics.AngleLimitError`.
- `plant_sample(t, pos, vel, q, w, act)` and `plant_rhs(t, y, n_act, command)` — mirrors of `simulate._plant_sample` / `simulate._plant_rhs`.

`x31_sim.py` uses these at its three plant sites (closed-loop RHS, open-loop RHS, CSV logging pass), and `x31_guidance.py` at its three `_plant_sample` sites (initial sense, RHS, logging). Controllers are untouched.

## Proof

TDD per task. Tolerances: coefficients `|Δ| <= 1e-12 * max(1, |ref|)`; plant derivatives per component `|Δ| <= 1e-10 * max(1, |ref|)`.

- Schema: loader accepts a valid all-zero file and rejects each malformed case above.
- Evaluator: hand-built single-slot files prove each basis term, each table kind (inside grid, below, above), `sgn(0) = 0`, CG shift, and the wind→body rule.
- Morelli, Stevens, X-31: translated coefficients match the old evaluator on a grid of α −30°..85° (includes Stevens extrapolation below −10° and above 45°), β ∈ {−30°, −7°, 0, 4°, 30°}, every surface at −limit/0/+limit and one interior value, nonzero `p, q, r`.
- Files: `python3 -m most31.build` into a temp dir equals the committed files byte-for-byte.
- F-16: `subf16_derivative` on MOST31 matches the old path for `morelli` and `stevens` at trim and at perturbed states.
- X-31: `x31_plant.rates` matches `dynamics.rates` at trim and perturbed states, for nonzero canard/aileron/rudder/thrust vectoring. Because `dynamics.rates` uses the same table the controllers read, this also pins controller–plant agreement.
- All existing F-16 and X-31 tests pass unchanged, including `test_x31_vendored_port.py`.
- `README.md` architecture rows for `python/f16/`, `python/x31_sim.py`, `python/x31_guidance.py`, and a new `python/most31/` row; `UPDATES.md` minor bump.
