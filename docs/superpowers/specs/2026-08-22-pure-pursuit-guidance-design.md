# Pure-pursuit guidance: accel → attitude → thrust/speed

Date: 2026-08-22  
Status: approved (pipeline approach; JSONC plants; polar default + geometric; AoA hybrid)

## Goal

Replace balloon-race **on-target** attitude chase (LOS bank/pitch look-at + `thrust_for_hold` / `chase_speed_mps`) with a separable pipeline:

1. **Desired acceleration** from a pluggable law (v1: 3D pure pursuit).
2. **Desired quaternion** + axial accel from \(\mathbf{a}_\mathrm{des}\) (polar and geometric translators).
3. **Required thrust** from longitudinal dynamics with quadratic drag; **speed governor** when thrust saturates or airspeed is near stall.

Off-screen path-hold and straight-flight remain unchanged in v1.

## Non-goals

- Replacing path-hold / straight-flight with pure pursuit.
- Online aero identification or wind estimation.
- `cmd_mode=rates`.
- Matching miss distances of the old LOS law (PP will be re-tuned via plant files).
- Strict RFC8259 JSON (comments are required → JSONC).

## Decisions (locked)

| Topic | Choice |
|-------|--------|
| Architecture | Pipeline modules (`AccelLaw` → `AttitudeFromAccel` → `ThrustEnergy`); wire into `AttitudeChaseController` when chasing a balloon |
| Accel law (v1) | \(\mathbf{a}_\mathrm{des} = k\,(\hat u - \hat v)\); \(a_\parallel\) feeds thrust; \(\mathbf{a}_\perp\) feeds attitude |
| Attitude | Both polar (Euler) and geometric (R→q); plant flag; **default polar** |
| \(\psi^c\) | LOS azimuth (fallback: velocity heading if LOS nearly vertical) |
| AoA | \(\alpha \approx \theta - \gamma\); if \(\|\alpha\| < \alpha_\mathrm{small}\): \(\cos\alpha\approx 1\), small-\(\alpha\) \(C_D\) |
| Drag | \(D = \tfrac12\rho V^2 S\,C_D(\alpha)\). Small-\(\alpha\): \(C_D\approx C_{D0}\) (or \(C_{D0}+k_\mathrm{ind}C_L^2\) with \(C_L\) from load \(\|\mathbf{a}_\mathrm{des}-\mathbf{g}\|\,m/(qS)\)). Else: \(C_L=C_{L\alpha}\alpha\) (plant `cl_alpha`) then same \(C_D\) polar |
| Thrust | Use PP \(a_\parallel=\mathbf{a}_\mathrm{des}\cdot\hat v\) as \(\dot V_a\) (not body \(a_\mathrm{axial}\)). \(T=(m a_\parallel+D+mg\sin\gamma)/\cos\alpha\); PX4 fraction \(T/T_\mathrm{max}\). Body \(a_\mathrm{axial}\) is diagnostic only |
| Speed vs thrust | If \(T>T_\mathrm{max}\): lower \(V\) until \(T=0.8\,T_\mathrm{max}\), floor \(1.2 V_\mathrm{stall}\). If \(V<1.1 V_\mathrm{stall}\): raise to \(1.2 V_\mathrm{stall}\) (beats homing). If \(T<0.8\,T_\mathrm{max}\): ramp \(V\) up toward cruise at `v_up_mps_s` |
| Plant data | One **JSONC** file per plant under `python/fw_sitl/plants/`; comments on every parameter; loader feeds `PlantGains` |
| HSV vs geom | Still selects \(\hat u\) only; PP does not use bank look-at |

## Architecture

```text
LOS û, velocity v̂ / V, q_act, plant
        │
        ▼
┌────────────────────┐
│ AccelLaw (PP v1)   │  a_des = k (û − v̂)
└─────────┬──────────┘
          │ a_des
          ▼
┌──────────────────────────────┐
│ AttitudeFromAccel            │  polar (default) | geometric
│ → q_des, a_axial             │
└─────────┬────────────────────┘
          │
          ▼
┌──────────────────────────────┐
│ ThrustEnergy (stateful Vcmd) │
│ α, D(α), T, speed governor   │
└─────────┬────────────────────┘
          │ q_des, thrust_frac, V_cmd
          ▼
  SET_ATTITUDE_TARGET (existing MAVLink path)
```

Off-screen: existing `q_des_from_path` + attitude PID + `thrust_for_hold`.

## Components

### `python/fw_sitl/plants/<plant_id>.jsonc`

One file per `KNOWN_PLANT_IDS` entry. Migrates today’s `plant_gains.py` tables and adds aero / PP / governor fields. Every key has a `//` or `/* */` comment explaining meaning and units.

Required new fields (names may match loader schema exactly):

- `mass_kg`, `wing_area_m2`, `cd0`, `k_induced`, `cl_alpha` (\(C_{L\alpha}\) per rad; used when \(|\alpha|\ge\alpha_\mathrm{small}\))
- `rho_kg_m3` — air density for \(q=\tfrac12\rho V^2\) (default sea-level `1.225` if present in file)
- `t_max_n` — max thrust (N); fraction = \(T / t_\mathrm{max\_n}\)
- `v_stall_mps`
- `pp_gain` — \(k\) in \(a = k(\hat u-\hat v)\)
- `thrust_target_frac` — default `0.8`
- `v_min_mult` — default `1.1` (forbidden band below this × stall)
- `v_recover_mult` — default `1.2` (recover / thrust-limit floor)
- `v_up_mps_s` — gradual speed-up when under target thrust fraction
- `attitude_from_accel` — `"polar"` | `"geometric"` (default `"polar"`)
- `alpha_small_rad` — threshold for small-\(\alpha\) shortcuts
- Existing PID / bank / thrust frac clamps / speeds / `FW_*` overlay unchanged in meaning

### `fw_sitl/plant_loader.py`

- Strip JSONC comments → `json.loads` → validate → `PlantGains`.
- Unknown plant id fails loud (same contract as today).
- `plant_gains.py` becomes a thin façade: `get_plant(id)` delegates to loader; keep helper methods (`path_kwargs`, `make_pid`, …) so callers stay stable.

### `fw_sitl/accel_laws.py`

- Protocol / callable: `(u_hat, v_hat, *, gain) -> a_des` (NED).
- `PurePursuitAccel`: \(\mathbf{a}_\mathrm{des} = k(\hat u - \hat v)\).
- Helpers: split \(a_\parallel = \mathbf{a}\cdot\hat v\), \(\mathbf{a}_\perp = \mathbf{a} - a_\parallel\hat v\).
- Future laws implement the same surface without touching attitude/thrust.

### `fw_sitl/attitude_from_accel.py`

- **Polar:** vehicle-1 / vehicle-2 projection of \(\mathbf{a}_\mathrm{des}-\mathbf{g}\); \(\theta^c=\mathrm{atan2}(A_x^{v1},-A_z^{v1})\); \(\phi^c=\mathrm{atan2}(A_y^{v2},-A_z^{v2})\); \(\psi^c\) from caller; 3-2-1 → \(q^c\); axial accel along body \(x\) as in the approved sample.
- **Geometric:** \(\mathbf{k}^d = -(\mathbf{a}_\mathrm{des}-\mathbf{g})/\|\cdot\|\); heading \(\mathbf{i}_\mathrm{head}=(\cos\psi^c,\sin\psi^c,0)\); complete \(R^d\); extract quaternion; \(a_\mathrm{axial}=(\mathbf{a}_\mathrm{des}-\mathbf{g})\cdot\mathbf{i}^d\).
- Dispatcher reads plant `attitude_from_accel`.
- After Euler extract (or from \(q\)): clamp roll/pitch to plant max roll / max pitch.

### `fw_sitl/thrust_energy.py`

Inputs: PP \(a_\parallel=\mathbf{a}_\mathrm{des}\cdot\hat v\) as \(\dot V_a\), commanded/current \(V\), \(\gamma\), \(\theta\), \(\mathbf{a}_\mathrm{des}\) (for load-factor \(C_L\) when small-\(\alpha\)), plant aero, previous \(V_\mathrm{cmd}\), dt.

\[
\dot V_a = a_\parallel,\quad
\alpha \approx \theta - \gamma,\quad
T = \frac{m\,\dot V_a + D(\alpha) + mg\sin\gamma}{\cos\alpha}
\]

with \(D=\tfrac12\rho V^2 S\,C_D(\alpha)\). \(\rho\) from plant `rho_kg_m3` (seed `1.225`).

Speed governor (order):

1. If \(V < 1.1\,V_\mathrm{stall}\): set \(V_\mathrm{cmd} \leftarrow 1.2\,V_\mathrm{stall}\) (hard priority).
2. Else compute \(T(V_\mathrm{cmd})\). If \(T > T_\mathrm{max}\): reduce \(V_\mathrm{cmd}\) until \(T = 0.8\,T_\mathrm{max}\) or hit floor \(1.2\,V_\mathrm{stall}\).
3. Else if \(T < 0.8\,T_\mathrm{max}\): \(V_\mathrm{cmd} \leftarrow \min(V_\mathrm{cruise}, V_\mathrm{cmd} + v_\mathrm{up}\,dt)\).
4. Recompute \(T\) at final \(V_\mathrm{cmd}\); output thrust fraction clamped to plant `min_thrust`/`max_thrust`.

### Wiring (`AttitudeChaseController`)

When chasing balloon (`in_view` / current LOS chase branch):

1. Build \(\hat u\) from `dir_ned`; \(\hat v\), \(V\) from velocity (NED); \(\gamma=\mathrm{asin}(-v_d/\max(|v|,\epsilon))\); \(\theta\) from `q_act`.
2. `a_des = PurePursuitAccel(...)`.
3. `q_des, a_axial = attitude_from_accel(...)`.
4. Keep existing roll/pitch LPF + slew on commanded Euler if still useful for PX4 tracking.
5. `thrust, V_cmd = thrust_energy(...)` (controller holds governor state).
6. `send_attitude_target`; set `last_law` to `"pp"` (or `"pp_polar"` / `"pp_geom"`).

Do not call `q_des_from_los` / `chase_speed_mps` on that branch.

## Error handling

- Near-zero \(\|v\|\): hold last \(\hat v\) / skip PP update; do not command NaN.
- Near-zero \(\|\mathbf{a}_\mathrm{des}-\mathbf{g}\|\) (geometric): fall back to polar or identity attitude from heading.
- \(\cos\alpha\) near zero: clamp \(\alpha\) to plant-safe range before dividing.
- Missing plant file or schema field: fail at load with a clear error (no silent defaults for aero).

## Testing

- Unit PP: aligned \(\hat u=\hat v\) → \(\|\mathbf{a}\|\approx 0\); offset → lateral component dominates.
- Polar vs geometric on mild coordinated climb: high \(|q_1\cdot q_2|\); axial accel close.
- Thrust: level \(a_\parallel=0,\gamma=0\) → \(T\approx D\); climb increases \(T\).
- Governor: over-thrust lowers \(V\) to 0.8 band; never below \(1.2 V_s\); under \(1.1 V_s\) recovers; under-thrust ramps \(V\) up.
- JSONC: each plant loads; comments ignored; unknown id fails.
- Controller: balloon chase sets `last_law` starting with `pp`; path branch unchanged.

## Migration

1. Add JSONC plants + loader; keep `get_plant` working.
2. Implement accel / attitude / thrust modules + unit tests.
3. Switch `AttitudeChaseController` LOS branch to pipeline.
4. Retune `pp_gain` / aero per plant from live races (separate from this design).
5. Remove or leave unused `q_des_from_los` until path/viz callers are clear (do not delete in first PR if tests still import it).

## Open calibration (not blocking design)

Numeric `mass_kg`, \(S\), \(C_{D0}\), \(T_\mathrm{max}\), \(V_\mathrm{stall}\) start as plausible per-airframe seeds in each JSONC; live miss/energy tuning follows implementation.
