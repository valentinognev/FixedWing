# Flightbench — Linear Analysis Design

Date: 2026-10-10
Status: design approved in chat; this file is the written spec for review before the implementation plan.

## Goal

On a `linear`-law run, the right-hand results show the controller that flew, the linear plant it closes, and how each of its coefficients shapes that loop. LQR and NDI runs are unchanged: time histories and the modes table.

The page renders what the run returns. It does not form a transfer function, a locus, or a Nyquist curve itself.

## Decisions (locked)

- Computed with the run, at that run's trim and gains. Editing a gain does not refresh the block until the next Run.
- Open-loop cards are the plant alone, one channel per task (`trim_cruise` has two). Yaw orientation is the exception below.
- One root locus and one Nyquist plot per coefficient of the task. The other coefficients stay at the run's values.
- The locus and the Nyquist use the same controller object the time histories use (`closed_loop_matrix` and the task builder). No second block diagram in algebra, and no `python-control` dependency.
- A controller diagram is drawn from that same topology, with the run's gain values written in the blocks.

## Where it appears

Under the time histories, for `law == "linear"` only, in this order:

1. Controller diagram.
2. Open-loop cards.
3. One root-locus chart and one Nyquist chart per coefficient, in the task's gain order.

`RunResponse` gains a `linear_analysis` object on a `linear` run. LQR and NDI omit the field, and the page draws nothing for it.

## Open-loop cards

One card per row. `trim_cruise` has both rows and they share `A`.

| Task | Input | Output | Model |
|---|---|---|---|
| `pitch_disturbance`, `lead_pitch` | pitch | θ | longitudinal plant |
| `short_period_phugoid` | pitch | q | longitudinal plant |
| `acceleration` | pitch | nz | longitudinal plant |
| `airspeed` | throttle | vt | longitudinal plant |
| `steady_descent` | pitch | γ | longitudinal plant |
| `trim_cruise` | pitch, and throttle | θ, and vt | longitudinal plant |
| `dutch_roll` | yaw | r | lateral plant |
| `turn_coordination`, `sideslip_turn` | roll | φ | lateral plant |
| `yaw_orientation` | r_cmd | ψ | lateral plant with the inner loops closed |

The longitudinal plant is `linearize.longitudinal`: states `(vt, alpha, theta, q, altitude)` plus the plane's engine extras, inputs `(pitch, throttle)`. The lateral plant is `linearize.lateral`: states `(beta, phi, psi, p, r)`, inputs `(roll, yaw)`. A card takes the subsystem `A`, the column of `B` for its input, the row `C` for its output, and `D`. In the JSON, `B` is `n×1`, `C` is `1×n`, and `D` is `1×1`. Poles and zeros are ordered by descending real part, then descending imaginary part.

`C` and `D` for an output that is a state are the unit row and `D = 0`. `γ` and `nz` use `measure.linear_measurement`: linear `γ = theta − alpha`, and `nz = (vt0/g)·(q − alpha_dot_free)` with `alpha_dot_free = (A dx)[alpha]`. Both have `D = 0`, because that `nz` holds the channels at trim.

**Yaw orientation.** Bank (`kp_phi`, `ki_phi`, `kp_p`), sideslip (`k_beta`, `k_betadot`), and yaw-rate tracking (`kr_track`) stay closed at the run's gains. `kp_psi` stays open: the card's input is `r_cmd`. The state vector is the lateral plant plus the bank PI state (the only inner-loop state on this task). At the zero deviation the ±30° bank clip is not binding, so the card uses `φ_ref = (vt0/g)·r_cmd`. The diagram still draws the clip.

Each card shows:

- `A`, `B`, `C`, `D` with state names, the input name, and the output name. Units are the model's (radians, seconds).
- The minimal Laplace form `G(s) = N(s)/D(s)`. A pole/zero pair is cancelled when `|p − z| ≤ 1e-6·max(1, |p|)`. The denominator is monic. The API sends full-precision coefficients; the page prints 6 significant figures.
- The poles and zeros of that cancelled `G(s)`, as real numbers or conjugate pairs. The matrices stay the unreduced system, so `A` can contain a mode the transfer function has cancelled.

## Controller diagram

One diagram per task, SVG from `linear_analysis.diagram`. The airframe is one block. Rectangles carry the current expression; circles are sums; a wire into a sum carries `+` or `−`. Expressions use 6 significant figures:

- PI: `kp + ki/s`
- Lead: `(p/z)·(s+z)/(s+p)` with the run's `lead_pole` and `lead_zero`
- Washout: `s/(s+1/τ)` with the run's `tau_w`
- A plain gain: its value

The lead sits on the command, ahead of the error sum, which is where `tasks` puts it: `e = Lead(command) − measured`.

| Task | Diagram |
|---|---|
| `pitch_disturbance` | `θ_ref` into a PI, minus `θ`, then minus `kq·q`, into pitch. A +1° step sums onto the pitch channel. |
| `short_period_phugoid` | `q` through `kq` only, minus, into pitch. The doublet sums onto that channel. No PI. |
| `lead_pitch` | `θ_ref` through the lead, then the PI and `kq` loop above. |
| `trim_cruise` | The pitch loop, plus a `Δpitch` step summed at the pitch output, and a speed PI on `V_ref − V` with a `Δthrottle` step summed at the throttle output. |
| `airspeed` | Pitch loop holding trim `θ`, and a speed PI on `V_ref − V` into throttle. |
| `acceleration` | PI on `nz_ref − nz`, minus `kq·q`, into pitch, and a speed PI holding trim `V`. |
| `steady_descent` | PI on `γ_ref − γ` produces `θ_ref` for the pitch loop. A speed PI holds trim `V`. |
| `dutch_roll` | `r` through the washout and `kr`, minus, into yaw. The +2° pulse sums onto yaw. |
| `turn_coordination` | Bank PI minus `kp_p·p` into roll. Yaw sums the washout damper, `k_beta·β`, and `k_ari` times the roll command. |
| `sideslip_turn` | The same bank and damper. Yaw sums `k_beta·β`, `ki_beta/s`, and `k_betadot·β̇_est`. |
| `yaw_orientation` | `kp_psi·(ψ_ref − ψ)` is `r_cmd`. That becomes `φ_ref = clip((vt0/g)·r_cmd, ±30°)` for the bank loop, and `kr_track·(r_cmd − r)` on yaw, together with `k_beta·β` and `k_betadot·β̇_est`. |

`diagram` is a list of loops. Each loop has `input`, `output`, `feedback`, `nodes` (`id`, `kind`, `label`, `text`), and `wires` (`from`, `to`, `sign`). `feedback` is the primary measured signal of that loop (`θ`, `q`, `nz`, `V`, `γ`, `r`, `φ`, or `ψ`); a rate or cross-feed path is only wires. `kind` is `input`, `sum`, `block`, or `plant`. Disturbance arrows, the trim-cruise feed-forward steps, and the yaw-orientation clip are `block` nodes. The plant node id is `plant` on every loop, and the page draws that box once. A task with no gain of that name omits its block (pitch tasks other than `lead_pitch` have no lead).

## Root locus

For an ordinary coefficient the closed-loop matrix is affine, `M(K) = M(0) + K·(M(1) − M(0))`. `M(0)` and `M(1)` come from `engine.closed_loop_matrix` with that coefficient set to 0 and to 1 and the others left at the run's values. Ordinary coefficients are every task gain except `lead_zero`, `lead_pole`, and `tau_w`.

The sweep is the unique sorted union of `0`, the current value `K`, and 80 values spaced logarithmically from `1e-3·S` to `1e3·S`, where `S = max(|K|, 1)`. A negative `K` adds the same logarithmic grid on the negative side.

`lead_zero`, `lead_pole`, and `tau_w` are not affine. Each grid point rebuilds the controller. The sweep is 80 values spaced logarithmically from `0.05·K` to `20·K`, the union with `K`, and every value positive. A lead point that would set `lead_zero == lead_pole` is dropped, because that equality removes the lead state and the branch count would change.

Branches are joined greedily by nearest eigenvalue in the complex plane, taking source eigenvalues in order of increasing real part and then increasing imaginary part. A tie takes the unmatched target with the smaller real part, then the smaller imaginary part. The current value is marked.

The command-path lead does not put `lead_zero` into the closed-loop state matrix: the lead state is driven by the command, and the plant does not feed it. A `lead_zero` locus therefore does not move. On a `lead_pole` locus, at every grid value `p` one eigenvalue equals `−p`, and the plant eigenvalues stay put. Both loci are still drawn. `tau_w` moves the washout eigenvalue and the branches that couple through `r`.

The run uses the affine formula directly. The match against a direct `closed_loop_matrix` is the synthetic test below, not a second Jacobian on each request.

## Nyquist

A coefficient `K` contributes the term `c(K)·σ` in the control law. An external input `v` replaces that term. With `Σ(s)/V(s)` the transfer function from `v` to `σ`,

```text
L(s) = −c(K) · Σ(s)/V(s)
```

so the loop closes as `1 + L(s) = 0` and the critical point is −1. The broken-loop state-space is the same Jacobian, with one extra input column for `v` and one output row for `σ`. `L(jω)` is evaluated by a linear solve at each frequency.

`c(K)` and `σ`:

| Coefficient | `c(K)` | `σ` |
|---|---|---|
| `kp_theta`, `kp_phi`, `kp_nz`, `kp_gamma`, `kp_v`, `kp_psi` | `+K` | the loop error (`ψ_ref − ψ` for `kp_psi`; `v` replaces `r_cmd`) |
| `ki_theta`, `ki_phi`, `ki_nz`, `ki_gamma`, `ki_v`, `ki_beta` | `+K` | that loop's integrator state |
| `kq` | `−K` | `q` |
| `kp_p` | `−K` | `p` |
| `kr` | `−K` | washout output |
| `k_beta` | `+K` | `β` |
| `k_betadot` | `+K` | `β̇_est` |
| `k_ari` | `+K` | the roll command |
| `kr_track` | `+K` | `r_cmd − r` |

`ω` is 400 values spaced logarithmically from `1e-2` to `1e2` rad/s. Samples with `|L| > 1e6` are omitted.

`lead_zero` and `lead_pole` have no injection of their own. Both Nyquist series are the pitch-loop return ratio at the current lead, which is the `kp_theta` series. `tau_w` has none either: its series is the washout-path return ratio at the current `τ` and `kr`, which is the `kr` series. The page still draws one chart per coefficient.

## API

`linear_analysis` on a `linear` `RunResponse`:

```text
linear_analysis = {
  open_loop: [{
    input, output, states,
    A, B, C, D,                  # nested lists, full precision
    numerator, denominator,      # descending powers, denominator monic
    poles, zeros                 # [{real, imag}, ...], cancelled pairs removed
  }],
  diagram: [{
    input, output, feedback,
    nodes: [{id, kind, label, text}],
    wires: [{from, to, sign}]    # sign is "+", "-", or ""
  }],
  coefficients: [{
    name,
    locus: {values, branches},   # branches[i] is [{real, imag}, ...] over values
    nyquist: {omega, real, imag}
  }]
}
```

`coefficients` follows the task's gain order. `locus.values` is the sweep in ascending order. Each branch has the same length as `values`.

## Failures

A gain the run accepted is finite, and a lead zero or pole the run accepted is positive, so this block adds no new rejection. A non-finite locus sample or frequency sample is dropped from that series. The time histories still return. A coefficient whose series is empty after that drop still appears, with empty arrays, and the page leaves that chart blank.

## Tests

- A fixed state-space model: numerator, denominator, poles, and zeros match the known transfer function, including one cancelled pair that must disappear from `G(s)` and must remain an eigenvalue of the returned `A`.
- One ordinary gain on a synthetic affine loop: after nearest-eigenvalue pairing, each locus point at the current value is within `1e-6·max(1, |λ|)` of the direct closed-loop matrix.
- One frequency on that loop: `L(jω)` is within `1e-6·max(1, |L|)` of the solve `(jωI − A)⁻¹` for the broken loop, on both real and imaginary parts.
- `lead_zero` on the lead-pitch topology: every locus branch is constant. `lead_pole`: at each grid value `p`, one eigenvalue equals `−p`.
- Each of the eleven tasks: the diagram contains that task's gain names and the numeric values from the request, and omits a block the task does not close. The `trim_cruise` diagram text contains both feed-forward values.
- API: a `linear` run includes `linear_analysis`; an LQR run does not.
- Page: a fixture renders the matrix state names, the Laplace text, the pitch-disturbance sums, the PI block, and the `kq` block, plus one locus chart and one Nyquist chart. Plotly stays mocked.

Python: `cd python && python3 -m unittest discover -s tests -p 'test_flightbench*.py'`. Web: `cd web/flightbench && npx vitest run`.

## Out of scope

LQR and NDI analysis and diagrams. Asymptotes, gain margins, and Nichols charts. A locus or Nyquist that updates without Run. Editing the linear model from the page.
