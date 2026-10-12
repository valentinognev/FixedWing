# Flightbench — Analysis Readability Design

Date: 2026-10-12
Status: written with the implementation plan for review. Implements the five requests on the linear-analysis view. Extends `docs/superpowers/specs/2026-10-10-flightbench-linear-analysis-design.md`.

## Goal

On every linear-law task, the results show the feedback path, a readable open-loop card, a Bode plot of that open-loop channel, and a gain slider on each root-locus chart and each Nyquist chart. Every chart on the page can be panned, boxed-zoomed, and returned to its first view, and a box zoom stretches the axes independently. LQR and NDI runs stay without `linear_analysis`.

The page still draws what the run returns. It does not form a transfer function. A slider only walks data the run already returned, or scales a Nyquist series that is proportional to its gain.

## Decisions

- The pitch-disturbance screenshot is the example. The same components render all eleven tasks. No task gets its own diagram, card, or chart.
- Return wires already exist in `diagram.py` (`plant` → error sum, `plant` → rate block). The defect is the drawing: center-to-center segments sit under the boxes. The data model stays.
- Feedback is a polyline on a lane under that loop. Two return wires in one loop use two lanes. The drop from the shared airframe stays in the gap to the left of the airframe, so a lower loop’s boxes are clear of it.
- Return wires are sky (`#38bdf8`). Forward wires stay gray (`#737373`). A `+` or `−` sits at the destination end of its wire.
- Open-loop text keeps 6 significant figures. Color and size come from `data-role` rules in `index.css`, with a light-theme override for each role. Tailwind amber/emerald utilities are not used: light mode remaps only the neutral ramp, so those utilities stay the dark-theme colors on a light page.
- A matrix entry with absolute value `≤ 1e-8` is the zero role. Any larger positive entry is positive; any larger negative entry is negative.
- A pole with real part `< -1e-8` is stable, `> 1e-8` is unstable, and the rest sit on the axis. A conjugate pair shares that role. Zeros use the zero role.
- The open-loop plot is the Bode chart of the card’s `G(jω)`: magnitude in dB and phase in degrees, same 400-point grid as Nyquist (`1e-2` to `1e2` rad/s). One chart per card. `trim_cruise` has two.
- `G(jω)` is the plant channel. It has no controller coefficient, so the Bode chart has no gain slider.
- Each locus chart has a slider over that coefficient’s `locus.values`. The mark is the branch sample at the slider index. The gain form still does not move the mark; only the slider and a new Run do.
- Each Nyquist chart has its own slider, starting at the same index. For a proportional coefficient, displayed `L` is the returned series times `sliderGain / runGain`. `lead_zero`, `lead_pole`, and `tau_w` are not proportional: the Nyquist slider is disabled and the series stays the returned samples. A run gain of `0` also leaves the series unscaled.
- Editing a gain and pressing Run is still how a new analysis is computed.
- Time-series charts, locus charts, Nyquist charts, and Bode charts share one Plotly config. The mode bar shows Zoom, Pan, Zoom in, Zoom out, Autoscale, and Reset axes. Reset axes is the back control: it restores the ranges the chart opened with. The Plotly logo, lasso, and box-select stay off. Wheel zoom is on.
- No axis sets `scaleanchor` or `scaleratio`. Locus and Nyquist drop the current `scaleanchor: 'x'` and `scaleratio: 1`, so a box zoom is free rather than square. Bode’s magnitude and phase axes stay independent of each other and of the frequency axis. Time charts already omit the anchor; they keep the shared time range as the range Reset axes returns to.

## Roles

| `data-role` | Size | Dark | Light |
|---|---|---|---|
| `card-title` | 18px semibold | `#38bdf8` | `#0369a1` |
| `matrix-a` | 18px bold | `#fbbf24` | `#b45309` |
| `matrix-b` | 18px bold | `#38bdf8` | `#0369a1` |
| `matrix-c` | 18px bold | `#34d399` | `#047857` |
| `matrix-d` | 18px bold | `#a78bfa` | `#6d28d9` |
| `axis-name` | 14px semibold | `#fcd34d` | `#92400e` |
| `value-positive` | 14px | `#34d399` | `#047857` |
| `value-negative` | 14px | `#fb7185` | `#be123c` |
| `value-zero` | 14px | `#737373` | `#737373` |
| `tf-label` | 18px semibold | `#38bdf8` | `#0369a1` |
| `tf-num` | 16px | `#34d399` | `#047857` |
| `tf-den` | 16px | `#fbbf24` | `#b45309` |
| `pole-stable` | 16px | `#34d399` | `#047857` |
| `pole-unstable` | 16px | `#fb7185` | `#be123c` |
| `pole-axis` | 16px | `#fbbf24` | `#b45309` |
| `zero` | 16px | `#38bdf8` | `#0369a1` |

`G(s) =` stays in the parent paragraph so the existing `/G\(s\)/` query still sees the ratio. Numerator and denominator are inner spans.

## API additions

`open_loop[]` gains:

```text
bode: { omega, magnitude_db, phase_deg }   # equal length; dropped samples omitted
```

`coefficients[]` gains `proportional: boolean`. False only for `lead_zero`, `lead_pole`, and `tau_w`.

Bode uses `G(jω) = C (jωI − A)^{-1} B + D`. A non-finite frequency, a singular resolvent, a non-finite sample, or a sample with `|G| = 0` is omitted. Phase is `atan2(imag, real)` in degrees, in `(-180, 180]`.

## Out of scope

Time-history trace colors. A slider that re-runs the plant or rebuilds the locus. Asymptotes, margins, and Nichols charts. LQR and NDI diagrams.
