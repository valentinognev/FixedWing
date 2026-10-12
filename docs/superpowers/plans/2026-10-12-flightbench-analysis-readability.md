# Flightbench Analysis Readability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Show the airframe feedback on every controller diagram, make each open-loop card readable in both themes, plot `G(jω)` once per card, put a gain slider on every locus chart and every Nyquist chart, and give every chart a navigation bar whose box zoom is free of a locked aspect ratio.

**Architecture:** Diagram geometry moves into a pure layout function the SVG renders. Card color is `data-role` plus theme CSS. Bode samples are computed once inside `siso_card`, so all eleven tasks and both `trim_cruise` channels get them. Sliders only index the returned locus sweep or scale a proportional Nyquist series. Time, locus, Nyquist, and Bode charts all pass one `chartConfig()`, and no axis sets `scaleanchor`.

**Tech Stack:** Python 3 unittest, NumPy; React, TypeScript, Vitest, Testing Library, Plotly.

**Spec:** `docs/superpowers/specs/2026-10-12-flightbench-analysis-readability-design.md`

## Global Constraints

- All eleven linear tasks share `ControllerDiagram`, `OpenLoopCard`, and `CoefficientCharts`. A task-specific branch is a defect.
- LQR and NDI responses omit `linear_analysis`. The page draws none of this analysis for them.
- The page does not compute a transfer function, a locus, or a Nyquist curve. Bode samples come from the API.
- Gain-form edits do not move a chart. The slider and the next Run do.
- `lead_zero`, `lead_pole`, and `tau_w` set `proportional` false. Every other coefficient sets it true.
- A matrix entry with absolute value `≤ 1e-8` uses `value-zero`. Pole roles use the same threshold on the real part.
- Bode and Nyquist share `NYQUIST_OMEGA`: 400 log-spaced points from `1e-2` to `1e2` rad/s.
- Colors and sizes are the spec role table. Light overrides live under `html[data-theme='light']`.
- `Plot`, `ComplexPlot`, and `BodePlot` pass `chartConfig()`. The mode bar is Zoom, Pan, Zoom in, Zoom out, Autoscale, Reset axes. No axis sets `scaleanchor` or `scaleratio`.
- Implementers and reviewers use `grok-4.7-high` only.
- Leave the work uncommitted. Record the diff; do not run `git commit`.
- After the last task, `UPDATES.md` is `0.105.0` and the Flightbench sentence in `README.md` names the return lanes, the Bode chart, the sliders, and the chart navigation bar.

## Review Focus

- Two return wires in one loop (pitch: `plant → sum_e` and `plant → kq`) can draw on one lane. Expect lane Y values 16 px apart. Task 1.
- A return from the shared airframe can cut through a lower loop. Expect the vertical segment at `plant.x - 14`, to the right of every other box. Task 1.
- Light mode remaps only neutrals, so a dark-theme green utility fails on a light card. Expect the light hex from the role table in `index.css`. Task 2.
- Locus and Nyquist keep `scaleanchor: 'x'` and `scaleratio: 1`, so a box zoom stays square. Expect both properties absent on every chart axis, and the mode bar to list Pan, Autoscale, and Reset axes. Task 4. Bode joins them in Task 5.
- A run gain of 0 makes `slider / runGain` undefined. Expect scale `1`. Task 7.
- Scaling `lead_zero`, `lead_pole`, or `tau_w` draws a false Nyquist. Expect `proportional: false`, a disabled Nyquist slider, and unscaled samples. Task 7.

---

### Task 1: Return lanes on the controller diagram

**Files:**
- Create: `web/flightbench/src/diagramLayout.ts`
- Modify: `web/flightbench/src/components/ControllerDiagram.tsx`
- Test: `web/flightbench/src/__tests__/diagram-layout.test.ts`

**Interfaces:**
- Consumes: `DiagramLoop` from `web/flightbench/src/types.ts` (`nodes[].id/kind/label/text`, `wires[].from/to/sign`).
- Produces: `layoutDiagram(diagram: DiagramLoop[]): DiagramLayout`

```ts
export interface DiagramBox {
  key: string
  node: DiagramNode
  x: number
  y: number
  w: number
  h: number
}
export interface RoutedWire {
  from: string
  to: string
  sign: '' | '+' | '-'
  feedback: boolean
  points: { x: number; y: number }[]
  signAt: { x: number; y: number } | null
}
export interface DiagramLayout {
  width: number
  height: number
  boxes: DiagramBox[]
  wires: RoutedWire[]
}
```

`key` is `plant` for the airframe and `${loopIndex}:${id}` otherwise. `ControllerDiagram` renders `wires` as polylines. Feedback stroke `#38bdf8`, width `1.75`. Forward stroke `#737373`, width `1.25`. `marker-end` uses a matching arrow. `signAt` renders the sign in `#fbbf24` at 13 px.

- [x] **Step 1: Write the failing test**

```ts
import { describe, expect, it } from 'vitest'
import { layoutDiagram } from '../diagramLayout'
import type { DiagramLoop } from '../types'

function node(id: string, kind: DiagramLoop['nodes'][number]['kind'], label: string, text = '') {
  return { id, kind, label, text }
}

const PITCH: DiagramLoop[] = [
  {
    input: 'theta',
    output: 'pitch',
    feedback: 'theta',
    nodes: [
      node('theta_ref', 'input', 'theta_ref'),
      node('sum_e', 'sum', 'sum'),
      node('pi', 'block', 'PI', '2 + 0.5/s'),
      node('kq', 'block', 'kq', '0.5'),
      node('sum_rate', 'sum', 'sum'),
      node('disturbance', 'block', 'disturbance', '+1 deg'),
      node('sum_out', 'sum', 'sum'),
      node('plant', 'plant', 'airframe'),
    ],
    wires: [
      { from: 'theta_ref', to: 'sum_e', sign: '+' },
      { from: 'plant', to: 'sum_e', sign: '-' },
      { from: 'sum_e', to: 'pi', sign: '' },
      { from: 'pi', to: 'sum_rate', sign: '+' },
      { from: 'plant', to: 'kq', sign: '' },
      { from: 'kq', to: 'sum_rate', sign: '-' },
      { from: 'disturbance', to: 'sum_out', sign: '+' },
      { from: 'sum_rate', to: 'sum_out', sign: '+' },
      { from: 'sum_out', to: 'plant', sign: '' },
    ],
  },
]

describe('diagram return lanes', () => {
  it('routes both pitch returns under the row and keeps the forward wire on the edges', () => {
    const layout = layoutDiagram(PITCH)
    const sumE = layout.boxes.find((box) => box.key === '0:sum_e')
    const theta = layout.boxes.find((box) => box.key === '0:theta_ref')
    const plant = layout.boxes.find((box) => box.key === 'plant')
    expect(sumE).toBeDefined()
    expect(theta).toBeDefined()
    expect(plant).toBeDefined()
    if (sumE === undefined || theta === undefined || plant === undefined) return

    const returns = layout.wires.filter((wire) => wire.feedback)
    expect(returns.map((wire) => `${wire.from}->${wire.to}`)).toEqual([
      'plant->sum_e',
      'plant->kq',
    ])
    const laneY = returns.map((wire) => wire.points.find((point) => point.y > sumE.y + sumE.h)?.y)
    expect(laneY[1]! - laneY[0]!).toBe(16)
    expect(laneY[0]).toBeGreaterThan(sumE.y + sumE.h)

    const elbow = returns[0].points.find((point) => point.x === plant.x - 14)
    expect(elbow).toBeDefined()
    const rightmost = Math.max(
      ...layout.boxes.filter((box) => box.key !== 'plant').map((box) => box.x + box.w),
    )
    expect(plant.x - 14).toBeGreaterThan(rightmost)

    const forward = layout.wires.find((wire) => wire.from === 'theta_ref' && wire.to === 'sum_e')
    expect(forward?.points).toEqual([
      { x: theta.x + theta.w, y: theta.y + theta.h / 2 },
      { x: sumE.x, y: sumE.y + sumE.h / 2 },
    ])
    expect(returns[0].signAt?.y).toBeGreaterThan(sumE.y + sumE.h)
    const rateSign = layout.wires.find((wire) => wire.from === 'kq' && wire.to === 'sum_rate')
    const sumRate = layout.boxes.find((box) => box.key === '0:sum_rate')
    expect(rateSign?.signAt?.x).toBeLessThan(sumRate!.x)
  })

  it('drops a lower loop’s airframe return in the gap beside the airframe', () => {
    const second: DiagramLoop = {
      input: 'vt',
      output: 'throttle',
      feedback: 'vt',
      nodes: [
        node('sum_rate', 'sum', 'sum'),
        node('plant', 'plant', 'airframe'),
      ],
      wires: [{ from: 'plant', to: 'sum_rate', sign: '-' }],
    }
    const layout = layoutDiagram([PITCH[0], second])
    const plant = layout.boxes.find((box) => box.key === 'plant')!
    const lower = layout.boxes.find((box) => box.key === '1:sum_rate')!
    const upper = layout.boxes.filter((box) => box.key.startsWith('0:') || box.key === 'plant')
    const wire = layout.wires.find((item) => item.from === 'plant' && item.to === 'sum_rate')!
    expect(wire.feedback).toBe(true)
    expect(wire.points.some((point) => point.x === plant.x - 14)).toBe(true)
    const rightmost = Math.max(
      ...layout.boxes.filter((box) => box.key !== 'plant').map((box) => box.x + box.w),
    )
    expect(plant.x - 14).toBeGreaterThan(rightmost)
    const lane = wire.points.find((point) => point.y > lower.y + lower.h)!
    expect(lane.y).toBeGreaterThan(lower.y + lower.h)
    for (const box of upper) {
      if (box.key === 'plant') continue
      expect(lane.y).toBeGreaterThan(box.y + box.h)
    }
  })
})
```

- [x] **Step 2: Run test to verify it fails**

Run: `cd web/flightbench && npx vitest run src/__tests__/diagram-layout.test.ts`

Expected: FAIL with `layoutDiagram` not exported.

- [x] **Step 3: Implement `layoutDiagram`**

Keep today’s box sizes: `PAD = 20`, `GAP_X = 36`, `BOX_H = 64`, width `max(128, max(label.length, text.length, 4) * 8 + 24)`, sums are `BOX_H` wide. Place non-plant nodes left to right per loop. Count a wire as feedback when `from === 'plant'` or the source node’s index in that loop is greater than the destination’s. Row step is `BOX_H + 28` when that count is 0, otherwise `BOX_H + 12 + count * 16 + 12`. Place the single `plant` box to the right of the stack, vertically centered, as today.

Feedback polyline, lane `k` (wire order within the loop): `laneY = rowTop + BOX_H + 12 + k * 16`. From `plant`: left edge at the row’s mid-Y clamped into the plant, then `x = plant.x - 14`, down to `laneY`, across to the destination’s center X, up to the destination’s bottom center. From a same-row source: bottom center, down to `laneY`, across, up to the destination bottom. Forward, same row: right center to left center. Forward into `plant`: right center, elbow at `plant.x - 14`, into the plant’s left center. Sign `+`/`−` only: feedback at `(dest.cx + 14, dest.bottom + 11)`; forward at `(dest.x - 12, dest.cy)`. `width` and `height` include a 20 px pad past the furthest point or box.

`ControllerDiagram` draws this layout. Drop the center-to-center `line`.

- [x] **Step 4: Run test to verify it passes**

Run: `cd web/flightbench && npx vitest run src/__tests__/diagram-layout.test.ts src/__tests__/analysis.test.tsx`

Expected: PASS. The analysis test still finds one SVG, `2 + 0.5/s`, and a single `plant` label.

- [x] **Step 5: Leave the work uncommitted**

---

### Task 2: Color and size the open-loop card

**Files:**
- Modify: `web/flightbench/src/components/OpenLoopCard.tsx`
- Modify: `web/flightbench/src/index.css`
- Test: `web/flightbench/src/__tests__/open-loop-card.test.tsx`

**Interfaces:**
- Consumes: `OpenLoop` from `types.ts`. `formatCoeff` stays `toPrecision(6)`.
- Produces: `data-role` values from the spec table on the card title, each matrix caption, each axis name, each cell, `G(s)` numerator, `G(s)` denominator, each pole token, and each zero token. CSS rules for those roles, dark hex at the root and light hex under `html[data-theme='light']`.

- [x] **Step 1: Write the failing test**

```tsx
import { readFileSync } from 'node:fs'
import { describe, expect, it } from 'vitest'
import { render, screen } from '@testing-library/react'
import { OpenLoopCard } from '../components/OpenLoopCard'

const css = readFileSync(new URL('../index.css', import.meta.url), 'utf8')

describe('open-loop card roles', () => {
  it('paints signed entries, a tiny entry, G(s), and pole stability', () => {
    render(
      <OpenLoopCard
        card={{
          input: 'pitch',
          output: 'theta',
          states: ['theta', 'q'],
          A: [[6.0142, -9.80542], [-4.50919e-17, 0]],
          B: [[0.124826], [10.5916]],
          C: [[1, 0]],
          D: [[0]],
          numerator: [1, 0.25],
          denominator: [1, 2, 1],
          poles: [
            { real: 0.10333, imag: 0 },
            { real: -1.93302, imag: 0 },
            { real: -0.149519, imag: 0.128339 },
            { real: -0.149519, imag: -0.128339 },
          ],
          zeros: [{ real: -1.07248, imag: 0 }],
        }}
      />,
    )

    expect(screen.getByRole('heading', { name: 'pitch → theta' })).toHaveAttribute(
      'data-role',
      'card-title',
    )
    expect(screen.getByText('A')).toHaveAttribute('data-role', 'matrix-a')
    expect(screen.getByText('B')).toHaveAttribute('data-role', 'matrix-b')
    expect(screen.getByText('C')).toHaveAttribute('data-role', 'matrix-c')
    expect(screen.getByText('D')).toHaveAttribute('data-role', 'matrix-d')
    expect(screen.getByRole('columnheader', { name: 'theta' })).toHaveAttribute(
      'data-role',
      'axis-name',
    )
    expect(screen.getByText('6.01420')).toHaveAttribute('data-role', 'value-positive')
    expect(screen.getByText('-9.80542')).toHaveAttribute('data-role', 'value-negative')
    expect(screen.getByText('-4.50919e-17')).toHaveAttribute('data-role', 'value-zero')
    expect(screen.getByText(/G\(s\)/).textContent).toMatch(/1\.00000 s \+ 0\.250000/)
    expect(screen.getByText(/G\(s\)/).querySelector("[data-role='tf-num']")).not.toBeNull()
    expect(screen.getByText(/G\(s\)/).querySelector("[data-role='tf-den']")).not.toBeNull()
    expect(screen.getByText('0.103330')).toHaveAttribute('data-role', 'pole-unstable')
    expect(screen.getByText('-1.93302')).toHaveAttribute('data-role', 'pole-stable')
    expect(screen.getByText(/-0\.149519 ± 0\.128339j/)).toHaveAttribute('data-role', 'pole-stable')
    expect(screen.getByText('-1.07248')).toHaveAttribute('data-role', 'zero')
  })

  it('defines both theme colors and the larger title size', () => {
    expect(css).toMatch(/\[data-role='value-negative'\]\s*\{[^}]*color:\s*#fb7185/)
    expect(css).toMatch(/\[data-role='value-negative'\]\s*\{[^}]*font-size:\s*14px/)
    expect(css).toMatch(
      /html\[data-theme='light'\] \[data-role='value-negative'\]\s*\{[^}]*color:\s*#be123c/,
    )
    expect(css).toMatch(/\[data-role='matrix-a'\]\s*\{[^}]*font-size:\s*18px/)
    expect(css).toMatch(
      /html\[data-theme='light'\] \[data-role='matrix-a'\]\s*\{[^}]*color:\s*#b45309/,
    )
    expect(css).toMatch(/\[data-role='pole-unstable'\]\s*\{[^}]*color:\s*#fb7185/)
    expect(css).toMatch(
      /html\[data-theme='light'\] \[data-role='pole-unstable'\]\s*\{[^}]*color:\s*#be123c/,
    )
  })
})
```

Add a CSS rule for every role in the spec table, not only the ones this test names. Dark hex on the unscoped rule, light hex on the `html[data-theme='light']` rule, `font-size` and `font-weight` on the unscoped rule.

- [x] **Step 2: Run test to verify it fails**

Run: `cd web/flightbench && npx vitest run src/__tests__/open-loop-card.test.tsx`

Expected: FAIL because `data-role` is absent.

- [x] **Step 3: Implement the roles**

In `OpenLoopCard`, set `data-role` from the spec. Keep the polynomial and conjugate-pair text. Put `G(s) =` in the parent `<p data-role="tf-label">` and the numerator and denominator in child spans. One pole token, including a `±` pair, is one span. In `index.css`, add the role rules after the light-theme neutral block.

- [x] **Step 4: Run test to verify it passes**

Run: `cd web/flightbench && npx vitest run src/__tests__/open-loop-card.test.tsx src/__tests__/analysis.test.tsx`

Expected: PASS. `getByText(/G\(s\)/)` still contains the ratio.

- [x] **Step 5: Leave the work uncommitted**

---

### Task 3: Bode samples of G(jω)

**Files:**
- Create: `python/flightbench/analysis/bode.py`
- Test: `python/tests/test_flightbench_analysis_bode.py`

**Interfaces:**
- Consumes: `NYQUIST_OMEGA` from `flightbench.analysis.nyquist`.
- Produces: `bode_response(A, b, c, d, omega=None) -> dict` with keys `omega`, `magnitude_db`, `phase_deg`, equal-length lists of floats. Default `omega` is `NYQUIST_OMEGA`.

- [x] **Step 1: Write the failing test**

```python
import math
import unittest

import numpy as np

from flightbench.analysis.bode import bode_response
from flightbench.analysis.nyquist import NYQUIST_OMEGA


class TestBode(unittest.TestCase):
    def test_first_order_at_one_rad_per_s(self):
        # G(s) = 1/(s+1). |G(j1)| = 1/sqrt(2), phase = -45 deg.
        sample = bode_response([[-1.0]], [1.0], [1.0], 0.0, omega=np.array([1.0]))
        self.assertEqual(sample["omega"], [1.0])
        self.assertAlmostEqual(sample["magnitude_db"][0], 20.0 * math.log10(1.0 / math.sqrt(2.0)))
        self.assertAlmostEqual(sample["phase_deg"][0], -45.0)

    def test_default_grid_matches_nyquist_and_drops_a_singular_sample(self):
        sample = bode_response([[-1.0]], [1.0], [1.0], 0.0)
        self.assertEqual(len(sample["omega"]), len(NYQUIST_OMEGA))
        self.assertEqual(len(sample["omega"]), len(sample["magnitude_db"]))
        self.assertEqual(len(sample["omega"]), len(sample["phase_deg"]))
        dropped = bode_response([[0.0]], [1.0], [1.0], 0.0, omega=np.array([0.0, np.nan]))
        self.assertEqual(dropped, {"omega": [], "magnitude_db": [], "phase_deg": []})


if __name__ == "__main__":
    unittest.main()
```

- [x] **Step 2: Run test to verify it fails**

Run: `cd python && python3 -m unittest tests.test_flightbench_analysis_bode -v`

Expected: FAIL with `bode_response` import error.

- [x] **Step 3: Implement `bode_response`**

One `numpy.linalg.solve` per finite frequency, same resolvent as `nyquist` (`jωI − A`). `G = c·x + d`. Omit a singular solve, a non-finite `G`, and `|G| == 0`. `magnitude_db = 20 * log10(|G|)`. `phase_deg = degrees(atan2(imag, real))` wrapped into `(-180, 180]`.

- [x] **Step 4: Run test to verify it passes**

Run: `cd python && python3 -m unittest tests.test_flightbench_analysis_bode -v`

Expected: PASS.

- [x] **Step 5: Leave the work uncommitted**

---

### Task 4: Chart navigation and free zoom

**Files:**
- Create: `web/flightbench/src/chartConfig.ts`
- Modify: `web/flightbench/src/components/Plot.tsx`
- Modify: `web/flightbench/src/components/ComplexPlot.tsx`
- Test: `web/flightbench/src/__tests__/chart-navigation.test.tsx`

**Interfaces:**
- Consumes: Plotly `Config`. The time-series `Plot` and the complex `ComplexPlot`.
- Produces: `chartConfig(): Partial<Config>` from `web/flightbench/src/chartConfig.ts`. Task 5’s `BodePlot` passes this same value. Buttons, in order: `zoom2d`, `pan2d`, `zoomIn2d`, `zoomOut2d`, `autoScale2d`, `resetScale2d`. `displayModeBar` is true, `displaylogo` is false, `scrollZoom` is true, `responsive` is true. `resetScale2d` is the back control.

- [x] **Step 1: Write the failing test**

```tsx
import { describe, expect, it, vi } from 'vitest'
import { render, waitFor } from '@testing-library/react'
import type { Config, Layout } from 'plotly.js-dist-min'
import { chartConfig } from '../chartConfig'
import { ComplexPlot } from '../components/ComplexPlot'
import { Plot } from '../components/Plot'

const { charts } = vi.hoisted(() => ({
  charts: [] as { layout: Partial<Layout>; config?: Partial<Config> }[],
}))

vi.mock('react-plotly.js/factory', () => ({
  default: () =>
    function Chart(props: { layout: Partial<Layout>; config?: Partial<Config> }) {
      charts.push({ layout: props.layout, config: props.config })
      return <div />
    },
}))

vi.mock('plotly.js-dist-min', () => ({ default: {} }))

describe('chart navigation', () => {
  it('lists pan, autoscale, and reset, and leaves both axes free', async () => {
    expect(chartConfig()).toEqual({
      displayModeBar: true,
      displaylogo: false,
      responsive: true,
      scrollZoom: true,
      modeBarButtons: [[
        'zoom2d',
        'pan2d',
        'zoomIn2d',
        'zoomOut2d',
        'autoScale2d',
        'resetScale2d',
      ]],
    })

    render(
      <Plot title="airspeed" traces={[{ name: 'vt', x: [0, 1], y: [1, 2] }]} unit="m/s" shapes={[]} />,
    )
    render(
      <ComplexPlot title="locus" series={[{ real: [0, 4], imag: [0, 1] }]} marks={[]} />,
    )

    await waitFor(() => expect(charts).toHaveLength(2))
    expect(charts[0].config).toEqual(chartConfig())
    expect(charts[1].config).toEqual(chartConfig())
    expect(charts[1].layout.yaxis?.scaleanchor).toBeUndefined()
    expect(charts[1].layout.yaxis?.scaleratio).toBeUndefined()
    expect(charts[0].layout.yaxis?.scaleanchor).toBeUndefined()
  })
})
```

- [x] **Step 2: Run test to verify it fails**

Run: `cd web/flightbench && npx vitest run src/__tests__/chart-navigation.test.tsx`

Expected: FAIL because `chartConfig` is missing. After it exists, FAIL because `ComplexPlot` still sets `scaleanchor` and both charts pass `displayModeBar: false`.

- [x] **Step 3: Implement `chartConfig` and pass it from both charts**

`chartConfig` returns the object in the test. `Plot` and `ComplexPlot` pass `config={chartConfig()}`. Delete `scaleanchor` and `scaleratio` from the complex y axis. Widen `ComplexPlot`’s config prop type so it accepts this object. Leave the shared time `xRange` on `Plot` as the range Reset axes restores.

- [x] **Step 4: Run test to verify it passes**

Run: `cd web/flightbench && npx vitest run src/__tests__/chart-navigation.test.tsx src/__tests__/chart-theme.test.tsx src/__tests__/plots.test.tsx`

Expected: PASS. The light-ground theme assertions still hold.

- [x] **Step 5: Leave the work uncommitted**

---

### Task 5: One Bode chart per open-loop card

**Files:**
- Modify: `python/flightbench/analysis/siso.py` (`siso_card` return dict)
- Modify: `web/flightbench/src/types.ts` (`OpenLoop`)
- Create: `web/flightbench/src/plotlyChart.ts`
- Modify: `web/flightbench/src/components/ComplexPlot.tsx` (import the shared loader)
- Create: `web/flightbench/src/components/BodePlot.tsx`
- Modify: `web/flightbench/src/components/OpenLoopCard.tsx`
- Modify: `web/flightbench/src/__tests__/analysis.test.tsx`
- Modify: `python/tests/test_flightbench_analysis_cards.py`
- Test: `web/flightbench/src/__tests__/analysis.test.tsx` and the cards unittest

**Interfaces:**
- Consumes: `bode_response(A, b, c, d) -> {omega, magnitude_db, phase_deg}` from Task 3. `chartConfig()` from Task 4. `loadPlotlyChart()` extracted from `ComplexPlot` into `plotlyChart.ts` with the same factory promise.
- Produces: `siso_card(...)["bode"]` is that dict. `OpenLoop.bode?: { omega: number[]; magnitude_db: number[]; phase_deg: number[] }`.

```ts
export interface BodePlotProps {
  title: string
  omega: number[]
  magnitudeDb: number[]
  phaseDeg: number[]
}
```

`BodePlot` title is `` `${input} → ${output}` ``. X axis type `log`. Trace `magnitude (dB)` on `yaxis`, trace `phase (deg)` on `yaxis2`. Paper and font come from `chartChrome`. `config` is `chartConfig()`. Neither `yaxis` nor `yaxis2` sets `scaleanchor` or `scaleratio`. An absent `bode` or an empty `omega` draws no chart. The card text still renders.

- [x] **Step 1: Write the failing test**

Python, added to `TestCards` in `test_flightbench_analysis_cards.py`:

```python
def test_every_built_card_carries_a_bode_series(self):
    cards = []
    cards.append(open_loop_cards("pitch_disturbance", _long_model(), _trim(), _PITCH)[0])
    cards.extend(open_loop_cards("trim_cruise", _long_model(), _trim(), _CRUISE))
    cards.append(open_loop_cards("steady_descent", _long_model(), _trim(), _DESCENT)[0])
    cards.append(open_loop_cards("dutch_roll", _lat_model(), _trim(), _GAINS["dutch_roll"])[0])
    cards.append(open_loop_cards("yaw_orientation", _lat_model(), _trim(), _YAW)[0])
    self.assertEqual(len(cards), 6)
    for card in cards:
        bode = card["bode"]
        self.assertGreater(len(bode["omega"]), 0)
        self.assertEqual(len(bode["omega"]), len(bode["magnitude_db"]))
        self.assertEqual(len(bode["omega"]), len(bode["phase_deg"]))
```

Web, in `analysis.test.tsx`, mock `BodePlot` the way `ComplexPlot` is mocked (`bodePlots` array, `data-testid={`bode-${title}`}`). Extend the fixture card with:

```ts
bode: {
  omega: [0.01, 1, 100],
  magnitude_db: [20, 0, -20],
  phase_deg: [0, -45, -90],
}
```

Add a second card `{ ...first, input: 'throttle', output: 'vt' }` only in this test’s response. Assert `bodePlots` titles are `pitch → theta` and `throttle → vt`, and the first plot’s `magnitudeDb` is `[20, 0, -20]`. Assert a card with `bode: { omega: [], magnitude_db: [], phase_deg: [] }` produces no Bode plot and still shows `G(s)`.

In `chart-navigation.test.tsx`, add a case that renders `BodePlot` with `title="pitch → theta"`, `omega={[0.01, 1]}`, `magnitudeDb={[0, -3]}`, `phaseDeg={[0, -45]}`. Expect `config` equal to `chartConfig()`, and `yaxis.scaleanchor` and `yaxis2.scaleanchor` both undefined.

- [x] **Step 2: Run test to verify it fails**

Run: `cd python && python3 -m unittest tests.test_flightbench_analysis_cards.TestCards.test_every_built_card_carries_a_bode_series -v`

Expected: FAIL with `bode` missing.

Run: `cd web/flightbench && npx vitest run src/__tests__/analysis.test.tsx src/__tests__/chart-navigation.test.tsx`

Expected: FAIL because no Bode plot is rendered, and `BodePlot` is not exported.

- [x] **Step 3: Implement the card field and `BodePlot`**

Call `bode_response` at the end of `siso_card` on the unreduced `A`, `b`, `c`, and `d` (the matrices, before cancellation). Move the Plotly factory promise into `plotlyChart.ts`. `OpenLoopCard` renders `BodePlot` under the zeros line when `omega.length > 0`. Pass `chartConfig()`. Leave `scaleanchor` and `scaleratio` unset on `yaxis` and `yaxis2`.

- [x] **Step 4: Run test to verify it passes**

Run: `cd python && python3 -m unittest tests.test_flightbench_analysis_cards tests.test_flightbench_analysis_siso tests.test_flightbench_analysis_bode -v`

Run: `cd web/flightbench && npx vitest run src/__tests__/analysis.test.tsx src/__tests__/chart-theme.test.tsx src/__tests__/chart-navigation.test.tsx src/__tests__/open-loop-card.test.tsx`

Expected: PASS. Chart-theme still sees a light complex-figure ground.

- [x] **Step 5: Leave the work uncommitted**

---

### Task 6: Locus gain slider

**Files:**
- Modify: `web/flightbench/src/components/CoefficientCharts.tsx`
- Modify: `web/flightbench/src/__tests__/analysis.test.tsx`

**Interfaces:**
- Consumes: `CoefficientAnalysis.locus.values` and `branches`, and `values: Record<string, number>` (the gains posted with the run).
- Produces: a range input per locus chart, `aria-label={`${name} locus gain`}`. Its value is an index into `locus.values`. The locus mark is `branches[*][index]`. The numeric readout is `locus.values[index].toPrecision(6)`. Initial index is the existing `gainIndex` of the run gain, or `0` when the run gain is absent. A new `values` object resets the index. An empty `locus.values` still omits the chart and omits the slider.

- [x] **Step 1: Write the failing test**

In `analysis.test.tsx`:

```tsx
it('moves the locus mark along the sweep and ignores a gain-form edit', async () => {
  await runShown()
  const slider = await screen.findByRole('slider', { name: 'kp_theta locus gain' })
  expect(slider).toHaveValue('1')
  expect(screen.getByText('2.00000')).toBeInTheDocument()

  fireEvent.change(slider, { target: { value: '2' } })

  const locus = complexPlots.filter((plot) => plot.title === 'kp_theta locus').at(-1)
  expect(locus?.marks).toContainEqual({ real: -8, imag: 0 })
  expect(locus?.marks).not.toContainEqual(KP_SAMPLE)

  fireEvent.change(screen.getByLabelText('kp_theta'), { target: { value: '50' } })
  const afterForm = complexPlots.filter((plot) => plot.title === 'kp_theta locus').at(-1)
  expect(afterForm?.marks).toContainEqual({ real: -8, imag: 0 })
})
```

`COEFFICIENTS[0].locus.values` is `[0.01, 2, 50]`, so index `2` is the sample `{ real: -8, imag: 0 }`. The readout `2.00000` is `toPrecision(6)` of the run gain `2`.

- [x] **Step 2: Run test to verify it fails**

Run: `cd web/flightbench && npx vitest run src/__tests__/analysis.test.tsx`

Expected: FAIL because the slider is missing.

- [x] **Step 3: Implement the locus slider**

Hold `Record<string, number>` in `CoefficientCharts`. Render the range under the locus title, `min={0}`, `max={values.length - 1}`, `step={1}`. Pass `locus.values[index]` into `locusMarks`. Keep the existing form-versus-mark test green: it changes the gain form, and the slider state stays on the posted run.

- [x] **Step 4: Run test to verify it passes**

Run: `cd web/flightbench && npx vitest run src/__tests__/analysis.test.tsx`

Expected: PASS, including `keeps the locus mark on the gain sent with the run after the form changes` until this test moves the slider. That older test still expects the run sample when only the form changes.

- [x] **Step 5: Leave the work uncommitted**

---

### Task 7: Nyquist gain slider

**Files:**
- Create: `web/flightbench/src/nyquistScale.ts`
- Modify: `python/flightbench/analysis/loops.py` (add `proportional` on each coefficient dict)
- Modify: `web/flightbench/src/types.ts` (`CoefficientAnalysis.proportional: boolean`)
- Modify: `web/flightbench/src/components/CoefficientCharts.tsx`
- Modify: `web/flightbench/src/__tests__/analysis.test.tsx`
- Modify: `python/tests/test_flightbench_analysis_loops.py`
- Modify: `README.md` (the Flightbench linear-analysis sentence)
- Modify: `UPDATES.md` (new top entry `0.105.0`)

**Interfaces:**
- Consumes: Task 6’s locus slider, which stays independent. `coefficient_charts` dicts from `loops.py`.
- Produces:

```ts
export function nyquistScale(
  proportional: boolean,
  runGain: number,
  sliderGain: number,
): number
```

Returns `sliderGain / runGain` only when `proportional` is true, `runGain` is finite, and `runGain !== 0`. Otherwise returns `1`.

Python key `"proportional": name not in _NYQUIST_SOURCE` on every coefficient, including the three aliases. Nyquist range input `aria-label={`${name} nyquist gain`}`. Disabled when `proportional` is false. Displayed real/imag are the returned arrays times `nyquistScale`. The critical mark stays `{ real: -1, imag: 0 }`.

- [x] **Step 1: Write the failing test**

```python
def test_proportional_flag_matches_the_nyquist_alias_table(self):
    pitch = {c["name"]: c["proportional"] for c in coefficient_charts(
        "pitch_disturbance", _pitch_model(), _trim(), _PITCH, _ctx())}
    self.assertTrue(pitch["kq"])
    lead = {c["name"]: c["proportional"] for c in coefficient_charts(
        "lead_pitch", _pitch_model(), _trim(), _LEAD, _ctx())}
    self.assertFalse(lead["lead_zero"])
    self.assertFalse(lead["lead_pole"])
    dutch = {c["name"]: c["proportional"] for c in coefficient_charts(
        "dutch_roll", _lat_model(), _trim(), {"kr": 0.5, "tau_w": 1.0}, _ctx())}
    self.assertTrue(dutch["kr"])
    self.assertFalse(dutch["tau_w"])
```

`_trim` and `_ctx` are the helpers already in `test_flightbench_analysis_loops.py`. The class is `TestLoops`.

```ts
import { nyquistScale } from '../nyquistScale'

describe('nyquist scale', () => {
  it('scales a proportional gain and holds a zero run and an alias', () => {
    expect(nyquistScale(true, 2, 4)).toBe(2)
    expect(nyquistScale(true, 0, 4)).toBe(1)
    expect(nyquistScale(false, 1, 4)).toBe(1)
  })
})
```

In `analysis.test.tsx`, set `proportional: true` on the existing coefficients. Add:

```tsx
it('scales the nyquist curve from its own slider and leaves the critical point', async () => {
  await runShown()
  const slider = await screen.findByRole('slider', { name: 'kq nyquist gain' })
  fireEvent.change(slider, { target: { value: '2' } })
  const nyquist = complexPlots.filter((plot) => plot.title === 'kq nyquist').at(-1)
  // run kq is 0.5 at index 1; index 2 is 10, so the scale is 20.
  expect(nyquist?.series[0].real[1]).toBeCloseTo(-1.5 * 20)
  expect(nyquist?.marks).toContainEqual({ real: -1, imag: 0 })
})

it('keeps an aliased nyquist fixed while its locus slider moves', async () => {
  vi.mocked(postRun).mockResolvedValue(aliasRun())
  await runShown()
  const nyquistSlider = await screen.findByRole('slider', { name: 'tau_w nyquist gain' })
  expect(nyquistSlider).toBeDisabled()
  fireEvent.change(screen.getByRole('slider', { name: 'tau_w locus gain' }), {
    target: { value: '2' },
  })
  const nyquist = complexPlots.filter((plot) => plot.title === 'tau_w nyquist').at(-1)
  expect(nyquist?.series[0].real).toEqual([-0.4, -1.5, -0.3])
})
```

`aliasRun()` is `ANALYSIS_RUN` with one coefficient: `name: 'tau_w'`, `proportional: false`, locus values `[0.05, 1, 20]`, one branch, and the `kq` Nyquist samples from the fixture (`real: [-0.4, -1.5, -0.3]`).

- [x] **Step 2: Run test to verify it fails**

Run: `cd python && python3 -m unittest tests.test_flightbench_analysis_loops.TestLoops.test_proportional_flag_matches_the_nyquist_alias_table -v`

Expected: FAIL with `proportional` missing.

Run: `cd web/flightbench && npx vitest run src/__tests__/analysis.test.tsx src/__tests__/nyquist-scale.test.ts`

Expected: FAIL with the slider missing and `nyquistScale` missing. Put the scale test in `src/__tests__/nyquist-scale.test.ts`.

- [x] **Step 3: Implement the flag, the scale, and the slider**

Set `proportional` in both branches of `coefficient_charts` (the computed chart and the alias copy). Multiply Nyquist `real` and `imag` by `nyquistScale(coefficient.proportional, values[name], locus.values[nyquistIndex])`. The Nyquist slider’s index state is separate from the locus slider. Disable it when `proportional` is false. Update `README.md` Architecture flightbench sentence to: the linear analysis returns the controller diagram with return wires on lanes under each loop, the open-loop matrices and minimal Laplace form, a Bode plot of each open-loop `G(jω)`, and a root locus and Nyquist plot per gain, each with a gain slider; every time-series, locus, Nyquist, and Bode chart shows Zoom, Pan, Zoom in, Zoom out, Autoscale, and Reset axes, and box zoom does not lock the aspect ratio; LQR and NDI omit `linear_analysis`. Prepend `UPDATES.md` with `0.105.0 - Flightbench analysis readability` and one bullet each for the return lanes, the card roles, the Bode series, the sliders (including the non-proportional and zero-gain holds), and the shared navigation bar with free box zoom.

- [x] **Step 4: Run test to verify it passes**

Run: `cd python && python3 -m unittest tests.test_flightbench_analysis_loops tests.test_flightbench_api -v`

Run: `cd web/flightbench && npx vitest run && npm run build`

Expected: PASS. `test_linear_response_passes_linear_analysis_through` still sees the same three top-level keys. The web build succeeds.

- [x] **Step 5: Leave the work uncommitted**
