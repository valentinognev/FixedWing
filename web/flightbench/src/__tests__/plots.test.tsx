import { beforeEach, afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, render, screen } from '@testing-library/react'
import { RunBanner } from '../components/RunBanner'
import { ModesTable } from '../components/ModesTable'
import { RunPlots } from '../components/RunPlots'
import {
  ANGLE_SIGNALS,
  LAT_SIGNALS,
  LONG_SIGNALS,
  toDisplay,
} from '../plotting'
import type { Mode, RunResponse, Series, Trim } from '../types'
import type { PlotProps } from '../components/Plot'

const { plots } = vi.hoisted(() => ({ plots: [] as PlotProps[] }))

// Plot is the only importer of react-plotly.js, so tests stub it and read the
// figure props the charts were handed.
vi.mock('../components/Plot', () => ({
  Plot: ({ title, traces, xRange, shapes, unit }: PlotProps) => {
    plots.push({ title, traces, xRange, shapes, unit })
    return <div data-testid={`plot-${title}`} />
  },
}))

function series(time: number[], value = 0): Series {
  const column = () => time.map((_, index) => value + index)
  return {
    time,
    vt: column(),
    alpha: column(),
    beta: column(),
    phi: column(),
    theta: column(),
    psi: column(),
    p: column(),
    q: column(),
    r: column(),
    altitude: column(),
    gamma: column(),
    nz: column(),
    throttle: column(),
    pitch: column(),
    roll: column(),
    yaw: column(),
  }
}

const TRIM: Trim = {
  vt_mps: 153.0096,
  altitude_m: 457.2,
  alpha_rad: 0.0215,
  theta_rad: 0.0215,
  controls: { throttle: 0.42, pitch: 0, roll: 0, yaw: 0 },
}

const OPEN_MODES: Mode[] = [
  { name: 'short_period', wn: 4.123456, zeta: 0.5678, real: -2.34, imag: 3.39 },
  { name: 'phugoid', wn: 0.0876543, zeta: 0.12345, real: -0.0108, imag: 0.087 },
]

const CLOSED_MODES: Mode[] = [
  { name: 'short_period', wn: 5.4321, zeta: 0.8123, real: -4.4, imag: 3.2 },
  { name: 'phugoid', wn: 0.0912, zeta: 0.24, real: -0.0219, imag: 0.0885 },
]

const TIME = [0, 1, 2]

const LINEAR_RUN: RunResponse = {
  plane: 'f16',
  law: 'linear',
  task: 'lead_pitch',
  aero: 'morelli',
  trim: TRIM,
  runs: { nonlinear: series(TIME, 10), linear: series(TIME, 10) },
  reference: null,
  metrics: { open_loop_modes: OPEN_MODES, closed_loop_modes: CLOSED_MODES },
  stopped_at: null,
  stop_reason: null,
}

const LQR_RUN: RunResponse = {
  ...LINEAR_RUN,
  law: 'lqr',
  runs: { nonlinear: series(TIME, 10) },
  metrics: { open_loop_modes: OPEN_MODES },
}

const LATERAL_RUN: RunResponse = {
  ...LINEAR_RUN,
  task: 'dutch_roll',
  reference: { signal: 'psi', time: TIME, values: [0.1, 0.2, 0.3] },
}

const STOPPED_RUN: RunResponse = {
  ...LINEAR_RUN,
  stopped_at: 3.2,
  stop_reason: 'engine stop: fuel exhausted at 3.200 s',
}

afterEach(cleanup)

beforeEach(() => {
  plots.length = 0
})

describe('toDisplay', () => {
  it('shows radian angle signals in degrees', () => {
    expect(toDisplay('theta', [0.1])[0]).toBeCloseTo(5.729578, 6)
    expect(toDisplay('pitch', [0.1])[0]).toBeCloseTo(5.729578, 6)
    expect(toDisplay('roll', [0.1])[0]).toBeCloseTo(5.729578, 6)
    expect(toDisplay('yaw', [0.1])[0]).toBeCloseTo(5.729578, 6)
    expect(toDisplay('alpha', [0, Math.PI])).toEqual([0, 180])
  })

  it('leaves the other signals in the unit the API reports', () => {
    expect(toDisplay('vt', [153.0096])).toEqual([153.0096])
    expect(toDisplay('q', [0.25])).toEqual([0.25])
    expect(toDisplay('altitude', [457.2])).toEqual([457.2])
    expect(toDisplay('throttle', [0.42])).toEqual([0.42])
  })

  it('knows which signals are radians', () => {
    expect(ANGLE_SIGNALS.has('pitch')).toBe(true)
    expect(ANGLE_SIGNALS.has('roll')).toBe(true)
    expect(ANGLE_SIGNALS.has('yaw')).toBe(true)
    expect(ANGLE_SIGNALS.has('vt')).toBe(false)
    expect(ANGLE_SIGNALS.has('altitude')).toBe(false)
  })
})

describe('RunPlots', () => {
  it('charts every longitudinal signal with one trace per run', () => {
    render(<RunPlots response={LINEAR_RUN} family="longitudinal" />)

    expect(plots.map((plot) => plot.title)).toEqual([...LONG_SIGNALS])
    expect(plots.map((plot) => plot.traces.map((trace) => trace.name))).toEqual(
      LONG_SIGNALS.map(() => ['linear', 'nonlinear']),
    )
    expect(plots[0].traces[0].x).toEqual(TIME)
  })

  it('charts one trace when the response has no linear run', () => {
    render(<RunPlots response={LQR_RUN} family="longitudinal" />)

    expect(plots).toHaveLength(LONG_SIGNALS.length)
    expect(
      plots.every((plot) => plot.traces.map((trace) => trace.name).join() === 'nonlinear'),
    ).toBe(true)
  })

  it('charts the lateral signals in degrees on their angle charts', () => {
    render(<RunPlots response={LATERAL_RUN} family="lateral" />)

    expect(plots.map((plot) => plot.title)).toEqual([...LAT_SIGNALS])
    const psi = plots[LAT_SIGNALS.indexOf('psi')]
    expect(psi.traces.map((trace) => trace.name)).toEqual([
      'linear',
      'nonlinear',
      'reference',
    ])
    expect(psi.traces[2].y[0]).toBeCloseTo(5.729578, 6)
  })

  it('shows theta in degrees', () => {
    const theta = series([0.5])
    const response: RunResponse = {
      ...LINEAR_RUN,
      runs: { nonlinear: { ...theta, theta: [0.1] }, linear: { ...theta, theta: [0.1] } },
    }

    render(<RunPlots response={response} family="longitudinal" />)

    expect(plots[0].traces[0].name).toBe('linear')
    expect(plots[0].traces[0].y[0]).toBeCloseTo(5.729578, 6)
    expect(plots[0].traces[1].name).toBe('nonlinear')
    expect(plots[0].traces[1].y[0]).toBeCloseTo(5.729578, 6)
  })

  it('dashes the reference trace only on the chart it commands', () => {
    const response: RunResponse = {
      ...LINEAR_RUN,
      reference: { signal: 'theta', time: TIME, values: [0.05, 0.1, 0.15] },
    }

    render(<RunPlots response={response} family="longitudinal" />)

    expect(plots[0].traces).toHaveLength(3)
    expect(plots[0].traces[2].name).toBe('reference')
    expect(plots[0].traces[2].line?.dash).toBe('dash')
    expect(plots[0].traces[2].y[0]).toBeCloseTo(2.864789, 6)
    expect(
      plots.slice(1).every((plot) => !plot.traces.some((t) => t.name === 'reference')),
    ).toBe(true)
  })

  it('shares one time range across the charts', () => {
    render(<RunPlots response={LINEAR_RUN} family="longitudinal" />)

    expect(plots[0].xRange).toEqual([0, 2])
    expect(plots.every((plot) => JSON.stringify(plot.xRange) === '[0,2]')).toBe(true)
    expect(plots.every((plot) => plot.shapes.length === 0)).toBe(true)
  })

  it('marks the plant stop on every chart', () => {
    render(<RunPlots response={STOPPED_RUN} family="longitudinal" />)

    expect(plots[0].xRange).toEqual([0, 3.2])
    for (const plot of plots) {
      expect(plot.shapes.some((shape) => shape.x0 === 3.2 && shape.x1 === 3.2)).toBe(true)
    }
  })

  it('labels each chart with the unit of its signal', () => {
    render(<RunPlots response={LINEAR_RUN} family="longitudinal" />)

    expect(plots[0].unit).toBe('deg')
    expect(plots[LONG_SIGNALS.indexOf('q')].unit).toBe('rad/s')
    expect(plots[LONG_SIGNALS.indexOf('vt')].unit).toBe('m/s')
    expect(plots[LONG_SIGNALS.indexOf('altitude')].unit).toBe('m')
    expect(plots[LONG_SIGNALS.indexOf('nz')].unit).toBe('g')
  })
})

describe('RunBanner', () => {
  it('shows the stop reason of a plant stop', () => {
    render(<RunBanner stopReason={STOPPED_RUN.stop_reason} error={null} />)

    expect(screen.getByRole('alert')).toHaveTextContent(
      'engine stop: fuel exhausted at 3.200 s',
    )
  })

  it('shows the detail of an API error', () => {
    render(<RunBanner stopReason={null} error="max(altitude_m) = 15000" />)

    expect(screen.getByRole('alert')).toHaveTextContent('max(altitude_m) = 15000')
  })

  it('renders nothing for a clean run', () => {
    const { container } = render(<RunBanner stopReason={null} error={null} />)

    expect(container).toBeEmptyDOMElement()
  })
})

describe('ModesTable', () => {
  it('lists the open-loop and closed-loop modes to three decimals', () => {
    render(
      <ModesTable
        metrics={{
          open_loop_modes: OPEN_MODES,
          closed_loop_modes: CLOSED_MODES,
        }}
      />,
    )

    expect(screen.getByRole('heading', { name: 'Open loop' })).toBeInTheDocument()
    expect(screen.getByRole('heading', { name: 'Closed loop' })).toBeInTheDocument()
    expect(screen.getAllByText('Short period')).toHaveLength(2)
    expect(screen.getByText('4.123')).toBeInTheDocument()
    expect(screen.getByText('0.568')).toBeInTheDocument()
    expect(screen.getByText('0.088')).toBeInTheDocument()
    expect(screen.getByText('0.123')).toBeInTheDocument()
    expect(screen.getByText('5.432')).toBeInTheDocument()
    expect(screen.getByText('0.812')).toBeInTheDocument()
    expect(screen.getAllByRole('row')).toHaveLength(6)
  })

  it('omits the closed-loop modes when the API does not send them', () => {
    render(<ModesTable metrics={{ open_loop_modes: OPEN_MODES }} />)

    expect(screen.getByRole('heading', { name: 'Open loop' })).toBeInTheDocument()
    expect(
      screen.queryByRole('heading', { name: 'Closed loop' }),
    ).not.toBeInTheDocument()
  })
})
