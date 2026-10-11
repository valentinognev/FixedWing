import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import App from '../App'
import { fetchGains, fetchPlanes, fetchTasks, postRun } from '../api'
import type {
  CoefficientAnalysis,
  Complex,
  GainSpec,
  PlaneInfo,
  RunResponse,
  Series,
  TaskInfo,
} from '../types'

interface ComplexSeries {
  real: number[]
  imag: number[]
}

interface ComplexPlotProps {
  title: string
  series: ComplexSeries[]
  marks: Complex[]
}

const { complexPlots } = vi.hoisted(() => ({
  complexPlots: [] as ComplexPlotProps[],
}))

// ComplexPlot is the only analysis importer of plotly.js, so tests stub it
// and read the figure props the charts were handed.
vi.mock('../components/ComplexPlot', () => ({
  ComplexPlot: ({ title, series, marks }: ComplexPlotProps) => {
    complexPlots.push({ title, series, marks })
    return <div data-testid={`complex-${title}`} />
  },
}))

vi.mock('../components/Plot', () => ({
  Plot: ({ title }: { title: string }) => <div data-testid={`plot-${title}`} />,
}))

vi.mock('../api', async (importOriginal) => {
  const actual = await importOriginal<typeof import('../api')>()
  return {
    ...actual,
    fetchPlanes: vi.fn(),
    fetchTasks: vi.fn(),
    fetchGains: vi.fn(),
    postRun: vi.fn(),
  }
})

const F16: PlaneInfo = {
  id: 'f16',
  label: 'F-16',
  laws: ['linear', 'lqr'],
  aero_models: ['morelli'],
  default_trim: { vt_mps: 153.0096, altitude_m: 457.2 },
  channel_labels: {
    throttle: 'Throttle',
    pitch: 'Elevator',
    roll: 'Aileron',
    yaw: 'Rudder',
  },
}

const TASKS: TaskInfo[] = [
  {
    id: 'pitch_disturbance',
    label: 'Pitch disturbance',
    family: 'longitudinal',
    lesson: 'pitch',
    duration_s: 10,
    description: 'theta step',
  },
]

const GAINS: GainSpec[] = [
  { name: 'kp_theta', label: 'kp_theta', value: 2, unit: '' },
  { name: 'ki_theta', label: 'ki_theta', value: 0.5, unit: '' },
  { name: 'kq', label: 'kq', value: 0.5, unit: 's' },
]

const KP_GAIN = 2
const KQ_GAIN = 0.5

const KP_SAMPLE: Complex = { real: -3, imag: 1 }
const KQ_SAMPLE: Complex = { real: -1.5, imag: 2 }

const COEFFICIENTS: CoefficientAnalysis[] = [
  {
    name: 'kp_theta',
    locus: {
      values: [0.01, KP_GAIN, 50],
      branches: [
        [
          { real: -0.2, imag: 0.1 },
          KP_SAMPLE,
          { real: -8, imag: 0 },
        ],
      ],
    },
    nyquist: {
      omega: [0.01, 1, 100],
      real: [-0.2, -0.8, -0.1],
      imag: [0.4, 0.3, -0.2],
    },
  },
  {
    name: 'kq',
    locus: {
      values: [0.001, KQ_GAIN, 10],
      branches: [
        [
          { real: -0.1, imag: 0.2 },
          KQ_SAMPLE,
          { real: -4, imag: 0.5 },
        ],
      ],
    },
    nyquist: {
      omega: [0.01, 1, 100],
      real: [-0.4, -1.5, -0.3],
      imag: [0.6, 0.1, -0.4],
    },
  },
]

const ANALYSIS_RUN: RunResponse = {
  plane: 'f16',
  law: 'linear',
  task: 'pitch_disturbance',
  aero: 'morelli',
  trim: {
    vt_mps: 153.0096,
    altitude_m: 457.2,
    alpha_rad: 0.02,
    theta_rad: 0.02,
    controls: { throttle: 0.4, pitch: 0, roll: 0, yaw: 0 },
  },
  runs: { nonlinear: series(), linear: series() },
  reference: null,
  metrics: {
    open_loop_modes: [
      { name: 'short_period', wn: 4.123456, zeta: 0.5678, real: -2.34, imag: 3.39 },
    ],
  },
  stopped_at: null,
  stop_reason: null,
  linear_analysis: {
    open_loop: [
      {
        input: 'pitch',
        output: 'theta',
        states: ['theta', 'q'],
        A: [
          [-0.2, 1],
          [-2, -0.4],
        ],
        B: [[0], [1]],
        C: [[1, 0]],
        D: [[0]],
        numerator: [1],
        denominator: [1, 2],
        poles: [{ real: -2, imag: 0 }],
        zeros: [],
      },
    ],
    diagram: [
      {
        input: 'pitch',
        output: 'theta',
        feedback: 'θ',
        nodes: [
          { id: 'theta_ref', kind: 'input', label: 'θ_ref', text: '' },
          { id: 'sum_e', kind: 'sum', label: 'sum', text: '' },
          { id: 'pi', kind: 'block', label: 'PI', text: '2 + 0.5/s' },
          { id: 'kq', kind: 'block', label: 'kq', text: '0.5' },
          { id: 'plant', kind: 'plant', label: 'plant', text: '' },
        ],
        wires: [
          { from: 'theta_ref', to: 'sum_e', sign: '+' },
          { from: 'plant', to: 'sum_e', sign: '-' },
          { from: 'sum_e', to: 'pi', sign: '' },
          { from: 'pi', to: 'kq', sign: '' },
          { from: 'kq', to: 'plant', sign: '' },
        ],
      },
      {
        input: 'pitch',
        output: 'q',
        feedback: 'q',
        nodes: [
          { id: 'sum_rate', kind: 'sum', label: 'rate', text: '' },
          { id: 'pi', kind: 'block', label: 'speed PI', text: '3 + 0.1/s' },
          { id: 'plant', kind: 'plant', label: 'plant', text: '' },
        ],
        wires: [{ from: 'plant', to: 'sum_rate', sign: '-' }],
      },
    ],
    coefficients: COEFFICIENTS,
  },
}

function series(): Series {
  const time = [0, 1, 2]
  const column = () => time.map((_, index) => index)
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

function withoutCoefficientSeries(
  name: string,
  seriesName: 'nyquist' | 'locus',
): RunResponse {
  const analysis = ANALYSIS_RUN.linear_analysis
  if (analysis === undefined) return ANALYSIS_RUN
  return {
    ...ANALYSIS_RUN,
    linear_analysis: {
      ...analysis,
      coefficients: analysis.coefficients.map((coefficient) => {
        if (coefficient.name !== name) return coefficient
        if (seriesName === 'nyquist') {
          return {
            ...coefficient,
            nyquist: { omega: [], real: [], imag: [] },
          }
        }
        return {
          ...coefficient,
          locus: { values: [], branches: [] },
        }
      }),
    },
  }
}

function sampleAt(coefficient: CoefficientAnalysis, gain: number): Complex {
  const index = coefficient.locus.values.indexOf(gain)
  return coefficient.locus.branches[0][index]
}

function isBefore(earlier: Element, later: Element): boolean {
  return (earlier.compareDocumentPosition(later) & Node.DOCUMENT_POSITION_FOLLOWING) !== 0
}

async function runShown() {
  render(<App />)
  await screen.findByRole('combobox', { name: 'Plane' })
  await waitFor(() => expect(screen.getByRole('button', { name: 'Run' })).toBeEnabled())
  fireEvent.click(screen.getByRole('button', { name: 'Run' }))
}

afterEach(cleanup)

beforeEach(() => {
  complexPlots.length = 0
  vi.clearAllMocks()
  vi.mocked(fetchPlanes).mockResolvedValue([F16])
  vi.mocked(fetchTasks).mockResolvedValue(TASKS)
  vi.mocked(fetchGains).mockResolvedValue(GAINS)
  vi.mocked(postRun).mockResolvedValue(ANALYSIS_RUN)
})

describe('linear analysis', () => {
  it('draws the pitch-disturbance diagram, the card, and both charts', async () => {
    await runShown()

    expect(await screen.findByText('2 + 0.5/s')).toBeInTheDocument()
    expect(screen.getByText('3 + 0.1/s')).toBeInTheDocument()
    expect(screen.getByText('0.5')).toBeInTheDocument()
    expect(screen.getAllByText('plant')).toHaveLength(1)
    expect(document.querySelectorAll('svg')).toHaveLength(1)

    const laplace = screen.getByText(/G\(s\)/)
    const ratio = laplace.textContent?.replace('G(s)', '') ?? ''
    expect(ratio).toContain('1')
    expect(ratio).toMatch(/s/)
    expect(screen.getAllByRole('columnheader', { name: 'theta' }).length).toBeGreaterThan(0)
    expect(screen.getAllByRole('columnheader', { name: 'q' }).length).toBeGreaterThan(0)

    // Two coefficients, each a locus and a Nyquist.
    expect(complexPlots).toHaveLength(4)
    const kpLocus = complexPlots.find((plot) => plot.title === 'kp_theta locus')
    const kqLocus = complexPlots.find((plot) => plot.title === 'kq locus')
    expect(kpLocus?.marks).toContainEqual(sampleAt(COEFFICIENTS[0], KP_GAIN))
    expect(kqLocus?.marks).toContainEqual(sampleAt(COEFFICIENTS[1], KQ_GAIN))
    expect(kpLocus?.series.some((one) => one.real.includes(KP_SAMPLE.real))).toBe(true)
    for (const plot of complexPlots.filter((one) => one.title.endsWith(' nyquist'))) {
      expect(plot.marks).toContainEqual({ real: -1, imag: 0 })
    }

    expect(
      isBefore(
        screen.getByRole('heading', { name: 'Open loop' }),
        screen.getByTestId('plot-theta'),
      ),
    ).toBe(true)
    expect(isBefore(screen.getByTestId('plot-theta'), screen.getByText('2 + 0.5/s'))).toBe(
      true,
    )
    expect(isBefore(screen.getByText('2 + 0.5/s'), laplace)).toBe(true)
    expect(isBefore(laplace, screen.getByRole('heading', { name: 'kp_theta' }))).toBe(true)
  })

  it('keeps the locus mark on the gain sent with the run after the form changes', async () => {
    await runShown()
    expect(await screen.findByText('2 + 0.5/s')).toBeInTheDocument()

    fireEvent.change(screen.getByLabelText('kp_theta'), { target: { value: '50' } })

    const locus = complexPlots.filter((plot) => plot.title === 'kp_theta locus').at(-1)
    expect(locus?.marks).toContainEqual(sampleAt(COEFFICIENTS[0], KP_GAIN))
    expect(locus?.marks).not.toContainEqual(sampleAt(COEFFICIENTS[0], 50))
  })

  it('omits the Nyquist chart when omega is empty and keeps the coefficient name', async () => {
    vi.mocked(postRun).mockResolvedValue(withoutCoefficientSeries('kq', 'nyquist'))
    await runShown()

    expect(await screen.findByRole('heading', { name: 'kq' })).toBeInTheDocument()
    expect(complexPlots.find((plot) => plot.title === 'kq nyquist')).toBeUndefined()
    expect(complexPlots.find((plot) => plot.title === 'kq locus')).toBeDefined()
  })

  it('omits the locus chart when values is empty and keeps the coefficient name', async () => {
    vi.mocked(postRun).mockResolvedValue(withoutCoefficientSeries('kp_theta', 'locus'))
    await runShown()

    expect(await screen.findByRole('heading', { name: 'kp_theta' })).toBeInTheDocument()
    expect(complexPlots.find((plot) => plot.title === 'kp_theta locus')).toBeUndefined()
    expect(complexPlots.find((plot) => plot.title === 'kp_theta nyquist')).toBeDefined()
  })

  it('draws none of the analysis when the response has no linear_analysis', async () => {
    const { linear_analysis: _omitted, ...plain } = ANALYSIS_RUN
    vi.mocked(postRun).mockResolvedValue(plain)
    await runShown()

    await waitFor(() =>
      expect(screen.getByRole('heading', { name: 'Open loop' })).toBeInTheDocument(),
    )
    expect(screen.queryByText('2 + 0.5/s')).not.toBeInTheDocument()
    expect(screen.queryByText(/G\(s\)/)).not.toBeInTheDocument()
    expect(document.querySelector('svg')).toBeNull()
    expect(complexPlots).toHaveLength(0)
    expect(screen.getByTestId('plot-theta')).toBeInTheDocument()
  })
})
