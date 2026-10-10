import type { RunResponse, Series, TaskFamily } from './types'

/**
 * Everything the charts need to turn a `RunResponse` into Plotly figures. The
 * API reports SI units and radians; the UI shows degrees.
 */

/** A sampled column of a `Series`, i.e. every key but `time`. */
export type Signal = Exclude<keyof Series, 'time'>

/** Longitudinal tasks chart these signals, in this order. */
export const LONG_SIGNALS: Signal[] = [
  'theta',
  'q',
  'alpha',
  'vt',
  'gamma',
  'nz',
  'altitude',
  'pitch',
  'throttle',
]

/** Lateral tasks chart these signals, in this order. */
export const LAT_SIGNALS: Signal[] = [
  'phi',
  'p',
  'beta',
  'r',
  'psi',
  'roll',
  'yaw',
]

/** The signals the API reports in radians, so the charts can show degrees. */
export const ANGLE_SIGNALS: ReadonlySet<Signal> = new Set<Signal>([
  'alpha',
  'beta',
  'gamma',
  'phi',
  'psi',
  'theta',
  'pitch',
  'roll',
  'yaw',
])

const UNITS: Partial<Record<Signal, string>> = {
  vt: 'm/s',
  q: 'rad/s',
  p: 'rad/s',
  r: 'rad/s',
  alpha: 'deg',
  beta: 'deg',
  gamma: 'deg',
  phi: 'deg',
  psi: 'deg',
  theta: 'deg',
  nz: 'g',
  altitude: 'm',
  throttle: '-',
  pitch: 'deg',
  roll: 'deg',
  yaw: 'deg',
}

const SIGNALS: Record<TaskFamily, Signal[]> = {
  longitudinal: LONG_SIGNALS,
  lateral: LAT_SIGNALS,
}

const DEG_PER_RAD = 180 / Math.PI

/** A Plotly dash pattern. */
export type Dash = 'solid' | 'dot' | 'dash' | 'longdash' | 'dashdot' | 'longdashdot'

/** One plotted line: a named pair of equal-length columns. */
export interface PlotTrace {
  name: string
  x: number[]
  y: number[]
  line?: { dash?: Dash; color?: string; width?: number }
}

/** A vertical rule drawn across a chart, in Plotly's shape language. */
export interface PlotShape {
  type: 'line'
  x0: number
  x1: number
  yref: 'paper'
  y0: number
  y1: number
  line: { color: string; width: number; dash: Dash }
}

const STOP_COLOR = '#f87171'

export function signalsFor(family: TaskFamily): Signal[] {
  return SIGNALS[family]
}

export function unitOf(signal: Signal): string {
  return UNITS[signal] ?? ''
}

/** Radians to degrees for angle signals; everything else is already SI. */
export function toDisplay(signal: Signal, values: number[]): number[] {
  if (!ANGLE_SIGNALS.has(signal)) return values
  return values.map((value) => value * DEG_PER_RAD)
}

/** One trace per run of the response, plus the dashed reference it commands. */
export function tracesFor(signal: Signal, response: RunResponse): PlotTrace[] {
  const { nonlinear, linear } = response.runs
  const traces: PlotTrace[] = []
  if (linear !== undefined) {
    traces.push({
      name: 'linear',
      x: linear.time,
      y: toDisplay(signal, linear[signal]),
    })
  }
  traces.push({
    name: 'nonlinear',
    x: nonlinear.time,
    y: toDisplay(signal, nonlinear[signal]),
  })
  const reference = response.reference
  if (reference !== null && reference.signal === signal) {
    traces.push({
      name: 'reference',
      x: reference.time,
      y: toDisplay(signal, reference.values),
      line: { dash: 'dash' },
    })
  }
  return traces
}

/** The time range every chart of a response shares, covering a plant stop. */
export function xRangeOf(response: RunResponse): [number, number] {
  const runs = [response.runs.nonlinear, response.runs.linear].filter(
    (run): run is Series => run !== undefined,
  )
  const from = Math.min(...runs.map((run) => run.time[0] ?? 0))
  const last = runs.map((run) => run.time[run.time.length - 1] ?? 0)
  const to = Math.max(...last, response.stopped_at ?? Number.NEGATIVE_INFINITY)
  return [from, to]
}

/** The vertical rule that marks where the nonlinear run stopped. */
export function stopShapes(stopped_at: number | null): PlotShape[] {
  if (stopped_at === null) return []
  return [
    {
      type: 'line',
      x0: stopped_at,
      x1: stopped_at,
      yref: 'paper',
      y0: 0,
      y1: 1,
      line: { color: STOP_COLOR, width: 2, dash: 'dash' },
    },
  ]
}
