/**
 * Mirrors the Flightbench REST API (docs/superpowers/specs/2026-10-09-flightbench-design.md,
 * "API"). SI units, angles in radians, absolute values except `nz`, which is the
 * load-factor increment in g. This app never computes flight dynamics: it only
 * types and renders what the API returns.
 */

export type Law = 'linear' | 'lqr' | 'ndi'

export type Channel = 'throttle' | 'pitch' | 'roll' | 'yaw'

export type TaskFamily = 'longitudinal' | 'lateral'

/** A plane/law/task that the server can run. */
export interface PlaneInfo {
  id: string
  label: string
  laws: Law[]
  aero_models: string[]
  default_trim: { vt_mps: number; altitude_m: number }
  channel_labels: Record<Channel, string>
}

export interface TaskInfo {
  id: string
  label: string
  family: TaskFamily
  lesson: string
  duration_s: number
  description: string
}

/** One tunable gain of the loop the task closes, with its default value. */
export interface GainSpec {
  name: string
  label: string
  value: number
  unit: string
}

export interface Trim {
  vt_mps: number
  altitude_m: number
  alpha_rad: number
  theta_rad: number
  controls: Record<Channel, number>
}

export type ModeName =
  | 'short_period'
  | 'phugoid'
  | 'dutch_roll'
  | 'roll'
  | 'spiral'

export interface Mode {
  name: ModeName
  wn: number
  zeta: number
  real: number
  imag: number
}

export interface RunRequest {
  plane: string
  law: Law
  task: string
  aero?: string
  trim?: { vt_mps: number; altitude_m: number }
  gains?: Record<string, number>
}

/** Sampled every 0.02 s; every key is the same length as `time`. */
export interface Series {
  time: number[]
  vt: number[]
  alpha: number[]
  beta: number[]
  phi: number[]
  theta: number[]
  psi: number[]
  p: number[]
  q: number[]
  r: number[]
  altitude: number[]
  gamma: number[]
  nz: number[]
  throttle: number[]
  pitch: number[]
  roll: number[]
  yaw: number[]
}

/** The commanded trace, absolute (not a deviation from trim). */
export interface Reference {
  signal: string
  time: number[]
  values: number[]
}

export interface RunMetrics {
  open_loop_modes: Mode[]
  closed_loop_modes?: Mode[]
  trims?: Trim[]
}

/** A complex sample from the analysis API: a pole, zero, or locus point. */
export interface Complex {
  real: number
  imag: number
}

/** One open-loop channel. Matrices are the unreduced system; G(s) is cancelled. */
export interface OpenLoop {
  input: string
  output: string
  states: string[]
  A: number[][]
  B: number[][]
  C: number[][]
  D: number[][]
  numerator: number[]
  denominator: number[]
  poles: Complex[]
  zeros: Complex[]
  bode?: {
    omega: number[]
    magnitude_db: number[]
    phase_deg: number[]
  }
}

export type DiagramKind = 'input' | 'sum' | 'block' | 'plant'

export interface DiagramNode {
  id: string
  kind: DiagramKind
  label: string
  text: string
}

export interface DiagramWire {
  from: string
  to: string
  sign: '' | '+' | '-'
  /** Measurement carried on a feedback wire. Empty on every other wire. */
  signal?: string
}

export interface DiagramLoop {
  input: string
  output: string
  feedback: string
  nodes: DiagramNode[]
  wires: DiagramWire[]
}

export interface Locus {
  values: number[]
  branches: Complex[][]
}

export interface Nyquist {
  omega: number[]
  real: number[]
  imag: number[]
}

export interface CoefficientAnalysis {
  name: string
  proportional: boolean
  locus: Locus
  nyquist: Nyquist
}

/** Controller diagram, open-loop cards, and per-gain locus/Nyquist from a linear run. */
export interface LinearAnalysis {
  open_loop: OpenLoop[]
  diagram: DiagramLoop[]
  coefficients: CoefficientAnalysis[]
}

export interface RunResponse {
  plane: string
  law: Law
  task: string
  aero: string
  trim: Trim
  runs: { nonlinear: Series; linear?: Series }
  reference: Reference | null
  metrics: RunMetrics
  stopped_at: number | null
  stop_reason: string | null
  /** Present on a linear-law run. LQR and NDI omit it, and the page draws nothing. */
  linear_analysis?: LinearAnalysis
}
