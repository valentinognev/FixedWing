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
}
