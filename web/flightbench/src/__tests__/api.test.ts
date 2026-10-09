import { afterEach, describe, expect, it, vi } from 'vitest'
import { ApiError, fetchGains, fetchPlanes, fetchTasks, postRun } from '../api'
import type {
  GainSpec,
  PlaneInfo,
  RunRequest,
  RunResponse,
  Series,
  TaskInfo,
} from '../types'

const DETAIL = "law 'lqr' is not available on cessna172; available: linear"

const PLANES: PlaneInfo[] = [
  {
    id: 'cessna172',
    label: 'Cessna 172',
    laws: ['linear'],
    aero_models: ['tornado'],
    default_trim: { vt_mps: 55.0, altitude_m: 500.0 },
    channel_labels: {
      throttle: 'Throttle',
      pitch: 'Elevator',
      roll: 'Aileron',
      yaw: 'Rudder',
    },
  },
]

const TASKS: TaskInfo[] = [
  {
    id: 'lead_pitch',
    label: 'Lead-compensated pitch',
    family: 'longitudinal',
    lesson: '2023-06-19_lead-compensated',
    duration_s: 10,
    description: 'theta_ref +5 deg step at 1 s',
  },
]

const GAINS: GainSpec[] = [
  { name: 'kp_theta', label: 'Pitch P gain', value: 2, unit: 'rad/rad' },
]

function series(): Series {
  return {
    time: [0, 0.02],
    vt: [55, 55],
    alpha: [0.05, 0.05],
    beta: [0, 0],
    phi: [0, 0],
    theta: [0.05, 0.05],
    psi: [0, 0],
    p: [0, 0],
    q: [0, 0],
    r: [0, 0],
    altitude: [500, 500],
    gamma: [0, 0],
    nz: [0, 0],
    throttle: [0.5, 0.5],
    pitch: [0, 0],
    roll: [0, 0],
    yaw: [0, 0],
  }
}

const RUN: RunResponse = {
  plane: 'cessna172',
  law: 'linear',
  task: 'lead_pitch',
  aero: 'tornado',
  trim: {
    vt_mps: 55,
    altitude_m: 500,
    alpha_rad: 0.05,
    theta_rad: 0.05,
    controls: { throttle: 0.5, pitch: 0, roll: 0, yaw: 0 },
  },
  runs: { nonlinear: series(), linear: series() },
  reference: { signal: 'theta', time: [0, 0.02], values: [0.05, 0.05] },
  metrics: {
    open_loop_modes: [
      { name: 'short_period', wn: 3.1, zeta: 0.4, real: -1.2, imag: 2.9 },
    ],
    closed_loop_modes: [
      { name: 'phugoid', wn: 0.1, zeta: 0.8, real: -0.02, imag: 0.1 },
    ],
    trims: [],
  },
  stopped_at: null,
  stop_reason: null,
}

function respond(body: unknown, status = 200) {
  return {
    ok: status >= 200 && status < 300,
    status,
    statusText: '',
    json: async () => body,
    text: async () => JSON.stringify(body),
  }
}

function mockFetch(body: unknown, status = 200) {
  const fetchMock = vi.fn(async (_url: string, _init?: RequestInit) =>
    respond(body, status),
  )
  vi.stubGlobal('fetch', fetchMock)
  return fetchMock
}

describe('flightbench api client', () => {
  afterEach(() => {
    vi.unstubAllGlobals()
  })

  it('postRun posts JSON to /api/runs and returns the parsed body', async () => {
    const fetchMock = mockFetch(RUN)
    const request: RunRequest = {
      plane: 'cessna172',
      law: 'linear',
      task: 'lead_pitch',
      gains: { kp_theta: 2 },
    }

    const response = await postRun(request)

    expect(response).toEqual(RUN)
    expect(fetchMock).toHaveBeenCalledTimes(1)
    const [url, init] = fetchMock.mock.calls[0]
    expect(url).toBe('/api/runs')
    expect(init?.method).toBe('POST')
    expect(init?.headers).toEqual({ 'Content-Type': 'application/json' })
    expect(init?.body).toBe(JSON.stringify(request))
  })

  it('postRun rejects with an ApiError carrying the status and detail of a 400', async () => {
    mockFetch({ detail: DETAIL }, 400)
    const request: RunRequest = {
      plane: 'cessna172',
      law: 'lqr',
      task: 'lead_pitch',
    }

    await expect(postRun(request)).rejects.toBeInstanceOf(ApiError)
    await expect(postRun(request)).rejects.toMatchObject({
      status: 400,
      detail: DETAIL,
    })
    await expect(postRun(request)).rejects.toThrow(DETAIL)
  })

  it('fetchGains requests /api/gains?plane&law&task', async () => {
    const fetchMock = mockFetch(GAINS)

    const gains = await fetchGains('f16', 'linear', 'lead_pitch')

    expect(gains).toEqual(GAINS)
    expect(fetchMock.mock.calls[0][0]).toBe(
      '/api/gains?plane=f16&law=linear&task=lead_pitch',
    )
  })

  it.each<[string, () => Promise<unknown>, unknown]>([
    ['/api/planes', fetchPlanes, PLANES],
    ['/api/tasks', fetchTasks, TASKS],
  ])('GET %s returns the parsed body', async (path, call, body) => {
    const fetchMock = mockFetch(body)

    expect(await call()).toEqual(body)
    expect(fetchMock.mock.calls[0][0]).toBe(path)
  })
})
