import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import App from '../App'
import { ApiError, fetchGains, fetchPlanes, fetchTasks, postRun } from '../api'
import { LAT_SIGNALS, LONG_SIGNALS } from '../plotting'
import type { GainSpec, PlaneInfo, RunResponse, Series, TaskInfo } from '../types'

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

const { plots } = vi.hoisted(() => ({
  plots: [] as { title: string; traces: { name: string }[] }[],
}))

vi.mock('../components/Plot', () => ({
  Plot: ({ title, traces }: { title: string; traces: { name: string }[] }) => {
    plots.push({ title, traces })
    return <div />
  },
}))

const F16: PlaneInfo = {
  id: 'f16',
  label: 'F-16',
  laws: ['linear', 'lqr'],
  aero_models: ['morelli', 'stevens'],
  default_trim: { vt_mps: 153.0096, altitude_m: 457.2 },
  channel_labels: {
    throttle: 'Throttle',
    pitch: 'Elevator',
    roll: 'Aileron',
    yaw: 'Rudder',
  },
}

const CESSNA: PlaneInfo = {
  id: 'cessna172',
  label: 'Cessna 172',
  laws: ['linear'],
  aero_models: ['tornado'],
  default_trim: { vt_mps: 23.114578, altitude_m: 100.0 },
  channel_labels: {
    throttle: 'Throttle',
    pitch: 'Elevator',
    roll: 'Aileron',
    yaw: 'Rudder',
  },
}

const TASKS: TaskInfo[] = [
  {
    id: 'lead_pitch',
    label: 'Lead-compensated pitch',
    family: 'longitudinal',
    lesson: '2023-06-19_lead-compensated',
    duration_s: 10,
    description: 'theta_ref +5 deg step at 1 s',
  },
  {
    id: 'dutch_roll',
    label: 'Dutch roll',
    family: 'lateral',
    lesson: '2024-12-18_dutch-roll-control',
    duration_s: 15,
    description: 'yaw-channel pulse +2 deg on [1, 1.5)',
  },
]

const GAINS: GainSpec[] = [
  { name: 'kq', label: 'kq', value: 0.5, unit: 's' },
  { name: 'k_theta', label: 'Pitch P gain', value: 2.0, unit: 'rad/rad' },
]

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

const RUN: RunResponse = {
  plane: 'f16',
  law: 'linear',
  task: 'lead_pitch',
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
}

async function loaded() {
  render(<App />)
  await screen.findByRole('combobox', { name: 'Plane' })
}

function runButton() {
  return screen.getByRole('button', { name: 'Run' })
}

afterEach(cleanup)

beforeEach(() => {
  plots.length = 0
  vi.clearAllMocks()
  vi.mocked(fetchPlanes).mockResolvedValue([F16, CESSNA])
  vi.mocked(fetchTasks).mockResolvedValue(TASKS)
  vi.mocked(fetchGains).mockResolvedValue(GAINS)
  vi.mocked(postRun).mockResolvedValue(RUN)
})

describe('App selection', () => {
  it('refetches the gains for the plane and task it was given', async () => {
    await loaded()

    expect(fetchGains).toHaveBeenCalledWith('f16', 'linear', 'lead_pitch')

    fireEvent.change(screen.getByRole('combobox', { name: 'Plane' }), {
      target: { value: 'cessna172' },
    })

    expect(fetchGains).toHaveBeenCalledWith('cessna172', 'linear', 'lead_pitch')

    fireEvent.change(screen.getByRole('combobox', { name: 'Law' }), {
      target: { value: 'linear' },
    })
    fireEvent.click(screen.getByRole('button', { name: /Dutch roll/ }))

    expect(fetchGains).toHaveBeenCalledWith('cessna172', 'linear', 'dutch_roll')
  })

  it('seeds the gain form with every gain the server returns', async () => {
    await loaded()

    expect(await screen.findByLabelText('kq')).toHaveValue(0.5)
    expect(screen.getByLabelText('Pitch P gain')).toHaveValue(2)
    await waitFor(() => expect(runButton()).toBeEnabled())
  })

  it('leaves Run disabled until the gains are seeded', () => {
    render(<App />)

    expect(runButton()).toBeDisabled()
  })
})

describe('App run flow', () => {
  it('posts the selection and the seeded gains', async () => {
    await loaded()
    await waitFor(() => expect(runButton()).toBeEnabled())

    fireEvent.click(runButton())

    expect(postRun).toHaveBeenCalledWith({
      plane: 'f16',
      law: 'linear',
      task: 'lead_pitch',
      aero: 'morelli',
      trim: { vt_mps: 153.0096, altitude_m: 457.2 },
      gains: { kq: 0.5, k_theta: 2 },
    })
  })

  it('feeds the response into the charts and the modes table', async () => {
    await loaded()
    await waitFor(() => expect(runButton()).toBeEnabled())

    fireEvent.click(runButton())

    await waitFor(() => expect(plots).toHaveLength(LONG_SIGNALS.length))
    expect(plots.map((plot) => plot.title)).toEqual([...LONG_SIGNALS])
    expect(
      plots.every(
        (plot) => plot.traces.map((trace) => trace.name).join() === 'linear,nonlinear',
      ),
    ).toBe(true)
    expect(screen.getByRole('heading', { name: 'Open loop' })).toBeInTheDocument()
    expect(screen.getByText('4.123')).toBeInTheDocument()
    expect(screen.queryByRole('alert')).not.toBeInTheDocument()
  })

  it('charts the lateral signals for a lateral task', async () => {
    vi.mocked(fetchPlanes).mockResolvedValue([CESSNA])
    vi.mocked(postRun).mockResolvedValue({
      ...RUN,
      plane: CESSNA.id,
      aero: 'tornado',
      task: 'dutch_roll',
    })
    await loaded()
    await waitFor(() => expect(runButton()).toBeEnabled())
    fireEvent.click(screen.getByRole('button', { name: /Dutch roll/ }))
    // Run drops out while the gains of the new task are fetched.
    await waitFor(() => expect(runButton()).toBeDisabled())
    await waitFor(() => expect(runButton()).toBeEnabled())

    fireEvent.click(runButton())

    await waitFor(() => expect(plots).toHaveLength(LAT_SIGNALS.length))
    expect(plots.map((plot) => plot.title)).toEqual([...LAT_SIGNALS])
  })

  it('hides the plots of a run that no longer matches the selection', async () => {
    await loaded()
    await waitFor(() => expect(runButton()).toBeEnabled())
    fireEvent.click(runButton())
    await waitFor(() =>
      expect(screen.getByRole('heading', { name: 'Open loop' })).toBeInTheDocument(),
    )
    const drawn = plots.length

    fireEvent.change(screen.getByRole('combobox', { name: 'Plane' }), {
      target: { value: 'cessna172' },
    })

    await waitFor(() =>
      expect(
        screen.queryByRole('heading', { name: 'Open loop' }),
      ).not.toBeInTheDocument(),
    )
    expect(plots).toHaveLength(drawn)
  })

  it('shows the detail of a rejected run', async () => {
    vi.mocked(postRun).mockRejectedValue(
      new ApiError(422, 'max(altitude_m) = 15000'),
    )
    await loaded()
    await waitFor(() => expect(runButton()).toBeEnabled())

    fireEvent.click(runButton())

    expect(await screen.findByRole('alert')).toHaveTextContent('max(altitude_m) = 15000')
    expect(plots).toHaveLength(0)
  })

  it('keeps Run disabled while the run is in flight', async () => {
    let release: (response: RunResponse) => void = () => undefined
    vi.mocked(postRun).mockImplementation(
      () =>
        new Promise((resolve) => {
          release = resolve
        }),
    )
    await loaded()
    await waitFor(() => expect(runButton()).toBeEnabled())

    fireEvent.click(runButton())

    expect(runButton()).toBeDisabled()
    await act(async () => {
      release(RUN)
    })
    expect(runButton()).toBeEnabled()
  })

  it('blocks Run while a gain is not a number', async () => {
    await loaded()
    await screen.findByLabelText('kq')

    fireEvent.change(screen.getByLabelText('kq'), { target: { value: 'abc' } })

    expect(runButton()).toBeDisabled()
    fireEvent.change(screen.getByLabelText('kq'), { target: { value: '0.75' } })
    expect(runButton()).toBeEnabled()
  })
})
