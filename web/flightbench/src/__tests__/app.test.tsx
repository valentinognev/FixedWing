import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, fireEvent, render, screen, waitFor, within } from '@testing-library/react'
import App from '../App'
import { ApiError, fetchGains, fetchPlanes, fetchTasks, postRun } from '../api'
import { LAT_SIGNALS, LONG_SIGNALS } from '../plotting'
import type { GainSpec, PlaneInfo, RunRequest, RunResponse, Series, TaskInfo } from '../types'

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

const DUTCH_GAINS: GainSpec[] = [{ name: 'kr', label: 'kr', value: 0.3, unit: 's' }]

/** A run whose open-loop wn identifies which task produced it. */
function responseFor(task: string, wn: number): RunResponse {
  return {
    ...RUN,
    task,
    metrics: {
      open_loop_modes: [
        {
          name: task === 'dutch_roll' ? 'dutch_roll' : 'short_period',
          wn,
          zeta: 0.5,
          real: -1,
          imag: 1,
        },
      ],
    },
  }
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

function updateButton() {
  return screen.getByRole('button', { name: 'Update' })
}

/** Nearest ancestor that scrolls on its own, or null when the window scrolls. */
function scrollBox(element: HTMLElement): HTMLElement | null {
  let node: HTMLElement | null = element.parentElement
  while (node !== null) {
    if (/\boverflow-y-auto\b/.test(node.className)) return node
    node = node.parentElement
  }
  return null
}

afterEach(cleanup)

beforeEach(() => {
  localStorage.clear()
  delete document.documentElement.dataset.theme
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

describe('App panels', () => {
  it('places Run above the loop gains panel', async () => {
    await loaded()

    const setup = scrollBox(runButton())
    const order = [...(setup?.querySelectorAll('h2, button') ?? [])].map(
      (node) => node.textContent,
    )

    expect(order.indexOf('Airframe')).toBeLessThan(order.indexOf('Run'))
    expect(order.indexOf('Run')).toBeLessThan(order.indexOf('Loop gains'))
    expect(order.indexOf('Loop gains')).toBeLessThan(order.indexOf('Task'))
  })

  it('scrolls the setup column apart from the figure column', async () => {
    await loaded()

    const setup = scrollBox(screen.getByRole('combobox', { name: 'Plane' }))
    const figures = scrollBox(screen.getByText(/Pick a task and press Run/))

    expect(setup).not.toBeNull()
    expect(figures).not.toBeNull()
    expect(setup).not.toBe(figures)
    expect(setup?.className).toMatch(/\bmin-h-0\b/)
    expect(figures?.className).toMatch(/\bmin-h-0\b/)
    const shell = setup?.closest('.h-dvh')
    expect(shell).not.toBeNull()
    expect(shell?.className).toMatch(/\boverflow-hidden\b/)
    expect(shell?.contains(screen.getByRole('banner'))).toBe(true)
    expect(setup?.contains(screen.getByRole('banner'))).toBe(false)
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
    const releases: Array<(response: RunResponse) => void> = []
    vi.mocked(postRun).mockImplementation(
      () =>
        new Promise((resolve) => {
          releases.push(resolve)
        }),
    )
    await loaded()
    await waitFor(() => expect(runButton()).toBeEnabled())

    fireEvent.click(runButton())

    expect(runButton()).toBeDisabled()
    await act(async () => {
      releases[0](RUN)
    })
    await waitFor(() => expect(releases).toHaveLength(TASKS.length))
    expect(runButton()).toBeDisabled()
    await act(async () => {
      for (const release of releases.slice(1)) release(RUN)
    })
    await waitFor(() => expect(runButton()).toBeEnabled())
  })

  it('Update in the loop gains panel posts the edited gains and draws the traces', async () => {
    await loaded()
    const panel = screen.getByRole('heading', { name: 'Loop gains' }).closest('section')
    expect(panel).not.toBeNull()
    expect(within(panel as HTMLElement).getByRole('button', { name: 'Update' })).toBe(
      updateButton(),
    )
    await screen.findByLabelText('kq')
    await waitFor(() => expect(updateButton()).toBeEnabled())

    fireEvent.change(screen.getByLabelText('kq'), { target: { value: '1.25' } })
    fireEvent.click(updateButton())

    expect(postRun).toHaveBeenCalledWith({
      plane: 'f16',
      law: 'linear',
      task: 'lead_pitch',
      aero: 'morelli',
      trim: { vt_mps: 153.0096, altitude_m: 457.2 },
      gains: { kq: 1.25, k_theta: 2 },
    })
    expect(postRun).toHaveBeenCalledTimes(1)
    await waitFor(() => expect(plots).toHaveLength(LONG_SIGNALS.length))
  })

  it('posts every task, the open form for the open task and each other task its own gains', async () => {
    vi.mocked(fetchGains).mockImplementation(async (_plane, _law, task) =>
      task === 'dutch_roll' ? DUTCH_GAINS : GAINS,
    )
    vi.mocked(postRun).mockImplementation(async (request: RunRequest) =>
      responseFor(request.task, request.task === 'dutch_roll' ? 1.5 : 4.123456),
    )
    await loaded()
    await screen.findByLabelText('kq')
    fireEvent.change(screen.getByLabelText('kq'), { target: { value: '9' } })
    await waitFor(() => expect(runButton()).toBeEnabled())

    fireEvent.click(runButton())

    await waitFor(() => expect(postRun).toHaveBeenCalledTimes(2))
    expect(postRun).toHaveBeenNthCalledWith(1, {
      plane: 'f16',
      law: 'linear',
      task: 'lead_pitch',
      aero: 'morelli',
      trim: { vt_mps: 153.0096, altitude_m: 457.2 },
      gains: { kq: 9, k_theta: 2 },
    })
    expect(postRun).toHaveBeenNthCalledWith(2, {
      plane: 'f16',
      law: 'linear',
      task: 'dutch_roll',
      aero: 'morelli',
      trim: { vt_mps: 153.0096, altitude_m: 457.2 },
      gains: { kr: 0.3 },
    })
    expect(screen.getByText('4.123')).toBeInTheDocument()
  })

  it('shows the stored traces of the task you switch to', async () => {
    vi.mocked(postRun).mockImplementation(async (request: RunRequest) =>
      responseFor(request.task, request.task === 'dutch_roll' ? 1.5 : 4.123456),
    )
    await loaded()
    await waitFor(() => expect(runButton()).toBeEnabled())
    fireEvent.click(runButton())
    expect(await screen.findByText('4.123')).toBeInTheDocument()

    fireEvent.click(screen.getByRole('button', { name: /Dutch roll/ }))

    expect(await screen.findByText('1.500')).toBeInTheDocument()
    expect(screen.queryByText('4.123')).not.toBeInTheDocument()

    fireEvent.click(screen.getByRole('button', { name: /Lead-compensated pitch/ }))

    expect(screen.getByText('4.123')).toBeInTheDocument()
    expect(screen.queryByText('1.500')).not.toBeInTheDocument()
    expect(postRun).toHaveBeenCalledTimes(2)
  })

  it('keeps a task gain edits when you leave it and come back', async () => {
    vi.mocked(fetchGains).mockImplementation(async (_plane, _law, task) =>
      task === 'dutch_roll' ? DUTCH_GAINS : GAINS,
    )
    await loaded()
    await screen.findByLabelText('kq')
    fireEvent.change(screen.getByLabelText('kq'), { target: { value: '1.25' } })

    fireEvent.click(screen.getByRole('button', { name: /Dutch roll/ }))
    expect(await screen.findByLabelText('kr')).toHaveValue(0.3)

    fireEvent.click(screen.getByRole('button', { name: /Lead-compensated pitch/ }))

    expect(screen.getByLabelText('kq')).toHaveValue(1.25)
  })

  it("keeps a failed task's error on that task and the other task's traces", async () => {
    vi.mocked(postRun).mockImplementation(async (request: RunRequest) => {
      if (request.task === 'dutch_roll') throw new ApiError(422, 'dutch failed')
      return responseFor(request.task, 4.123456)
    })
    await loaded()
    await waitFor(() => expect(runButton()).toBeEnabled())
    fireEvent.click(runButton())
    expect(await screen.findByText('4.123')).toBeInTheDocument()
    expect(screen.queryByRole('alert')).not.toBeInTheDocument()

    fireEvent.click(screen.getByRole('button', { name: /Dutch roll/ }))

    expect(await screen.findByRole('alert')).toHaveTextContent('dutch failed')
    expect(screen.queryByText('4.123')).not.toBeInTheDocument()

    fireEvent.click(screen.getByRole('button', { name: /Lead-compensated pitch/ }))

    expect(screen.getByText('4.123')).toBeInTheDocument()
    expect(screen.queryByRole('alert')).not.toBeInTheDocument()
  })

  it('Update replaces the open task and leaves the other task stored', async () => {
    vi.mocked(postRun).mockImplementation(async (request: RunRequest) =>
      responseFor(
        request.task,
        request.task === 'dutch_roll' ? 1.5 : request.gains?.kq === 9 ? 7.25 : 4.123456,
      ),
    )
    await loaded()
    await waitFor(() => expect(runButton()).toBeEnabled())
    fireEvent.click(runButton())
    expect(await screen.findByText('4.123')).toBeInTheDocument()

    fireEvent.change(screen.getByLabelText('kq'), { target: { value: '9' } })
    fireEvent.click(updateButton())
    expect(await screen.findByText('7.250')).toBeInTheDocument()

    fireEvent.click(screen.getByRole('button', { name: /Dutch roll/ }))
    expect(await screen.findByText('1.500')).toBeInTheDocument()

    fireEvent.click(screen.getByRole('button', { name: /Lead-compensated pitch/ }))
    expect(screen.getByText('7.250')).toBeInTheDocument()
    expect(postRun).toHaveBeenCalledTimes(3)
  })

  it('keeps Run disabled until every task has returned', async () => {
    const releases: Array<(response: RunResponse) => void> = []
    vi.mocked(postRun).mockImplementation(
      () =>
        new Promise((resolve) => {
          releases.push(resolve)
        }),
    )
    await loaded()
    await waitFor(() => expect(runButton()).toBeEnabled())

    fireEvent.click(runButton())

    await waitFor(() => expect(releases).toHaveLength(1))
    expect(runButton()).toBeDisabled()
    expect(updateButton()).toBeDisabled()
    await act(async () => {
      releases[0](responseFor('lead_pitch', 4.123456))
    })
    await waitFor(() => expect(releases).toHaveLength(2))
    expect(runButton()).toBeDisabled()
    await act(async () => {
      releases[1](responseFor('dutch_roll', 1.5))
    })
    await waitFor(() => expect(runButton()).toBeEnabled())
  })

  it('blocks Update while a gain is not a number', async () => {
    await loaded()
    await screen.findByLabelText('kq')
    await waitFor(() => expect(updateButton()).toBeEnabled())

    fireEvent.change(screen.getByLabelText('kq'), { target: { value: 'abc' } })

    expect(updateButton()).toBeDisabled()
    fireEvent.change(screen.getByLabelText('kq'), { target: { value: '0.75' } })
    expect(updateButton()).toBeEnabled()
  })

  it('keeps Update disabled while the run is in flight', async () => {
    let release: (response: RunResponse) => void = () => undefined
    vi.mocked(postRun).mockImplementation(
      () =>
        new Promise((resolve) => {
          release = resolve
        }),
    )
    await loaded()
    await waitFor(() => expect(updateButton()).toBeEnabled())

    fireEvent.click(updateButton())

    expect(updateButton()).toBeDisabled()
    await act(async () => {
      release(RUN)
    })
    expect(updateButton()).toBeEnabled()
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

describe('theme switch', () => {
  it('offers Light in the header while the page is dark', () => {
    render(<App />)

    expect(
      within(screen.getByRole('banner')).getByRole('button', { name: 'Light' }),
    ).toBeInTheDocument()
    expect(document.documentElement.dataset.theme).toBe('dark')
  })

  it('switches to light and remembers that choice', () => {
    render(<App />)

    fireEvent.click(screen.getByRole('button', { name: 'Light' }))

    expect(screen.getByRole('button', { name: 'Dark' })).toBeInTheDocument()
    expect(document.documentElement.dataset.theme).toBe('light')
    expect(localStorage.getItem('flightbench-theme')).toBe('light')
  })

  it('opens in light when that choice was saved', () => {
    localStorage.setItem('flightbench-theme', 'light')

    render(<App />)

    expect(screen.getByRole('button', { name: 'Dark' })).toBeInTheDocument()
    expect(document.documentElement.dataset.theme).toBe('light')
  })
})
