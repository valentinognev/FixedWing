import { afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen } from '@testing-library/react'
import { AeroPicker } from '../components/AeroPicker'
import { LawPicker } from '../components/LawPicker'
import { PlanePicker } from '../components/PlanePicker'
import { TaskList } from '../components/TaskList'
import { TrimPanel } from '../components/TrimPanel'
import {
  aeroChanged,
  lawChanged,
  planeChanged,
  selectionReducer,
  taskChanged,
  trimChanged,
  type Selection,
} from '../state'
import type { PlaneInfo, TaskInfo } from '../types'

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

const X31: PlaneInfo = {
  id: 'x31',
  label: 'X-31',
  laws: ['linear', 'ndi'],
  aero_models: ['most31'],
  default_trim: { vt_mps: 50.0, altitude_m: 457.2 },
  channel_labels: {
    throttle: 'Thrust',
    pitch: 'Canard',
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

const PLANES: PlaneInfo[] = [F16, X31, CESSNA]

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
    id: 'short_period_phugoid',
    label: 'Short period and phugoid',
    family: 'longitudinal',
    lesson: '2023-06-05_lead-compensation',
    duration_s: 60,
    description: 'pitch-channel doublet',
  },
  {
    id: 'dutch_roll',
    label: 'Dutch roll',
    family: 'lateral',
    lesson: '2024-12-18_dutch-roll-control',
    duration_s: 15,
    description: 'yaw-channel pulse +2 deg on [1, 1.5)',
  },
  {
    id: 'turn_coordination',
    label: 'Turn coordination',
    family: 'lateral',
    lesson: '2025-03-01_automatic-turn-coordination',
    duration_s: 20,
    description: 'phi_ref +20 deg step at 1 s',
  },
]

function selection(over: Partial<Selection> = {}): Selection {
  return {
    plane: 'f16',
    law: 'linear',
    aero: 'morelli',
    task: 'lead_pitch',
    trim: {
      vt_mps: F16.default_trim.vt_mps,
      altitude_m: F16.default_trim.altitude_m,
    },
    ...over,
  }
}

afterEach(cleanup)

describe('selectionReducer', () => {
  it('a plane whose laws lack the current law resets law, aero and trim', () => {
    const state = selection({
      plane: 'f16',
      law: 'lqr',
      aero: 'stevens',
      task: 'dutch_roll',
      trim: { vt_mps: 153.0096, altitude_m: 457.2 },
    })

    const next = selectionReducer(state, planeChanged(CESSNA))

    expect(next.plane).toBe('cessna172')
    expect(next.law).toBe('linear')
    expect(next.aero).toBe('tornado')
    expect(next.trim).toEqual(CESSNA.default_trim)
    expect(next.task).toBe('dutch_roll')
    expect(state.law).toBe('lqr')
    expect(state.aero).toBe('stevens')
  })

  it('a plane that shares the current law keeps it', () => {
    const state = selection({ plane: 'f16', law: 'linear', aero: 'stevens' })

    const next = selectionReducer(state, planeChanged(X31))

    expect(next.plane).toBe('x31')
    expect(next.law).toBe('linear')
    expect(next.aero).toBe('most31')
    expect(next.trim).toEqual(X31.default_trim)
  })

  it('lawChanged, aeroChanged and taskChanged replace only their own field', () => {
    const state = selection()

    expect(selectionReducer(state, lawChanged('lqr'))).toEqual({ ...state, law: 'lqr' })
    expect(selectionReducer(state, aeroChanged('stevens'))).toEqual({
      ...state,
      aero: 'stevens',
    })
    expect(selectionReducer(state, taskChanged('dutch_roll'))).toEqual({
      ...state,
      task: 'dutch_roll',
    })
  })

  it('trimChanged replaces the whole trim', () => {
    const state = selection()
    const trim = { vt_mps: 120.0, altitude_m: 1000.0 }

    expect(selectionReducer(state, trimChanged(trim))).toEqual({ ...state, trim })
    expect(state.trim).toEqual(F16.default_trim)
  })

  it('an unknown action returns the same state', () => {
    const state = selection()

    expect(selectionReducer(state, { type: 'nope' } as never)).toBe(state)
  })
})

describe('LawPicker', () => {
  it('lists only the plane laws', () => {
    render(<LawPicker plane={X31} value="linear" onChange={vi.fn()} />)

    const options = screen.getAllByRole('option')
    expect(options).toHaveLength(2)
    expect(options.map((option) => option.getAttribute('value'))).toEqual([
      'linear',
      'ndi',
    ])
    expect(screen.getByRole('combobox', { name: 'Law' })).toHaveValue('linear')
  })

  it('lists every law of a plane that has several', () => {
    render(<LawPicker plane={F16} value="lqr" onChange={vi.fn()} />)

    expect(screen.getAllByRole('option').map((o) => o.getAttribute('value'))).toEqual(
      ['linear', 'lqr'],
    )
    expect(screen.getByRole('combobox', { name: 'Law' })).toHaveValue('lqr')
  })
})

describe('AeroPicker', () => {
  it('selects among several aero models', () => {
    render(<AeroPicker plane={F16} value="morelli" onChange={vi.fn()} />)

    const select = screen.getByRole('combobox', { name: 'Aero' })
    expect(screen.getAllByRole('option').map((o) => o.getAttribute('value'))).toEqual([
      'morelli',
      'stevens',
    ])
    expect(select).toHaveValue('morelli')
  })

  it('renders nothing when the plane has one aero model', () => {
    const { container } = render(
      <AeroPicker plane={CESSNA} value="tornado" onChange={vi.fn()} />,
    )

    expect(container).toBeEmptyDOMElement()
    expect(screen.queryByRole('combobox')).not.toBeInTheDocument()
  })
})

describe('PlanePicker', () => {
  it('lists the plane labels and reports the chosen PlaneInfo', () => {
    const onChange = vi.fn()
    render(<PlanePicker planes={PLANES} value="x31" onChange={onChange} />)

    expect(screen.getAllByRole('option').map((o) => o.textContent)).toEqual([
      'F-16',
      'X-31',
      'Cessna 172',
    ])
    expect(screen.getByRole('combobox', { name: 'Plane' })).toHaveValue('x31')

    fireEvent.change(screen.getByRole('combobox', { name: 'Plane' }), {
      target: { value: 'cessna172' },
    })

    expect(onChange).toHaveBeenCalledWith(CESSNA)
  })
})

describe('TrimPanel', () => {
  it('edits each trim field and emits the whole trim', () => {
    const onChange = vi.fn()
    render(
      <TrimPanel
        value={{ vt_mps: 153.0096, altitude_m: 457.2 }}
        onChange={onChange}
      />,
    )

    const vt = screen.getByLabelText('Airspeed')
    const altitude = screen.getByLabelText('Altitude')
    expect(vt).toHaveValue(153.0096)
    expect(vt).toHaveAttribute('type', 'number')
    expect(altitude).toHaveValue(457.2)
    expect(screen.getByText('m/s')).toBeInTheDocument()
    expect(screen.getByText('m')).toBeInTheDocument()

    fireEvent.change(vt, { target: { value: '120' } })
    expect(onChange).toHaveBeenLastCalledWith({ vt_mps: 120, altitude_m: 457.2 })

    fireEvent.change(altitude, { target: { value: '1000' } })
    expect(onChange).toHaveBeenLastCalledWith({ vt_mps: 153.0096, altitude_m: 1000 })
  })
})

describe('TaskList', () => {
  it('groups tasks by family under Longitudinal and Lateral headings', () => {
    render(<TaskList tasks={TASKS} value="lead_pitch" onChange={vi.fn()} />)

    expect(screen.getByRole('heading', { name: 'Longitudinal' })).toBeInTheDocument()
    expect(screen.getByRole('heading', { name: 'Lateral' })).toBeInTheDocument()
    expect(screen.getByRole('button', { name: /Lead-compensated pitch/ })).toHaveAttribute(
      'aria-pressed',
      'true',
    )
    expect(screen.getByText('2024-12-18_dutch-roll-control')).toBeInTheDocument()
    expect(screen.getByRole('button', { name: /Turn coordination/ })).toHaveAttribute(
      'aria-pressed',
      'false',
    )
  })

  it('reports the clicked task', () => {
    const onChange = vi.fn()
    render(<TaskList tasks={TASKS} value="lead_pitch" onChange={onChange} />)

    fireEvent.click(screen.getByRole('button', { name: /Dutch roll/ }))

    expect(onChange).toHaveBeenCalledWith('dutch_roll')
  })
})
