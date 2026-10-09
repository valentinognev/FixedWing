import { afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen } from '@testing-library/react'
import { GainForm, isValid } from '../components/GainForm'
import type { GainSpec } from '../types'

const SPECS: GainSpec[] = [
  { name: 'kq', label: 'kq', value: 0.5, unit: 's' },
  { name: 'k_theta', label: 'Pitch P gain', value: 2.0, unit: 'rad/rad' },
]

function values(over: Record<string, number> = {}): Record<string, number> {
  return {
    kq: SPECS[0].value,
    k_theta: SPECS[1].value,
    ...over,
  }
}

afterEach(cleanup)

describe('GainForm', () => {
  it('renders one numeric field per gain, with its unit and current value', () => {
    render(<GainForm specs={SPECS} values={values()} onChange={vi.fn()} onReset={vi.fn()} />)

    const kq = screen.getByLabelText('kq')
    expect(kq).toHaveAttribute('type', 'number')
    expect(kq).toHaveValue(0.5)
    expect(screen.getByLabelText('Pitch P gain')).toHaveValue(2)
    expect(screen.getAllByRole('spinbutton')).toHaveLength(SPECS.length)
    expect(screen.getByText('s')).toBeInTheDocument()
    expect(screen.getByText('rad/rad')).toBeInTheDocument()
  })

  it('reports the edited gain and its parsed value', () => {
    const onChange = vi.fn()
    render(<GainForm specs={SPECS} values={values()} onChange={onChange} onReset={vi.fn()} />)

    fireEvent.change(screen.getByLabelText('kq'), { target: { value: '1.25' } })

    expect(onChange).toHaveBeenCalledWith('kq', 1.25)
  })

  it('a non-number gain makes isValid false, so Run stays blocked', () => {
    const onChange = vi.fn()
    render(<GainForm specs={SPECS} values={values()} onChange={onChange} onReset={vi.fn()} />)

    fireEvent.change(screen.getByLabelText('kq'), { target: { value: 'abc' } })

    const [name, gain] = onChange.mock.calls.at(-1) as [string, number]
    expect(name).toBe('kq')
    expect(Number.isFinite(gain)).toBe(false)
    expect(isValid({ ...values(), [name]: gain })).toBe(false)
  })

  it('an emptied gain makes isValid false', () => {
    const onChange = vi.fn()
    render(<GainForm specs={SPECS} values={values()} onChange={onChange} onReset={vi.fn()} />)

    fireEvent.change(screen.getByLabelText('kq'), { target: { value: '' } })

    const [name, gain] = onChange.mock.calls.at(-1) as [string, number]
    expect(isValid({ ...values(), [name]: gain })).toBe(false)
  })

  it('Reset to defaults calls onReset', () => {
    const onReset = vi.fn()
    render(<GainForm specs={SPECS} values={values()} onChange={vi.fn()} onReset={onReset} />)

    fireEvent.click(screen.getByRole('button', { name: 'Reset to defaults' }))

    expect(onReset).toHaveBeenCalledTimes(1)
  })
})

describe('isValid', () => {
  it('accepts a full set of finite gains', () => {
    expect(isValid(values())).toBe(true)
  })

  it('rejects NaN and infinite gains', () => {
    expect(isValid(values({ kq: Number.NaN }))).toBe(false)
    expect(isValid(values({ kq: Number.POSITIVE_INFINITY }))).toBe(false)
  })
})
