import { describe, expect, it } from 'vitest'
import { nyquistScale } from '../nyquistScale'

describe('nyquist scale', () => {
  it('scales a proportional gain and holds a zero run and an alias', () => {
    expect(nyquistScale(true, 2, 4)).toBe(2)
    expect(nyquistScale(true, 0, 4)).toBe(1)
    expect(nyquistScale(false, 1, 4)).toBe(1)
  })
})
