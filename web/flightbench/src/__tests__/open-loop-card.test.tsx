import { readFileSync } from 'node:fs'
import { dirname, join } from 'node:path'
import { fileURLToPath } from 'node:url'
import { describe, expect, it } from 'vitest'
import { render, screen } from '@testing-library/react'
import { OpenLoopCard } from '../components/OpenLoopCard'

// Vite rewrites `new URL('../index.css', import.meta.url)` to http://localhost:3000/...,
// so readFileSync throws before any assertion. Resolve the stylesheet from the file URL.
const css = readFileSync(join(dirname(fileURLToPath(import.meta.url)), '../index.css'), 'utf8')

describe('open-loop card roles', () => {
  it('paints signed entries, a tiny entry, G(s), and pole stability', () => {
    render(
      <OpenLoopCard
        card={{
          input: 'pitch',
          output: 'theta',
          states: ['theta', 'q'],
          A: [[6.0142, -9.80542], [-4.50919e-17, 0]],
          B: [[0.124826], [10.5916]],
          C: [[1, 0]],
          D: [[0]],
          numerator: [1, 0.25],
          denominator: [1, 2, 1],
          poles: [
            { real: 0.10333, imag: 0 },
            { real: -1.93302, imag: 0 },
            { real: -0.149519, imag: 0.128339 },
            { real: -0.149519, imag: -0.128339 },
          ],
          zeros: [{ real: -1.07248, imag: 0 }],
        }}
      />,
    )

    expect(screen.getByRole('heading', { name: 'pitch → theta' })).toHaveAttribute(
      'data-role',
      'card-title',
    )
    expect(screen.getByText('A')).toHaveAttribute('data-role', 'matrix-a')
    expect(screen.getByText('B')).toHaveAttribute('data-role', 'matrix-b')
    expect(screen.getByText('C')).toHaveAttribute('data-role', 'matrix-c')
    expect(screen.getByText('D')).toHaveAttribute('data-role', 'matrix-d')
    // Matrices A and C each expose a theta columnheader, so getByRole throws.
    for (const header of screen.getAllByRole('columnheader', { name: 'theta' })) {
      expect(header).toHaveAttribute('data-role', 'axis-name')
    }
    expect(screen.getByText('6.01420')).toHaveAttribute('data-role', 'value-positive')
    expect(screen.getByText('-9.80542')).toHaveAttribute('data-role', 'value-negative')
    expect(screen.getByText('-4.50919e-17')).toHaveAttribute('data-role', 'value-zero')
    expect(screen.getByText(/G\(s\)/).textContent).toMatch(/1\.00000 s \+ 0\.250000/)
    expect(screen.getByText(/G\(s\)/).querySelector("[data-role='tf-num']")).not.toBeNull()
    expect(screen.getByText(/G\(s\)/).querySelector("[data-role='tf-den']")).not.toBeNull()
    expect(screen.getByText('0.103330')).toHaveAttribute('data-role', 'pole-unstable')
    expect(screen.getByText('-1.93302')).toHaveAttribute('data-role', 'pole-stable')
    expect(screen.getByText(/-0\.149519 ± 0\.128339j/)).toHaveAttribute('data-role', 'pole-stable')
    expect(screen.getByText('-1.07248')).toHaveAttribute('data-role', 'zero')
  })

  it('defines both theme colors and the larger title size', () => {
    expect(css).toMatch(/\[data-role='value-negative'\]\s*\{[^}]*color:\s*#fb7185/)
    expect(css).toMatch(/\[data-role='value-negative'\]\s*\{[^}]*font-size:\s*14px/)
    expect(css).toMatch(
      /html\[data-theme='light'\] \[data-role='value-negative'\]\s*\{[^}]*color:\s*#be123c/,
    )
    expect(css).toMatch(/\[data-role='matrix-a'\]\s*\{[^}]*color:\s*#f472b6/)
    expect(css).toMatch(/\[data-role='matrix-a'\]\s*\{[^}]*font-size:\s*18px/)
    expect(css).toMatch(
      /html\[data-theme='light'\] \[data-role='matrix-a'\]\s*\{[^}]*color:\s*#be185d/,
    )
    expect(css).toMatch(/\[data-role='axis-name'\]\s*\{[^}]*color:\s*#fcd34d/)
    expect(css).toMatch(
      /html\[data-theme='light'\] \[data-role='axis-name'\]\s*\{[^}]*color:\s*#92400e/,
    )
    expect(css).toMatch(/\[data-role='pole-unstable'\]\s*\{[^}]*color:\s*#fb7185/)
    expect(css).toMatch(
      /html\[data-theme='light'\] \[data-role='pole-unstable'\]\s*\{[^}]*color:\s*#be123c/,
    )
  })
})
