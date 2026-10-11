import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { cleanup, render, waitFor } from '@testing-library/react'
import type { Layout } from 'plotly.js-dist-min'
import { ComplexPlot } from '../components/ComplexPlot'
import { Plot } from '../components/Plot'

const { layouts } = vi.hoisted(() => ({ layouts: [] as Partial<Layout>[] }))

vi.mock('react-plotly.js/factory', () => ({
  default: () =>
    function Chart(props: { layout: Partial<Layout> }) {
      layouts.push(props.layout)
      return <div />
    },
}))

vi.mock('plotly.js-dist-min', () => ({ default: {} }))

afterEach(cleanup)

beforeEach(() => {
  layouts.length = 0
  localStorage.clear()
})

describe('chart theme', () => {
  it('draws a time figure on a light ground when the theme is light', () => {
    localStorage.setItem('flightbench-theme', 'light')

    render(
      <Plot
        title="airspeed"
        traces={[{ name: 'vt', x: [0], y: [1] }]}
        unit="m/s"
        shapes={[]}
      />,
    )

    const layout = layouts[0]
    expect(layout?.paper_bgcolor).toBe('#fafafa')
    expect(layout?.plot_bgcolor).toBe('#fafafa')
    expect(layout?.font).toMatchObject({ color: '#404040' })
    expect(layout?.title).toMatchObject({ font: { color: '#262626' } })
    expect(layout?.xaxis).toMatchObject({
      gridcolor: '#e5e5e5',
      linecolor: '#d4d4d4',
      tickfont: { color: '#525252' },
    })
  })

  it('draws a complex figure on a light ground when the theme is light', async () => {
    localStorage.setItem('flightbench-theme', 'light')

    render(
      <ComplexPlot
        title="locus"
        series={[{ real: [0], imag: [1] }]}
        marks={[]}
      />,
    )

    await waitFor(() => expect(layouts.length).toBeGreaterThan(0))
    expect(layouts.at(-1)?.paper_bgcolor).toBe('#fafafa')
    expect(layouts.at(-1)?.plot_bgcolor).toBe('#fafafa')
  })
})
