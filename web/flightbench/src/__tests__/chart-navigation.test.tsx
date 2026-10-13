import { describe, expect, it, vi } from 'vitest'
import { fireEvent, render, waitFor } from '@testing-library/react'
import type { Config, Layout } from 'plotly.js-dist-min'
import { chartConfig } from '../chartConfig'
import { BodePlot } from '../components/BodePlot'
import { CoefficientCharts } from '../components/CoefficientCharts'
import { ComplexPlot } from '../components/ComplexPlot'
import { Plot } from '../components/Plot'
import type { CoefficientAnalysis } from '../types'

const { charts } = vi.hoisted(() => ({
  charts: [] as { layout: Partial<Layout>; config?: Partial<Config> }[],
}))

vi.mock('react-plotly.js/factory', () => ({
  default: () =>
    function Chart(props: { layout: Partial<Layout>; config?: Partial<Config> }) {
      charts.push({ layout: props.layout, config: props.config })
      return <div />
    },
}))

vi.mock('plotly.js-dist-min', () => ({ default: {} }))

describe('chart navigation', () => {
  it('lists pan, autoscale, and reset, and leaves both axes free', async () => {
    expect(chartConfig()).toEqual({
      displayModeBar: true,
      displaylogo: false,
      responsive: true,
      scrollZoom: true,
      modeBarButtons: [[
        'zoom2d',
        'pan2d',
        'zoomIn2d',
        'zoomOut2d',
        'autoScale2d',
        'resetScale2d',
      ]],
    })

    render(
      <Plot title="airspeed" traces={[{ name: 'vt', x: [0, 1], y: [1, 2] }]} unit="m/s" shapes={[]} />,
    )
    render(
      <ComplexPlot title="locus" series={[{ real: [0, 4], imag: [0, 1] }]} marks={[]} />,
    )

    await waitFor(() => expect(charts).toHaveLength(2))
    expect(charts[0].config).toEqual(chartConfig())
    expect(charts[1].config).toEqual(chartConfig())
    expect(charts[1].layout.yaxis?.scaleanchor).toBeUndefined()
    expect(charts[1].layout.yaxis?.scaleratio).toBeUndefined()
    expect(charts[0].layout.yaxis?.scaleanchor).toBeUndefined()
  })

  it('gives the Bode chart the shared config and free magnitude and phase axes', async () => {
    const before = charts.length
    render(
      <BodePlot
        title="pitch → theta"
        omega={[0.01, 1]}
        magnitudeDb={[0, -3]}
        phaseDeg={[0, -45]}
      />,
    )

    await waitFor(() => expect(charts.length).toBe(before + 1))
    const bode = charts.at(-1)
    expect(bode?.config).toEqual(chartConfig())
    expect(bode?.layout.yaxis?.scaleanchor).toBeUndefined()
    expect(bode?.layout.yaxis2?.scaleanchor).toBeUndefined()
  })

  it('keeps a chart view across a slider move and resets it for a new run', async () => {
    const coefficient: CoefficientAnalysis = {
      name: 'kq',
      proportional: true,
      locus: {
        values: [0.1, 0.5, 10],
        branches: [[{ real: -1, imag: 0 }, { real: -2, imag: 1 }, { real: -8, imag: 0 }]],
      },
      nyquist: { omega: [0.1, 1], real: [-0.2, -1.5], imag: [0.1, 0.4] },
    }
    const values = { kq: 0.5 }
    const { rerender } = render(<CoefficientCharts coefficients={[coefficient]} values={values} />)
    await waitFor(() => expect(charts.length).toBeGreaterThanOrEqual(2))
    const viewOf = (title: string) =>
      charts.filter((chart) => chart.layout.title && typeof chart.layout.title === 'object' && chart.layout.title.text === title).at(-1)?.layout.uirevision

    const locusView = viewOf('kq locus')
    const nyquistView = viewOf('kq nyquist')
    expect(locusView).toBeDefined()
    expect(nyquistView).toBeDefined()

    fireEvent.change(document.querySelector('[aria-label="kq locus gain"]') as HTMLInputElement, {
      target: { value: '2' },
    })
    fireEvent.change(document.querySelector('[aria-label="kq nyquist gain"]') as HTMLInputElement, {
      target: { value: '0' },
    })

    expect(viewOf('kq locus')).toBe(locusView)
    expect(viewOf('kq nyquist')).toBe(nyquistView)

    rerender(<CoefficientCharts coefficients={[{ ...coefficient }]} values={{ ...values }} />)
    expect(viewOf('kq locus')).not.toBe(locusView)
    expect(viewOf('kq nyquist')).not.toBe(nyquistView)
  })
})
