import { useEffect, useState, type ComponentType } from 'react'
import type { Data, Layout } from 'plotly.js-dist-min'
import type { Complex } from '../types'

/**
 * The only analysis module that loads Plotly, and only once a chart is shown.
 * Axes are real and imag, never time. The import stays inside this file, same
 * rule as Plot, but it is deferred so the rest of the page does not initialize
 * Plotly.
 */

interface ChartProps {
  className?: string
  data: Data[]
  layout: Partial<Layout>
  config?: { displayModeBar?: boolean; responsive?: boolean }
  useResizeHandler?: boolean
}

export interface ComplexSeries {
  name?: string
  real: number[]
  imag: number[]
}

export interface ComplexPlotProps {
  title: string
  series: ComplexSeries[]
  marks: Complex[]
}

const TICK = '#a3a3a3'
const GRID = '#262626'
const LINE = '#404040'

const AXIS: Layout['xaxis'] = {
  gridcolor: GRID,
  linecolor: LINE,
  zerolinecolor: LINE,
  tickfont: { color: TICK, size: 10 },
}

let chartPromise: Promise<ComponentType<ChartProps>> | undefined

function loadChart(): Promise<ComponentType<ChartProps>> {
  chartPromise ??= Promise.all([
    import('react-plotly.js/factory'),
    import('plotly.js-dist-min'),
  ]).then(([factory, Plotly]) => factory.default(Plotly.default))
  return chartPromise
}

export function ComplexPlot({ title, series, marks }: ComplexPlotProps) {
  const [Chart, setChart] = useState<ComponentType<ChartProps> | null>(null)

  useEffect(() => {
    let cancelled = false
    void loadChart()
      .then((Loaded) => {
        if (!cancelled) setChart(() => Loaded)
      })
      .catch(() => {
        chartPromise = undefined
      })
    return () => {
      cancelled = true
    }
  }, [])

  const data: Data[] = series.map((one) => ({
    name: one.name,
    x: one.real,
    y: one.imag,
    type: 'scatter',
    mode: 'lines+markers',
    marker: { size: 5 },
    line: { width: 1.5 },
  }))
  if (marks.length > 0) {
    data.push({
      name: 'mark',
      x: marks.map((mark) => mark.real),
      y: marks.map((mark) => mark.imag),
      type: 'scatter',
      mode: 'markers',
      marker: { size: 11, symbol: 'x', color: '#f87171' },
    })
  }

  const xaxis: Layout['xaxis'] = { ...AXIS, title: { text: 'real' } }
  const yaxis: Layout['yaxis'] = {
    ...AXIS,
    title: { text: 'imag' },
    scaleanchor: 'x',
    scaleratio: 1,
  }
  const layout: Partial<Layout> = {
    title: {
      text: title,
      font: { color: '#e5e5e5', size: 12 },
      x: 0,
      xanchor: 'left',
    },
    margin: { l: 52, r: 16, t: 30, b: 40 },
    paper_bgcolor: '#0a0a0a',
    plot_bgcolor: '#0a0a0a',
    font: { color: '#d4d4d4', size: 11 },
    showlegend: data.length > 1,
    legend: { orientation: 'h', y: -0.22, font: { size: 10 } },
    hovermode: 'closest',
    xaxis,
    yaxis,
  }

  if (Chart === null) return <div className="h-72 w-full" />

  return (
    <Chart
      className="h-72 w-full"
      data={data}
      layout={layout}
      config={{ displayModeBar: false, responsive: true }}
      useResizeHandler
    />
  )
}
