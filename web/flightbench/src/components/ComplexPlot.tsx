import { useEffect, useState, type ComponentType } from 'react'
import type { Data, Layout } from 'plotly.js-dist-min'
import { chartChrome, readTheme } from '../theme'
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

let chartPromise: Promise<ComponentType<ChartProps>> | undefined

function loadChart(): Promise<ComponentType<ChartProps>> {
  chartPromise ??= Promise.all([
    import('react-plotly.js/factory'),
    import('plotly.js-dist-min'),
  ]).then(([factory, Plotly]) => factory.default(Plotly.default))
  return chartPromise
}

export function ComplexPlot({ title, series, marks }: ComplexPlotProps) {
  const chrome = chartChrome(readTheme())
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

  const axis: Layout['xaxis'] = {
    gridcolor: chrome.grid,
    linecolor: chrome.line,
    zerolinecolor: chrome.line,
    tickfont: { color: chrome.tick, size: 10 },
  }
  const xaxis: Layout['xaxis'] = { ...axis, title: { text: 'real' } }
  const yaxis: Layout['yaxis'] = {
    ...axis,
    title: { text: 'imag' },
    scaleanchor: 'x',
    scaleratio: 1,
  }
  const layout: Partial<Layout> = {
    title: {
      text: title,
      font: { color: chrome.title, size: 12 },
      x: 0,
      xanchor: 'left',
    },
    margin: { l: 52, r: 16, t: 30, b: 40 },
    paper_bgcolor: chrome.paper,
    plot_bgcolor: chrome.paper,
    font: { color: chrome.font, size: 11 },
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
