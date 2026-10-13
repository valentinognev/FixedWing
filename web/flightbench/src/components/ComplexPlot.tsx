import { useEffect, useState, type ComponentType } from 'react'
import type { Data, Layout } from 'plotly.js-dist-min'
import { chartConfig } from '../chartConfig'
import { loadPlotlyChart, type ChartProps } from '../plotlyChart'
import { chartChrome, readTheme } from '../theme'
import type { Complex } from '../types'

/**
 * Axes are real and imag, never time. Plotly loads through the shared
 * deferred loader so the rest of the page does not initialize it.
 */

export interface ComplexSeries {
  name?: string
  real: number[]
  imag: number[]
}

export interface ComplexPlotProps {
  title: string
  series: ComplexSeries[]
  marks: Complex[]
  /** Stable across slider edits. A new value refits the axes. */
  viewKey?: string
}

export function ComplexPlot({ title, series, marks, viewKey }: ComplexPlotProps) {
  const chrome = chartChrome(readTheme())
  const [Chart, setChart] = useState<ComponentType<ChartProps> | null>(null)

  useEffect(() => {
    let cancelled = false
    void loadPlotlyChart()
      .then((Loaded) => {
        if (!cancelled) setChart(() => Loaded)
      })
      .catch(() => undefined)
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
    uirevision: viewKey,
  }

  if (Chart === null) return <div className="h-72 w-full" />

  return (
    <Chart
      className="h-72 w-full"
      data={data}
      layout={layout}
      config={chartConfig()}
      useResizeHandler
    />
  )
}
