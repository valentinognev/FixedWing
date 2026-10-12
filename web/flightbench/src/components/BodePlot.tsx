import { useEffect, useState, type ComponentType } from 'react'
import type { Data, Layout } from 'plotly.js-dist-min'
import { chartConfig } from '../chartConfig'
import { loadPlotlyChart, type ChartProps } from '../plotlyChart'
import { chartChrome, readTheme } from '../theme'

/**
 * Magnitude and phase of one open-loop channel. Frequency is logarithmic.
 * The two value axes are independent of each other and of the frequency axis.
 */

export interface BodePlotProps {
  title: string
  omega: number[]
  magnitudeDb: number[]
  phaseDeg: number[]
}

export function BodePlot({ title, omega, magnitudeDb, phaseDeg }: BodePlotProps) {
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

  const data: Data[] = [
    {
      name: 'magnitude (dB)',
      x: omega,
      y: magnitudeDb,
      type: 'scatter',
      mode: 'lines',
      yaxis: 'y',
    },
    {
      name: 'phase (deg)',
      x: omega,
      y: phaseDeg,
      type: 'scatter',
      mode: 'lines',
      yaxis: 'y2',
    },
  ]

  const axis: Layout['xaxis'] = {
    gridcolor: chrome.grid,
    linecolor: chrome.line,
    zerolinecolor: chrome.line,
    tickfont: { color: chrome.tick, size: 10 },
  }
  const xaxis: Layout['xaxis'] = {
    ...axis,
    type: 'log',
    title: { text: 'ω (rad/s)' },
  }
  const yaxis: Layout['yaxis'] = {
    ...axis,
    title: { text: 'magnitude (dB)' },
  }
  const yaxis2: Layout['yaxis'] = {
    ...axis,
    title: { text: 'phase (deg)' },
    overlaying: 'y',
    side: 'right',
  }
  const layout: Partial<Layout> = {
    title: {
      text: title,
      font: { color: chrome.title, size: 12 },
      x: 0,
      xanchor: 'left',
    },
    margin: { l: 52, r: 52, t: 30, b: 40 },
    paper_bgcolor: chrome.paper,
    plot_bgcolor: chrome.paper,
    font: { color: chrome.font, size: 11 },
    showlegend: true,
    legend: { orientation: 'h', y: -0.22, font: { size: 10 } },
    hovermode: 'x unified',
    xaxis,
    yaxis,
    yaxis2,
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
