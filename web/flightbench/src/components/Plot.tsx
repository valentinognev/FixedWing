import createPlotlyComponent from 'react-plotly.js/factory'
import Plotly from 'plotly.js-dist-min'
import type { Layout } from 'plotly.js-dist-min'
import { chartConfig } from '../chartConfig'
import type { PlotShape, PlotTrace } from '../plotting'
import { chartChrome, readTheme } from '../theme'

/**
 * The only module that touches Plotly. It renders pre-built traces, so tests
 * (and the rest of the app) never need a Plotly runtime.
 */

const PlotlyChart = createPlotlyComponent(Plotly)

export interface PlotProps {
  title: string
  traces: PlotTrace[]
  unit: string
  xRange?: [number, number]
  shapes: PlotShape[]
}

export function Plot({ title, traces, unit, xRange, shapes }: PlotProps) {
  const chrome = chartChrome(readTheme())
  const axis: Layout['xaxis'] = {
    gridcolor: chrome.grid,
    linecolor: chrome.line,
    zerolinecolor: chrome.line,
    tickfont: { color: chrome.tick, size: 10 },
  }
  const xaxis: Layout['xaxis'] = {
    ...axis,
    title: { text: 'time (s)' },
    ...(xRange !== undefined ? { range: xRange } : {}),
  }
  const yaxis: Layout['yaxis'] = { ...axis, title: { text: unit } }

  return (
    <PlotlyChart
      className="h-60 w-full"
      data={traces.map((trace) => ({
        name: trace.name,
        x: trace.x,
        y: trace.y,
        type: 'scatter',
        mode: 'lines',
        line: trace.line,
      }))}
      layout={{
        title: {
          text: title,
          font: { color: chrome.title, size: 12 },
          x: 0,
          xanchor: 'left',
        },
        margin: { l: 52, r: 12, t: 30, b: 34 },
        paper_bgcolor: chrome.paper,
        plot_bgcolor: chrome.paper,
        font: { color: chrome.font, size: 11 },
        showlegend: traces.length > 1,
        legend: { orientation: 'h', y: -0.22, font: { size: 10 } },
        hovermode: 'x unified',
        xaxis,
        yaxis,
        shapes,
      }}
      config={chartConfig()}
      useResizeHandler
    />
  )
}
