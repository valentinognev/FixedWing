import createPlotlyComponent from 'react-plotly.js/factory'
import Plotly from 'plotly.js-dist-min'
import type { Layout } from 'plotly.js-dist-min'
import type { PlotShape, PlotTrace } from '../plotting'

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

const TICK = '#a3a3a3'
const GRID = '#262626'
const LINE = '#404040'

const AXIS: Layout['xaxis'] = {
  gridcolor: GRID,
  linecolor: LINE,
  zerolinecolor: LINE,
  tickfont: { color: TICK, size: 10 },
}

export function Plot({ title, traces, unit, xRange, shapes }: PlotProps) {
  const xaxis: Layout['xaxis'] = {
    ...AXIS,
    title: { text: 'time (s)' },
    ...(xRange !== undefined ? { range: xRange } : {}),
  }
  const yaxis: Layout['yaxis'] = { ...AXIS, title: { text: unit } }

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
          font: { color: '#e5e5e5', size: 12 },
          x: 0,
          xanchor: 'left',
        },
        margin: { l: 52, r: 12, t: 30, b: 34 },
        paper_bgcolor: '#0a0a0a',
        plot_bgcolor: '#0a0a0a',
        font: { color: '#d4d4d4', size: 11 },
        showlegend: traces.length > 1,
        legend: { orientation: 'h', y: -0.22, font: { size: 10 } },
        hovermode: 'x unified',
        xaxis,
        yaxis,
        shapes,
      }}
      config={{ displayModeBar: false, responsive: true }}
      useResizeHandler
    />
  )
}
