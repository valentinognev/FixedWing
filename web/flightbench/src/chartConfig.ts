import type { Config } from 'plotly.js-dist-min'

/** Shared Plotly mode bar: zoom, pan, and the reset that restores the authored ranges. */
export function chartConfig(): Partial<Config> {
  return {
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
  }
}
