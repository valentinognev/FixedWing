import type { ComponentType } from 'react'
import type { Config, Data, Layout } from 'plotly.js-dist-min'

/**
 * Shared deferred Plotly factory. The import runs once, the first time a
 * chart is shown, so the rest of the page does not initialize Plotly.
 */

export interface ChartProps {
  className?: string
  data: Data[]
  layout: Partial<Layout>
  config?: Partial<Config>
  useResizeHandler?: boolean
}

let chartPromise: Promise<ComponentType<ChartProps>> | undefined

export function loadPlotlyChart(): Promise<ComponentType<ChartProps>> {
  chartPromise ??= Promise.all([
    import('react-plotly.js/factory'),
    import('plotly.js-dist-min'),
  ])
    .then(([factory, Plotly]) => factory.default(Plotly.default))
    .catch((error: unknown) => {
      chartPromise = undefined
      throw error
    })
  return chartPromise
}
