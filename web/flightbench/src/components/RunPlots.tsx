import type { RunResponse, TaskFamily } from '../types'
import { signalsFor, stopShapes, tracesFor, unitOf, xRangeOf } from '../plotting'
import { Plot } from './Plot'

export interface RunPlotsProps {
  response: RunResponse
  family: TaskFamily
}

/** One chart per signal of the task family, all sharing the response time axis. */
export function RunPlots({ response, family }: RunPlotsProps) {
  const signals = signalsFor(family)
  const range = xRangeOf(response)
  const shapes = stopShapes(response.stopped_at)

  return (
    <div className="grid gap-4 xl:grid-cols-2">
      {signals.map((signal) => (
        <Plot
          key={signal}
          title={signal}
          traces={tracesFor(signal, response)}
          unit={unitOf(signal)}
          xRange={range}
          shapes={shapes}
        />
      ))}
    </div>
  )
}
