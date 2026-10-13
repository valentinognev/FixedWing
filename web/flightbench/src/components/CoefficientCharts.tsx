import { useState } from 'react'
import { nyquistScale } from '../nyquistScale'
import type { CoefficientAnalysis, Complex, Locus } from '../types'
import { ComplexPlot, type ComplexSeries } from './ComplexPlot'

export interface CoefficientChartsProps {
  coefficients: CoefficientAnalysis[]
  values: Record<string, number>
}

const PANEL = 'flex flex-col gap-4 rounded border border-neutral-800 bg-neutral-900/50 p-4'
const TITLE = 'text-xs font-semibold tracking-wide text-neutral-400 uppercase'

const CRITICAL: Complex = { real: -1, imag: 0 }

function gainIndex(sweep: number[], gain: number): number {
  const scale = Math.max(1, Math.abs(gain))
  return sweep.findIndex((value) => Math.abs(value - gain) <= 1e-9 * scale)
}

/** Index of the gain posted with the run, or 0 when that gain is missing. */
function initialIndex(locus: Locus, gain: number | undefined): number {
  if (gain === undefined || !Number.isFinite(gain) || locus.values.length === 0) return 0
  const index = gainIndex(locus.values, gain)
  return index < 0 ? 0 : index
}

function indexesFor(
  coefficients: CoefficientAnalysis[],
  values: Record<string, number>,
): Record<string, number> {
  const indexes: Record<string, number> = {}
  for (const coefficient of coefficients) {
    indexes[coefficient.name] = initialIndex(coefficient.locus, values[coefficient.name])
  }
  return indexes
}

function chosenIndex(locus: Locus, gain: number | undefined, stored: number | undefined): number {
  if (stored !== undefined && stored >= 0 && stored < locus.values.length) return stored
  return initialIndex(locus, gain)
}

/** Branch samples at one sweep gain. An unknown gain leaves the mark off. */
function locusMarks(locus: Locus, gain: number | undefined): Complex[] {
  if (gain === undefined || !Number.isFinite(gain)) return []
  const index = gainIndex(locus.values, gain)
  if (index < 0) return []
  return locus.branches.flatMap((branch) => {
    const point = branch[index]
    return point === undefined ? [] : [point]
  })
}

function locusSeries(locus: Locus): ComplexSeries[] {
  return locus.branches.map((branch, index) => ({
    name: `branch ${index + 1}`,
    real: branch.map((point) => point.real),
    imag: branch.map((point) => point.imag),
  }))
}

/**
 * One locus and one Nyquist per coefficient. An empty sweep leaves that chart
 * and its slider out. Each chart keeps its own slider index; a new `values`
 * object (the next Run) puts each index back on the posted gain. A gain-form
 * edit keeps `values`. A slider move keeps the chart's view; a new coefficient
 * list refits it.
 */
export function CoefficientCharts({ coefficients, values }: CoefficientChartsProps) {
  const [indexes, setIndexes] = useState<Record<string, number>>(() =>
    indexesFor(coefficients, values),
  )
  const [nyquistIndexes, setNyquistIndexes] = useState<Record<string, number>>(() =>
    indexesFor(coefficients, values),
  )
  const [trackedValues, setTrackedValues] = useState(values)
  const [trackedCoefficients, setTrackedCoefficients] = useState(coefficients)
  const [viewEpoch, setViewEpoch] = useState(0)
  if (trackedCoefficients !== coefficients) {
    setTrackedCoefficients(coefficients)
    setViewEpoch((epoch) => epoch + 1)
  }
  if (trackedValues !== values) {
    setTrackedValues(values)
    const reset = indexesFor(coefficients, values)
    setIndexes(reset)
    setNyquistIndexes(reset)
  }

  return (
    <>
      {coefficients.map((coefficient) => {
        const locus = coefficient.locus
        const index = chosenIndex(locus, values[coefficient.name], indexes[coefficient.name])
        const sweepGain = locus.values[index]
        const nyquistIndex = chosenIndex(
          locus,
          values[coefficient.name],
          nyquistIndexes[coefficient.name],
        )
        const nyquistGain = locus.values[nyquistIndex]
        const scale =
          nyquistGain === undefined
            ? 1
            : nyquistScale(coefficient.proportional, values[coefficient.name], nyquistGain)
        return (
          <section key={coefficient.name} className={PANEL}>
            <h2 className={TITLE}>{coefficient.name}</h2>
            {locus.values.length > 0 && sweepGain !== undefined && (
              <>
                <ComplexPlot
                  title={`${coefficient.name} locus`}
                  series={locusSeries(locus)}
                  marks={locusMarks(locus, sweepGain)}
                  viewKey={`${coefficient.name}:locus:${viewEpoch}`}
                />
                <div className="flex items-center gap-3">
                  <input
                    type="range"
                    aria-label={`${coefficient.name} locus gain`}
                    min={0}
                    max={locus.values.length - 1}
                    step={1}
                    value={index}
                    onChange={(event) => {
                      const next = Number(event.target.value)
                      setIndexes((current) => ({ ...current, [coefficient.name]: next }))
                    }}
                    className="min-w-0 flex-1"
                  />
                  <span className="shrink-0 text-xs tabular-nums text-neutral-200">
                    {sweepGain.toPrecision(6)}
                  </span>
                </div>
              </>
            )}
            {coefficient.nyquist.omega.length > 0 && (
              <>
                <ComplexPlot
                  title={`${coefficient.name} nyquist`}
                  viewKey={`${coefficient.name}:nyquist:${viewEpoch}`}
                  series={[
                    {
                      name: 'L(jω)',
                      real: coefficient.nyquist.real.map((sample) => sample * scale),
                      imag: coefficient.nyquist.imag.map((sample) => sample * scale),
                    },
                  ]}
                  marks={[CRITICAL]}
                />
                {nyquistGain !== undefined && (
                  <div className="flex items-center gap-3">
                    <input
                      type="range"
                      aria-label={`${coefficient.name} nyquist gain`}
                      min={0}
                      max={locus.values.length - 1}
                      step={1}
                      value={nyquistIndex}
                      disabled={!coefficient.proportional}
                      onChange={(event) => {
                        const next = Number(event.target.value)
                        setNyquistIndexes((current) => ({
                          ...current,
                          [coefficient.name]: next,
                        }))
                      }}
                      className="min-w-0 flex-1"
                    />
                    <span className="shrink-0 text-xs tabular-nums text-neutral-200">
                      {nyquistGain.toPrecision(6)}
                    </span>
                  </div>
                )}
              </>
            )}
          </section>
        )
      })}
    </>
  )
}
