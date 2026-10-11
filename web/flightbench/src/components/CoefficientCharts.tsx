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

/** The branch samples at the gain the run sent. The sweep itself is already in the payload. */
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

/** One locus and one Nyquist per coefficient. An empty sweep leaves that chart blank. */
export function CoefficientCharts({ coefficients, values }: CoefficientChartsProps) {
  return (
    <>
      {coefficients.map((coefficient) => (
        <section key={coefficient.name} className={PANEL}>
          <h2 className={TITLE}>{coefficient.name}</h2>
          {coefficient.locus.values.length > 0 && (
            <ComplexPlot
              title={`${coefficient.name} locus`}
              series={locusSeries(coefficient.locus)}
              marks={locusMarks(coefficient.locus, values[coefficient.name])}
            />
          )}
          {coefficient.nyquist.omega.length > 0 && (
            <ComplexPlot
              title={`${coefficient.name} nyquist`}
              series={[
                {
                  name: 'L(jω)',
                  real: coefficient.nyquist.real,
                  imag: coefficient.nyquist.imag,
                },
              ]}
              marks={[CRITICAL]}
            />
          )}
        </section>
      ))}
    </>
  )
}
