import type { Complex, OpenLoop } from '../types'

export interface OpenLoopCardProps {
  card: OpenLoop
}

const PANEL = 'flex flex-col gap-4 rounded border border-neutral-800 bg-neutral-900/50 p-4'
const TITLE = 'text-xs font-semibold tracking-wide text-neutral-400 uppercase'
const HEAD = 'px-2 py-1 text-left text-xs font-semibold tracking-wide text-neutral-400 uppercase'
const CELL = 'px-2 py-1 text-sm text-neutral-200'

/** Six significant figures, the page equivalent of Python format(value, ".6g"). */
function formatCoeff(value: number): string {
  return value.toPrecision(6)
}

function factor(magnitude: number, power: number): string {
  const coeff = formatCoeff(magnitude)
  if (power === 0) return coeff
  if (power === 1) return `${coeff} s`
  return `${coeff} s^${power}`
}

/** Descending-power polynomial. The API already cancelled and made the denominator monic. */
function polynomial(coeffs: number[]): string {
  const degree = coeffs.length - 1
  const parts: string[] = []
  for (let index = 0; index < coeffs.length; index += 1) {
    const coeff = coeffs[index]
    if (coeff === 0) continue
    const body = factor(Math.abs(coeff), degree - index)
    if (parts.length === 0) {
      parts.push(coeff < 0 ? `-${body}` : body)
    } else {
      parts.push(`${coeff < 0 ? '-' : '+'} ${body}`)
    }
  }
  return parts.length === 0 ? '0' : parts.join(' ')
}

function isReal(sample: Complex): boolean {
  return Math.abs(sample.imag) <= 1e-9 * Math.max(1, Math.abs(sample.real))
}

function nearly(left: number, right: number): boolean {
  return Math.abs(left - right) <= 1e-9 * Math.max(1, Math.abs(left), Math.abs(right))
}

function formatSamples(samples: Complex[]): string {
  const used = samples.map(() => false)
  const parts: string[] = []
  samples.forEach((sample, index) => {
    if (used[index]) return
    used[index] = true
    if (isReal(sample)) {
      parts.push(formatCoeff(sample.real))
      return
    }
    const pair = samples.findIndex(
      (other, otherIndex) =>
        !used[otherIndex] &&
        nearly(other.real, sample.real) &&
        nearly(other.imag, -sample.imag),
    )
    if (pair >= 0) {
      used[pair] = true
      parts.push(`${formatCoeff(sample.real)} ± ${formatCoeff(Math.abs(sample.imag))}j`)
      return
    }
    const sign = sample.imag < 0 ? '-' : '+'
    parts.push(`${formatCoeff(sample.real)} ${sign} ${formatCoeff(Math.abs(sample.imag))}j`)
  })
  return parts.length === 0 ? 'none' : parts.join(', ')
}

function MatrixTable({
  title,
  columns,
  rows,
  data,
}: {
  title: string
  columns: string[]
  rows: string[]
  data: number[][]
}) {
  return (
    <table className="w-full border-collapse">
      <caption className={`${HEAD} caption-top`}>{title}</caption>
      <thead>
        <tr className="border-b border-neutral-800">
          <th className={HEAD} scope="col" />
          {columns.map((column, index) => (
            <th key={`${column}-${index}`} className={HEAD} scope="col">
              {column}
            </th>
          ))}
        </tr>
      </thead>
      <tbody>
        {data.map((row, rowIndex) => (
          <tr key={`${rows[rowIndex] ?? 'row'}-${rowIndex}`} className="border-b border-neutral-900">
            <th className={HEAD} scope="row">
              {rows[rowIndex] ?? ''}
            </th>
            {row.map((value, columnIndex) => (
              <td key={`${columns[columnIndex] ?? 'col'}-${columnIndex}`} className={`${CELL} tabular-nums`}>
                {formatCoeff(value)}
              </td>
            ))}
          </tr>
        ))}
      </tbody>
    </table>
  )
}

/** State-space headings and the cancelled Laplace ratio. Coefficients print at 6 significant figures. */
export function OpenLoopCard({ card }: OpenLoopCardProps) {
  const ratio = `${polynomial(card.numerator)} / (${polynomial(card.denominator)})`
  return (
    <section className={PANEL}>
      <h2 className={TITLE}>
        {card.input} → {card.output}
      </h2>
      <p className="text-xs text-neutral-500">Units are the model&apos;s (radians, seconds).</p>
      <MatrixTable title="A" columns={card.states} rows={card.states} data={card.A} />
      <MatrixTable title="B" columns={[card.input]} rows={card.states} data={card.B} />
      <MatrixTable title="C" columns={card.states} rows={[card.output]} data={card.C} />
      <MatrixTable title="D" columns={[card.input]} rows={[card.output]} data={card.D} />
      <p className="text-sm text-neutral-100">{`G(s) = ${ratio}`}</p>
      <p className="text-sm text-neutral-300">Poles: {formatSamples(card.poles)}</p>
      <p className="text-sm text-neutral-300">Zeros: {formatSamples(card.zeros)}</p>
    </section>
  )
}
