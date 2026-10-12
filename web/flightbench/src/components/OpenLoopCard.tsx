import { Fragment } from 'react'
import type { Complex, OpenLoop } from '../types'
import { BodePlot } from './BodePlot'

export interface OpenLoopCardProps {
  card: OpenLoop
}

const PANEL = 'flex flex-col gap-4 rounded border border-neutral-800 bg-neutral-900/50 p-4'
const TITLE = 'text-xs font-semibold tracking-wide text-neutral-400 uppercase'
const HEAD = 'px-2 py-1 text-left text-xs font-semibold tracking-wide text-neutral-400 uppercase'
const CELL = 'px-2 py-1 text-sm text-neutral-200'

/** |entry| and a pole real part at or under this are value-zero and pole-axis. */
const AXIS_BAND = 1e-8

const MATRIX_ROLE: Record<string, string> = {
  A: 'matrix-a',
  B: 'matrix-b',
  C: 'matrix-c',
  D: 'matrix-d',
}

/** Six significant figures, the page equivalent of Python format(value, ".6g"). */
function formatCoeff(value: number): string {
  return value.toPrecision(6)
}

function valueRole(value: number): string {
  if (Math.abs(value) <= AXIS_BAND) return 'value-zero'
  return value > 0 ? 'value-positive' : 'value-negative'
}

function poleRole(real: number): string {
  if (Math.abs(real) <= AXIS_BAND) return 'pole-axis'
  return real > 0 ? 'pole-unstable' : 'pole-stable'
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

interface SampleToken {
  text: string
  role: string
}

function sampleTokens(samples: Complex[], roleFor: (sample: Complex) => string): SampleToken[] {
  const used = samples.map(() => false)
  const parts: SampleToken[] = []
  samples.forEach((sample, index) => {
    if (used[index]) return
    used[index] = true
    if (isReal(sample)) {
      parts.push({ text: formatCoeff(sample.real), role: roleFor(sample) })
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
      parts.push({
        text: `${formatCoeff(sample.real)} ± ${formatCoeff(Math.abs(sample.imag))}j`,
        role: roleFor(sample),
      })
      return
    }
    const sign = sample.imag < 0 ? '-' : '+'
    parts.push({
      text: `${formatCoeff(sample.real)} ${sign} ${formatCoeff(Math.abs(sample.imag))}j`,
      role: roleFor(sample),
    })
  })
  return parts
}

function SampleLine({ label, tokens }: { label: string; tokens: SampleToken[] }) {
  return (
    <p className="text-sm text-neutral-300">
      {label}:{' '}
      {tokens.length === 0
        ? 'none'
        : tokens.map((token, index) => (
            <Fragment key={`${token.text}-${index}`}>
              {index > 0 ? ', ' : null}
              <span data-role={token.role}>{token.text}</span>
            </Fragment>
          ))}
    </p>
  )
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
      <caption className={`${HEAD} caption-top`} data-role={MATRIX_ROLE[title]}>
        {title}
      </caption>
      <thead>
        <tr className="border-b border-neutral-800">
          <th className={HEAD} scope="col" />
          {columns.map((column, index) => (
            <th
              key={`${column}-${index}`}
              className={HEAD}
              scope="col"
              {...(column ? { 'data-role': 'axis-name' } : {})}
            >
              {column}
            </th>
          ))}
        </tr>
      </thead>
      <tbody>
        {data.map((row, rowIndex) => (
          <tr key={`${rows[rowIndex] ?? 'row'}-${rowIndex}`} className="border-b border-neutral-900">
            <th
              className={HEAD}
              scope="row"
              {...(rows[rowIndex] ? { 'data-role': 'axis-name' } : {})}
            >
              {rows[rowIndex] ?? ''}
            </th>
            {row.map((value, columnIndex) => (
              <td
                key={`${columns[columnIndex] ?? 'col'}-${columnIndex}`}
                className={`${CELL} tabular-nums`}
                data-role={valueRole(value)}
              >
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
  return (
    <section className={PANEL}>
      <h2 className={TITLE} data-role="card-title">
        {card.input} → {card.output}
      </h2>
      <p className="text-xs text-neutral-500">Units are the model&apos;s (radians, seconds).</p>
      <MatrixTable title="A" columns={card.states} rows={card.states} data={card.A} />
      <MatrixTable title="B" columns={[card.input]} rows={card.states} data={card.B} />
      <MatrixTable title="C" columns={card.states} rows={[card.output]} data={card.C} />
      <MatrixTable title="D" columns={[card.input]} rows={[card.output]} data={card.D} />
      <p className="text-sm text-neutral-100" data-role="tf-label">
        {'G(s) = '}
        <span data-role="tf-num">{polynomial(card.numerator)}</span>
        {' / ('}
        <span data-role="tf-den">{polynomial(card.denominator)}</span>
        {')'}
      </p>
      <SampleLine label="Poles" tokens={sampleTokens(card.poles, (sample) => poleRole(sample.real))} />
      <SampleLine label="Zeros" tokens={sampleTokens(card.zeros, () => 'zero')} />
      {card.bode !== undefined && card.bode.omega.length > 0 ? (
        <BodePlot
          title={`${card.input} → ${card.output}`}
          omega={card.bode.omega}
          magnitudeDb={card.bode.magnitude_db}
          phaseDeg={card.bode.phase_deg}
        />
      ) : null}
    </section>
  )
}
