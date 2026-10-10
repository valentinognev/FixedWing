import type { Mode, RunMetrics } from '../types'

export interface ModesTableProps {
  metrics: RunMetrics
}

const LABELS: Record<Mode['name'], string> = {
  short_period: 'Short period',
  phugoid: 'Phugoid',
  dutch_roll: 'Dutch roll',
  roll: 'Roll',
  spiral: 'Spiral',
}

const HEAD = 'px-2 py-1 text-left text-xs font-semibold tracking-wide text-neutral-400 uppercase'
const CELL = 'px-2 py-1 text-sm text-neutral-200'

function Modes({ title, modes }: { title: string; modes: Mode[] }) {
  if (modes.length === 0) return null

  return (
    <section className="flex flex-col gap-1">
      <h3 className="text-xs font-semibold tracking-wide text-neutral-400 uppercase">
        {title}
      </h3>
      <table className="w-full border-collapse">
        <thead>
          <tr className="border-b border-neutral-800">
            <th className={HEAD}>Mode</th>
            <th className={HEAD}>wn</th>
            <th className={HEAD}>zeta</th>
          </tr>
        </thead>
        <tbody>
          {modes.map((mode) => (
            <tr key={mode.name} className="border-b border-neutral-900">
              <td className={CELL}>{LABELS[mode.name]}</td>
              <td className={`${CELL} tabular-nums`}>{mode.wn.toFixed(3)}</td>
              <td className={`${CELL} tabular-nums`}>{mode.zeta.toFixed(3)}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </section>
  )
}

/** The open-loop modes of the plane, plus the closed-loop ones when it ran. */
export function ModesTable({ metrics }: ModesTableProps) {
  const closed = metrics.closed_loop_modes
  return (
    <div className="flex flex-col gap-4">
      <Modes title="Open loop" modes={metrics.open_loop_modes} />
      {closed !== undefined && <Modes title="Closed loop" modes={closed} />}
    </div>
  )
}
