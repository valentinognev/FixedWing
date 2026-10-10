import type { Selection } from '../state'

export interface TrimPanelProps {
  value: Selection['trim']
  onChange: (trim: Selection['trim']) => void
}

const INPUT =
  'w-24 rounded border border-neutral-700 bg-neutral-800 px-2 py-1 text-sm text-neutral-100'

function shown(value: number): number | '' {
  return Number.isFinite(value) ? value : ''
}

export function TrimPanel({ value, onChange }: TrimPanelProps) {
  const field = (name: 'vt_mps' | 'altitude_m', label: string, unit: string) => (
    <label className="flex flex-col gap-1">
      <span className="text-xs font-semibold tracking-wide text-neutral-400 uppercase">
        {label}
      </span>
      <span className="flex items-center gap-1">
        <input
          type="number"
          aria-label={label}
          className={INPUT}
          value={shown(value[name])}
          onChange={(event) =>
            onChange({ ...value, [name]: event.target.valueAsNumber })
          }
        />
        <span className="text-xs text-neutral-500">{unit}</span>
      </span>
    </label>
  )

  return (
    <fieldset className="flex gap-4 rounded border border-neutral-800 p-3">
      <legend className="px-1 text-xs font-semibold tracking-wide text-neutral-400 uppercase">
        Trim
      </legend>
      {field('vt_mps', 'Airspeed', 'm/s')}
      {field('altitude_m', 'Altitude', 'm')}
    </fieldset>
  )
}
