import type { GainSpec } from '../types'

export interface GainFormProps {
  specs: GainSpec[]
  values: Record<string, number>
  onChange: (name: string, value: number) => void
  onReset: () => void
}

const INPUT =
  'w-24 rounded border border-neutral-700 bg-neutral-800 px-2 py-1 text-sm text-neutral-100'

function shown(value: number | undefined): number | '' {
  return value !== undefined && Number.isFinite(value) ? value : ''
}

/** A blocked field (NaN from an empty or non-numeric entry) keeps Run disabled. */
export function isValid(values: Record<string, number>): boolean {
  return Object.values(values).every((value) => Number.isFinite(value))
}

export function GainForm({ specs, values, onChange, onReset }: GainFormProps) {
  return (
    <div className="flex flex-col gap-3">
      {specs.map((spec) => (
        <label key={spec.name} className="flex items-center justify-between gap-2">
          <span className="text-sm text-neutral-300">{spec.label}</span>
          <span className="flex items-center gap-1">
            <input
              type="number"
              aria-label={spec.label}
              className={INPUT}
              value={shown(values[spec.name])}
              onChange={(event) => onChange(spec.name, event.target.valueAsNumber)}
            />
            <span className="w-16 text-xs text-neutral-500">{spec.unit}</span>
          </span>
        </label>
      ))}
      <button
        type="button"
        className="self-start rounded border border-neutral-700 px-2 py-1 text-sm text-neutral-200 hover:bg-neutral-800"
        onClick={onReset}
      >
        Reset to defaults
      </button>
    </div>
  )
}
