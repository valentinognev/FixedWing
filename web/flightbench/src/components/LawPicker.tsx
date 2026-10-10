import type { Law, PlaneInfo } from '../types'

export interface LawPickerProps {
  plane: PlaneInfo
  value: Law
  onChange: (law: Law) => void
}

const SELECT =
  'rounded border border-neutral-700 bg-neutral-800 px-2 py-1 text-sm text-neutral-100'

export function LawPicker({ plane, value, onChange }: LawPickerProps) {
  return (
    <label className="flex flex-col gap-1">
      <span className="text-xs font-semibold tracking-wide text-neutral-400 uppercase">
        Law
      </span>
      <select
        aria-label="Law"
        className={SELECT}
        value={value}
        onChange={(event) => {
          const law = plane.laws.find((one) => one === event.target.value)
          if (law) onChange(law)
        }}
      >
        {plane.laws.map((law) => (
          <option key={law} value={law}>
            {law}
          </option>
        ))}
      </select>
    </label>
  )
}
