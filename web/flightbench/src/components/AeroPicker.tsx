import type { PlaneInfo } from '../types'

export interface AeroPickerProps {
  plane: PlaneInfo
  value: string
  onChange: (aero: string) => void
}

const SELECT =
  'rounded border border-neutral-700 bg-neutral-800 px-2 py-1 text-sm text-neutral-100'

export function AeroPicker({ plane, value, onChange }: AeroPickerProps) {
  if (plane.aero_models.length < 2) return null

  return (
    <label className="flex flex-col gap-1">
      <span className="text-xs font-semibold tracking-wide text-neutral-400 uppercase">
        Aero
      </span>
      <select
        aria-label="Aero"
        className={SELECT}
        value={value}
        onChange={(event) => {
          const aero = plane.aero_models.find((one) => one === event.target.value)
          if (aero) onChange(aero)
        }}
      >
        {plane.aero_models.map((aero) => (
          <option key={aero} value={aero}>
            {aero}
          </option>
        ))}
      </select>
    </label>
  )
}
