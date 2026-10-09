import type { PlaneInfo } from '../types'

export interface PlanePickerProps {
  planes: PlaneInfo[]
  value: string
  onChange: (plane: PlaneInfo) => void
}

const SELECT =
  'rounded border border-neutral-700 bg-neutral-800 px-2 py-1 text-sm text-neutral-100'

export function PlanePicker({ planes, value, onChange }: PlanePickerProps) {
  return (
    <label className="flex flex-col gap-1">
      <span className="text-xs font-semibold tracking-wide text-neutral-400 uppercase">
        Plane
      </span>
      <select
        aria-label="Plane"
        className={SELECT}
        value={value}
        onChange={(event) => {
          const plane = planes.find((one) => one.id === event.target.value)
          if (plane) onChange(plane)
        }}
      >
        {planes.map((plane) => (
          <option key={plane.id} value={plane.id}>
            {plane.label}
          </option>
        ))}
      </select>
    </label>
  )
}
