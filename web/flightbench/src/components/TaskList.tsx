import type { TaskFamily, TaskInfo } from '../types'

export interface TaskListProps {
  tasks: TaskInfo[]
  value: string
  onChange: (task: string) => void
}

const FAMILIES: { family: TaskFamily; heading: string }[] = [
  { family: 'longitudinal', heading: 'Longitudinal' },
  { family: 'lateral', heading: 'Lateral' },
]

const ITEM =
  'flex w-full flex-col gap-0.5 rounded border border-neutral-800 bg-neutral-900 px-3 py-2 text-left aria-pressed:border-neutral-500 aria-pressed:bg-neutral-800'

export function TaskList({ tasks, value, onChange }: TaskListProps) {
  return (
    <div className="flex flex-col gap-4">
      {FAMILIES.map(({ family, heading }) => {
        const group = tasks.filter((task) => task.family === family)
        if (group.length === 0) return null
        return (
          <section key={family} className="flex flex-col gap-1">
            <h3 className="text-xs font-semibold tracking-wide text-neutral-400 uppercase">
              {heading}
            </h3>
            <ul className="flex flex-col gap-1">
              {group.map((task) => (
                <li key={task.id}>
                  <button
                    type="button"
                    aria-pressed={value === task.id}
                    className={ITEM}
                    onClick={() => onChange(task.id)}
                  >
                    <span className="text-sm text-neutral-100">{task.label}</span>
                    <span className="text-xs text-neutral-500">{task.lesson}</span>
                  </button>
                </li>
              ))}
            </ul>
          </section>
        )
      })}
    </div>
  )
}
