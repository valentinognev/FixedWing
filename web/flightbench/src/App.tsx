import { useEffect, useLayoutEffect, useReducer, useRef, useState } from 'react'
import { ApiError, fetchGains, fetchPlanes, fetchTasks, postRun } from './api'
import { AeroPicker } from './components/AeroPicker'
import { CoefficientCharts } from './components/CoefficientCharts'
import { ControllerDiagram } from './components/ControllerDiagram'
import { GainForm, isValid } from './components/GainForm'
import { LawPicker } from './components/LawPicker'
import { ModesTable } from './components/ModesTable'
import { OpenLoopCard } from './components/OpenLoopCard'
import { PlanePicker } from './components/PlanePicker'
import { RunBanner } from './components/RunBanner'
import { RunPlots } from './components/RunPlots'
import { TaskList } from './components/TaskList'
import { TrimPanel } from './components/TrimPanel'
import {
  aeroChanged,
  lawChanged,
  planeChanged,
  selectionReducer,
  taskChanged,
  trimChanged,
  type Selection,
} from './state'
import { applyTheme, readTheme, type Theme } from './theme'
import type { GainSpec, PlaneInfo, RunResponse, TaskInfo } from './types'

const EMPTY: Selection = {
  plane: '',
  law: 'linear',
  aero: '',
  task: '',
  trim: { vt_mps: 0, altitude_m: 0 },
}

const PANEL = 'flex flex-col gap-4 rounded border border-neutral-800 bg-neutral-900/50 p-4'
const TITLE = 'text-xs font-semibold tracking-wide text-neutral-400 uppercase'
const RUN_BUTTON =
  'rounded border border-sky-700 bg-sky-800 px-4 py-2 text-sm font-semibold ' +
  'text-white hover:bg-sky-700 disabled:cursor-not-allowed ' +
  'disabled:border-neutral-800 disabled:bg-neutral-900 disabled:text-neutral-600'

/** What the API said, whether it said it as a status or as a thrown error. */
function detailOf(error: unknown): string {
  if (error instanceof ApiError) return error.detail
  if (error instanceof Error) return error.message
  return String(error)
}

/** Defaults for a gain form, so a fresh App never blocks on user input. */
function seed(specs: GainSpec[]): Record<string, number> {
  return Object.fromEntries(specs.map((spec) => [spec.name, spec.value]))
}

/** Gains loaded for one plane, law, and task. Edits stay here across task switches. */
interface GainEntry {
  specs: GainSpec[]
  values: Record<string, number>
}

/** The last run of one plane, law, aero, and task, or the error that run returned. */
type ResultSlot =
  | { ok: true; response: RunResponse; gains: Record<string, number> }
  | { ok: false; detail: string }

function gainKey(plane: string, law: string, task: string): string {
  return `${plane} ${law} ${task}`
}

function resultKey(plane: string, law: string, aero: string, task: string): string {
  return `${plane} ${law} ${aero} ${task}`
}

/**
 * Flightbench: pick a plane, law and task, tune the loop gains, run the
 * simulation on the server, then read its traces and modes. The app never
 * computes flight dynamics — it renders what the API returns.
 */
export default function App() {
  const [planes, setPlanes] = useState<PlaneInfo[]>([])
  const [tasks, setTasks] = useState<TaskInfo[]>([])
  const gainsRef = useRef<Record<string, GainEntry>>({})
  const [gainsByKey, setGainsByKey] = useState<Record<string, GainEntry>>({})
  const [results, setResults] = useState<Record<string, ResultSlot>>({})
  const [error, setError] = useState<string | null>(null)
  const [running, setRunning] = useState(false)
  const [selection, dispatch] = useReducer(selectionReducer, EMPTY)
  const [theme, setTheme] = useState<Theme>(readTheme)

  function commitGains(next: Record<string, GainEntry>) {
    gainsRef.current = next
    setGainsByKey(next)
  }

  useLayoutEffect(() => {
    applyTheme(theme)
  }, [theme])

  function switchTheme() {
    const next: Theme = theme === 'dark' ? 'light' : 'dark'
    // Charts read storage during render, so the write has to land first.
    applyTheme(next)
    setTheme(next)
  }

  const plane = planes.find((one) => one.id === selection.plane)
  const task = tasks.find((one) => one.id === selection.task)
  const key =
    selection.plane === ''
      ? ''
      : gainKey(selection.plane, selection.law, selection.task)
  const entry = key === '' ? undefined : gainsByKey[key]

  // The gain form drives the run: every spec of the current selection must
  // hold a finite value, so an unseeded form keeps Run disabled.
  const values = entry?.values ?? {}
  const seeded = (entry?.specs ?? []).every((spec) => Number.isFinite(values[spec.name]))
  const canRun = !running && entry !== undefined && seeded && isValid(values)

  const slot =
    results[resultKey(selection.plane, selection.law, selection.aero, selection.task)]
  const response = slot?.ok === true ? slot.response : null
  // A response only belongs to the selection it was run for.
  const shown =
    response !== null &&
    response.plane === selection.plane &&
    response.law === selection.law &&
    response.task === selection.task &&
    response.aero === selection.aero
  const taskError = slot?.ok === false ? slot.detail : null

  useEffect(() => {
    let cancelled = false
    Promise.all([fetchPlanes(), fetchTasks()])
      .then(([planeList, taskList]) => {
        if (cancelled) return
        setPlanes(planeList)
        setTasks(taskList)
        if (planeList.length > 0) dispatch(planeChanged(planeList[0]))
        if (taskList.length > 0) dispatch(taskChanged(taskList[0].id))
      })
      .catch((cause: unknown) => {
        if (!cancelled) setError(detailOf(cause))
      })
    return () => {
      cancelled = true
    }
  }, [])

  useEffect(() => {
    if (selection.plane === '' || selection.task === '') return
    const wanted = gainKey(selection.plane, selection.law, selection.task)
    if (gainsRef.current[wanted] !== undefined) {
      setError(null)
      return
    }
    let cancelled = false
    fetchGains(selection.plane, selection.law, selection.task)
      .then((returned) => {
        if (gainsRef.current[wanted] !== undefined) return
        commitGains({
          ...gainsRef.current,
          [wanted]: { specs: returned, values: seed(returned) },
        })
        if (!cancelled) setError(null)
      })
      .catch((cause: unknown) => {
        if (!cancelled) setError(detailOf(cause))
      })
    return () => {
      cancelled = true
    }
  }, [selection.plane, selection.law, selection.task])

  function remember(wanted: string, slot: ResultSlot) {
    setResults((current) => ({ ...current, [wanted]: slot }))
  }

  /**
   * Run one task. A task whose gains are already stored posts before the first
   * await, so the open task's request is visible as soon as Run is clicked.
   */
  async function execute(task: string, picked: Selection): Promise<void> {
    const wanted = resultKey(picked.plane, picked.law, picked.aero, task)
    const storedKey = gainKey(picked.plane, picked.law, task)
    let gains = gainsRef.current[storedKey]
    if (gains === undefined) {
      try {
        const specs = await fetchGains(picked.plane, picked.law, task)
        const again = gainsRef.current[storedKey]
        if (again !== undefined) {
          gains = again
        } else {
          gains = { specs, values: seed(specs) }
          commitGains({ ...gainsRef.current, [storedKey]: gains })
        }
      } catch (cause: unknown) {
        remember(wanted, { ok: false, detail: detailOf(cause) })
        return
      }
    }
    const sent = { ...gains.values }
    try {
      const response = await postRun({
        plane: picked.plane,
        law: picked.law,
        task,
        aero: picked.aero,
        trim: { ...picked.trim },
        gains: sent,
      })
      remember(wanted, { ok: true, response, gains: sent })
    } catch (cause: unknown) {
      remember(wanted, { ok: false, detail: detailOf(cause) })
    }
  }

  async function run() {
    setRunning(true)
    setError(null)
    const picked = selection
    const list = tasks
    try {
      for (const task of list) {
        await execute(task.id, picked)
      }
    } finally {
      setRunning(false)
    }
  }

  async function update() {
    setRunning(true)
    setError(null)
    try {
      await execute(selection.task, selection)
    } finally {
      setRunning(false)
    }
  }

  return (
    <div className="flex h-dvh flex-col overflow-hidden bg-neutral-950 text-neutral-100">
      <header className="flex shrink-0 items-center justify-between border-b border-neutral-800 px-6 py-4">
        <h1 className="text-base font-semibold tracking-wide">Flightbench</h1>
        <button
          type="button"
          className="rounded border border-neutral-700 px-3 py-1 text-sm text-neutral-100 hover:bg-neutral-800"
          onClick={switchTheme}
        >
          {theme === 'dark' ? 'Light' : 'Dark'}
        </button>
      </header>
      <main className="grid min-h-0 flex-1 grid-rows-[minmax(0,1fr)_minmax(0,1fr)] gap-6 px-6 py-6 xl:grid-cols-[20rem_minmax(0,1fr)] xl:grid-rows-1">
        <div className="flex min-h-0 flex-col gap-4 overflow-y-auto">
          {plane !== undefined && (
            <section className={PANEL}>
              <h2 className={TITLE}>Airframe</h2>
              <PlanePicker
                planes={planes}
                value={selection.plane}
                onChange={(one) => dispatch(planeChanged(one))}
              />
              <LawPicker
                plane={plane}
                value={selection.law}
                onChange={(law) => dispatch(lawChanged(law))}
              />
              <AeroPicker
                plane={plane}
                value={selection.aero}
                onChange={(aero) => dispatch(aeroChanged(aero))}
              />
              <TrimPanel
                value={selection.trim}
                onChange={(trim) => dispatch(trimChanged(trim))}
              />
            </section>
          )}
          <button
            type="button"
            className={RUN_BUTTON}
            disabled={!canRun}
            onClick={run}
          >
            Run
          </button>
          {plane !== undefined && (
            <section className={PANEL}>
              <h2 className={TITLE}>Loop gains</h2>
              <GainForm
                specs={entry?.specs ?? []}
                values={values}
                onChange={(name, value) => {
                  const current = gainsRef.current
                  const open = current[key]
                  if (open === undefined) return
                  commitGains({
                    ...current,
                    [key]: { ...open, values: { ...open.values, [name]: value } },
                  })
                }}
                onReset={() => {
                  const current = gainsRef.current
                  const open = current[key]
                  if (open === undefined) return
                  commitGains({
                    ...current,
                    [key]: { ...open, values: seed(open.specs) },
                  })
                }}
              />
              <button
                type="button"
                className={RUN_BUTTON}
                disabled={!canRun}
                onClick={update}
              >
                Update
              </button>
            </section>
          )}
          <section className={PANEL}>
            <h2 className={TITLE}>Task</h2>
            <TaskList
              tasks={tasks}
              value={selection.task}
              onChange={(next) => dispatch(taskChanged(next))}
            />
          </section>
        </div>
        <div className="flex min-h-0 min-w-0 flex-col gap-6 overflow-y-auto">
          <RunBanner
            stopReason={shown && response !== null ? response.stop_reason : null}
            error={taskError ?? error}
          />
          {response !== null && shown ? (
            <>
              <ModesTable metrics={response.metrics} />
              <RunPlots response={response} family={task?.family ?? 'longitudinal'} />
              {response.linear_analysis !== undefined && (
                <>
                  <ControllerDiagram diagram={response.linear_analysis.diagram} />
                  {response.linear_analysis.open_loop.map((card, index) => (
                    <OpenLoopCard
                      key={`${card.input}:${card.output}:${index}`}
                      card={card}
                    />
                  ))}
                  <CoefficientCharts
                    coefficients={response.linear_analysis.coefficients}
                    values={slot?.ok === true ? slot.gains : {}}
                  />
                </>
              )}
            </>
          ) : (
            <p className="text-sm text-neutral-500">
              Pick a task and press Run; the traces and modes of the run appear
              here.
            </p>
          )}
        </div>
      </main>
    </div>
  )
}
