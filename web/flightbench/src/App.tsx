import { useEffect, useReducer, useState } from 'react'
import { ApiError, fetchGains, fetchPlanes, fetchTasks, postRun } from './api'
import { AeroPicker } from './components/AeroPicker'
import { GainForm, isValid } from './components/GainForm'
import { LawPicker } from './components/LawPicker'
import { ModesTable } from './components/ModesTable'
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
  'text-neutral-50 hover:bg-sky-700 disabled:cursor-not-allowed ' +
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

/** The gains of one plane/law/task, tagged with the selection they belong to. */
interface GainState {
  key: string
  specs: GainSpec[]
  values: Record<string, number>
}

const NO_GAINS: GainState = { key: '', specs: [], values: {} }

/**
 * Flightbench: pick a plane, law and task, tune the loop gains, run the
 * simulation on the server, then read its traces and modes. The app never
 * computes flight dynamics — it renders what the API returns.
 */
export default function App() {
  const [planes, setPlanes] = useState<PlaneInfo[]>([])
  const [tasks, setTasks] = useState<TaskInfo[]>([])
  const [gains, setGains] = useState<GainState>(NO_GAINS)
  const [response, setResponse] = useState<RunResponse | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [running, setRunning] = useState(false)
  const [selection, dispatch] = useReducer(selectionReducer, EMPTY)

  const plane = planes.find((one) => one.id === selection.plane)
  const task = tasks.find((one) => one.id === selection.task)
  const key =
    selection.plane === ''
      ? ''
      : `${selection.plane} ${selection.law} ${selection.task}`

  // The gain form drives the run: every spec of the current selection must
  // hold a finite value, so an unseeded form keeps Run disabled.
  const loaded = key !== '' && gains.key === key
  const values = loaded ? gains.values : {}
  const seeded = gains.specs.every((spec) => Number.isFinite(values[spec.name]))
  const canRun = !running && loaded && seeded && isValid(values)

  // A response only belongs to the selection it was run for.
  const shown =
    response !== null &&
    response.plane === selection.plane &&
    response.law === selection.law &&
    response.task === selection.task &&
    response.aero === selection.aero

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
    const wanted = `${selection.plane} ${selection.law} ${selection.task}`
    let cancelled = false
    fetchGains(selection.plane, selection.law, selection.task)
      .then((returned) => {
        if (cancelled) return
        setGains({ key: wanted, specs: returned, values: seed(returned) })
        setError(null)
      })
      .catch((cause: unknown) => {
        if (!cancelled) setError(detailOf(cause))
      })
    return () => {
      cancelled = true
    }
  }, [selection.plane, selection.law, selection.task])

  async function run() {
    setRunning(true)
    setError(null)
    try {
      setResponse(
        await postRun({
          plane: selection.plane,
          law: selection.law,
          task: selection.task,
          aero: selection.aero,
          trim: { ...selection.trim },
          gains: values,
        }),
      )
    } catch (cause: unknown) {
      setResponse(null)
      setError(detailOf(cause))
    } finally {
      setRunning(false)
    }
  }

  return (
    <div className="min-h-screen bg-neutral-950 text-neutral-100">
      <header className="border-b border-neutral-800 px-6 py-4">
        <h1 className="text-base font-semibold tracking-wide">Flightbench</h1>
      </header>
      <main className="grid gap-6 px-6 py-6 xl:grid-cols-[20rem_minmax(0,1fr)]">
        <div className="flex flex-col gap-4">
          {plane !== undefined && (
            <>
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
              <section className={PANEL}>
                <h2 className={TITLE}>Loop gains</h2>
                <GainForm
                  specs={gains.specs}
                  values={values}
                  onChange={(name, value) =>
                    setGains((current) => ({
                      ...current,
                      values: { ...current.values, [name]: value },
                    }))
                  }
                  onReset={() =>
                    setGains((current) => ({
                      ...current,
                      values: seed(current.specs),
                    }))
                  }
                />
              </section>
            </>
          )}
          <section className={PANEL}>
            <h2 className={TITLE}>Task</h2>
            <TaskList
              tasks={tasks}
              value={selection.task}
              onChange={(next) => dispatch(taskChanged(next))}
            />
          </section>
          <button
            type="button"
            className={RUN_BUTTON}
            disabled={!canRun}
            onClick={run}
          >
            Run
          </button>
        </div>
        <div className="flex min-w-0 flex-col gap-6">
          <RunBanner
            stopReason={shown && response !== null ? response.stop_reason : null}
            error={error}
          />
          {response !== null && shown ? (
            <>
              <ModesTable metrics={response.metrics} />
              <RunPlots response={response} family={task?.family ?? 'longitudinal'} />
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
