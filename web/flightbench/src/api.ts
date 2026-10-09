import type {
  GainSpec,
  PlaneInfo,
  RunRequest,
  RunResponse,
  TaskInfo,
} from './types'

/** The FastAPI app is same-origin in production and proxied by Vite in dev. */
const API = '/api'

/** A non-2xx response, carrying the server's `detail` for the UI to show. */
export class ApiError extends Error {
  readonly status: number
  readonly detail: string

  constructor(status: number, detail: string) {
    super(detail)
    this.name = 'ApiError'
    this.status = status
    this.detail = detail
  }
}

async function detailOf(response: Response): Promise<string> {
  const text = await response.text()
  try {
    const body: unknown = JSON.parse(text)
    if (typeof body === 'object' && body !== null && 'detail' in body) {
      const { detail } = body as { detail: unknown }
      return typeof detail === 'string' ? detail : JSON.stringify(detail)
    }
  } catch {
    // Not JSON: the raw body is the most useful detail there is.
  }
  return text || response.statusText
}

async function request<T>(path: string, init?: RequestInit): Promise<T> {
  const response = await fetch(path, init)
  if (!response.ok) {
    throw new ApiError(response.status, await detailOf(response))
  }
  return (await response.json()) as T
}

export function fetchPlanes(): Promise<PlaneInfo[]> {
  return request<PlaneInfo[]>(`${API}/planes`)
}

export function fetchTasks(): Promise<TaskInfo[]> {
  return request<TaskInfo[]>(`${API}/tasks`)
}

export function fetchGains(
  plane: string,
  law: string,
  task: string,
): Promise<GainSpec[]> {
  const query = new URLSearchParams({ plane, law, task }).toString()
  return request<GainSpec[]>(`${API}/gains?${query}`)
}

export function postRun(run: RunRequest): Promise<RunResponse> {
  return request<RunResponse>(`${API}/runs`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(run),
  })
}
