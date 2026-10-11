#!/usr/bin/env bash
# Start the flightbench control UI on 127.0.0.1:4321 (background).
# A previous instance from this script is stopped first.
#   ./start.sh         stop any previous UI, then serve it
#   ./start.sh --stop  stop the UI and exit (used by ./kill.sh)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT=4321
PIDFILE="/tmp/fixedwing-flightbench-ui.pid"
LOG="/tmp/fixedwing-flightbench-ui.log"
UVICORN_MATCH="[u]vicorn flightbench.api:app --host 127.0.0.1 --port ${PORT}"

stop_ui() {
  local report_absent="${1:-0}"
  local stopped=0
  local pid=""

  if [[ -f "${PIDFILE}" ]]; then
    pid="$(tr -d '[:space:]' < "${PIDFILE}")"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -- "-${pid}" 2>/dev/null || kill "${pid}" 2>/dev/null || true
      stopped=1
    fi
    rm -f "${PIDFILE}"
  fi

  local pids=()
  mapfile -t pids < <(pgrep -f "${UVICORN_MATCH}" || true)
  if [[ ${#pids[@]} -gt 0 ]]; then
    kill -- "${pids[@]}" 2>/dev/null || true
    stopped=1
  fi

  local i
  for i in $(seq 1 50); do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      sleep 0.1
      continue
    fi
    if pgrep -f "${UVICORN_MATCH}" >/dev/null; then
      sleep 0.1
      continue
    fi
    break
  done

  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill -9 -- "-${pid}" 2>/dev/null || kill -9 "${pid}" 2>/dev/null || true
  fi
  mapfile -t pids < <(pgrep -f "${UVICORN_MATCH}" || true)
  if [[ ${#pids[@]} -gt 0 ]]; then
    kill -9 -- "${pids[@]}" 2>/dev/null || true
  fi

  if [[ "${stopped}" -eq 1 ]]; then
    echo "Stopped control UI on port ${PORT}"
  elif [[ "${report_absent}" -eq 1 ]]; then
    echo "No control UI on port ${PORT}"
  fi
}

if [[ "${1:-}" == "--stop" ]]; then
  if [[ $# -ne 1 ]]; then
    echo "Usage: $0 [--stop]" >&2
    exit 2
  fi
  stop_ui 1
  exit 0
fi

if [[ $# -gt 0 ]]; then
  echo "Usage: $0 [--stop]" >&2
  exit 2
fi

stop_ui 0

if [[ ! -f "${ROOT}/web/flightbench/dist/index.html" ]]; then
  (cd "${ROOT}/web/flightbench" && npm ci && npm run build)
fi

cd "${ROOT}/python"
setsid python3 -m uvicorn flightbench.api:app --host 127.0.0.1 --port "${PORT}" >>"${LOG}" 2>&1 </dev/null &
pid=$!
echo "${pid}" >"${PIDFILE}"

ready=0
for _ in $(seq 1 50); do
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "Control UI exited. Log: ${LOG}" >&2
    rm -f "${PIDFILE}"
    exit 1
  fi
  if ss -ltn 2>/dev/null | grep -q ":${PORT} "; then
    ready=1
    break
  fi
  sleep 0.1
done

echo "flightbench: http://127.0.0.1:${PORT} (pid ${pid})"
if [[ "${ready}" -eq 0 ]]; then
  echo "Port ${PORT} is not listening yet. Log: ${LOG}" >&2
  exit 1
fi
