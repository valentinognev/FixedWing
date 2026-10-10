#!/usr/bin/env bash
# Flightbench: the FastAPI app and the React page on one loopback port.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT=8765
DEV=0

usage() {
  cat <<EOF
Usage: $0 [--dev] [--port N]

Default: if web/flightbench/dist is absent, build it (npm ci && npm run build),
then serve flightbench.api:app and print flightbench: http://127.0.0.1:8765
(the port is the one --port set, otherwise 8765).

  --dev       uvicorn --reload and npm run dev; Ctrl-C kills both
  --port N    listen port (default 8765)
  -h, --help  show this text and exit

An unknown flag exits 2.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dev)
      DEV=1
      shift
      ;;
    --port)
      if [[ $# -lt 2 || ! "$2" =~ ^[0-9]+$ || "$2" -lt 1 || "$2" -gt 65535 ]]; then
        usage >&2
        exit 2
      fi
      PORT="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
done

echo "flightbench: http://127.0.0.1:${PORT}"

if [[ "${DEV}" -eq 1 ]]; then
  api_pid=""
  web_pid=""
  # Background jobs are their own process groups (set -m). Killing the group
  # stops uvicorn's reloader and Vite, which npm does not take down with it.
  stop_group() {
    local pid="$1"
    if [[ -z "${pid}" ]]; then
      return 0
    fi
    kill -- "-${pid}" 2>/dev/null || kill "${pid}" 2>/dev/null || true
  }
  cleanup() {
    trap - INT TERM EXIT
    stop_group "${api_pid}"
    stop_group "${web_pid}"
    wait 2>/dev/null || true
  }
  trap cleanup EXIT
  trap 'cleanup; exit 130' INT
  trap 'cleanup; exit 143' TERM
  # Own process groups so the trap can stop uvicorn's reloader and Vite's children.
  set -m
  (
    cd "${ROOT}/python"
    exec python3 -m uvicorn flightbench.api:app --host 127.0.0.1 --port "${PORT}" --reload
  ) &
  api_pid=$!
  (
    cd "${ROOT}/web/flightbench"
    export FLIGHTBENCH_API_PORT="${PORT}"
    exec npm run dev
  ) &
  web_pid=$!
  status=0
  wait -n || status=$?
  exit "${status}"
fi

if [[ ! -f "${ROOT}/web/flightbench/dist/index.html" ]]; then
  (cd "${ROOT}/web/flightbench" && npm ci && npm run build)
fi

cd "${ROOT}/python"
exec python3 -m uvicorn flightbench.api:app --host 127.0.0.1 --port "${PORT}"
