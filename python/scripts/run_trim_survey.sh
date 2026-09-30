#!/usr/bin/env bash
# Host-only wings-level trim survey. No Docker, PX4, or MAVLink.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PLANE="f16"
MODEL="morelli"
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      cat <<EOF
Usage: $0 [--plane PLANE] [--model MODEL]

Solve the wings-level (true airspeed, altitude) trim grid and write
data/planes/<plane>/<model>.json. Defaults: plane f16, model morelli.
Stevens is the other F-16 aero model. The survey takes several minutes.

Options:
  --plane PLANE   Aircraft id (default f16)
  --model MODEL   Aero model (default morelli)
  -h, --help      Show this text and exit

EXAMPLES:
  $0
  $0 --model stevens
  $0 --plane f16 --model morelli
EOF
      exit 0
      ;;
    --plane)
      if [[ $# -lt 2 ]]; then
        echo "missing value for --plane" >&2
        exit 2
      fi
      PLANE="$2"
      shift 2
      ;;
    --model)
      if [[ $# -lt 2 ]]; then
        echo "missing value for --model" >&2
        exit 2
      fi
      MODEL="$2"
      shift 2
      ;;
    *)
      echo "unknown argument $1" >&2
      exit 2
      ;;
  esac
done
cd "${PYTHON_ROOT}"
python3 - "$PLANE" "$MODEL" <<'PY'
import sys

from f16.trim_table import write_trim_survey

try:
    path = write_trim_survey(sys.argv[1], sys.argv[2])
except ValueError as exc:
    print(exc, file=sys.stderr)
    sys.exit(2)
print(path)
PY
