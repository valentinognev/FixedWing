#!/usr/bin/env bash
# Host-only F-16 gcas_upright mission. No Docker, PX4, or MAVLink.
# Default setup ships gcas_upright, so --maneuver is always passed.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
CSV=""
DURATION="3.51"
EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      cat <<EOF
Usage: $0 [--csv PATH] [--duration SECONDS] [--setup PATH] [--anim] [--fg] [extra run_f16.py args]

Host-only gcas_upright (default 3.51 s): upright GCAS dive-recovery incl. return to standby. No Docker, PX4, or MAVLink.

Options:
  --csv PATH          CSV output (default /tmp/f16_gcas_upright_<stamp>.csv)
  --duration SECONDS  Integration horizon (default 3.51)
  --setup PATH        Setup JSON (default ${PYTHON_ROOT}/f16_setup.json). This script always
                      overrides maneuver via --maneuver; --duration overrides the setup duration.
  --anim              Matplotlib replay after the CSV is on disk
  --fg                Send replay to FlightGear after the CSV is on disk
  -h, --help          Show this text and exit

--maneuver is fixed by this script. Any other args pass through to run_f16.py.

EXAMPLES:
  $0
  $0 --csv /tmp/f16_gcas_upright.csv
  $0 --duration 3
EOF
      exit 0
      ;;
    --csv)
      CSV="$2"
      shift 2
      ;;
    --duration)
      DURATION="$2"
      shift 2
      ;;
    *)
      EXTRA+=("$1")
      shift
      ;;
  esac
done
CSV="${CSV:-/tmp/f16_gcas_upright_${STAMP}.csv}"
cd "${PYTHON_ROOT}"
python3 run_f16.py --maneuver gcas_upright --duration "${DURATION}" --csv "${CSV}" ${EXTRA[@]+"${EXTRA[@]}"}
echo "${CSV}"
