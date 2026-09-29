#!/usr/bin/env bash
# Host-only F-16 gcas_long mission. No Docker, PX4, or MAVLink.
# Default setup ships gcas_upright, so --maneuver is always passed.
# Runner default is 120.0 s (LONG_HORIZONS / SCENARIO_HORIZONS). Python compare
# uses 115 s (GCAS_LONG_PYTHON_T_END) because AeroBench raises below 60.96 m/s
# near t=118 s; the runner horizon stays 120 s.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
CSV=""
DURATION="120.0"
EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      cat <<EOF
Usage: $0 [--csv PATH] [--duration SECONDS] [--setup PATH] [--anim] [--fg] [extra run_f16.py args]

Host-only gcas_long (default 120.0 s): long-horizon upright GCAS. Runner horizon is 120 s; Python
reference compare uses 115 s (GCAS_LONG_PYTHON_T_END). No Docker, PX4, or MAVLink.

Options:
  --csv PATH          CSV output (default /tmp/f16_gcas_long_<stamp>.csv)
  --duration SECONDS  Integration horizon (default 120.0)
  --setup PATH        Setup JSON (default ${PYTHON_ROOT}/f16_setup.json). This script always
                      overrides maneuver via --maneuver; --duration overrides the setup duration.
  --anim              Matplotlib replay after the CSV is on disk
  --fg                Send replay to FlightGear after the CSV is on disk
  -h, --help          Show this text and exit

--maneuver is fixed by this script. Any other args pass through to run_f16.py.

EXAMPLES:
  $0
  $0 --csv /tmp/f16_gcas_long.csv
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
CSV="${CSV:-/tmp/f16_gcas_long_${STAMP}.csv}"
cd "${PYTHON_ROOT}"
python3 run_f16.py --maneuver gcas_long --duration "${DURATION}" --csv "${CSV}" ${EXTRA[@]+"${EXTRA[@]}"}
echo "${CSV}"
