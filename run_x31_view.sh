#!/usr/bin/env bash
# Host-only X-31 third-person view. The window follows the aircraft.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_ROOT="${ROOT}/python"
STAMP="$(date +%Y%m%d_%H%M%S)"
CSV=""
MANEUVER="waypoint"
DURATION=""
EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      cat <<EOF
Usage: $0 [--csv PATH] [--duration SECONDS] [--maneuver NAME] [extra run_x31.py args]

Flies an X-31 trajectory, then opens a third-person matplotlib window on the aircraft (elevation 30°, azimuth 45°, short trail). This is the F-16 --anim camera. The path-wide picture is run_x31.py --anim; this script passes --follow, and the runner refuses both together.

Default maneuver is waypoint (40 s, the AeroBench four-point list). No Docker, PX4, or MAVLink.

Options:
  --csv PATH          CSV output (default /tmp/x31_view_<stamp>.csv)
  --duration SECONDS  Integration horizon (default: the maneuver's own)
  --maneuver NAME     gcas_upright, gcas_inverted, gcas_long, ahead, waypoint, or a data-file scenario
  -h, --help          Show this text and exit

Any other args pass through to run_x31.py. --maneuver is consumed here, so a second --maneuver in the extras is the runner's.

EXAMPLES:
  $0
  $0 --maneuver gcas_upright
  $0 --duration 12 --csv /tmp/x31_view.csv
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
    --maneuver)
      MANEUVER="$2"
      shift 2
      ;;
    *)
      EXTRA+=("$1")
      shift
      ;;
  esac
done
CSV="${CSV:-/tmp/x31_view_${STAMP}.csv}"
cd "${PYTHON_ROOT}"
ARGS=(run_x31.py --maneuver "${MANEUVER}" --csv "${CSV}" --follow)
if [[ -n "${DURATION}" ]]; then
  ARGS+=(--duration "${DURATION}")
fi
python3 "${ARGS[@]}" ${EXTRA[@]+"${EXTRA[@]}"}
echo "${CSV}"
