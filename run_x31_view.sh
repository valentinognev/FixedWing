#!/usr/bin/env bash
# Host-only X-31 replay. --follow stays on the aircraft; --anim keeps the whole path.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_ROOT="${ROOT}/python"
STAMP="$(date +%Y%m%d_%H%M%S)"
CSV=""
MANEUVER="waypoint"
DURATION=""
CAMERA=""
EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      cat <<EOF
Usage: $0 [--csv PATH] [--duration SECONDS] [--maneuver NAME] [extra run_x31.py args]

Flies an X-31 trajectory, then opens one matplotlib window. With no camera flag, and with --follow, the window stays on the aircraft (elevation 30°, azimuth 45°, short trail). --anim keeps the whole path in one frame and the aircraft moves across it. The runner prints which one opened. Passing both exits 2 before the flight.

Default maneuver is waypoint (40 s, the AeroBench four-point list). No Docker, PX4, or MAVLink.

Options:
  --csv PATH          CSV output (default /tmp/x31_view_<stamp>.csv)
  --duration SECONDS  Integration horizon (default: the maneuver's own)
  --maneuver NAME     gcas_upright, gcas_inverted, gcas_long, ahead, waypoint, or a data-file scenario
  --follow            Third-person window on the aircraft (the default)
  --anim              Path-wide frame; the aircraft translates through it
  -h, --help          Show this text and exit

Any other args pass through to run_x31.py. --maneuver, --anim, and --follow are consumed here.

EXAMPLES:
  $0
  $0 --follow
  $0 --anim
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
    --anim|--follow)
      if [[ -n "${CAMERA}" && "${CAMERA}" != "$1" ]]; then
        echo "--follow and --anim open different cameras; pass one" >&2
        exit 2
      fi
      CAMERA="$1"
      shift
      ;;
    *)
      EXTRA+=("$1")
      shift
      ;;
  esac
done
CAMERA="${CAMERA:---follow}"
CSV="${CSV:-/tmp/x31_view_${STAMP}.csv}"
cd "${PYTHON_ROOT}"
ARGS=(run_x31.py --maneuver "${MANEUVER}" --csv "${CSV}" "${CAMERA}")
if [[ -n "${DURATION}" ]]; then
  ARGS+=(--duration "${DURATION}")
fi
python3 "${ARGS[@]}" ${EXTRA[@]+"${EXTRA[@]}"}
echo "${CSV}"
