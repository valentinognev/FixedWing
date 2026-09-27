#!/usr/bin/env bash
# Production-course race_quat e2e (same course as ./run_balloon_race.sh --duration 120).
# Scores first-circuit misses on a typical JSBSim spawn; does not gate at 5 m.
# Requires Docker SITL, tmux; viz needs DISPLAY/FG.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
export FW_SITL_E2E=1
export FW_SITL_E2E_DURATION_S="${FW_SITL_E2E_DURATION_S:-120}"
export FW_SITL_E2E_WAIT_SLACK_S="${FW_SITL_E2E_WAIT_SLACK_S:-240}"
# Default headless JSBSim (user ./run_balloon_race.sh). Add viz: jsbsim,viz
export FW_SITL_E2E_PLATFORMS="${FW_SITL_E2E_PLATFORMS:-jsbsim}"
cd "${PYTHON_ROOT}"
exec python3 -m unittest tests.test_race_quat_production_e2e -v
