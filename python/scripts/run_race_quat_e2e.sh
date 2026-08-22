#!/usr/bin/env bash
# Live race_quat e2e for each sim.platform (jsbsim|viz|yasim|gz).
# Requires Docker SITL, tmux; viz/yasim need DISPLAY/FG; gz needs Gazebo image.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
export FW_SITL_E2E=1
export FW_SITL_E2E_DURATION_S="${FW_SITL_E2E_DURATION_S:-90}"
export FW_SITL_E2E_MIN_PASSES="${FW_SITL_E2E_MIN_PASSES:-1}"
export FW_SITL_E2E_WAIT_SLACK_S="${FW_SITL_E2E_WAIT_SLACK_S:-240}"
# Optional: FW_SITL_E2E_PLATFORMS=jsbsim,gz
cd "${PYTHON_ROOT}"
exec python3 -m unittest tests.test_race_quat_e2e -v
