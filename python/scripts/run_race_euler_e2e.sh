#!/usr/bin/env bash
# Live race_euler e2e on GZ Cessna (production 10 m course).
# Requires Docker SITL, tmux, Gazebo plant image.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
export FW_SITL_E2E=1
export FW_SITL_E2E_DURATION_S="${FW_SITL_E2E_DURATION_S:-90}"
export FW_SITL_E2E_WAIT_SLACK_S="${FW_SITL_E2E_WAIT_SLACK_S:-240}"
export FW_SITL_E2E_PLATFORMS="${FW_SITL_E2E_PLATFORMS:-gz}"
cd "${PYTHON_ROOT}"
exec python3 -m unittest tests.test_race_euler_e2e -v
