#!/usr/bin/env bash
# Root entry: stop balloon-race tmux + JSBSim (+ mavlink fan-out).
# Extra args are forwarded to python/scripts/kill.sh (e.g. --all, --fg, --gz).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="${BALLOON_RACE_TMUX_SESSION:-balloon_race}"
KILL_SH="${ROOT}/python/scripts/kill.sh"

# has-session / kill-session print "no server running..." when no tmux server exists.
if tmux has-session -t "${SESSION}" 2>/dev/null; then
  tmux kill-session -t "${SESSION}" 2>/dev/null || true
  echo "Killed tmux session: ${SESSION}"
else
  echo "No tmux session: ${SESSION}"
fi

if [[ $# -eq 0 ]]; then
  exec bash "${KILL_SH}" --jsbsim
fi
exec bash "${KILL_SH}" "$@"
