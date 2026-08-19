#!/usr/bin/env bash
# Root entry: headless balloon race (synth camera). Pass --viz for FG.
# Real launcher: python/scripts/run_balloon_race.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${ROOT}/python/scripts/run_balloon_race.sh" "$@"
