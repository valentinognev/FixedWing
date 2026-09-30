#!/usr/bin/env bash
# Root entry: wings-level trim survey. Real launcher python/scripts/run_trim_survey.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${ROOT}/python/scripts/run_trim_survey.sh" "$@"
