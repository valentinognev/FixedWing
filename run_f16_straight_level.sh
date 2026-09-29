#!/usr/bin/env bash
# Root entry: F-16 straight_level (3.0 s); real launcher python/scripts/run_f16_straight_level.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${ROOT}/python/scripts/run_f16_straight_level.sh" "$@"
