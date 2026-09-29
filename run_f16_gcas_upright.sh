#!/usr/bin/env bash
# Root entry: F-16 gcas_upright (3.51 s); real launcher python/scripts/run_f16_gcas_upright.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${ROOT}/python/scripts/run_f16_gcas_upright.sh" "$@"
