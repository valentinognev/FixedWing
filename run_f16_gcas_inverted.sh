#!/usr/bin/env bash
# Root entry: F-16 gcas_inverted (10.0 s); real launcher python/scripts/run_f16_gcas_inverted.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${ROOT}/python/scripts/run_f16_gcas_inverted.sh" "$@"
