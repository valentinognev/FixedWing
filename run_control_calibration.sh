#!/usr/bin/env bash
# Root entry: chirp SID. Plant from python/flightSetup.json; override with --gz/--yasim/--viz/--jsbsim/--model.
# Procedure: python/controlCallibration/procedure.json
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT}/python"
exec python3 -m controlCallibration run "$@"
