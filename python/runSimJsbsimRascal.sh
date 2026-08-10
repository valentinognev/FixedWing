#!/bin/bash
# Compat shim: real runner lives in scripts/.
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/runSimJsbsimRascal.sh" "$@"
