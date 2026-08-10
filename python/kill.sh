#!/bin/bash
# Compat shim: real script lives in scripts/.
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/kill.sh" "$@"
