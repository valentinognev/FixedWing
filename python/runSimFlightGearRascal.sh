#!/bin/bash
# Renamed: use runSimYasimRascal.sh (YASim FlightGear plant).
echo "note: runSimFlightGearRascal.sh → runSimYasimRascal.sh" >&2
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runSimYasimRascal.sh" "$@"
