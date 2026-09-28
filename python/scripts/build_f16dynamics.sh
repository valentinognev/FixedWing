#!/usr/bin/env bash
# Host-only build of the f16dynamics extension. Not invoked by unittest.
set -euo pipefail
REF="/home/valentin/Projects/FlightSimulation/F16/f16-flight-dynamics"
python3 -m pip install --upgrade scikit-build cmake ninja pybind11
python3 -m pip install "$REF"
python3 -c "import f16dynamics; print(f16dynamics.__file__)"
