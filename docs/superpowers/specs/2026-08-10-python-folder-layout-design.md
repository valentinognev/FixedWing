# Python folder layout (`fw_sitl` package)

## Goal
Separate utility Python modules from plant entrypoints and shell scripts by moving libraries into `python/fw_sitl/`, shells into `python/scripts/`, spawn data into `python/assets/`, and tests into `python/tests/`. Entrypoint script names/paths at `python/` root stay the same. No intentional flight/CLI behavior changes.

## Decisions (locked)
- Layout approach: entrypoints stay at `python/`; utilities in named package `fw_sitl/` (flat modules inside); shells in `scripts/`; assets in `assets/`; tests in `tests/`.
- No installable `pyproject.toml` in this pass.
- Thin shell shims remain at old `python/runSim*.sh` / `kill.sh` paths (exec into `scripts/`) for muscle-memory compatibility.
- Pure move + path/import fixes; no tuning.

## Target tree

```
python/
  run_straight_flight_jsbsim.py
  run_straight_flight_yasim.py
  run_straight_flight.py              # shim → yasim
  run_straight_flight_headless.py     # shim → jsbsim
  runSimJsbsimRascal.sh               # shim → scripts/
  runSimYasimRascal.sh                # shim → scripts/
  runSimFlightGearRascal.sh           # shim → scripts/runSimYasimRascal.sh
  kill.sh                             # shim → scripts/kill.sh
  fw_sitl/
    __init__.py
    path_geometry.py
    mavlink_io.py
    sim_lifecycle.py
    cli_common.py
    straight_flight_core.py
    flight_history.py
  scripts/
    runSimJsbsimRascal.sh
    runSimYasimRascal.sh
    kill.sh
  assets/
    jsb_spawn.xml
    fg_spawn.env
  tests/
    test_path_setpoint.py
```

## Path resolution

### Shells (`python/scripts/`)
- `SCRIPT_DIR` = directory of the script (`…/python/scripts`)
- `PYTHON_ROOT` = `SCRIPT_DIR/..`
- `REPO_ROOT` = `PYTHON_ROOT/..` (project root; used for `Dockerfiles/…` mounts)
- Spawn: `${PYTHON_ROOT}/assets/jsb_spawn.xml`, `${PYTHON_ROOT}/assets/fg_spawn.env`
- Patch scripts: `${REPO_ROOT}/Dockerfiles/…` (unchanged relative to repo root)

### Python (`fw_sitl.sim_lifecycle`)
- `PACKAGE_DIR` = `Path(__file__).resolve().parent` (`…/fw_sitl`)
- `PYTHON_ROOT` = `PACKAGE_DIR.parent`
- `SCRIPTS_DIR` = `PYTHON_ROOT / "scripts"`
- `ASSETS_DIR` = `PYTHON_ROOT / "assets"`
- `KILL_SCRIPT` = `SCRIPTS_DIR / "kill.sh"`
- Export `PYTHON_ROOT` / `SCRIPTS_DIR` (and keep a compatibility alias if useful). Entrypoints set `DEFAULT_SIM = SCRIPTS_DIR / "runSim….sh"`.
- `start_sim` cwd remains `sim_script.parent`.

### Entrypoint import bootstrap
Each runner (and shim that imports package symbols) inserts `python/` (script parent) onto `sys.path` before importing `fw_sitl`, so `python3 python/run_straight_flight_jsbsim.py` from repo root works without installing the package.

## Imports
- Internal: `from fw_sitl.path_geometry import …`, `from fw_sitl.mavlink_io import …`, etc.
- Intra-package relative or absolute `fw_sitl.*` (prefer absolute `fw_sitl.…` for clarity).
- JSBSim entrypoint keeps star-re-exports from `fw_sitl` modules so `from run_straight_flight_headless import path_setpoint_on_line` still works.
- Tests: `from fw_sitl.path_geometry import …` with `PYTHONPATH=python` or unittest from `python/` after path bootstrap in test module.

## Compatibility
- Public CLI flags/defaults unchanged.
- Python entrypoint paths unchanged.
- Old shell paths work via shims.
- Docker image / container names / mount destinations inside containers unchanged.

## Verification
1. `PYTHONPATH=python python3 -m unittest discover -s python/tests -v` (or equivalent path bootstrap) — path geometry tests pass.
2. `python3 -m py_compile` on entrypoints + `fw_sitl/*.py`.
3. `--help` on both runners and Python shims.
4. Shell `--help` for `scripts/runSim*.sh` and old shim paths; confirm spawn file existence checks resolve under `assets/`.

## Docs
- Update root `README.md` Architecture for the new tree.
- `UPDATES.md` top entry `0.9.0` (layout / packaging structure).

## Out of scope
- `pyproject.toml` / pip install
- Nested subpackages under `fw_sitl`
- Behavior or param tuning
- Moving Dockerfiles
