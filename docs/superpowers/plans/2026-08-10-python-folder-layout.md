# Python Folder Layout Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move utilities into `python/fw_sitl/`, shells into `python/scripts/`, assets into `python/assets/`, tests into `python/tests/`; keep entrypoints at `python/` root with `fw_sitl` imports and path bootstrap.

**Architecture:** Named package `fw_sitl` (flat modules); shells/assets/tests as sibling folders; thin shims at old shell paths; `sim_lifecycle` exposes `PYTHON_ROOT` / `SCRIPTS_DIR`.

**Tech Stack:** Python 3, bash, existing Docker runners.

**Spec:** `docs/superpowers/specs/2026-08-10-python-folder-layout-design.md`

## Global Constraints

- No flight/CLI behavior changes — layout + imports + path resolution only.
- Entrypoint paths `python/run_straight_flight_*.py` unchanged.
- Shell shims at old `python/runSim*.sh` / `kill.sh` paths.
- No `pyproject.toml`.
- **Commits:** skip unless user asks.

---

## File map

| Action | Path |
|--------|------|
| Create | `python/fw_sitl/__init__.py` |
| Move | `path_geometry.py`, `mavlink_io.py`, `sim_lifecycle.py`, `cli_common.py`, `straight_flight_core.py`, `flight_history.py` → `fw_sitl/` |
| Move | `runSimJsbsimRascal.sh`, `runSimYasimRascal.sh`, `kill.sh` → `scripts/` |
| Move | `jsb_spawn.xml`, `fg_spawn.env` → `assets/` |
| Move | `test_path_setpoint.py` → `tests/` |
| Rewrite | old shell paths → thin exec shims |
| Modify | entrypoints, package imports, shell REPO_ROOT/asset paths, README, UPDATES |

---

### Task 1: Create dirs and move files

- [ ] Create `fw_sitl/`, `scripts/`, `assets/`, `tests/`
- [ ] `git mv` (or `mv`) modules, shells, assets, test into place
- [ ] Write empty `fw_sitl/__init__.py`
- [ ] Leave entrypoints + Python shims at `python/` root

### Task 2: Fix package imports + `sim_lifecycle` paths

- [ ] Update intra-package imports to `from fw_sitl.…`
- [ ] `sim_lifecycle`: `PYTHON_ROOT`, `SCRIPTS_DIR`, `ASSETS_DIR`, `KILL_SCRIPT = SCRIPTS_DIR / "kill.sh"`; remove old `SCRIPT_DIR` or alias `SCRIPT_DIR = SCRIPTS_DIR` for any leftover refs
- [ ] Entrypoints: path bootstrap + `from fw_sitl.…`; `DEFAULT_SIM = SCRIPTS_DIR / "…"`
- [ ] Tests: import `fw_sitl.path_geometry`; bootstrap path in test file

### Task 3: Fix shells + shims

- [ ] Update `scripts/*.sh`: `PYTHON_ROOT`, `REPO_ROOT`, asset paths under `assets/`
- [ ] Replace root-level shell files with `exec` shims into `scripts/`

### Task 4: Docs + verify

- [ ] README Architecture tree
- [ ] UPDATES `0.9.0`
- [ ] `PYTHONPATH=python python3 -m unittest discover -s python/tests -v`
- [ ] `py_compile` + `--help` on runners/shims + shell `--help`

---

Plan saved. Executing inline in this session.
