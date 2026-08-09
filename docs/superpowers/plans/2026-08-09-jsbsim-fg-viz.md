# JSBSim + FG Viz (rename YASim) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rename JSBSim/YASim runners, keep YASim plant unchanged, and add `--viz` so FlightGear visualizes the existing JSBSim plant (same FDM as headless).

**Architecture:** Upstream `jsbsim_rascal` already starts `fgfs --fdm=null` when `HEADLESS` is unset. Extend `runSimJsbsimRascal.sh` with `--viz` (X11 + no `HEADLESS` + FG 2024 launch patch). Rename Python/shell entrypoints; leave thin compat shims.

**Tech Stack:** bash, Docker host networking, PX4 SITL `jsbsim_bridge` / `flightgear_bridge`, FlightGear 2024.1.6 AppImage in `px4-noble-sim-ros`, Python 3 + pymavlink.

**Spec:** `docs/superpowers/specs/2026-08-09-jsbsim-fg-viz-design.md`

## Global Constraints

- Do not replace YASim with FG-internal JSBSim via `flightgear_bridge`.
- Do not change PX4 airframe IDs `1033` / `1039`.
- JSBSim default remains headless (`HEADLESS=1`).
- Container names stay `px4-noble-jsbsim-rascal` (JSBSim) and `px4-noble-sim-ros` (YASim).
- `kill.sh` flags stay `--jsbsim` / `--fg` (docs call them JSBSim / YASim).
- Root `README.md` + `UPDATES.md` only markdown updates required besides this plan/spec tree.
- **Commits:** this repo forbids `git commit` unless the user explicitly asks; skip commit steps until asked (still stage mentally / leave clean diffs).

---

## File map

| File | Responsibility |
|------|----------------|
| `Dockerfiles/patch_px4_jsbsim_fg_viz.sh` | Runtime-patch JSBSim `sitl_run.sh` FG launch for FG 2024 + logs |
| `python/runSimJsbsimRascal.sh` | Headless default; `--viz` → X11 + no HEADLESS + apply patch |
| `python/runSimYasimRascal.sh` | Renamed YASim FG runner (former `runSimFlightGearRascal.sh`) |
| `python/runSimFlightGearRascal.sh` | Compat shim → `runSimYasimRascal.sh` |
| `python/run_straight_flight_jsbsim.py` | Renamed headless control; `--viz` passthrough |
| `python/run_straight_flight_yasim.py` | Renamed YASim control; import JSBSim core |
| `python/run_straight_flight_headless.py` | Compat shim → jsbsim |
| `python/run_straight_flight.py` | Compat shim → yasim |
| `README.md` / `UPDATES.md` / `Dockerfiles/README.md` | Document rename + `--viz` |

---

### Task 1: JSBSim FG viz runtime patch script

**Files:**
- Create: `Dockerfiles/patch_px4_jsbsim_fg_viz.sh`
- Test: shell dry-run against a copied `sitl_run.sh` snippet (or container file)

**Interfaces:**
- Consumes: PX4 tree path as `$1` (default `/home/valentin/PX4-Autopilot`)
- Produces: idempotent edits to `Tools/simulation/jsbsim/sitl_run.sh` FG launch block

- [ ] **Step 1: Create the patch script**

```bash
#!/usr/bin/env bash
# Harden JSBSim sitl_run.sh FlightGear viz launch for FG 2024 (TerraSync off, keep logs).
set -euo pipefail
PX4_ROOT="${1:-/home/valentin/PX4-Autopilot}"
SITL_RUN="${PX4_ROOT}/Tools/simulation/jsbsim/sitl_run.sh"
if [[ ! -f "${SITL_RUN}" ]]; then
  echo "missing ${SITL_RUN}" >&2
  exit 1
fi
# Idempotent: skip if already patched.
if grep -q 'FIXEDWING_JSBSIM_FG_VIZ' "${SITL_RUN}"; then
  echo "jsbsim sitl_run.sh already patched for FG viz"
  exit 0
fi
python3 - <<'PY' "${SITL_RUN}"
import pathlib, sys
path = pathlib.Path(sys.argv[1])
text = path.read_text()
old = """\t$FG_BINARY --fdm=null \\
\t\t--native-fdm=socket,in,60,,5550,udp \\
\t\t--aircraft=$JSBSIM_AIRCRAFT_MODEL \\
\t\t--airport=${world} \\
\t\t--disable-hud \\
\t\t--disable-ai-models &> /dev/null &
\tFGFS_PID=$!"""
new = """\t# FIXEDWING_JSBSIM_FG_VIZ
\tmkdir -p /tmp
\t$FG_BINARY --fdm=null \\
\t\t--native-fdm=socket,in,60,,5550,udp \\
\t\t--aircraft=$JSBSIM_AIRCRAFT_MODEL \\
\t\t--airport=${world} \\
\t\t--disable-hud \\
\t\t--disable-ai-models \\
\t\t--disable-terrasync \\
\t\t> /tmp/jsbsim_fgfs.log 2>&1 &
\tFGFS_PID=$!
\techo \"FlightGear viz PID=${FGFS_PID}; logs: /tmp/jsbsim_fgfs.log\""""
if old not in text:
    raise SystemExit("sitl_run.sh FG block not found; inspect upstream and update patch")
path.write_text(text.replace(old, new, 1))
print("patched", path)
PY
```

Make executable: `chmod +x Dockerfiles/patch_px4_jsbsim_fg_viz.sh`

- [ ] **Step 2: Verify patch finds the block in the image**

Run:

```bash
docker run --rm --entrypoint bash px4-noble-sim-ros:latest -c \
  'grep -A8 "FG_BINARY --fdm=null" /home/valentin/PX4-Autopilot/Tools/simulation/jsbsim/sitl_run.sh'
```

Expected: block matches the `old` string in the patch (tabs as shown). If whitespace differs, adjust `old`/`new` before merging.

- [ ] **Step 3: Dry-run patch on a host copy**

```bash
docker cp "$(docker create --name fw-jsb-tmp px4-noble-sim-ros:latest):/home/valentin/PX4-Autopilot/Tools/simulation/jsbsim/sitl_run.sh" /tmp/sitl_run.sh
docker rm fw-jsb-tmp
cp /tmp/sitl_run.sh /tmp/sitl_run.sh.bak
# Simulate: mount-less — run python replace against /tmp by temporarily pointing,
# or: bash -c 'PX4=/tmp/px4; mkdir -p $PX4/Tools/simulation/jsbsim; cp /tmp/sitl_run.sh.bak $PX4/Tools/simulation/jsbsim/sitl_run.sh; Dockerfiles/patch_px4_jsbsim_fg_viz.sh $PX4; grep FIXEDWING_JSBSIM_FG_VIZ $PX4/Tools/simulation/jsbsim/sitl_run.sh'
```

Expected: `FIXEDWING_JSBSIM_FG_VIZ` present; second run prints “already patched”.

- [ ] **Step 4: Commit** (only if user explicitly asks)

```bash
git add Dockerfiles/patch_px4_jsbsim_fg_viz.sh
git commit -m "feat: patch JSBSim sitl_run FG viz for FG 2024"
```

---

### Task 2: `runSimJsbsimRascal.sh --viz`

**Files:**
- Modify: `python/runSimJsbsimRascal.sh`
- Consumes: `Dockerfiles/patch_px4_jsbsim_fg_viz.sh` from Task 1

**Interfaces:**
- Produces: CLI `runSimJsbsimRascal.sh [--viz] [--kill] [--help]`
- Env: still honors `PX4_SITL_NO_DOCKER_TTY`, `PX4_JSBSIM_DOCKER_NAME`, `PX4_SITL_DOCKER_VER`

- [ ] **Step 1: Rewrite argument parse + viz docker path**

Replace the option loop and `docker run` section so the script supports `VIZ=0|1`. Structure (full file intent):

```bash
#!/bin/bash
# Start PX4 SITL + JSBSim Rascal. Default: headless. --viz: FG window (--fdm=null).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONTAINER_NAME="${PX4_JSBSIM_DOCKER_NAME:-px4-noble-jsbsim-rascal}"
IMAGE_TAG="${PX4_SITL_DOCKER_VER:-px4-noble-sim-ros:latest}"
SPAWN_XML="${SCRIPT_DIR}/jsb_spawn.xml"
CONTAINER_SCENE="/home/valentin/PX4-Autopilot/Tools/simulation/jsbsim/jsbsim_bridge/scene/LSZH.xml"
PATCH_SCRIPT="${REPO_ROOT}/Dockerfiles/patch_px4_jsbsim_fg_viz.sh"
CONTAINER_PATCH="/tmp/patch_px4_jsbsim_fg_viz.sh"
VIZ=0

cleanup_on_exit() {
	echo ""
	echo "Cleaning up..."
	docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true
	if [[ "${VIZ}" -eq 1 ]]; then
		xhost -local:docker 2>/dev/null || true
	fi
}

while [[ $# -gt 0 ]]; do
	case "$1" in
		--help|-h)
			echo "Usage: $0 [--viz] [--kill]"
			echo "  Starts JSBSim Rascal SITL with IC from ${SPAWN_XML}"
			echo "  --viz   FlightGear visualization (same JSBSim plant; no HEADLESS)"
			echo "  --kill  Remove container and exit"
			exit 0
			;;
		--viz) VIZ=1 ;;
		--kill) cleanup_on_exit; exit 0 ;;
		*) echo "Unknown option: $1 (use --help)" >&2; exit 1 ;;
	esac
	shift
done
```

Headless `docker run` stays as today (`HEADLESS=1`, spawn volume only).

For `VIZ=1`, require patch file; set up X11 like `runSimFlightGearRascal.sh` (DISPLAY, xhost, X11-unix, XAUTHORITY, XDG_RUNTIME_DIR); mount spawn + patch; run:

```bash
docker run ... \
  --env="DISPLAY=${DISPLAY}" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="XAUTHORITY=${XAUTH_FILE}" \
  --env="XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR}" \
  --volume="${SPAWN_XML}:${CONTAINER_SCENE}:ro" \
  --volume="${PATCH_SCRIPT}:${CONTAINER_PATCH}:ro" \
  ... X11 volumes ... \
  "${IMAGE_TAG}" \
  /bin/bash -lc "set -euo pipefail
    cd /home/valentin/PX4-Autopilot
    bash '${CONTAINER_PATCH}' /home/valentin/PX4-Autopilot
    BRIDGE_BIN=build/px4_sitl_default/build_jsbsim_bridge/jsbsim_bridge
    test -x \"\${BRIDGE_BIN}\"
    # Unset HEADLESS so sitl_run.sh starts fgfs --fdm=null
    unset HEADLESS || true
    make px4_sitl jsbsim_rascal
  "
```

Echo clearly: `Starting ... JSBSim Rascal with FG viz` vs `headless`.

- [ ] **Step 2: Syntax-check**

```bash
bash -n python/runSimJsbsimRascal.sh
python/runSimJsbsimRascal.sh --help
```

Expected: help lists `--viz`; exit 0.

- [ ] **Step 3: Smoke `--kill` only** (no full sim if CI-less)

```bash
python/runSimJsbsimRascal.sh --kill
```

Expected: cleans named container without error.

- [ ] **Step 4: Commit** (only if user asks)

---

### Task 3: Rename YASim shell runner + shim

**Files:**
- Create: `python/runSimYasimRascal.sh` (copy of current `runSimFlightGearRascal.sh` with comment renames)
- Replace: `python/runSimFlightGearRascal.sh` with shim

**Interfaces:**
- Produces: `runSimYasimRascal.sh [--kill] [--help]` — same behavior as old FG runner

- [ ] **Step 1: Copy and retitle**

```bash
cp python/runSimFlightGearRascal.sh python/runSimYasimRascal.sh
```

Edit header comments in `runSimYasimRascal.sh` to say YASim / FlightGear FDM (not “the” FG path as primary). Keep `fg_spawn.env`, patch script, container name, `make px4_sitl_nolockstep flightgear_rascal`.

- [ ] **Step 2: Replace old name with shim**

`python/runSimFlightGearRascal.sh`:

```bash
#!/bin/bash
# Renamed: use runSimYasimRascal.sh (YASim FlightGear plant).
echo "note: runSimFlightGearRascal.sh → runSimYasimRascal.sh" >&2
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/runSimYasimRascal.sh" "$@"
```

- [ ] **Step 3: Verify shim**

```bash
bash -n python/runSimYasimRascal.sh python/runSimFlightGearRascal.sh
python/runSimFlightGearRascal.sh --help
```

Expected: stderr rename note; help from YASim script.

- [ ] **Step 4: Commit** (only if user asks)

---

### Task 4: Rename JSBSim Python flight script + `--viz` + shim

**Files:**
- Create: `python/run_straight_flight_jsbsim.py` (from `run_straight_flight_headless.py`)
- Replace: `python/run_straight_flight_headless.py` with shim
- Modify: `start_sim` to accept extra argv

**Interfaces:**
- Produces: `start_sim(sim_script: Path, *, extra_args: list[str] | None = None) -> subprocess.Popen`
- CLI: `--viz` on jsbsim flight script

- [ ] **Step 1: Move file**

```bash
git mv python/run_straight_flight_headless.py python/run_straight_flight_jsbsim.py
# If git mv awkward with dirty tree: cp then write shim over old path
```

- [ ] **Step 2: Update module docstring / strings / plot title**

In `run_straight_flight_jsbsim.py`:
- Docstring: “JSBSim Rascal” not “headless-only”; document `--viz`.
- Argparse description: JSBSim.
- Plot title: `"JSBSim straight flight"` or `"JSBSim + FG viz straight flight"` when `--viz`.
- Help text for `--sim`: still `runSimJsbsimRascal.sh`.

- [ ] **Step 3: Extend `start_sim` and wire `--viz`**

```python
def start_sim(
    sim_script: Path,
    *,
    extra_args: list[str] | None = None,
) -> subprocess.Popen:
    if not sim_script.is_file():
        raise FileNotFoundError(sim_script)
    kill_sim(
        sim_script,
        label=f"Stopping any previous simulation container ({sim_script.name})...",
    )
    cmd = ["bash", str(sim_script), *(extra_args or [])]
    print(f"Starting simulation: {' '.join(cmd)}")
    env = os.environ.copy()
    env["PX4_SITL_NO_DOCKER_TTY"] = "1"
    return subprocess.Popen(
        cmd,
        cwd=str(sim_script.parent),
        start_new_session=True,
        env=env,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
```

Add argparse:

```python
parser.add_argument(
    "--viz",
    action="store_true",
    help="Start FlightGear as visualization for the JSBSim plant (same FDM as headless)",
)
```

When starting sim:

```python
extra = ["--viz"] if args.viz else []
start_sim(args.sim, extra_args=extra)
```

Any other `start_sim(sim_script)` call sites (engage retry full restart) must pass the same `extra` if the original run used `--viz`. Store `sim_extra_args = ["--viz"] if args.viz else []` and thread into `engage_offboard_with_retries` **or** simpler: when `full_sim_restart` and `args.viz`, pass extras.

Minimal approach that matches spec: add optional `sim_extra_args: list[str] | None = None` to `engage_offboard_with_retries` and use it in `start_sim(...)` inside the retry path. Default `None` → `[]`. YASim caller unchanged.

- [ ] **Step 4: Compat shim `run_straight_flight_headless.py`**

```python
#!/usr/bin/env python3
"""Renamed: use run_straight_flight_jsbsim.py."""
from __future__ import annotations
import runpy
import sys
from pathlib import Path

print("note: run_straight_flight_headless.py → run_straight_flight_jsbsim.py", file=sys.stderr)
sys.argv[0] = str(Path(__file__).resolve().parent / "run_straight_flight_jsbsim.py")
runpy.run_path(sys.argv[0], run_name="__main__")
```

- [ ] **Step 5: Import / syntax check**

```bash
python3 -m py_compile python/run_straight_flight_jsbsim.py python/run_straight_flight_headless.py
python3 python/run_straight_flight_jsbsim.py --help | head
```

Expected: `--viz` in help; shim prints note when invoked.

- [ ] **Step 6: Commit** (only if user asks)

---

### Task 5: Rename YASim Python flight script + shim

**Files:**
- Create: `python/run_straight_flight_yasim.py` (from `run_straight_flight.py`)
- Replace: `python/run_straight_flight.py` with shim

**Interfaces:**
- Consumes: `import run_straight_flight_jsbsim as core`
- Produces: `DEFAULT_SIM = .../runSimYasimRascal.sh`

- [ ] **Step 1: Move and retarget imports**

```bash
# prefer: copy content to run_straight_flight_yasim.py then shim the old file
```

In `run_straight_flight_yasim.py`:
- Docstring: YASim FlightGear plant; point at `run_straight_flight_jsbsim.py` for shared control.
- `import run_straight_flight_jsbsim as core`
- `DEFAULT_SIM = SCRIPT_DIR / "runSimYasimRascal.sh"`
- Plot title: `"YASim FlightGear straight flight"`
- Keep engage flags: `full_sim_restart=False`, `accept_unhealthy=True`, `arm_timeout_s=45.0`

- [ ] **Step 2: Shim old name**

`python/run_straight_flight.py` — same pattern as headless shim → `run_straight_flight_yasim.py`.

- [ ] **Step 3: Compile + help**

```bash
python3 -m py_compile python/run_straight_flight_yasim.py python/run_straight_flight.py
python3 python/run_straight_flight_yasim.py --help | head
```

Expected: default sim path mentions `runSimYasimRascal.sh`.

- [ ] **Step 4: Commit** (only if user asks)

---

### Task 6: Docs (`README` / `UPDATES` / Dockerfiles README)

**Files:**
- Modify: `README.md`
- Modify: `UPDATES.md` (feature bump → `0.7.0`)
- Modify: `Dockerfiles/README.md` (mention JSBSim `--viz` / patch script if FG/JSBSim launch is documented)

- [ ] **Step 1: Update root README architecture bullets**

Replace headless/FG naming with:
- `run_straight_flight_jsbsim.py` — JSBSim plant; optional `--viz` (FG window, same FDM)
- `run_straight_flight_yasim.py` — YASim FlightGear plant
- `runSimJsbsimRascal.sh` / `runSimYasimRascal.sh`
- Note old names still shim

- [ ] **Step 2: Top `UPDATES.md` entry**

```markdown
## 0.7.0 - JSBSim FG viz + YASim rename
- Rename headless → `run_straight_flight_jsbsim.py` / keep `runSimJsbsimRascal.sh`; add `--viz` (FG `--fdm=null` on same JSBSim plant).
- Rename FG YASim path → `run_straight_flight_yasim.py` + `runSimYasimRascal.sh`; old names shim.
- `Dockerfiles/patch_px4_jsbsim_fg_viz.sh`: TerraSync off + FG logs for JSBSim viz launch.
```

- [ ] **Step 3: Dockerfiles README one-liner** under JSBSim runtime: `runSimJsbsimRascal.sh --viz` applies `patch_px4_jsbsim_fg_viz.sh`.

- [ ] **Step 4: Commit** (only if user asks)

---

### Task 7: Manual verification

**Files:** none (runtime)

- [ ] **Step 1: JSBSim headless (no viz)**

```bash
python3 python/run_straight_flight_jsbsim.py --duration=20 --no-plot
```

Expected: no FG window; arm + hold; exits cleanly.

- [ ] **Step 2: JSBSim + viz**

```bash
python3 python/run_straight_flight_jsbsim.py --viz --duration=20 --no-plot
```

Expected: FG window with Rascal; container `px4-noble-jsbsim-rascal`; if FG fails, check `docker exec … cat /tmp/jsbsim_fgfs.log`.

- [ ] **Step 3: YASim smoke** (optional if time/GPU)

```bash
python3 python/run_straight_flight_yasim.py --duration=20 --no-plot
```

Expected: YASim container path still engages (or clear engage logs).

- [ ] **Step 4: Shim smoke**

```bash
python3 python/run_straight_flight_headless.py --help 2>&1 | head -5
python3 python/run_straight_flight.py --help 2>&1 | head -5
```

Expected: rename notes on stderr; help from new scripts.

---

## Spec coverage checklist

| Spec item | Task |
|-----------|------|
| JSBSim + optional FG viz same plant | 1, 2, 4, 7 |
| Keep YASim path | 3, 5, 7 |
| Renames + shims | 3–5 |
| `--viz` on JSBSim flight script | 4 |
| FG 2024 TerraSync + logs | 1, 2 |
| README / UPDATES | 6 |
| No airframe ID change / no dual-bridge | (non-goals; no task) |

## Plan self-review

- No TBD placeholders in steps.
- `start_sim(..., extra_args=)` and `engage_offboard_with_retries(..., sim_extra_args=)` named consistently in Task 4.
- Patch marker `FIXEDWING_JSBSIM_FG_VIZ` unique and idempotent.
- Commit steps gated on explicit user request (repo rule).
