#!/usr/bin/env bash
# tmux launcher for balloon-race: sim → balloons → heartbeat → image + camera + control.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP="${PYTHON_ROOT}/flightSetup.json"
SESSION="balloon_race"
MODE="synth"
VIZ=0
GZ=0
YASIM=0
XPLANE=0
PLATFORM_CLI=0
GZ_MODEL="rc_cessna"
MODEL_SET=0
NO_SIM=0
NO_PLOT=0
DETACH=0
DURATION=""
# --viz/--yasim: PX4's EKF dead-reckons (no GPS aiding) and drifts from
# FG/JSBSim ground truth. Guidance always rebases pos/att from FG telnet.
# --ekf-fix gps is disabled (crashed / never armed); see UPDATES.md 0.35.1.
# Kept as a reject-only flag so old command lines fail loudly.
EKF_FIX="rebase"
CONTAINER_NAME="${PX4_JSBSIM_DOCKER_NAME:-px4-noble-jsbsim-rascal}"
# Balloon race needs distinct UDP feeds for control + image-source.
MAVLINK_FANOUT="${MAVLINK_FANOUT:-1}"
MAVLINK_CONTROL_PORT="${MAVLINK_CONTROL_PORT:-14540}"
MAVLINK_IMAGE_PORT="${MAVLINK_IMAGE_PORT:-14541}"
# Wait for PX4 heartbeat on control UDP after fan-out before starting peers.
HEARTBEAT_TIMEOUT_S="${HEARTBEAT_TIMEOUT_S:-120}"

usage() {
  echo "Usage: $0 [--viz] [--gz] [--yasim] [--model rc_cessna|advanced_plane] [--setup PATH] [--session NAME] [--no-sim] [--duration SEC] [--no-plot] [--detach]"
  echo "  Defaults from flightSetup.json sim.platform / sim.gz_model / sim.duration_s;"
  echo "  plant flags and --duration override those fields when passed."
  echo "  sim.platform / flags: jsbsim (default) | viz | yasim | gz  (xplane not available)"
  echo "  --viz       FG viz sim + fg image capture"
  echo "  --gz        Gazebo plane + onboard camera (--mode gz); exclusive with --viz"
  echo "  --yasim     YASim FG Rascal FDM + fg camera; exclusive with --viz/--gz"
  echo "  --model     gz model (requires --gz or sim.platform=gz); default from setup"
  echo "  --no-sim    control connects to existing sim (do not kill docker)"
  echo "  --duration  race length seconds (0 = no time limit; default: sim.duration_s)"
  echo "  --no-plot   skip post-race plots (PNG + desktop viewer; default: show)"
  echo "  --detach    leave tmux in the background (default: attach, control pane)"
  echo "  --ekf-fix   rebase only (--viz/--yasim). gps is disabled (see UPDATES.md 0.35.1)"
  echo "  Enables mavlink-server fan-out by default (MAVLINK_FANOUT=1);"
  echo "  aborts image/control if fan-out is not running."
  echo "  After sim/fan-out: place FG/GZ balloons, then wait for HEARTBEAT on UDP ${MAVLINK_CONTROL_PORT}"
  echo "  (timeout HEARTBEAT_TIMEOUT_S=${HEARTBEAT_TIMEOUT_S}s, fail if none)."
  echo "  Timed end: host matplotlib window (zoom/pan, shared time axis)."
  exit 0
}

mavlink_fanout_up() {
  if pgrep -f '[m]avlink-server' >/dev/null 2>&1; then
    return 0
  fi
  if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}-mavlink"; then
    return 0
  fi
  if [[ "${GZ}" -eq 1 ]] && docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${PX4_GZ_DOCKER_NAME:-px4-noble-gz-plane}-mavlink"; then
    return 0
  fi
  if [[ "${YASIM}" -eq 1 ]] && docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${PX4_SITL_DOCKER_NAME:-px4-noble-sim-ros}-mavlink"; then
    return 0
  fi
  if [[ "${XPLANE}" -eq 1 ]] && docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${PX4_XP_DOCKER_NAME:-px4-noble-xplane-cessna}-mavlink"; then
    return 0
  fi
  return 1
}

# Bind control UDP briefly, wait for vehicle HEARTBEAT, then release the port.
wait_control_heartbeat() {
  local port="$1"
  local timeout_s="$2"
  echo "Waiting for MAVLink heartbeat on UDP ${port} (timeout ${timeout_s}s)..."
  if ! python3 - "$port" "$timeout_s" <<'PY'
import sys
import time

from pymavlink import mavutil

port = int(sys.argv[1])
timeout_s = float(sys.argv[2])
master = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}")
deadline = time.time() + timeout_s
try:
    while time.time() < deadline:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg is None:
            continue
        src = msg.get_srcSystem()
        comp = msg.get_srcComponent()
        if src in (0, 255):
            continue
        autopilot = int(getattr(msg, "autopilot", mavutil.mavlink.MAV_AUTOPILOT_INVALID))
        if comp == 191 and autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            continue
        if autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID and int(
            getattr(msg, "type", 0)
        ) in (
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_TYPE_GENERIC,
        ):
            continue
        print(f"Heartbeat from sys={src} comp={comp}")
        sys.exit(0)
finally:
    try:
        master.close()
    except Exception:
        pass
print(f"No MAVLink heartbeat on UDP {port} within {timeout_s:.0f}s", file=sys.stderr)
sys.exit(1)
PY
  then
    echo "Error: no heartbeat on control UDP ${port} within ${timeout_s}s; not launching image/camera/control" >&2
    return 1
  fi
  return 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --viz) VIZ=1; MODE="fg"; PLATFORM_CLI=1 ;;
    --gz) GZ=1; MODE="gz"; PLATFORM_CLI=1 ;;
    --yasim) YASIM=1; MODE="fg"; PLATFORM_CLI=1 ;;
    --xplane)
      echo "Error: platform xplane is not available (use jsbsim|viz|yasim|gz)" >&2
      exit 2
      ;;
    --model) MODEL_SET=1; GZ_MODEL="$2"; shift ;;
    --no-sim) NO_SIM=1 ;;
    --no-plot) NO_PLOT=1 ;;
    --detach) DETACH=1 ;;
    --duration) DURATION="$2"; shift ;;
    --ekf-fix) EKF_FIX="$2"; shift ;;
    --setup) SETUP="$2"; shift ;;
    --session) SESSION="$2"; shift ;;
    -h|--help) usage ;;
    *) echo "Unknown: $1" >&2; exit 1 ;;
  esac
  shift
done

if [[ "${VIZ}" -eq 1 && "${GZ}" -eq 1 ]]; then
  echo "Error: --viz and --gz are mutually exclusive" >&2
  exit 2
fi
if [[ "${VIZ}" -eq 1 && "${YASIM}" -eq 1 ]]; then
  echo "Error: --viz and --yasim are mutually exclusive" >&2
  exit 2
fi
if [[ "${GZ}" -eq 1 && "${YASIM}" -eq 1 ]]; then
  echo "Error: --gz and --yasim are mutually exclusive" >&2
  exit 2
fi
if [[ "${XPLANE}" -eq 1 && "${VIZ}" -eq 1 ]]; then
  echo "Error: --xplane and --viz are mutually exclusive" >&2
  exit 2
fi
if [[ "${XPLANE}" -eq 1 && "${GZ}" -eq 1 ]]; then
  echo "Error: --xplane and --gz are mutually exclusive" >&2
  exit 2
fi
if [[ "${XPLANE}" -eq 1 && "${YASIM}" -eq 1 ]]; then
  echo "Error: --xplane and --yasim are mutually exclusive" >&2
  exit 2
fi
if [[ "${MODEL_SET}" -eq 1 && "${PLATFORM_CLI}" -eq 1 && "${GZ}" -eq 0 ]]; then
  echo "Error: --model requires --gz" >&2
  exit 2
fi
if [[ "${EKF_FIX}" == "gps" ]]; then
  echo "Error: --ekf-fix gps is disabled. Mag+GPS crashed; GPS-only never armed." >&2
  echo "Use rebase (default) or see UPDATES.md 0.35.1." >&2
  exit 2
fi
if [[ "${EKF_FIX}" != "rebase" ]]; then
  echo "Error: --ekf-fix must be rebase (got '${EKF_FIX}'). gps is disabled." >&2
  exit 2
fi

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux required" >&2
  exit 1
fi

# Prefer the active conda env's interpreter (user prompt is often (pigeon)).
PYTHON="$(command -v "${PYTHON:-python3}")"

# Do not unset conda LD_LIBRARY_PATH: pyzmq/OpenCV need those libs. Unsetting
# mixed libzmq and aborted the camera pane (Assertion failed: !_more src/fq.cpp).
# tmux/bash "libtinfo no version information" lines are cosmetic.

# conda base currently ships OpenCV 5 (no Qt fonts) → black balloon_camera.
# Pin is opencv-python>=4.8,<5; fall back to envs/pigeon when present.
_cv_major="$("${PYTHON}" -c 'import cv2; print(cv2.__version__.split(".")[0])' 2>/dev/null | tail -1)"
_cv_major="${_cv_major:-0}"
if [[ "${_cv_major}" -ge 5 ]]; then
  _pigeon=""
  if [[ -n "${CONDA_PREFIX:-}" && -x "${CONDA_PREFIX}/envs/pigeon/bin/python3" ]]; then
    _pigeon="${CONDA_PREFIX}/envs/pigeon/bin/python3"
  elif [[ -x "${HOME}/anaconda/envs/pigeon/bin/python3" ]]; then
    _pigeon="${HOME}/anaconda/envs/pigeon/bin/python3"
  fi
  if [[ -n "${_pigeon}" ]] && "${_pigeon}" -c 'import cv2, numpy, zmq, pymavlink, matplotlib; assert int(cv2.__version__.split(".")[0]) < 5' >/dev/null 2>&1; then
    echo "Warning: ${PYTHON} has OpenCV ${_cv_major} (need <5 for balloon_camera). Using ${_pigeon}"
    PYTHON="${_pigeon}"
  else
    echo "Error: balloon_camera needs opencv-python>=4.8,<5; ${PYTHON} has OpenCV ${_cv_major}." >&2
    echo "  conda activate pigeon" >&2
    echo "  or: PYTHON=/path/to/python3 ./run_balloon_race.sh ..." >&2
    exit 1
  fi
fi

if ! "${PYTHON}" -c "import cv2, numpy, zmq, pymavlink, matplotlib" >/dev/null 2>&1; then
  echo "Error: balloon-race Python deps missing in $("${PYTHON}" -c 'import sys; print(sys.executable)')." >&2
  echo "  Install: ${PYTHON} -m pip install -r ${PYTHON_ROOT}/requirements.txt" >&2
  exit 1
fi

export PYTHONPATH="${PYTHON_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"

CLI_PLATFORM=""
if [[ "${PLATFORM_CLI}" -eq 1 ]]; then
  if [[ "${VIZ}" -eq 1 ]]; then
    CLI_PLATFORM="viz"
  elif [[ "${GZ}" -eq 1 ]]; then
    CLI_PLATFORM="gz"
  elif [[ "${YASIM}" -eq 1 ]]; then
    CLI_PLATFORM="yasim"
  elif [[ "${XPLANE}" -eq 1 ]]; then
    CLI_PLATFORM="xplane"
  else
    CLI_PLATFORM="jsbsim"
  fi
fi
CLI_MODEL=""
if [[ "${MODEL_SET}" -eq 1 ]]; then
  CLI_MODEL="${GZ_MODEL}"
fi
CLI_DURATION="${DURATION}"
if [[ -z "${CLI_DURATION}" && -n "${BALLOON_RACE_DURATION:-}" ]]; then
  CLI_DURATION="${BALLOON_RACE_DURATION}"
fi

read -r RESOLVED_PLATFORM RESOLVED_GZ_MODEL RESOLVED_DURATION < <(
  SETUP_PATH="${SETUP}" \
  CLI_PLATFORM="${CLI_PLATFORM}" \
  CLI_MODEL="${CLI_MODEL}" \
  CLI_DURATION="${CLI_DURATION}" \
  "${PYTHON}" - <<'PY'
import os
import sys
from pathlib import Path

from fw_sitl.flight_setup import load_flight_setup, resolve_race_sim

setup = load_flight_setup(Path(os.environ["SETUP_PATH"]))
plat = os.environ.get("CLI_PLATFORM") or None
model = os.environ.get("CLI_MODEL") or None
dur_raw = os.environ.get("CLI_DURATION") or ""
dur = float(dur_raw) if dur_raw.strip() else None
try:
    p, m, d = resolve_race_sim(
        setup, platform=plat, gz_model=model, duration_s=dur
    )
except ValueError as exc:
    print(f"Error: {exc}", file=sys.stderr)
    sys.exit(2)
print(p, m, d)
PY
) || exit $?

VIZ=0
GZ=0
YASIM=0
XPLANE=0
MODE="synth"
case "${RESOLVED_PLATFORM}" in
  jsbsim) ;;
  viz) VIZ=1; MODE="fg" ;;
  gz) GZ=1; MODE="gz" ;;
  yasim) YASIM=1; MODE="fg" ;;
  xplane) XPLANE=1; MODE="xp" ;;
  *)
    echo "Error: unknown resolved platform '${RESOLVED_PLATFORM}'" >&2
    exit 2
    ;;
esac
GZ_MODEL="${RESOLVED_GZ_MODEL}"
DURATION="${RESOLVED_DURATION}"

echo "Race sim: platform=${RESOLVED_PLATFORM} gz_model=${GZ_MODEL} duration_s=${DURATION} homing_law=${FW_HOMING_LAW:-<flightSetup.json>}"

RACE_CSV="${BALLOON_RACE_CSV:-/tmp/balloon_race_$(date +%Y%m%d_%H%M%S).csv}"

# Quiet when no tmux server yet (fresh machine / after kill-server).
tmux kill-session -t "${SESSION}" 2>/dev/null || true

if [[ "${NO_SIM}" -eq 0 ]]; then
  echo "Removing leftover SITL docker stacks..."
  bash "${SCRIPT_DIR}/kill.sh" --all || true
fi

if [[ "${GZ}" -eq 1 ]]; then
  CONTAINER_NAME="${PX4_GZ_DOCKER_NAME:-px4-noble-gz-plane}"
  SIM_CMD="MAVLINK_FANOUT=${MAVLINK_FANOUT} bash ${PYTHON_ROOT}/scripts/runSimGzPlane.sh --mavlink-server --setup ${SETUP} --model ${GZ_MODEL}"
elif [[ "${YASIM}" -eq 1 ]]; then
  CONTAINER_NAME="${PX4_SITL_DOCKER_NAME:-px4-noble-sim-ros}"
  SIM_CMD="MAVLINK_FANOUT=${MAVLINK_FANOUT} bash ${PYTHON_ROOT}/scripts/runSimYasimRascal.sh --setup ${SETUP}"
  if [[ "${MAVLINK_FANOUT}" == "1" ]]; then
    SIM_CMD+=" --mavlink-server"
  fi
elif [[ "${XPLANE}" -eq 1 ]]; then
  CONTAINER_NAME="${PX4_XP_DOCKER_NAME:-px4-noble-xplane-cessna}"
  SIM_CMD="MAVLINK_FANOUT=${MAVLINK_FANOUT} bash ${PYTHON_ROOT}/scripts/runSimXplaneCessna.sh --mavlink-server --setup ${SETUP}"
else
  SIM_CMD="MAVLINK_FANOUT=${MAVLINK_FANOUT} bash ${PYTHON_ROOT}/scripts/runSimJsbsimRascal.sh --setup ${SETUP}"
  if [[ "${MAVLINK_FANOUT}" == "1" ]]; then
    SIM_CMD+=" --mavlink-server"
  fi
  if [[ "${VIZ}" -eq 1 ]]; then
    SIM_CMD+=" --viz"
  fi
fi

IMG_CMD="PYTHONUNBUFFERED=1 ${PYTHON} -u ${PYTHON_ROOT}/run_balloon_image_source.py --mode ${MODE} --setup ${SETUP} --udp ${MAVLINK_IMAGE_PORT}"
POSE_CMD="PYTHONUNBUFFERED=1 ${PYTHON} -u ${PYTHON_ROOT}/run_balloon_gz_pose.py --setup ${SETUP}"
CAM_CMD="DISPLAY=${DISPLAY:-:0} QT_X11_NO_MITSHM=1 PYTHONUNBUFFERED=1 ${PYTHON} -u ${PYTHON_ROOT}/run_balloon_camera.py --setup ${SETUP}"
# Headless e2e / no GUI: BALLOON_CAMERA_NO_DISPLAY=1 or --no-display
if [[ "${BALLOON_CAMERA_NO_DISPLAY:-0}" == "1" ]]; then
  CAM_CMD+=" --no-display"
fi
CTL_CMD="DISPLAY=${DISPLAY:-:0} MPLBACKEND=${MPLBACKEND:-Agg} PYTHONUNBUFFERED=1 ${PYTHON} -u ${PYTHON_ROOT}/run_balloon_control.py --setup ${SETUP} --udp ${MAVLINK_CONTROL_PORT}"
if [[ -n "${FW_HOMING_LAW:-}" ]]; then
  CTL_CMD="FW_HOMING_LAW=${FW_HOMING_LAW} ${CTL_CMD}"
fi
# Prefix PN tunables onto CTL_CMD: tmux server env may predate this shell.
for _pn_k in FW_PN_N FW_PN_TAU_S FW_PN_LPF_TAU_S FW_PN_A_MAX; do
  if [[ -n "${!_pn_k:-}" ]]; then
    CTL_CMD="${_pn_k}=${!_pn_k} ${CTL_CMD}"
  fi
done
if [[ "${NO_PLOT}" -eq 1 ]]; then
  CTL_CMD+=" --no-plot"
fi
CTL_CMD+=" --csv ${RACE_CSV}"
CTL_CMD+=" --duration ${DURATION}"
# MAVLINK ports with mavlink-server fan-out (started by runSimJsbsimRascal.sh):
#   14550 GCS/QGC, 14540 control, 14541 image-source
# Control always attaches: race owns sim, or user passed --no-sim for an existing sim.
CTL_CMD+=" --no-sim"
if [[ "${NO_SIM}" -eq 0 ]]; then
  CTL_CMD+=" --stop-sim-on-exit"
fi
if [[ "${VIZ}" -eq 1 ]]; then
  CTL_CMD+=" --viz --spawn-fg-balloons --ekf-fix ${EKF_FIX}"
fi
if [[ "${GZ}" -eq 1 ]]; then
  IMG_CMD+=" --container ${CONTAINER_NAME}"
  CTL_CMD+=" --gz --spawn-gz-balloons"
  CTL_CMD+=" --gz-container ${CONTAINER_NAME}"
  CTL_CMD+=" --model ${GZ_MODEL}"
  POSE_CMD+=" --container ${CONTAINER_NAME} --model ${GZ_MODEL}"
fi
if [[ "${YASIM}" -eq 1 ]]; then
  CTL_CMD+=" --yasim --spawn-fg-balloons --ekf-fix ${EKF_FIX}"
fi
if [[ "${XPLANE}" -eq 1 ]]; then
  CTL_CMD+=" --xplane --spawn-xp-balloons"
  POSE_CMD="PYTHONUNBUFFERED=1 ${PYTHON} -u ${PYTHON_ROOT}/run_balloon_xp_pose.py --setup ${SETUP}"
fi

# One window; peers are split panes (tiled) after heartbeat.
# remain-on-exit: if sim/fan-out dies, keep the pane so the real error is visible
# (otherwise the session vanishes and the next tmux call prints "no server running").
if [[ "${NO_SIM}" -eq 0 ]]; then
  tmux new-session -d -s "${SESSION}" -n race "${SIM_CMD}"
  tmux set-option -t "${SESSION}" remain-on-exit on
  sleep 8
else
  tmux new-session -d -s "${SESSION}" -n race "echo 'NO_SIM: using existing sim'; bash"
  tmux set-option -t "${SESSION}" remain-on-exit on
fi

if ! tmux has-session -t "${SESSION}" 2>/dev/null; then
  echo "Error: tmux session '${SESSION}' exited during sim startup." >&2
  echo "  Usual cause: mavlink-server fan-out failed (missing python/bin/mavlink-server)." >&2
  echo "  Fix: python/scripts/fetch_mavlink_server.sh   # then re-run ./run_balloon_race.sh" >&2
  echo "  Or inspect a prior sim log if remain-on-exit caught it." >&2
  exit 1
fi
tmux select-pane -t "${SESSION}:0.0" -T sim

if [[ "${MAVLINK_FANOUT}" == "1" ]] && ! mavlink_fanout_up; then
  echo "Error: mavlink-server fan-out failed to start; not launching image/camera/control" >&2
  echo "  Fix: python/scripts/fetch_mavlink_server.sh" >&2
  echo "  Or rebuild Noble image / set MAVLINK_FANOUT=0 (not recommended for race)." >&2
  echo "  Attach to see sim pane: tmux attach -t ${SESSION}" >&2
  echo "  Sim pane:" >&2
  tmux capture-pane -t "${SESSION}:0.0" -p -S -40 2>/dev/null | sed 's/^/    /' >&2 || true
  exit 1
fi

# Visual balloons before PX4 HEARTBEAT (aircraft in use). Headless synth has no models.
if [[ "${VIZ}" -eq 1 || "${YASIM}" -eq 1 ]]; then
  echo "Placing FG balloons from ${SETUP} before PX4 heartbeat..."
  if ! PYTHONPATH="${PYTHON_ROOT}${PYTHONPATH:+:${PYTHONPATH}}" \
      "${PYTHON}" -m fw_sitl.balloon_scene --setup "${SETUP}" --fg --timeout 90; then
    echo "Error: FG balloon spawn failed; not launching image/camera/control" >&2
    tmux kill-session -t "${SESSION}" 2>/dev/null || true
    exit 1
  fi
elif [[ "${GZ}" -eq 1 ]]; then
  echo "Placing GZ balloons from ${SETUP} before PX4 heartbeat..."
  if ! PYTHONPATH="${PYTHON_ROOT}${PYTHONPATH:+:${PYTHONPATH}}" \
      "${PYTHON}" -m fw_sitl.balloon_scene --setup "${SETUP}" --gz --timeout 90 \
      --container "${CONTAINER_NAME}"; then
    echo "Error: GZ balloon spawn failed; not launching image/camera/control" >&2
    tmux kill-session -t "${SESSION}" 2>/dev/null || true
    exit 1
  fi
elif [[ "${XPLANE}" -eq 1 ]]; then
  echo "Placing X-Plane balloons from ${SETUP} before PX4 heartbeat..."
  if ! PYTHONPATH="${PYTHON_ROOT}${PYTHONPATH:+:${PYTHONPATH}}" \
      "${PYTHON}" -m fw_sitl.balloon_scene --setup "${SETUP}" --xplane --timeout 90; then
    echo "Error: X-Plane balloon spawn failed; not launching image/camera/control" >&2
    tmux kill-session -t "${SESSION}" 2>/dev/null || true
    exit 1
  fi
fi

if ! wait_control_heartbeat "${MAVLINK_CONTROL_PORT}" "${HEARTBEAT_TIMEOUT_S}"; then
  tmux kill-session -t "${SESSION}" 2>/dev/null || true
  exit 1
fi

# Control first so OFFBOARD engages before the plane burns remaining AGL.
# -d keeps focus on sim; title panes by id (tiled layout may renumber indices).
_ctl_pane="$(tmux split-window -d -t "${SESSION}:0.0" -P -F '#{pane_id}' "${CTL_CMD}")"
tmux select-pane -t "${_ctl_pane}" -T control
: >/tmp/balloon_race_control.log
tmux pipe-pane -t "${_ctl_pane}" -o 'cat >> /tmp/balloon_race_control.log'
_img_pane="$(tmux split-window -d -t "${SESSION}:0.0" -P -F '#{pane_id}' "${IMG_CMD}")"
tmux select-pane -t "${_img_pane}" -T image
_cam_pane="$(tmux split-window -d -t "${SESSION}:0.0" -P -F '#{pane_id}' "${CAM_CMD}")"
tmux select-pane -t "${_cam_pane}" -T camera
if [[ "${GZ}" -eq 1 || "${XPLANE}" -eq 1 ]]; then
  # Streams the plane's true Gazebo/X-Plane pose to control (continuous);
  # replaces one-shot docker-exec polling that produced position jitter.
  _pose_pane="$(tmux split-window -d -t "${SESSION}:0.0" -P -F '#{pane_id}' "${POSE_CMD}")"
  tmux select-pane -t "${_pose_pane}" -T pose
fi
tmux select-layout -t "${SESSION}:0" tiled

if [[ "${NO_PLOT}" -eq 0 ]]; then
  if [[ -f /tmp/balloon_race_plot_waiter.pid ]]; then
    kill "$(cat /tmp/balloon_race_plot_waiter.pid)" 2>/dev/null || true
    rm -f /tmp/balloon_race_plot_waiter.pid
  fi
  if [[ -n "${DURATION}" && "${DURATION}" != "0" ]]; then
    # resolve_race_sim prints floats (e.g. 60.0); bash $(( )) only accepts ints.
    PLOT_TIMEOUT=$(( ${DURATION%%.*} + 300 ))
  else
    PLOT_TIMEOUT=3600
  fi
  # Do not inherit MPLBACKEND=Agg from the shell; the waiter needs a GUI backend.
  env -u MPLBACKEND DISPLAY="${DISPLAY:-:0}" PYTHONUNBUFFERED=1 "${PYTHON}" -u "${PYTHON_ROOT}/show_race_plots.py" \
    --csv "${RACE_CSV}" --timeout "${PLOT_TIMEOUT}" >>/tmp/balloon_race_plot.log 2>&1 &
  echo $! >/tmp/balloon_race_plot_waiter.pid
  disown || true
  echo "Plots will open when the race ends (matplotlib). csv=${RACE_CSV}"
fi

echo "tmux session '${SESSION}' started"
echo "panes (window race):"
tmux list-panes -t "${SESSION}:0" -F '  #{session_name}:#{window_index}.#{pane_index} #{pane_title}'
# Control pane has the 1 Hz t/x/y/z line. Attach by default so it is on screen.
tmux select-pane -t "${_ctl_pane}"
if [[ "${DETACH}" -eq 1 ]]; then
  echo "detached; attach: tmux attach -t ${SESSION}"
elif [[ -t 1 ]] && [[ -t 0 ]]; then
  if [[ -n "${TMUX:-}" ]]; then
    tmux switch-client -t "${SESSION}"
  else
    tmux attach -t "${SESSION}"
  fi
else
  echo "no TTY; session left detached (tmux attach -t ${SESSION})"
fi
