#!/bin/bash
# Start PX4 SITL + Gazebo plane (GUI). Default model: rc_cessna. In-air spawn.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PYTHON_ROOT}/.." && pwd)"
CONTAINER_NAME="${PX4_GZ_DOCKER_NAME:-px4-noble-gz-plane}"
IMAGE_TAG="${PX4_SITL_DOCKER_VER:-px4-noble-sim-ros:latest}"
MAVLINK_SERVER_SCRIPT="${REPO_ROOT}/Dockerfiles/scripts/start_mavlink_server.sh"
MAVLINK_SERVER_PID=""
MAVLINK_FANOUT="${MAVLINK_FANOUT:-0}"
GZ_MODEL="rc_cessna"
SETUP="${PYTHON_ROOT}/flightSetup.json"
POSE="${PX4_GZ_MODEL_POSE:-0,0,500,0,0,1.570796}"
HOST_GZ_ASSETS="${PYTHON_ROOT}/assets/gz"
HOST_PYTHON="${PYTHON_ROOT}"

cleanup_on_exit() {
	if [[ -n "${MAVLINK_SERVER_PID:-}" ]]; then
		kill "${MAVLINK_SERVER_PID}" 2>/dev/null || true
		wait "${MAVLINK_SERVER_PID}" 2>/dev/null || true
		MAVLINK_SERVER_PID=""
	fi
	docker rm -f "${CONTAINER_NAME}-mavlink" 2>/dev/null || true
	docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true
	xhost -local:docker 2>/dev/null || true
}

# Immediate GPU launch failure only. docker rm -f after a live run is 137/143
# (or a long elapsed time) — that must not start a second Gazebo.
gz_should_retry_without_gpu() {
	local rc="$1"
	local elapsed_s="$2"
	local used_gpu="$3"
	if [[ "${used_gpu}" != "gpu" ]]; then
		return 1
	fi
	if [[ "${elapsed_s}" -ge 30 ]]; then
		return 1
	fi
	if [[ "${rc}" -eq 137 || "${rc}" -eq 143 ]]; then
		return 1
	fi
	return 0
}

while [[ $# -gt 0 ]]; do
	case "$1" in
		--help|-h)
			echo "Usage: $0 [--model rc_cessna|advanced_plane] [--setup PATH] [--mavlink-server|--no-mavlink-server] [--kill]"
			exit 0
			;;
		--model)
			GZ_MODEL="$2"
			shift
			;;
		--setup)
			SETUP="$2"
			shift
			;;
		--mavlink-server) MAVLINK_FANOUT=1 ;;
		--no-mavlink-server) MAVLINK_FANOUT=0 ;;
		--kill) cleanup_on_exit; exit 0 ;;
		*) echo "Unknown option: $1" >&2; exit 1 ;;
	esac
	shift
done

if [[ "${GZ_MODEL}" != "rc_cessna" && "${GZ_MODEL}" != "advanced_plane" ]]; then
	echo "Error: --model must be rc_cessna or advanced_plane (got ${GZ_MODEL})" >&2
	exit 1
fi
if [[ ! -f "${SETUP}" ]]; then
	echo "Error: missing setup ${SETUP}" >&2
	exit 1
fi
if [[ -z "${DISPLAY:-}" ]]; then
	echo "Error: DISPLAY is not set (Gazebo GUI required)" >&2
	exit 1
fi
if ! docker image inspect "${IMAGE_TAG}" >/dev/null 2>&1; then
	echo "Error: Docker image '${IMAGE_TAG}' not found locally (will not pull from Docker Hub)." >&2
	echo "  Build: cd ${REPO_ROOT}/Dockerfiles && ./PX4_noble_sim_build.sh" >&2
	exit 1
fi

MAKE_TGT="gz_rc_cessna"
if [[ "${GZ_MODEL}" == "advanced_plane" ]]; then
	MAKE_TGT="gz_advanced_plane"
fi

# Return 0 if $1 is an executable mavlink-server for this host (not wrong arch).
mavlink_server_usable() {
	local candidate="$1"
	[[ -n "${candidate}" && -x "${candidate}" ]] || return 1
	# Wrong-arch ELF on PATH (e.g. aarch64 binary on x86_64) → Exec format error.
	"${candidate}" --version >/dev/null 2>&1
}

resolve_mavlink_server_bin() {
	local c=""
	# Prefer project-local musl binary (python/bin/mavlink-server).
	for c in \
		"${PYTHON_ROOT}/bin/mavlink-server" \
		"${MAVLINK_SERVER_BIN:-}" \
		"/usr/local/bin/mavlink-server" \
		"$(command -v mavlink-server 2>/dev/null || true)"
	do
		if mavlink_server_usable "${c}"; then
			echo "${c}"
			return 0
		fi
		if [[ -n "${c}" && -e "${c}" ]]; then
			echo "Warning: ignoring unusable mavlink-server at ${c} (wrong arch or broken)" >&2
		fi
	done
	return 1
}

ensure_host_mavlink_server() {
	# python/bin is gitignored; auto-fetch 0.10.1 when fan-out needs a host binary.
	if resolve_mavlink_server_bin >/dev/null; then
		return 0
	fi
	local fetch="${SCRIPT_DIR}/fetch_mavlink_server.sh"
	if [[ ! -x "${fetch}" && ! -f "${fetch}" ]]; then
		return 1
	fi
	echo "No usable host mavlink-server; fetching via ${fetch} ..." >&2
	if ! bash "${fetch}"; then
		echo "Error: fetch_mavlink_server.sh failed" >&2
		return 1
	fi
	resolve_mavlink_server_bin >/dev/null
}

start_mavlink_fanout() {
	# Host-side fan-out so control (14540) and image-source (14541) get distinct UDP feeds.
	# When requested (MAVLINK_FANOUT=1), fail loud — do not soft-skip.
	if [[ "${MAVLINK_FANOUT}" != "1" ]]; then
		return 0
	fi
	if [[ ! -f "${MAVLINK_SERVER_SCRIPT}" ]]; then
		echo "Error: missing ${MAVLINK_SERVER_SCRIPT}; cannot start mavlink fan-out" >&2
		return 1
	fi
	chmod +x "${MAVLINK_SERVER_SCRIPT}" 2>/dev/null || true

	# Host fan-out must not share the sim pane TTY: PX4 GCS (UDP 18570→14550) includes
	# dialect msgs mavlink-server may reject (InvalidCRC; seen on 0.9.0) and would spam the pane.
	local mav_log="${MAVLINK_SERVER_LOG:-/tmp/mavlink-server-fanout.log}"
	local bin=""
	ensure_host_mavlink_server || true
	if bin="$(resolve_mavlink_server_bin)"; then
		: >"${mav_log}" || true
		MAVLINK_SERVER_BIN="${bin}" \
			MAVLINK_SERVER_RUST_LOG="${MAVLINK_SERVER_RUST_LOG:-off}" \
			bash "${MAVLINK_SERVER_SCRIPT}" >>"${mav_log}" 2>&1 &
		MAVLINK_SERVER_PID=$!
		sleep 0.4
		if ! kill -0 "${MAVLINK_SERVER_PID}" 2>/dev/null; then
			echo "Error: mavlink-server host process exited immediately (${bin}); see ${mav_log}" >&2
			MAVLINK_SERVER_PID=""
			return 1
		fi
		echo "mavlink-server host PID=${MAVLINK_SERVER_PID} (${bin}); log ${mav_log}"
		echo "  PX4 GCS source port 18570 → :${MAVLINK_GCS_PORT:-14550} → :${MAVLINK_CONTROL_PORT:-14540}/:${MAVLINK_IMAGE_PORT:-14541}"
		return 0
	fi

	# Fall back to binary baked into the sim image (needs rebuild after Dockerfile bake).
	if ! docker image inspect "${IMAGE_TAG}" >/dev/null 2>&1; then
		echo "Error: no usable mavlink-server and Docker image '${IMAGE_TAG}' not found" >&2
		echo "  Install: curl -fsSL -o python/bin/mavlink-server \\" >&2
		echo "    https://github.com/bluerobotics/mavlink-server/releases/download/0.10.1/mavlink-server-x86_64-unknown-linux-musl" >&2
		echo "    && chmod +x python/bin/mavlink-server" >&2
		echo "  Or: python/scripts/fetch_mavlink_server.sh" >&2
		echo "  Or rebuild image: Dockerfiles/PX4_noble_sim_build.sh" >&2
		return 1
	fi

	docker rm -f "${CONTAINER_NAME}-mavlink" 2>/dev/null || true
	local docker_out=""
	local docker_rc=0
	docker_out=$(docker run --rm -d --net=host --name "${CONTAINER_NAME}-mavlink" \
		-e "RUST_LOG=${MAVLINK_SERVER_RUST_LOG:-off}" \
		--entrypoint /usr/local/bin/mavlink-server \
		"${IMAGE_TAG}" \
		--web-server "${MAVLINK_WEB_SERVER:-127.0.0.1:6040}" \
		--mavlink-heartbeat-frequency 0 \
		"udpserver://0.0.0.0:${MAVLINK_GCS_PORT:-14550}" \
		"udpclient://127.0.0.1:${MAVLINK_CONTROL_PORT:-14540}" \
		"udpclient://127.0.0.1:${MAVLINK_IMAGE_PORT:-14541}" 2>&1) && docker_rc=0 || docker_rc=$?
	if [[ "${docker_rc}" -ne 0 ]]; then
		echo "Error: failed to start mavlink-server docker sidecar:" >&2
		echo "${docker_out}" >&2
		echo "  Image may lack /usr/local/bin/mavlink-server — use python/bin/mavlink-server or rebuild." >&2
		return 1
	fi
	sleep 0.4
	if ! docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}-mavlink"; then
		echo "Error: mavlink-server sidecar exited immediately (${CONTAINER_NAME}-mavlink)" >&2
		echo "  Ensure the image bakes /usr/local/bin/mavlink-server (rebuild Noble sim image)." >&2
		return 1
	fi
	echo "Started mavlink-server sidecar ${CONTAINER_NAME}-mavlink"
	return 0
}

trap cleanup_on_exit EXIT INT TERM
docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true
if ! start_mavlink_fanout; then
	echo "Error: mavlink fan-out requested (MAVLINK_FANOUT=1) but failed to start" >&2
	exit 1
fi

xhost + 2>/dev/null || true
xhost +local:docker 2>/dev/null || true
XAUTH_FILE="${XAUTHORITY:-$HOME/.Xauthority}"
XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"
mkdir -p "${XDG_RUNTIME_DIR}"
chmod 700 "${XDG_RUNTIME_DIR}"

DOCKER_IT=(-i)
if [[ -t 0 && -z "${PX4_SITL_NO_DOCKER_TTY:-}" ]]; then
	DOCKER_IT=(-it)
fi

INNER_CMD="set -euo pipefail
		cd /home/valentin/PX4-Autopilot
		export PYTHONPATH=/opt/fixedwing/python:/opt/fixedwing/gz/systems\${PYTHONPATH:+:\$PYTHONPATH}
		STOCK=Tools/simulation/gz/models/${GZ_MODEL}/model.sdf
		if [[ ! -f \"\${STOCK}\" ]]; then
			echo \"missing stock SDF \${STOCK}\" >&2
			exit 1
		fi
		mkdir -p /tmp/fw_gz_overlay/models/${GZ_MODEL}
		cp -a Tools/simulation/gz/models/${GZ_MODEL}/. /tmp/fw_gz_overlay/models/${GZ_MODEL}/
		python3 - /tmp/fw_gz_overlay/models/${GZ_MODEL}/model.sdf <<'PY'
from pathlib import Path
import sys
sys.path.insert(0, '/opt/fixedwing/python')
from fw_sitl.flight_setup import load_flight_setup
from fw_sitl.gz_overlay import apply_plane_overlay
from fw_sitl.gz_pose import world_velocity_enu, DEFAULT_GZ_YAW_RAD
stock = Path(sys.argv[1])
setup = load_flight_setup(Path('/opt/fixedwing/flightSetup.json'))
stock.write_text(apply_plane_overlay(
    stock.read_text(),
    width=setup.camera.width_px,
    height=setup.camera.height_px,
    hfov_deg=setup.camera.hfov_deg,
    eye_forward_m=setup.camera.fg_eye_forward_m,
    update_rate_hz=setup.camera.rate_hz,
))
vx, vy, vz = world_velocity_enu(setup.guidance.speed_mps, DEFAULT_GZ_YAW_RAD)
Path('/tmp/fw_gz_vel.env').write_text(
    f'export FW_GZ_SPAWN_VX={vx}\\nexport FW_GZ_SPAWN_VY={vy}\\nexport FW_GZ_SPAWN_VZ={vz}\\n'
)
print(f'overlay {stock} v=({vx:.1f},{vy:.1f},{vz:.1f})')
PY
		cp -f /tmp/fw_gz_overlay/models/${GZ_MODEL}/model.sdf \"\${STOCK}\"
		# shellcheck disable=SC1091
		source /tmp/fw_gz_vel.env
		export GZ_SIM_RESOURCE_PATH=/tmp/fw_gz_overlay/models:/opt/fixedwing/gz/models\${GZ_SIM_RESOURCE_PATH:+:\$GZ_SIM_RESOURCE_PATH}
		export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/fixedwing/gz/systems\${GZ_SIM_SYSTEM_PLUGIN_PATH:+:\$GZ_SIM_SYSTEM_PLUGIN_PATH}
		python3 -m fw_sitl.gz_gui_follow --write-gui-config /tmp/fw_gz_gui.config --model ${GZ_MODEL}
		export GZ_GUI_CONFIG=/tmp/fw_gz_gui.config
		python3 -m fw_sitl.gz_gui_follow --follow --model ${GZ_MODEL} --timeout-s 0 >/tmp/fw_gz_follow.log 2>&1 &
		make px4_sitl ${MAKE_TGT}
"

run_gz_docker() {
	local -a gpu_args=()
	if [[ "${1}" == "gpu" ]]; then
		gpu_args=(--gpus all)
	fi
	docker run "${DOCKER_IT[@]}" --rm \
		--net=host --privileged "${gpu_args[@]}" \
		--name "${CONTAINER_NAME}" \
		--env="DISPLAY=${DISPLAY}" \
		--env="QT_X11_NO_MITSHM=1" \
		--env="XAUTHORITY=${XAUTH_FILE}" \
		--env="XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR}" \
		--env="PX4_GZ_WORLD=default" \
		--env="PX4_GZ_MODEL_POSE=${POSE}" \
		--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--volume="${XDG_RUNTIME_DIR}:${XDG_RUNTIME_DIR}" \
		--volume="${HOST_GZ_ASSETS}:/opt/fixedwing/gz:rw" \
		--volume="${HOST_PYTHON}:/opt/fixedwing/python:ro" \
		--volume="${SETUP}:/opt/fixedwing/flightSetup.json:ro" \
		${XAUTH_FILE:+--volume="${XAUTH_FILE}:${XAUTH_FILE}:ro"} \
		"${IMAGE_TAG}" \
		/bin/bash -lc "${INNER_CMD}"
}

echo "Starting ${IMAGE_TAG} Gazebo ${GZ_MODEL} pose=${POSE} setup=${SETUP}"
USE_GPU="gpu"
if [[ "${PX4_GZ_DOCKER_GPUS:-all}" == "none" ]]; then
	USE_GPU="nogpu"
	echo "PX4_GZ_DOCKER_GPUS=none — starting without --gpus all"
fi
GZ_T0=$(date +%s)
GZ_RC=0
run_gz_docker "${USE_GPU}" || GZ_RC=$?
GZ_ELAPSED=$(( $(date +%s) - GZ_T0 ))
if [[ "${GZ_RC}" -eq 0 ]]; then
	exit 0
fi
if gz_should_retry_without_gpu "${GZ_RC}" "${GZ_ELAPSED}" "${USE_GPU}"; then
	echo "Warning: docker --gpus all failed (rc=${GZ_RC} after ${GZ_ELAPSED}s); retrying without GPU (Intel/software GL)." >&2
	echo "  Install nvidia-container-toolkit for NVIDIA inside the container." >&2
	docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true
	if ! run_gz_docker nogpu; then
		echo "Error: docker run failed for ${IMAGE_TAG}." >&2
		exit 1
	fi
else
	echo "Gazebo container stopped (rc=${GZ_RC} after ${GZ_ELAPSED}s); not restarting."
	exit 0
fi
