#!/bin/bash
# Start PX4 SITL + JSBSim Rascal. Default: headless. --viz: FG window (--fdm=null).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PYTHON_ROOT}/.." && pwd)"
CONTAINER_NAME="${PX4_JSBSIM_DOCKER_NAME:-px4-noble-jsbsim-rascal}"
IMAGE_TAG="${PX4_SITL_DOCKER_VER:-px4-noble-sim-ros:latest}"
SPAWN_XML="${PYTHON_ROOT}/assets/jsb_spawn.xml"
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

if [[ ! -f "${SPAWN_XML}" ]]; then
	echo "Missing spawn IC: ${SPAWN_XML}" >&2
	exit 1
fi

if [[ "${VIZ}" -eq 1 && ! -f "${PATCH_SCRIPT}" ]]; then
	echo "Missing JSBSim FG viz patch: ${PATCH_SCRIPT}" >&2
	exit 1
fi

trap cleanup_on_exit EXIT INT TERM

docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true

# -t only when stdin is a real TTY *and* not launched from the Python helpers.
# Python passes stdin=DEVNULL so the debug console is never attached to PX4.
DOCKER_IT=(-i)
if [[ -t 0 && -z "${PX4_SITL_NO_DOCKER_TTY:-}" ]]; then
	DOCKER_IT=(-it)
fi

if [[ "${VIZ}" -eq 1 ]]; then
	xhost + 2>/dev/null || true
	xhost +local:docker 2>/dev/null || true

	if [[ -z "${DISPLAY:-}" ]]; then
		echo "Warning: DISPLAY is not set; defaulting to :0"
		export DISPLAY=:0
	fi

	if [[ -z "${XAUTHORITY:-}" && -f "${HOME}/.Xauthority" ]]; then
		export XAUTHORITY="${HOME}/.Xauthority"
	fi

	XAUTH_FILE="${XAUTHORITY:-$HOME/.Xauthority}"
	XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"
	mkdir -p "${XDG_RUNTIME_DIR}"
	chmod 700 "${XDG_RUNTIME_DIR}"

	DOCKER_VOLUMES=(
		--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
		--volume="${XDG_RUNTIME_DIR}:${XDG_RUNTIME_DIR}"
		--volume="${SPAWN_XML}:${CONTAINER_SCENE}:ro"
		--volume="${PATCH_SCRIPT}:${CONTAINER_PATCH}:ro"
	)

	if [[ -f "${XAUTH_FILE}" ]]; then
		DOCKER_VOLUMES+=(--volume="${XAUTH_FILE}:${XAUTH_FILE}:ro")
	fi

	echo "Starting ${IMAGE_TAG} JSBSim Rascal with FG viz (IC ${SPAWN_XML})"
	docker run "${DOCKER_IT[@]}" --rm \
		--net=host \
		--privileged \
		--name "${CONTAINER_NAME}" \
		--env="DISPLAY=${DISPLAY}" \
		--env="QT_X11_NO_MITSHM=1" \
		--env="XAUTHORITY=${XAUTH_FILE}" \
		--env="XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR}" \
		"${DOCKER_VOLUMES[@]}" \
		"${IMAGE_TAG}" \
		/bin/bash -lc "set -euo pipefail
			cd /home/valentin/PX4-Autopilot
			bash '${CONTAINER_PATCH}' /home/valentin/PX4-Autopilot
			BRIDGE_BIN=build/px4_sitl_default/build_jsbsim_bridge/jsbsim_bridge
			if [[ ! -x \"\${BRIDGE_BIN}\" ]]; then
				echo \"jsbsim_bridge missing in image (\${BRIDGE_BIN}). Rebuild: Dockerfiles/PX4_noble_sim_build.sh\" >&2
				exit 1
			fi
			# Unset HEADLESS so sitl_run.sh starts fgfs --fdm=null
			unset HEADLESS || true
			make px4_sitl jsbsim_rascal
		"
else
	echo "Starting ${IMAGE_TAG} headless JSBSim Rascal with IC ${SPAWN_XML}"
	docker run "${DOCKER_IT[@]}" --rm \
		--net=host \
		--privileged \
		--name "${CONTAINER_NAME}" \
		--volume="${SPAWN_XML}:${CONTAINER_SCENE}:ro" \
		"${IMAGE_TAG}" \
		/bin/bash -lc "set -euo pipefail
			cd /home/valentin/PX4-Autopilot
			BRIDGE_BIN=build/px4_sitl_default/build_jsbsim_bridge/jsbsim_bridge
			if [[ ! -x \"\${BRIDGE_BIN}\" ]]; then
				echo \"jsbsim_bridge missing in image (\${BRIDGE_BIN}). Rebuild: Dockerfiles/PX4_noble_sim_build.sh\" >&2
				exit 1
			fi
			# HEADLESS=1 skips FlightGear visualization; JSBSim FDM + bridge only.
			# Bridge is prebuilt in the image — this should only launch, not compile.
			HEADLESS=1 make px4_sitl jsbsim_rascal
		"
fi
