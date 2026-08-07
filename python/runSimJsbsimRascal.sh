#!/bin/bash
# Start PX4 SITL + headless JSBSim Rascal in the Noble sim container (no FlightGear UI).
# Mounts python/jsb_spawn.xml over the default LSZH scene for in-air spawn.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONTAINER_NAME="${PX4_JSBSIM_DOCKER_NAME:-px4-noble-jsbsim-rascal}"
IMAGE_TAG="${PX4_SITL_DOCKER_VER:-px4-noble-sim-ros:latest}"
SPAWN_XML="${SCRIPT_DIR}/jsb_spawn.xml"
CONTAINER_SCENE="/home/valentin/PX4-Autopilot/Tools/simulation/jsbsim/jsbsim_bridge/scene/LSZH.xml"

cleanup_on_exit() {
	echo ""
	echo "Cleaning up..."
	docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true
}

while [[ $# -gt 0 ]]; do
	case "$1" in
		--help|-h)
			echo "Usage: $0 [--kill]"
			echo "  Starts headless JSBSim Rascal SITL with IC from ${SPAWN_XML}"
			echo "  --kill  Remove container and exit"
			exit 0
			;;
		--kill)
			cleanup_on_exit
			exit 0
			;;
		*)
			echo "Unknown option: $1 (use --help)"
			exit 1
			;;
	esac
done

if [[ ! -f "${SPAWN_XML}" ]]; then
	echo "Missing spawn IC: ${SPAWN_XML}" >&2
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
