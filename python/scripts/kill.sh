#!/bin/bash
# Stop FixedWing SITL containers (FlightGear and/or headless JSBSim).

set -euo pipefail

FG_NAME="${PX4_SITL_DOCKER_NAME:-px4-noble-sim-ros}"
JSB_NAME="${PX4_JSBSIM_DOCKER_NAME:-px4-noble-jsbsim-rascal}"

usage() {
	echo "Usage: $0 [--fg|--jsbsim|--all]"
	echo "  --fg      Remove FlightGear container (${FG_NAME}) [default]"
	echo "  --jsbsim  Remove headless JSBSim container (${JSB_NAME})"
	echo "  --all     Remove both"
	echo "Env: PX4_SITL_DOCKER_NAME, PX4_JSBSIM_DOCKER_NAME"
}

kill_container() {
	local name="$1"
	if docker rm -f "${name}" >/dev/null 2>&1; then
		echo "Removed container: ${name}"
	else
		echo "No container to remove: ${name}"
	fi
}

TARGET="${1:---fg}"
case "${TARGET}" in
	--help|-h)
		usage
		exit 0
		;;
	--fg)
		kill_container "${FG_NAME}"
		xhost -local:docker 2>/dev/null || true
		;;
	--jsbsim)
		kill_container "${JSB_NAME}"
		;;
	--all)
		kill_container "${FG_NAME}"
		kill_container "${JSB_NAME}"
		xhost -local:docker 2>/dev/null || true
		;;
	*)
		echo "Unknown option: ${TARGET}" >&2
		usage >&2
		exit 1
		;;
esac
