#!/bin/bash
# Stop FixedWing SITL containers (FlightGear, JSBSim, Gazebo, and/or X-Plane).

set -euo pipefail

FG_NAME="${PX4_SITL_DOCKER_NAME:-px4-noble-sim-ros}"
JSB_NAME="${PX4_JSBSIM_DOCKER_NAME:-px4-noble-jsbsim-rascal}"
GZ_NAME="${PX4_GZ_DOCKER_NAME:-px4-noble-gz-plane}"
XP_NAME="${PX4_XP_DOCKER_NAME:-px4-noble-xplane-cessna}"

usage() {
	echo "Usage: $0 [--fg|--jsbsim|--gz|--xplane|--all]"
	echo "  --fg      Remove FlightGear container (${FG_NAME}) [default]"
	echo "  --jsbsim  Remove headless JSBSim container (${JSB_NAME})"
	echo "  --gz      Remove Gazebo plane container (${GZ_NAME})"
	echo "  --xplane  Remove X-Plane Cessna container (${XP_NAME})"
	echo "  --all     Remove FlightGear, JSBSim, Gazebo, and X-Plane"
	echo "Env: PX4_SITL_DOCKER_NAME, PX4_JSBSIM_DOCKER_NAME, PX4_GZ_DOCKER_NAME, PX4_XP_DOCKER_NAME"
}

kill_container() {
	local name="$1"
	if docker rm -f "${name}" >/dev/null 2>&1; then
		echo "Removed container: ${name}"
	else
		echo "No container to remove: ${name}"
	fi
}

kill_jsbsim_stack() {
	# Sim plant + optional mavlink-server sidecar from --mavlink-server / balloon race.
	kill_container "${JSB_NAME}"
	kill_container "${JSB_NAME}-mavlink"
	# Host mavlink-server (if started outside Docker).
	if pgrep -f '[m]avlink-server' >/dev/null 2>&1; then
		pkill -f '[m]avlink-server' 2>/dev/null || true
		echo "Stopped host mavlink-server process(es)"
	fi
}

kill_gz_stack() {
	# Gazebo plane + optional mavlink-server sidecar from --mavlink-server / balloon race.
	kill_container "${GZ_NAME}"
	kill_container "${GZ_NAME}-mavlink"
	# Host mavlink-server (if started outside Docker).
	if pgrep -f '[m]avlink-server' >/dev/null 2>&1; then
		pkill -f '[m]avlink-server' 2>/dev/null || true
		echo "Stopped host mavlink-server process(es)"
	fi
}

kill_fg_stack() {
	kill_container "${FG_NAME}"
	kill_container "${FG_NAME}-mavlink"
	if pgrep -f '[m]avlink-server' >/dev/null 2>&1; then
		pkill -f '[m]avlink-server' 2>/dev/null || true
		echo "Stopped host mavlink-server process(es)"
	fi
}

kill_xplane_stack() {
	kill_container "${XP_NAME}"
	kill_container "${XP_NAME}-mavlink"
	if pgrep -f '[m]avlink-server' >/dev/null 2>&1; then
		pkill -f '[m]avlink-server' 2>/dev/null || true
		echo "Stopped host mavlink-server process(es)"
	fi
}

TARGET="${1:---fg}"
case "${TARGET}" in
	--help|-h)
		usage
		exit 0
		;;
	--fg)
		kill_fg_stack
		xhost -local:docker 2>/dev/null || true
		;;
	--jsbsim)
		kill_jsbsim_stack
		;;
	--gz)
		kill_gz_stack
		;;
	--xplane)
		kill_xplane_stack
		xhost -local:docker 2>/dev/null || true
		;;
	--all)
		kill_fg_stack
		kill_jsbsim_stack
		kill_gz_stack
		kill_xplane_stack
		xhost -local:docker 2>/dev/null || true
		;;
	*)
		echo "Unknown option: ${TARGET}" >&2
		usage >&2
		exit 1
		;;
esac
