#!/bin/bash
# Start PX4 SITL + X-Plane 12 demo Cessna (host install bind-mounted into Noble).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PYTHON_ROOT}/.." && pwd)"

CONTAINER_NAME="${PX4_XP_DOCKER_NAME:-px4-noble-xplane-cessna}"
IMAGE_TAG="${PX4_SITL_DOCKER_VER:-px4-noble-sim-ros:latest}"
XP12_HOME="${XP12_HOME:-${HOME}/X-Plane 12}"
XP12_AIRPORT="${XP12_AIRPORT:-LOWS}"
CONTAINER_XP="/opt/xplane12"
AIRFRAME_HOST="${PYTHON_ROOT}/assets/xplane/5001_xplane_cessna172"
PX4XPLANE_HOST="${PYTHON_ROOT}/assets/xplane/px4xplane"
BALLOON_PLUGIN_HOST="${PYTHON_ROOT}/assets/xplane/plugin"
MAVLINK_SERVER_SCRIPT="${REPO_ROOT}/Dockerfiles/scripts/start_mavlink_server.sh"
MAVLINK_SERVER_PID=""
MAVLINK_FANOUT="${MAVLINK_FANOUT:-0}"
SETUP="${PYTHON_ROOT}/flightSetup.json"
SPAWN_CSV=""

usage() {
	echo "Usage: $0 [--setup PATH] [--mavlink-server|--no-mavlink-server] [--kill]"
	echo "  Bind-mounts XP12_HOME (${XP12_HOME}) → ${CONTAINER_XP}"
	echo "  Requires ${XP12_HOME}/X-Plane-x86_64 and Cessna 172 SP"
	echo "  Airport default: ${XP12_AIRPORT} (demo scenery)"
	echo "  Plugins: px4xplane + fixedwing_balloons under Resources/plugins"
}

cleanup_on_exit() {
	echo ""
	echo "Cleaning up..."
	if [[ -n "${MAVLINK_SERVER_PID:-}" ]]; then
		kill "${MAVLINK_SERVER_PID}" 2>/dev/null || true
		wait "${MAVLINK_SERVER_PID}" 2>/dev/null || true
		MAVLINK_SERVER_PID=""
	fi
	docker rm -f "${CONTAINER_NAME}-mavlink" 2>/dev/null || true
	docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true
	xhost -local:docker 2>/dev/null || true
}

while [[ $# -gt 0 ]]; do
	case "$1" in
		--help|-h) usage; exit 0 ;;
		--setup) SETUP="$2"; shift ;;
		--mavlink-server) MAVLINK_FANOUT=1 ;;
		--no-mavlink-server) MAVLINK_FANOUT=0 ;;
		--kill) cleanup_on_exit; exit 0 ;;
		*) echo "Unknown option: $1" >&2; exit 1 ;;
	esac
	shift
done

if [[ ! -x "${XP12_HOME}/X-Plane-x86_64" ]]; then
	echo "Error: missing X-Plane binary at ${XP12_HOME}/X-Plane-x86_64" >&2
	echo "  Set XP12_HOME to your X-Plane 12 demo install." >&2
	exit 1
fi
ACF="${XP12_HOME}/Aircraft/Laminar Research/Cessna 172 SP/Cessna_172SP.acf"
if [[ ! -f "${ACF}" ]]; then
	echo "Error: missing Cessna at ${ACF}" >&2
	exit 1
fi
if [[ ! -d "${XP12_HOME}/Global Scenery/X-Plane 12 Demo Areas" ]]; then
	echo "Error: missing demo scenery under ${XP12_HOME}/Global Scenery" >&2
	exit 1
fi
if [[ ! -f "${AIRFRAME_HOST}" ]]; then
	echo "Error: missing airframe ${AIRFRAME_HOST}" >&2
	exit 1
fi
if [[ ! -f "${SETUP}" ]]; then
	echo "Error: missing setup ${SETUP}" >&2
	exit 1
fi
if [[ -z "${DISPLAY:-}" ]]; then
	echo "Error: DISPLAY is not set (X-Plane GUI required)" >&2
	exit 1
fi
if ! docker image inspect "${IMAGE_TAG}" >/dev/null 2>&1; then
	echo "Error: Docker image '${IMAGE_TAG}' not found locally." >&2
	echo "  Build: cd ${REPO_ROOT}/Dockerfiles && ./PX4_noble_sim_build.sh" >&2
	exit 1
fi

# Ensure Linux px4xplane is present (gitignored download).
if [[ ! -f "${PX4XPLANE_HOST}/64/lin.xpl" && ! -f "${PX4XPLANE_HOST}/lin.xpl" ]]; then
	echo "px4xplane missing; fetching..."
	bash "${SCRIPT_DIR}/fetch_px4xplane.sh"
fi

SPAWN_CSV="$(
	PYTHONPATH="${PYTHON_ROOT}${PYTHONPATH:+:${PYTHONPATH}}" \
		python3 -m fw_sitl.spawn_ic --setup "${SETUP}" --xp-geodetic
)"
echo "XP spawn geodetic: ${SPAWN_CSV}"

trap cleanup_on_exit EXIT INT TERM
docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true

mavlink_server_usable() {
	local candidate="$1"
	[[ -n "${candidate}" && -x "${candidate}" ]] || return 1
	"${candidate}" --version >/dev/null 2>&1
}

resolve_mavlink_server_bin() {
	local c=""
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
	done
	return 1
}

ensure_host_mavlink_server() {
	if resolve_mavlink_server_bin >/dev/null; then
		return 0
	fi
	if [[ -x "${SCRIPT_DIR}/fetch_mavlink_server.sh" ]]; then
		bash "${SCRIPT_DIR}/fetch_mavlink_server.sh" || true
	fi
	resolve_mavlink_server_bin >/dev/null
}

start_mavlink_fanout() {
	[[ "${MAVLINK_FANOUT}" == "1" ]] || return 0
	if [[ ! -f "${MAVLINK_SERVER_SCRIPT}" ]]; then
		echo "Error: missing ${MAVLINK_SERVER_SCRIPT}" >&2
		return 1
	fi
	ensure_host_mavlink_server || true
	local mav_log="/tmp/${CONTAINER_NAME}-mavlink-server.log"
	local bin=""
	if bin="$(resolve_mavlink_server_bin)"; then
		: >"${mav_log}" || true
		MAVLINK_SERVER_BIN="${bin}" \
			MAVLINK_SERVER_RUST_LOG="${MAVLINK_SERVER_RUST_LOG:-off}" \
			bash "${MAVLINK_SERVER_SCRIPT}" >>"${mav_log}" 2>&1 &
		MAVLINK_SERVER_PID=$!
		sleep 0.4
		if ! kill -0 "${MAVLINK_SERVER_PID}" 2>/dev/null; then
			echo "Error: mavlink-server host process exited; see ${mav_log}" >&2
			MAVLINK_SERVER_PID=""
			return 1
		fi
		echo "mavlink-server host PID=${MAVLINK_SERVER_PID} (${bin})"
		return 0
	fi
	docker rm -f "${CONTAINER_NAME}-mavlink" 2>/dev/null || true
	docker run --rm -d --net=host --name "${CONTAINER_NAME}-mavlink" \
		-e "RUST_LOG=${MAVLINK_SERVER_RUST_LOG:-off}" \
		--entrypoint /usr/local/bin/mavlink-server \
		"${IMAGE_TAG}" \
		--web-server "${MAVLINK_WEB_SERVER:-127.0.0.1:6040}" \
		--mavlink-heartbeat-frequency 0 \
		"udpserver://0.0.0.0:${MAVLINK_GCS_PORT:-14550}" \
		"udpclient://127.0.0.1:${MAVLINK_CONTROL_PORT:-14540}" \
		"udpclient://127.0.0.1:${MAVLINK_IMAGE_PORT:-14541}" >/dev/null
	sleep 0.4
	if ! docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${CONTAINER_NAME}-mavlink"; then
		echo "Error: mavlink-server sidecar exited" >&2
		return 1
	fi
	echo "Started mavlink-server sidecar ${CONTAINER_NAME}-mavlink"
	return 0
}

if ! start_mavlink_fanout; then
	echo "Error: mavlink fan-out requested but failed" >&2
	exit 1
fi

xhost + 2>/dev/null || true
xhost +local:docker 2>/dev/null || true
export DISPLAY="${DISPLAY:-:0}"
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
	--volume="${XP12_HOME}:${CONTAINER_XP}"
	--volume="${AIRFRAME_HOST}:/tmp/5001_xplane_cessna172:ro"
	--volume="${PX4XPLANE_HOST}:/tmp/px4xplane:ro"
	--volume="${BALLOON_PLUGIN_HOST}:/tmp/fixedwing_balloons_src:ro"
)
if [[ -f "${XAUTH_FILE}" ]]; then
	DOCKER_VOLUMES+=(--volume="${XAUTH_FILE}:${XAUTH_FILE}:ro")
fi

DOCKER_IT=(-i)
if [[ -t 0 && -z "${PX4_SITL_NO_DOCKER_TTY:-}" ]]; then
	DOCKER_IT=(-it)
fi

echo "Starting ${IMAGE_TAG} as ${CONTAINER_NAME}"
echo "  XP12_HOME=${XP12_HOME} → ${CONTAINER_XP}  airport=${XP12_AIRPORT}"
echo "  spawn=${SPAWN_CSV}"

# shellcheck disable=SC2086
docker run "${DOCKER_IT[@]}" --rm \
	--net=host \
	--privileged \
	--gpus all \
	--name "${CONTAINER_NAME}" \
	--env="DISPLAY=${DISPLAY}" \
	--env="QT_X11_NO_MITSHM=1" \
	--env="XAUTHORITY=${XAUTH_FILE}" \
	--env="XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR}" \
	--env="XP12_AIRPORT=${XP12_AIRPORT}" \
	--env="XP_SPAWN_CSV=${SPAWN_CSV}" \
	"${DOCKER_VOLUMES[@]}" \
	"${IMAGE_TAG}" \
	/bin/bash -lc "set -euo pipefail
		XP='${CONTAINER_XP}'
		AIRFRAME_DST=''
		for d in \
			/home/valentin/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes \
			/home/valentin/PX4-Autopilot/build/px4_sitl_nolockstep/etc/init.d-posix/airframes \
			/home/valentin/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes
		do
			if [[ -d \"\$d\" ]]; then
				cp -f /tmp/5001_xplane_cessna172 \"\$d/5001_xplane_cessna172\"
				chmod +x \"\$d/5001_xplane_cessna172\"
				AIRFRAME_DST=\"\$d/5001_xplane_cessna172\"
			fi
		done
		if [[ -z \"\${AIRFRAME_DST}\" ]]; then
			echo 'Error: no PX4 airframes dir found in image' >&2
			exit 1
		fi
		echo \"Installed airframe → \${AIRFRAME_DST}\"

		# Install px4xplane into the bind-mounted XP tree (persists on host).
		mkdir -p \"\${XP}/Resources/plugins/px4xplane\"
		cp -a /tmp/px4xplane/. \"\${XP}/Resources/plugins/px4xplane/\"
		# Prefer a prebuilt lin.xpl; otherwise compile from sources in-container.
		mkdir -p \"\${XP}/Resources/plugins/fixedwing_balloons/64\"
		if [[ -f /tmp/fixedwing_balloons_src/64/lin.xpl ]]; then
			cp -f /tmp/fixedwing_balloons_src/64/lin.xpl \\
				\"\${XP}/Resources/plugins/fixedwing_balloons/64/lin.xpl\"
		elif [[ -f /tmp/fixedwing_balloons_src/Makefile ]]; then
			env -u CXXFLAGS -u LDFLAGS -u CPPFLAGS \\
				make -C /tmp/fixedwing_balloons_src \\
				CXX=/usr/bin/g++ \\
				OUTDIR=/tmp/fw_balloons_build SDK_DIR=/tmp/xplane_sdk
			cp -f /tmp/fw_balloons_build/lin.xpl \\
				\"\${XP}/Resources/plugins/fixedwing_balloons/64/lin.xpl\"
		else
			echo 'Error: fixedwing_balloons sources/binary missing' >&2
			exit 1
		fi
		if [[ -f /tmp/fixedwing_balloons_src/balloon_sphere.obj ]]; then
			cp -f /tmp/fixedwing_balloons_src/balloon_sphere.obj \\
				\"\${XP}/Resources/plugins/fixedwing_balloons/balloon_sphere.obj\"
		fi
		echo \"Plugins ready under \${XP}/Resources/plugins/\"

		# Launch X-Plane (demo airport + Cessna). --no_sound reduces host noise.
		ACF=\"\${XP}/Aircraft/Laminar Research/Cessna 172 SP/Cessna_172SP.acf\"
		\"\${XP}/X-Plane-x86_64\" \
			--airport \"\${XP12_AIRPORT}\" \
			--flight_model \"\${ACF}\" \
			--no_sound \
			>/tmp/xplane.log 2>&1 &
		XP_PID=\$!
		echo \"X-Plane PID=\${XP_PID}; log /tmp/xplane.log\"
		# Give the GUI + plugins time to load before PX4 HIL connect.
		sleep 25
		if ! kill -0 \"\${XP_PID}\" 2>/dev/null; then
			echo 'Error: X-Plane exited early; last log lines:' >&2
			tail -n 40 /tmp/xplane.log >&2 || true
			exit 1
		fi

		cd /home/valentin/PX4-Autopilot
		PX4_BIN=''
		for b in build/px4_sitl_default/bin/px4 build/px4_sitl_nolockstep/bin/px4; do
			if [[ -x \"\$b\" ]]; then PX4_BIN=\"\$b\"; break; fi
		done
		if [[ -z \"\${PX4_BIN}\" ]]; then
			echo 'Error: no px4 binary in image' >&2
			exit 1
		fi
		export PX4_SIMULATOR=xplane
		export PX4_SYS_AUTOSTART=5001
		export PX4_SIM_MODEL=xplane_cessna172
		echo \"Launching \${PX4_BIN} (SYS_AUTOSTART=5001, spawn=\${XP_SPAWN_CSV})\"
		exec \"\${PX4_BIN}\"
	"
