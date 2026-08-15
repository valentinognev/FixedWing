#!/bin/bash
# Start PX4 SITL + YASim FlightGear FDM (Rascal) in the Noble sim container.
# Mounts fixedwing/fg_spawn.env so spawn altitude/speed can change without rebuilding the image.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PYTHON_ROOT}/.." && pwd)"

CONTAINER_NAME="${PX4_SITL_DOCKER_NAME:-px4-noble-sim-ros}"
IMAGE_TAG="${PX4_SITL_DOCKER_VER:-${CONTAINER_NAME}:latest}"
SPAWN_ENV="${PYTHON_ROOT}/assets/fg_spawn.env"
CONTAINER_SPAWN_ENV="/tmp/fg_spawn.env"
PATCH_SCRIPT="${REPO_ROOT}/Dockerfiles/patch_px4_flightgear_sitl.sh"
CONTAINER_PATCH="/tmp/patch_px4_flightgear_sitl.sh"
# Balloon .ac models: host → container path used by FG AI model-path / spawn_balloons_fg.
BALLOONS_HOST="${PYTHON_ROOT}/assets/balloons"
CONTAINER_BALLOONS="/opt/fixedwing/balloons"
MAVLINK_SERVER_SCRIPT="${REPO_ROOT}/Dockerfiles/scripts/start_mavlink_server.sh"
MAVLINK_SERVER_PID=""
# Default off for straight-flight/QGC; balloon race sets MAVLINK_FANOUT=1.
MAVLINK_FANOUT="${MAVLINK_FANOUT:-0}"

cleanup_on_exit() {
	echo ""
	echo "Cleaning up..."
	if [[ -n "${MAVLINK_SERVER_PID}" ]]; then
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
		--help|-h)
			echo "Usage: $0 [--kill] [--mavlink-server|--no-mavlink-server]"
			echo "  Starts YASim FlightGear FDM Rascal SITL with spawn args from ${SPAWN_ENV}"
			echo "  --kill  Remove container and exit"
			echo "  --mavlink-server     Start mavlink-server fan-out (14550→14540/14541); fail if unavailable"
			echo "  --no-mavlink-server  Skip fan-out (default; restores single-client 14540/14550)"
			echo "  Balloons bind-mount: ${BALLOONS_HOST} → ${CONTAINER_BALLOONS}"
			exit 0
			;;
		--mavlink-server) MAVLINK_FANOUT=1 ;;
		--no-mavlink-server) MAVLINK_FANOUT=0 ;;
		--kill)
			cleanup_on_exit
			exit 0
			;;
		*)
			echo "Unknown option: $1 (use --help)" >&2
			exit 1
			;;
	esac
	shift
done

if [[ ! -f "${SPAWN_ENV}" ]]; then
	echo "Missing spawn config: ${SPAWN_ENV}" >&2
	exit 1
fi
if [[ ! -f "${PATCH_SCRIPT}" ]]; then
	echo "Missing FG/PX4 patch script: ${PATCH_SCRIPT}" >&2
	exit 1
fi

trap cleanup_on_exit EXIT INT TERM

docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true

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

if ! start_mavlink_fanout; then
	echo "Error: mavlink fan-out requested (MAVLINK_FANOUT=1) but failed to start" >&2
	exit 1
fi

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

if [[ ! -d "${BALLOONS_HOST}" ]]; then
	echo "Warning: balloon assets missing: ${BALLOONS_HOST}" >&2
fi

DOCKER_VOLUMES=(
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
	--volume="${XDG_RUNTIME_DIR}:${XDG_RUNTIME_DIR}"
	--volume="${SPAWN_ENV}:${CONTAINER_SPAWN_ENV}:ro"
	--volume="${PATCH_SCRIPT}:${CONTAINER_PATCH}:ro"
	# Stable in-container path for FG AI balloon models (see balloon_scene.CONTAINER_BALLOONS_DIR).
	--volume="${BALLOONS_HOST}:${CONTAINER_BALLOONS}:ro"
)

if [[ -f "${XAUTH_FILE}" ]]; then
	DOCKER_VOLUMES+=(--volume="${XAUTH_FILE}:${XAUTH_FILE}:ro")
fi

# -t only when stdin is a real TTY *and* not launched from the Python helpers.
# Python passes stdin=DEVNULL so the debug console is never attached to PX4.
DOCKER_IT=(-i)
if [[ -t 0 && -z "${PX4_SITL_NO_DOCKER_TTY:-}" ]]; then
	DOCKER_IT=(-it)
fi

echo "Starting ${IMAGE_TAG} with spawn config ${SPAWN_ENV}"
echo "Balloons mount: ${BALLOONS_HOST} → ${CONTAINER_BALLOONS}"
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
		set -a; source '${CONTAINER_SPAWN_ENV}'; set +a
		cd /home/valentin/PX4-Autopilot
		bash '${CONTAINER_PATCH}' /home/valentin/PX4-Autopilot
		if [[ -d '${CONTAINER_BALLOONS}' && -d /opt/flightgear/fgdata ]]; then
		  mkdir -p /opt/flightgear/fgdata/Models/FixedWing
		  cp -f '${CONTAINER_BALLOONS}'/balloon_*.ac '${CONTAINER_BALLOONS}'/balloon_*.xml \
		    /opt/flightgear/fgdata/Models/FixedWing/ 2>/dev/null || true
		fi
		# Rebuild bridge if HIL_SENSOR / RPM patches changed sources.
		ninja -C build/px4_sitl_nolockstep flightgear_bridge
		make px4_sitl_nolockstep flightgear_rascal
	"
