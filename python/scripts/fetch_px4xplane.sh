#!/usr/bin/env bash
# Download Linux px4xplane plugin into python/assets/xplane/px4xplane/
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEST="${PYTHON_ROOT}/assets/xplane/px4xplane"
TAG="${PX4XPLANE_TAG:-v4.2.1}"
ZIP_URL="${PX4XPLANE_ZIP_URL:-https://github.com/alireza787b/px4xplane/releases/download/${TAG}/px4xplane-linux-${TAG}.zip}"
TMP="$(mktemp -d /tmp/px4xplane.XXXXXX)"
trap 'rm -rf "${TMP}"' EXIT

mkdir -p "${DEST}"
echo "Fetching ${ZIP_URL} → ${DEST}"
curl -fsSL -o "${TMP}/px4xplane.zip" "${ZIP_URL}"
unzip -qo "${TMP}/px4xplane.zip" -d "${TMP}/out"
# Zip layouts vary; find lin.xpl and copy its plugin directory.
XPL="$(find "${TMP}/out" -name 'lin.xpl' -print -quit || true)"
if [[ -z "${XPL}" ]]; then
	echo "Error: no lin.xpl in ${ZIP_URL}" >&2
	exit 1
fi
PLUGIN_DIR="$(dirname "${XPL}")"
# If lin.xpl sits in 64/, take parent as plugin root.
if [[ "$(basename "${PLUGIN_DIR}")" == "64" ]]; then
	PLUGIN_DIR="$(dirname "${PLUGIN_DIR}")"
fi
rm -rf "${DEST:?}/"*
cp -a "${PLUGIN_DIR}/." "${DEST}/"
echo "Installed px4xplane (${TAG}) → ${DEST}"
find "${DEST}" -name 'lin.xpl' -print
