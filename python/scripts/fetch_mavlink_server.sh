#!/usr/bin/env bash
# Fetch host-arch mavlink-server into python/bin/ (balloon-race fan-out).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${ROOT}/bin/mavlink-server"
VER="${MAVLINK_SERVER_VERSION:-0.10.1}"
ARCH="$(uname -m)"
case "${ARCH}" in
  x86_64|amd64) ASSET="mavlink-server-x86_64-unknown-linux-musl" ;;
  aarch64|arm64) ASSET="mavlink-server-aarch64-unknown-linux-musl" ;;
  *) echo "Unsupported arch: ${ARCH}" >&2; exit 1 ;;
esac
URL="https://github.com/bluerobotics/mavlink-server/releases/download/${VER}/${ASSET}"
mkdir -p "${ROOT}/bin"
TMP="${OUT}.tmp.$$"
echo "Downloading ${URL} → ${OUT}"
# Download to a temp path then mv: overwriting a running binary fails (ETXTBSY / curl 23).
curl -fsSL -o "${TMP}" "${URL}"
chmod +x "${TMP}"
mv -f "${TMP}" "${OUT}"
"${OUT}" --version
echo "OK: ${OUT}"
