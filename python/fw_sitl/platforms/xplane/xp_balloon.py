"""UDP client for the X-Plane ``fixedwing_balloons`` plugin."""
from __future__ import annotations

import json
import socket
import time
from collections.abc import Callable
from typing import Any

from fw_sitl.balloon_scene import gz_balloon_model_name, ned_to_geodetic
from fw_sitl.flight_setup import FlightSetup
from fw_sitl.race_guidance import rebase_balloons_to_local_z
from fw_sitl.platforms.xplane.xp_origin import XP_AIRCRAFT_MSL_M, XP_ORIGIN_LAT_DEG, XP_ORIGIN_LON_DEG

XP_BALLOON_PORT = 49091

TransactFn = Callable[[bytes], dict[str, Any]]


def encode_clear() -> bytes:
    return json.dumps({"cmd": "clear"}, separators=(",", ":")).encode()


def encode_sitl_connect() -> bytes:
    return json.dumps({"cmd": "sitl_connect"}, separators=(",", ":")).encode()


def encode_pose_query() -> bytes:
    return json.dumps({"cmd": "pose_query"}, separators=(",", ":")).encode()


def encode_place(
    name: str,
    lat: float,
    lon: float,
    alt_msl_m: float,
    diameter_m: float,
    rgb: tuple[int, int, int],
) -> bytes:
    return json.dumps(
        {
            "cmd": "place",
            "name": str(name),
            "lat": float(lat),
            "lon": float(lon),
            "alt_msl_m": float(alt_msl_m),
            "diameter_m": float(diameter_m),
            "rgb": [int(rgb[0]), int(rgb[1]), int(rgb[2])],
        },
        separators=(",", ":"),
    ).encode()


def parse_reply(raw: bytes) -> dict[str, Any]:
    return json.loads(raw.decode("utf-8"))


def udp_transact(
    payload: bytes,
    *,
    host: str = "127.0.0.1",
    port: int = XP_BALLOON_PORT,
    timeout_s: float = 2.0,
) -> dict[str, Any]:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.settimeout(float(timeout_s))
        sock.sendto(payload, (host, int(port)))
        raw, _addr = sock.recvfrom(4096)
    finally:
        sock.close()
    return parse_reply(raw)


def origin_latlon_from_xp(
    lat_deg: float | None,
    lon_deg: float | None,
    *,
    fallback_lat_deg: float = XP_ORIGIN_LAT_DEG,
    fallback_lon_deg: float = XP_ORIGIN_LON_DEG,
) -> tuple[float, float]:
    """Reuse FG live-origin gates against X-Plane pose."""
    from fw_sitl.balloon_scene import origin_latlon_from_fg

    return origin_latlon_from_fg(
        lat_deg,
        lon_deg,
        fallback_lat_deg=fallback_lat_deg,
        fallback_lon_deg=fallback_lon_deg,
    )


def spawn_xp_from_setup(
    setup: FlightSetup,
    *,
    host: str = "127.0.0.1",
    port: int = XP_BALLOON_PORT,
    timeout_s: float = 90.0,
    transact: TransactFn | None = None,
) -> int:
    """Place visual-only balloons; return 0 on success, 1 on timeout/error."""
    send: TransactFn
    if transact is not None:
        send = transact
    else:
        def send(payload: bytes) -> dict[str, Any]:
            return udp_transact(
                payload, host=host, port=port, timeout_s=min(2.0, float(timeout_s))
            )

    deadline = time.time() + float(timeout_s)
    last_exc: BaseException | None = None
    while time.time() < deadline:
        try:
            pose = send(encode_pose_query())
            lat0, lon0 = origin_latlon_from_xp(
                pose.get("lat"), pose.get("lon")
            )
            alt0 = pose.get("alt_msl_m")
            if alt0 is None or not isinstance(alt0, (int, float)):
                alt0 = XP_AIRCRAFT_MSL_M
            else:
                alt0 = float(alt0)
                if alt0 < 100.0:
                    alt0 = XP_AIRCRAFT_MSL_M
            ack = send(encode_clear())
            if not ack.get("ok", True):
                raise RuntimeError(f"xp balloon clear failed: {ack}")
            balloons = rebase_balloons_to_local_z(setup.balloons, local_z=0.0)
            for spec in balloons:
                lat, lon, alt = ned_to_geodetic(
                    spec.ned[0],
                    spec.ned[1],
                    spec.ned[2],
                    lat0,
                    lon0,
                    alt0,
                )
                name = gz_balloon_model_name(spec.color)
                placed = send(
                    encode_place(
                        name, lat, lon, alt, spec.diameter_m, spec.color
                    )
                )
                if not placed.get("ok", True):
                    raise RuntimeError(f"xp balloon place failed: {placed}")
            return 0
        except Exception as exc:  # noqa: BLE001
            last_exc = exc
            if transact is not None:
                break
            time.sleep(1.0)
    print(f"XP balloon spawn failed: {last_exc}")
    return 1
