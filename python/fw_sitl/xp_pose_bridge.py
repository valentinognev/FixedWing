"""X-Plane aircraft pose → ZMQ (ENU), for balloon-race control rebase."""
from __future__ import annotations

import time
from dataclasses import dataclass

from fw_sitl.flight_setup import FlightSetup
from fw_sitl.xp_balloon import XP_BALLOON_PORT, encode_pose_query, udp_transact
from fw_sitl.xp_origin import (
    XP_ORIGIN_ALT_MSL_M,
    XP_ORIGIN_LAT_DEG,
    XP_ORIGIN_LON_DEG,
    geodetic_to_ned_xp,
)
from fw_sitl.zmq_bus import PosePublisher, PoseSample


@dataclass(frozen=True)
class XpPoseSample:
    stamp: float
    lat: float
    lon: float
    alt_msl_m: float
    roll_deg: float
    pitch_deg: float
    heading_deg: float

    def as_ned(
        self,
        *,
        origin_lat: float = XP_ORIGIN_LAT_DEG,
        origin_lon: float = XP_ORIGIN_LON_DEG,
        origin_alt_msl_m: float = XP_ORIGIN_ALT_MSL_M,
    ) -> tuple[float, float, float]:
        return geodetic_to_ned_xp(
            self.lat,
            self.lon,
            self.alt_msl_m,
            origin_lat_deg=origin_lat,
            origin_lon_deg=origin_lon,
            origin_alt_msl_m=origin_alt_msl_m,
        )


def fetch_xp_pose(
    *,
    host: str = "127.0.0.1",
    port: int = XP_BALLOON_PORT,
    timeout_s: float = 1.0,
) -> XpPoseSample | None:
    try:
        reply = udp_transact(
            encode_pose_query(), host=host, port=port, timeout_s=timeout_s
        )
    except OSError:
        return None
    if not reply.get("ok"):
        return None
    try:
        return XpPoseSample(
            stamp=time.time(),
            lat=float(reply["lat"]),
            lon=float(reply["lon"]),
            alt_msl_m=float(reply["alt_msl_m"]),
            roll_deg=float(reply.get("roll_deg", 0.0)),
            pitch_deg=float(reply.get("pitch_deg", 0.0)),
            heading_deg=float(reply.get("heading_deg", 0.0)),
        )
    except (KeyError, TypeError, ValueError):
        return None


def run_xp_pose_publisher(
    setup: FlightSetup,
    *,
    host: str = "127.0.0.1",
    port: int = XP_BALLOON_PORT,
    rate_hz: float = 40.0,
) -> None:
    """PUB PoseSample ENU (x=E, y=N, z=U) from LOWS-frame NED."""
    pub = PosePublisher(setup.zmq.pose)
    period = 1.0 / max(rate_hz, 1.0)
    print(
        f"XP pose publishing @ {rate_hz:.0f} Hz → {setup.zmq.pose} "
        f"(plugin UDP {host}:{port})"
    )
    next_t = time.time()
    try:
        while True:
            sample = fetch_xp_pose(host=host, port=port, timeout_s=0.5)
            if sample is not None:
                n, e, d = sample.as_ned()
                pub.publish(
                    PoseSample(stamp=sample.stamp, x=e, y=n, z=-d)
                )
            next_t += period
            sleep = next_t - time.time()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.time()
    finally:
        pub.close()
