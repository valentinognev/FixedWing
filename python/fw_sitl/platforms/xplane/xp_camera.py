"""X-Plane window capture for balloon race (mss, same path as FG)."""
from __future__ import annotations

import time
from dataclasses import replace

import cv2
from pymavlink import mavutil

from fw_sitl.platforms.yasim.fg_camera import (
    FG_GEO_REFRESH_PERIOD_S,
    capture_fg_frame,
    due_for_refresh,
    find_fg_window_geometry,
)
from fw_sitl.flight_setup import DEFAULT_FG_WINDOW_PATTERN, FlightSetup
from fw_sitl.mavlink_io import connect, poll_mavlink, request_local_position
from fw_sitl.zmq_bus import ImagePublisher

DEFAULT_XP_WINDOW_PATTERN = "X-Plane|X-System"


def xp_window_pattern(setup: FlightSetup) -> str:
    """Prefer camera.xp_window_pattern; fall back to FG pattern only if XP unset."""
    cam = setup.camera
    pat = getattr(cam, "xp_window_pattern", None)
    if pat and str(pat).strip():
        return str(pat).strip()
    if cam.fg_window_pattern and cam.fg_window_pattern != DEFAULT_FG_WINDOW_PATTERN:
        return cam.fg_window_pattern
    return DEFAULT_XP_WINDOW_PATTERN


def run_xp_publisher(setup: FlightSetup, *, udp_port: int = 14541) -> None:
    """Grab the X-Plane window and PUB RGB frames (no FG telnet view sync)."""
    master = connect(udp_port, timeout=180.0)
    request_local_position(master, hz=setup.render_rate_hz)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        int(1e6 / max(setup.render_rate_hz, 1.0)),
        0,
        0,
        0,
        0,
        0,
    )
    # Capture path expects CameraSpec.fg_window_pattern; overlay XP pattern.
    pattern = xp_window_pattern(setup)
    cam = replace(setup.camera, fg_window_pattern=pattern)
    pub = ImagePublisher(setup.zmq.image)
    period = 1.0 / setup.render_rate_hz
    next_t = time.time()
    last_geo_s = 0.0
    geo: dict[str, int] | None = None
    sct: object | None = None
    try:
        import mss

        sct = mss.mss()
    except Exception:  # noqa: BLE001
        sct = None
    print(
        f"XP capture publishing @ {setup.render_rate_hz} Hz → {setup.zmq.image} "
        f"(window /{pattern}/, mavlink UDP {udp_port})"
    )
    try:
        while True:
            while True:
                msg = master.recv_match(type="ATTITUDE", blocking=False)
                if msg is None:
                    break
            poll_mavlink(master)
            now = time.time()
            if due_for_refresh(now, last_geo_s, FG_GEO_REFRESH_PERIOD_S):
                geo = find_fg_window_geometry(pattern)
                last_geo_s = now
            frame = capture_fg_frame(
                cam,
                window_pattern=pattern,
                geometry=geo,
                sct=sct,
                locate=False,
            )
            if frame is not None:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                pub.publish(rgb)
            next_t += period
            sleep = next_t - time.time()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.time()
    finally:
        close = getattr(sct, "close", None)
        if callable(close):
            close()
