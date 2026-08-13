"""Synthetic pinhole renderer: colored balloon disks from MAVLink pose."""
from __future__ import annotations

import time

import cv2
import numpy as np
from pymavlink import mavutil

from fw_sitl.camera_model import CameraModel, project_ned_offset_to_pixel
from fw_sitl.flight_setup import BalloonSpec, CameraSpec, FlightSetup
from fw_sitl.mavlink_io import connect, poll_mavlink, request_local_position
from fw_sitl.race_guidance import rebase_balloons_to_local_z
from fw_sitl.zmq_bus import ImagePublisher


def render_frame(
    pos_ned: tuple[float, float, float],
    roll: float,
    pitch: float,
    yaw: float,
    balloons: tuple[BalloonSpec, ...],
    camera: CameraSpec,
    *,
    sky_rgb: tuple[int, int, int] = (135, 206, 235),
    ground_rgb: tuple[int, int, int] = (34, 120, 34),
    rebase_z_to_aircraft: bool = True,
) -> np.ndarray:
    model = CameraModel.from_spec(camera)
    img = np.zeros((model.height_px, model.width_px, 3), dtype=np.uint8)
    horizon_y = int(model.cy)
    img[:horizon_y, :] = np.array(sky_rgb, dtype=np.uint8)
    img[horizon_y:, :] = np.array(ground_rgb, dtype=np.uint8)

    pos = np.array(pos_ned, dtype=np.float64)
    draw_list: list[tuple[float, float, float, tuple[int, int, int], float]] = []
    render_balloons = (
        rebase_balloons_to_local_z(balloons, float(pos_ned[2]))
        if rebase_z_to_aircraft
        else balloons
    )

    for spec in render_balloons:
        target_ned = np.array(spec.ned, dtype=np.float64)
        rel_ned = tuple((target_ned - pos).tolist())
        range_m = float(np.linalg.norm(rel_ned))
        pixel = project_ned_offset_to_pixel(rel_ned, model, roll, pitch, yaw)
        if pixel is None:
            continue
        u, v = pixel
        angular_diam = spec.diameter_m / max(range_m, 1.0)
        radius_px = max(3.0, model.fx * angular_diam * 0.5)
        draw_list.append((u, v, radius_px, spec.color, range_m))

    draw_list.sort(key=lambda item: item[4], reverse=True)
    for u, v, radius_px, rgb, _ in draw_list:
        if 0 <= u < model.width_px and 0 <= v < model.height_px:
            cv2.circle(
                img,
                (int(round(u)), int(round(v))),
                int(round(radius_px)),
                rgb,
                -1,
            )

    return img


def run_synthetic_publisher(
    setup: FlightSetup,
    udp_port: int = 14541,
) -> None:
    master = connect(udp_port, timeout=120.0)
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

    pub = ImagePublisher(setup.zmq.image)
    period = 1.0 / setup.render_rate_hz
    pos = (0.0, 0.0, -80.0)
    att = (0.0, 0.0, 0.0)
    next_t = time.time()
    print(f"Synthetic camera publishing @ {setup.render_rate_hz} Hz → {setup.zmq.image}")

    while True:
        pos_msg, _, _ = poll_mavlink(master)
        while True:
            msg = master.recv_match(type="ATTITUDE", blocking=False)
            if msg is None:
                break
            att = (float(msg.roll), float(msg.pitch), float(msg.yaw))
        if pos_msg is not None:
            pos = pos_msg

        img = render_frame(pos, att[0], att[1], att[2], setup.balloons, setup.camera)
        pub.publish(img)
        next_t += period
        sleep = next_t - time.time()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_t = time.time()
