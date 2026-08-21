#!/usr/bin/env python3
"""Camera process: track commanded color, PUB camera-frame LOS, display window."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parent
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

import cv2
import numpy as np

from fw_sitl.balloon_tracker import track_balloon
from fw_sitl.camera_model import CameraModel
from fw_sitl.fg_camera import (
    FG_GEO_REFRESH_PERIOD_S,
    due_for_refresh,
    find_fg_window_geometry,
    fit_window_outside_rect,
    virtual_screen_rect,
)
from fw_sitl.flight_setup import load_flight_setup
from fw_sitl.race_guidance import ASSISTED_OVERLAY_TEXT, format_ned_pos_line, show_assisted_overlay
from fw_sitl.zmq_bus import ColorSubscriber, ImageSubscriber, TrackPublisher


def main() -> int:
    parser = argparse.ArgumentParser(description="Balloon race camera tracker")
    parser.add_argument(
        "--setup",
        type=Path,
        default=_PYTHON_ROOT / "flightSetup.json",
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Track and PUB without OpenCV window (for headless / e2e)",
    )
    args = parser.parse_args()

    setup = load_flight_setup(args.setup)
    camera = CameraModel.from_spec(setup.camera)
    img_sub = ImageSubscriber(setup.zmq.image)
    color_sub = ColorSubscriber(setup.zmq.color)
    track_pub = TrackPublisher(setup.zmq.track)

    target_rgb = setup.balloons[0].color
    assisted_flag = False
    period = 1.0 / setup.camera.rate_hz
    show_ui = not args.no_display
    win = "balloon_camera"
    last_place_s = 0.0
    if show_ui:
        # WINDOW_NORMAL is 0, same bit as WINDOW_GUI_EXPANDED. That Qt chrome
        # plus conda OpenCV 5 (no bundled fonts) paints balloon_camera black.
        cv2.namedWindow(win, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
        cv2.resizeWindow(win, camera.width_px, camera.height_px)
        # Do not leave the HighGUI default at (0,0): mss captures whatever
        # overlaps the FG rectangle, including this window (window-in-window).

        def _park_camera(fg_geo: dict[str, int] | None) -> None:
            px, py, pw, ph = fit_window_outside_rect(
                fg_geo,
                camera.width_px,
                camera.height_px,
                screen=virtual_screen_rect(),
            )
            cv2.resizeWindow(win, pw, ph)
            cv2.moveWindow(win, px, py)

        _park_camera(None)
        placeholder = np.zeros((camera.height_px, camera.width_px, 3), dtype=np.uint8)
        cv2.putText(
            placeholder,
            "waiting for image",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (200, 200, 200),
            2,
        )
        cv2.imshow(win, placeholder)
        cv2.waitKey(1)
        geo = find_fg_window_geometry(setup.camera.fg_window_pattern)
        if geo is not None:
            _park_camera(geo)
            cv2.waitKey(1)
            last_place_s = time.time()
    print(
        f"Camera @ {setup.camera.rate_hz} Hz; image={setup.zmq.image} "
        f"track→{setup.zmq.track}; display={'on' if show_ui else 'off'}"
    )

    next_t = time.time()
    while True:
        color_sub.poll_and_update()
        latest_color = color_sub.latest()
        if latest_color is not None:
            target_rgb = latest_color.as_tuple()
            assisted_flag = latest_color.assisted

        img_sub.poll_and_update()
        latest = img_sub.latest()
        if show_ui and due_for_refresh(
            time.time(), last_place_s, FG_GEO_REFRESH_PERIOD_S
        ):
            geo = find_fg_window_geometry(setup.camera.fg_window_pattern)
            last_place_s = time.time()
            _park_camera(geo)
        if latest is not None:
            frame_rgb = latest.as_numpy()
            result = track_balloon(frame_rgb, target_rgb, camera)
            track_pub.publish_track(
                result.in_view,
                result.dir_cam,
                result.centroid_uv,
                area_px=float(result.area_px),
            )
            if show_ui:
                display = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                if result.in_view and result.centroid_uv is not None:
                    cx, cy = result.centroid_uv
                    cv2.circle(display, (int(cx), int(cy)), 8, (0, 255, 255), 2)
                if show_assisted_overlay(assisted=assisted_flag, in_view=result.in_view):
                    cv2.putText(
                        display,
                        ASSISTED_OVERLAY_TEXT,
                        (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 200, 255),
                        2,
                    )
                elif result.in_view:
                    cv2.putText(
                        display,
                        f"track RGB{target_rgb}",
                        (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 255),
                        2,
                    )
                if latest_color is not None and latest_color.pos_ned is not None:
                    t_s = (
                        float(latest_color.t_s)
                        if latest_color.t_s is not None
                        else 0.0
                    )
                    cv2.putText(
                        display,
                        format_ned_pos_line(t_s, latest_color.pos_ned),
                        (10, 48),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (255, 255, 255),
                        2,
                    )
                cv2.imshow(win, display)
        next_t += period
        sleep = next_t - time.time()
        if show_ui:
            delay_ms = max(1, int(sleep * 1000.0)) if sleep > 0 else 1
            if cv2.waitKey(delay_ms) & 0xFF == ord("q"):
                break
            if sleep <= 0:
                next_t = time.time()
        elif sleep > 0:
            time.sleep(sleep)
        else:
            next_t = time.time()

    if show_ui:
        cv2.destroyAllWindows()
    img_sub.close()
    color_sub.close()
    track_pub.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
