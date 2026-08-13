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

from fw_sitl.balloon_tracker import track_balloon
from fw_sitl.camera_model import CameraModel
from fw_sitl.flight_setup import load_flight_setup
from fw_sitl.race_guidance import ASSISTED_OVERLAY_TEXT, show_assisted_overlay
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
    if show_ui:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
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
        if latest is not None:
            frame_rgb = latest.as_numpy()
            result = track_balloon(frame_rgb, target_rgb, camera)
            track_pub.publish_track(
                result.in_view,
                result.dir_cam,
                result.centroid_uv,
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
                cv2.imshow(win, display)
        if show_ui and (cv2.waitKey(1) & 0xFF == ord("q")):
            break
        next_t += period
        sleep = next_t - time.time()
        if sleep > 0:
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
