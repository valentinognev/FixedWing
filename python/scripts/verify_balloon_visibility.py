#!/usr/bin/env python3
"""One-shot: balloon 100 m ahead on camera boresight → synth (+ optional FG) PNGs."""

from __future__ import annotations

import argparse
import math
import re
import sys
import time
from pathlib import Path

import cv2
import numpy as np

_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.balloon_scene import (  # noqa: E402
    DEFAULT_ORIGIN_ALT_M,
    DEFAULT_ORIGIN_LAT_DEG,
    DEFAULT_ORIGIN_LON_DEG,
    FgTelnet,
    geodetic_to_ned,
    ned_to_geodetic,
    spawn_balloons_fg,
)
from fw_sitl.balloon_tracker import track_balloon  # noqa: E402
from fw_sitl.camera_model import CameraModel  # noqa: E402
from fw_sitl.fg_camera import capture_fg_frame, sync_camera_view  # noqa: E402
from fw_sitl.flight_setup import BalloonSpec, load_flight_setup  # noqa: E402
from fw_sitl.synthetic_camera import render_frame  # noqa: E402

OUT_DIR = _PYTHON_ROOT / "logs" / "e2e"
RANGE_M = 100.0
DIAMETER_M = 10.0
COLOR = (255, 0, 0)


def _getf(tel: FgTelnet, path: str) -> float:
    s = tel.command(f"get {path}")
    m = re.search(r"=\s*'([^']*)'", s)
    if not m or m.group(1) in ("", "true", "false"):
        raise RuntimeError(f"cannot parse float from {path}: {s!r}")
    return float(m.group(1))


def verify_synth(camera_spec, out: Path) -> tuple[bool, object]:
    pos = (0.0, 0.0, 0.0)
    balloon = BalloonSpec(ned=(RANGE_M, 0.0, 0.0), color=COLOR, diameter_m=DIAMETER_M)
    img = render_frame(
        pos, 0.0, 0.0, 0.0, (balloon,), camera_spec, rebase_z_to_aircraft=False
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    track = track_balloon(img, COLOR, CameraModel.from_spec(camera_spec))
    ok = bool(track.in_view and track.area_px >= 200.0)
    print(
        f"synth → {out} in_view={track.in_view} area_px={track.area_px:.1f} "
        f"centroid={track.centroid_uv} ok={ok}"
    )
    return ok, track


def _body_x_ned(roll_rad: float, pitch_rad: float, yaw_rad: float) -> tuple[float, float, float]:
    """Unit body +X (forward) expressed in NED (aerospace ZYX yaw-pitch-roll)."""
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    # Column 0 of R_ned_from_body
    return (cp * cy, cp * sy, -sp)


def verify_fg(camera_spec, out: Path) -> tuple[bool, object | None]:
    tel = FgTelnet(timeout=4.0)
    tel.connect(retries=20, delay_s=0.5)
    alat = _getf(tel, "/position/latitude-deg")
    alon = _getf(tel, "/position/longitude-deg")
    aalt = _getf(tel, "/position/altitude-ft") * 0.3048
    # Prefer Euler from orientation props (deg).
    hdg = _getf(tel, "/orientation/heading-deg")
    pitch = _getf(tel, "/orientation/pitch-deg")
    roll = _getf(tel, "/orientation/roll-deg")
    yaw = math.radians(hdg)
    # FG heading is compass; NED yaw is same convention (0=north).

    n0, e0, d0 = geodetic_to_ned(
        alat,
        alon,
        aalt,
        DEFAULT_ORIGIN_LAT_DEG,
        DEFAULT_ORIGIN_LON_DEG,
        DEFAULT_ORIGIN_ALT_M,
    )
    # Place on optical / body +X so banked failsafe orbits still center the disk.
    bx, by, bz = _body_x_ned(math.radians(roll), math.radians(pitch), yaw)
    balloon_ned = (
        n0 + RANGE_M * bx,
        e0 + RANGE_M * by,
        d0 + RANGE_M * bz,
    )
    balloon = BalloonSpec(ned=balloon_ned, color=COLOR, diameter_m=DIAMETER_M)
    lat, lon, alt = ned_to_geodetic(
        *balloon_ned,
        DEFAULT_ORIGIN_LAT_DEG,
        DEFAULT_ORIGIN_LON_DEG,
        DEFAULT_ORIGIN_ALT_M,
    )
    print(
        f"FG AC lat={alat:.6f} lon={alon:.6f} alt={aalt:.1f} "
        f"rpy=({roll:.1f},{pitch:.1f},{hdg:.1f}) → "
        f"balloon lat={lat:.6f} lon={lon:.6f} alt={alt:.1f}"
    )
    tel.close()

    spawn_balloons_fg(
        (balloon,),
        connect_retries=10,
        connect_delay_s=0.5,
    )

    tel = FgTelnet(timeout=4.0)
    tel.connect(retries=10, delay_s=0.3)
    sync_camera_view(tel, camera_spec, 0.0, 0.0, 0.0)
    # Allow model load + a couple of frames.
    time.sleep(2.0)
    sync_camera_view(tel, camera_spec, 0.0, 0.0, 0.0)
    tel.close()

    frame = None
    for _ in range(8):
        frame = capture_fg_frame(camera_spec)
        if frame is not None:
            break
        time.sleep(0.25)
    if frame is None:
        print("FG capture failed (no window frame)")
        return False, None

    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), frame)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    track = track_balloon(rgb, COLOR, CameraModel.from_spec(camera_spec), h_tol=15)
    ok = bool(track.in_view and track.area_px >= 80.0)
    print(
        f"fg → {out} in_view={track.in_view} area_px={track.area_px:.1f} "
        f"centroid={track.centroid_uv} ok={ok}"
    )
    return ok, track


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--setup",
        type=Path,
        default=_PYTHON_ROOT / "flightSetup.json",
    )
    parser.add_argument(
        "--fg",
        action="store_true",
        help="Also place+capture FG (requires running --viz sim, patch V6+)",
    )
    parser.add_argument("--out-dir", type=Path, default=OUT_DIR)
    args = parser.parse_args()
    setup = load_flight_setup(args.setup)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    synth_ok, _ = verify_synth(setup.camera, args.out_dir / "synth_balloon_100m.png")
    fg_ok = True
    if args.fg:
        fg_ok, _ = verify_fg(setup.camera, args.out_dir / "fg_balloon_100m.png")

    if not synth_ok:
        return 1
    if args.fg and not fg_ok:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
