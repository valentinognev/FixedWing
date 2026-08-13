"""FlightGear view sync + screenshot capture for balloon race viz."""
from __future__ import annotations

import re
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
from pymavlink import mavutil

from fw_sitl.balloon_scene import FgTelnet
from fw_sitl.flight_setup import CameraSpec, FlightSetup, DEFAULT_FG_WINDOW_PATTERN
from fw_sitl.mavlink_io import connect, poll_mavlink, request_local_position
from fw_sitl.zmq_bus import ImagePublisher


def window_matches_pattern(title: str, pattern: str) -> bool:
    """True if window title/class matches camera.fg_window_pattern (regex)."""
    if not pattern:
        return False
    try:
        return re.search(pattern, title or "", re.IGNORECASE) is not None
    except re.error:
        return pattern.lower() in (title or "").lower()


def _geometry_from_shell_vars(text: str) -> dict[str, int] | None:
    vals: dict[str, int] = {}
    for line in text.splitlines():
        if "=" not in line:
            continue
        key, _, raw = line.partition("=")
        key = key.strip().upper()
        if key in {"X", "Y", "WIDTH", "HEIGHT"}:
            try:
                vals[key.lower()] = int(raw.strip())
            except ValueError:
                return None
    if {"x", "y", "width", "height"} <= vals.keys() and vals["width"] > 0 and vals["height"] > 0:
        return vals
    return None


# Ignore helper/offscreen windows that match "fgfs" in the title (e.g. Qt
# selection-owner stubs at 3×3) — grabbing those yields a blank camera feed.
_MIN_FG_WINDOW_W = 64
_MIN_FG_WINDOW_H = 64


def _is_plausible_fg_window(
    title: str, width: int, height: int, *, line: str = ""
) -> bool:
    if width < _MIN_FG_WINDOW_W or height < _MIN_FG_WINDOW_H:
        return False
    blob = f"{title} {line}".lower()
    if "selection owner" in blob:
        return False
    return True


def _prefer_fg_geometry(
    candidates: list[tuple[dict[str, int], str]],
) -> dict[str, int] | None:
    """Pick the real OSG view: prefer title FlightGear, then largest area."""
    if not candidates:
        return None

    def score(item: tuple[dict[str, int], str]) -> tuple[int, int]:
        geo, title = item
        area = int(geo["width"]) * int(geo["height"])
        prefer = 1 if re.search(r"flightgear", title or "", re.I) else 0
        return (prefer, area)

    best_geo, _ = max(candidates, key=score)
    return best_geo


def find_fg_window_geometry(pattern: str) -> dict[str, int] | None:
    """Locate FG window bounds via xdotool / wmctrl / xwininfo (no heavy deps).

    Returns dict with x, y, width, height in root coordinates, or None.
    Fallback chain documented in capture_fg_frame.

    Important: pattern ``FlightGear|fgfs`` also matches tiny Qt helper windows
    titled like ``Qt Selection Owner for fgfs`` (3×3). Those are skipped;
    the largest plausible match (preferring title FlightGear) wins.
    """
    candidates: list[tuple[dict[str, int], str]] = []

    # 1) xdotool: search by name then class (pattern is already a regex)
    if _have("xdotool"):
        wids: list[str] = []
        for flag in ("--name", "--class"):
            try:
                found = subprocess.run(
                    ["xdotool", "search", "--onlyvisible", flag, pattern],
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=2.0,
                )
                wids.extend(found.stdout.split())
            except (OSError, subprocess.TimeoutExpired):
                break
        for wid in dict.fromkeys(wids):
            try:
                geo = subprocess.run(
                    ["xdotool", "getwindowgeometry", "--shell", wid],
                    check=True,
                    capture_output=True,
                    text=True,
                    timeout=1.0,
                )
                name = subprocess.run(
                    ["xdotool", "getwindowname", wid],
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=1.0,
                )
            except (OSError, subprocess.TimeoutExpired, subprocess.CalledProcessError):
                continue
            parsed = _geometry_from_shell_vars(geo.stdout)
            title = (name.stdout or "").strip()
            if parsed and _is_plausible_fg_window(
                title, parsed["width"], parsed["height"]
            ):
                candidates.append((parsed, title))

    # 2) wmctrl -l -G: id desktop x y w h host title
    if _have("wmctrl"):
        try:
            listing = subprocess.run(
                ["wmctrl", "-l", "-G"],
                check=True,
                capture_output=True,
                text=True,
                timeout=2.0,
            )
        except (OSError, subprocess.TimeoutExpired, subprocess.CalledProcessError):
            listing = None
        if listing is not None:
            for line in listing.stdout.splitlines():
                parts = line.split(None, 7)
                if len(parts) < 8:
                    continue
                title = parts[7]
                if not window_matches_pattern(title, pattern):
                    continue
                try:
                    geo = {
                        "x": int(parts[2]),
                        "y": int(parts[3]),
                        "width": int(parts[4]),
                        "height": int(parts[5]),
                    }
                except ValueError:
                    continue
                if _is_plausible_fg_window(title, geo["width"], geo["height"]):
                    candidates.append((geo, title))

    # 3) xwininfo -tree -root, then -id for *absolute* geometry
    #    (tree lines often show relative +0+0, not screen coords).
    if _have("xwininfo"):
        try:
            tree = subprocess.run(
                ["xwininfo", "-root", "-tree"],
                check=True,
                capture_output=True,
                text=True,
                timeout=3.0,
            )
        except (OSError, subprocess.TimeoutExpired, subprocess.CalledProcessError):
            tree = None
        if tree is not None:
            line_re = re.compile(
                r'^\s*(0x[0-9a-fA-F]+)\s+"([^"]*)"',
            )
            for line in tree.stdout.splitlines():
                m = line_re.search(line)
                if not m:
                    continue
                wid, title = m.group(1), m.group(2)
                if not window_matches_pattern(title, pattern) and not window_matches_pattern(
                    line, pattern
                ):
                    continue
                try:
                    info = subprocess.run(
                        ["xwininfo", "-id", wid],
                        check=True,
                        capture_output=True,
                        text=True,
                        timeout=1.0,
                    ).stdout
                except (OSError, subprocess.TimeoutExpired, subprocess.CalledProcessError):
                    continue
                abs_x = re.search(r"Absolute upper-left X:\s*(-?\d+)", info)
                abs_y = re.search(r"Absolute upper-left Y:\s*(-?\d+)", info)
                width = re.search(r"Width:\s*(\d+)", info)
                height = re.search(r"Height:\s*(\d+)", info)
                if not (abs_x and abs_y and width and height):
                    continue
                w, h = int(width.group(1)), int(height.group(1))
                if not _is_plausible_fg_window(title, w, h, line=line):
                    continue
                candidates.append(
                    (
                        {
                            "x": int(abs_x.group(1)),
                            "y": int(abs_y.group(1)),
                            "width": w,
                            "height": h,
                        },
                        title,
                    )
                )

    return _prefer_fg_geometry(candidates)


def _have(cmd: str) -> bool:
    try:
        return (
            subprocess.run(
                ["which", cmd],
                check=False,
                capture_output=True,
                timeout=1.0,
            ).returncode
            == 0
        )
    except (OSError, subprocess.TimeoutExpired):
        return False


def _resize_to_camera(bgr: np.ndarray, camera: CameraSpec) -> np.ndarray:
    h, w = bgr.shape[:2]
    th, tw = camera.height_px, camera.width_px
    target_aspect = tw / th
    src_aspect = w / max(h, 1)
    if src_aspect > target_aspect:
        new_w = max(int(h * target_aspect), 1)
        x0 = (w - new_w) // 2
        crop = bgr[:, x0 : x0 + new_w]
    else:
        new_h = max(int(w / target_aspect), 1)
        y0 = (h - new_h) // 2
        crop = bgr[y0 : y0 + new_h, :]
    return cv2.resize(crop, (tw, th), interpolation=cv2.INTER_AREA)


def sync_camera_view(
    tel: FgTelnet,
    camera: CameraSpec,
    roll: float,
    pitch: float,
    yaw: float,
) -> None:
    """Align FG view with fictional body-mounted camera (fuselage-free).

    Rascal cockpit view defaults to z-offset≈+0.9 m (aft of model origin) so the
    canopy/struts fill the frame. For balloon-race capture we:

    1) force Cockpit View (view 0, lookfrom, from-model),
    2) push eyepoint forward on body +X (FG view −Z) by ``fg_eye_forward_m``,
       writing both live and ``goal-*`` props so FG easing cannot snap back,
    3) hide ownship via ``/sim/rendering/draw-mask/aircraft=0`` (required —
       forward offset alone still leaves grey structure in-frame),
    4) match mount az/el + HFOV.

    ``roll``/``pitch``/``yaw`` kept for API stability; offsets are body-relative.
    """
    del roll, pitch, yaw  # body-relative mount; FG follows aircraft attitude

    # Hide airframe FIRST — draw-mask needs numeric 0/1 (string "false" is flaky).
    if camera.fg_hide_aircraft:
        tel.set_prop("/sim/rendering/draw-mask/aircraft", 0)

    # Cockpit / pilot lookfrom (aircraft-relative).
    tel.set_prop("/sim/current-view/view-number", 0)

    # FG aircraft view axes: +X right, +Y up, +Z aft → negative Z is forward.
    # Stock Rascal cockpit is ~+0.9 m; we want well ahead of the nose/canopy.
    z_eye = -abs(float(camera.fg_eye_forward_m))
    heading = float(camera.azimuth_deg)
    elev = float(camera.elevation_deg)
    fov = f"{float(camera.hfov_deg):.1f}"

    def _xyz(base: str) -> None:
        tel.set_prop(f"{base}/x-offset-m", 0)
        tel.set_prop(f"{base}/y-offset-m", 0)
        tel.set_prop(f"{base}/z-offset-m", f"{z_eye:.3f}")

    _xyz("/sim/current-view")
    # goal-* stops the view manager from interpolating back to cockpit defaults.
    tel.set_prop("/sim/current-view/goal-x-offset-m", 0)
    tel.set_prop("/sim/current-view/goal-y-offset-m", 0)
    tel.set_prop("/sim/current-view/goal-z-offset-m", f"{z_eye:.3f}")
    _xyz("/sim/view[0]/config")
    _xyz("/sim/view[0]")

    tel.set_prop("/sim/current-view/heading-offset-deg", f"{heading:.2f}")
    tel.set_prop("/sim/current-view/pitch-offset-deg", f"{elev:.2f}")
    tel.set_prop("/sim/current-view/roll-offset-deg", 0)
    tel.set_prop("/sim/current-view/goal-heading-offset-deg", f"{heading:.2f}")
    tel.set_prop("/sim/current-view/goal-pitch-offset-deg", f"{elev:.2f}")
    tel.set_prop("/sim/view[0]/heading-offset-deg", f"{heading:.2f}")
    tel.set_prop("/sim/view[0]/pitch-offset-deg", f"{elev:.2f}")

    tel.set_prop("/sim/current-view/field-of-view", fov)
    tel.set_prop("/sim/view[0]/config/field-of-view", fov)

    # Re-assert hide after view-number (some FG builds refresh draw defaults).
    if camera.fg_hide_aircraft:
        tel.set_prop("/sim/rendering/draw-mask/aircraft", 0)

    tel.set_prop("/sim/hud/visibility[0]", 0)
    tel.set_prop("/sim/hud/enable", 0)


def capture_fg_frame(
    camera: CameraSpec,
    *,
    display: str | None = None,  # noqa: ARG001 — reserved for DISPLAY override
    window_pattern: str | None = None,
) -> np.ndarray | None:
    """Grab FG *window* content; crop/resize to camera WxH.

    Primary: locate window by title/class regex (`camera.fg_window_pattern`,
    default ``FlightGear|fgfs``) via xdotool → wmctrl → xwininfo, then mss
    region grab (or xwd -id).

    Fallback: full X11 root (mss / xwd -root) if no matching window — last resort
    only; prefer installing xdotool or wmctrl on the host for reliable capture.
    """
    pattern = window_pattern or camera.fg_window_pattern or DEFAULT_FG_WINDOW_PATTERN
    geo = find_fg_window_geometry(pattern)
    bgr: np.ndarray | None = None

    if geo is not None:
        region = {
            "left": max(geo["x"], 0),
            "top": max(geo["y"], 0),
            "width": geo["width"],
            "height": geo["height"],
        }
        try:
            import mss

            with mss.mss() as sct:
                shot = np.array(sct.grab(region))
                bgr = cv2.cvtColor(shot, cv2.COLOR_BGRA2BGR)
        except Exception:  # noqa: BLE001
            if _have("xwd") and _have("convert"):
                # xwd cannot easily crop by geometry without window id; try root+numpy crop
                bgr = None
            else:
                bgr = None

    if bgr is None:
        # Fallback: full root, then center-crop (documented last resort)
        try:
            import mss

            with mss.mss() as sct:
                mon = sct.monitors[0]
                shot = np.array(sct.grab(mon))
                bgr = cv2.cvtColor(shot, cv2.COLOR_BGRA2BGR)
                if geo is not None:
                    x0 = max(geo["x"], 0)
                    y0 = max(geo["y"], 0)
                    x1 = min(x0 + geo["width"], bgr.shape[1])
                    y1 = min(y0 + geo["height"], bgr.shape[0])
                    if x1 > x0 and y1 > y0:
                        bgr = bgr[y0:y1, x0:x1]
        except Exception:  # noqa: BLE001
            tmp = Path("/tmp/fg_capture.xwd")
            try:
                subprocess.run(
                    ["xwd", "-root", "-out", str(tmp)],
                    check=True,
                    timeout=3.0,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                png = Path("/tmp/fg_capture.png")
                subprocess.run(
                    ["convert", str(tmp), str(png)],
                    check=True,
                    timeout=3.0,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                bgr = cv2.imread(str(png))
                if bgr is None:
                    return None
            except Exception:  # noqa: BLE001
                return None

    return _resize_to_camera(bgr, camera)


def run_fg_publisher(
    setup: FlightSetup,
    udp_port: int = 14541,
    telnet_host: str = "127.0.0.1",
    telnet_port: int = 5501,
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

    tel = FgTelnet(host=telnet_host, port=telnet_port)
    tel.connect()
    pub = ImagePublisher(setup.zmq.image)
    period = 1.0 / setup.render_rate_hz
    att = (0.0, 0.0, 0.0)
    pattern = setup.camera.fg_window_pattern
    next_t = time.time()
    print(
        f"FG capture publishing @ {setup.render_rate_hz} Hz → {setup.zmq.image} "
        f"(window /{pattern}/, mavlink UDP {udp_port})"
    )

    while True:
        while True:
            msg = master.recv_match(type="ATTITUDE", blocking=False)
            if msg is None:
                break
            att = (float(msg.roll), float(msg.pitch), float(msg.yaw))
        poll_mavlink(master)
        sync_camera_view(tel, setup.camera, att[0], att[1], att[2])
        frame = capture_fg_frame(setup.camera, window_pattern=pattern)
        if frame is not None:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pub.publish(rgb)
        next_t += period
        sleep = next_t - time.time()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_t = time.time()
