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

# View offsets are body-relative (FG follows the aircraft). Re-asserting ~26
# blocking telnet `set`s every render tick stalls capture for seconds because
# FG services props on the render thread. Same for re-running the X11 window
# hunt (xwininfo -id per class=fgfs child) every frame.
FG_VIEW_SYNC_PERIOD_S = 2.0
FG_GEO_REFRESH_PERIOD_S = 2.0

_TREE_SIZE_RE = re.compile(r"\s(\d+)x(\d+)[+-]")


def due_for_refresh(now_s: float, last_s: float, period_s: float) -> bool:
    """True on first call (last_s<=0) or once ``period_s`` has elapsed."""
    return last_s <= 0.0 or (now_s - last_s) >= period_s


def rects_overlap(
    ax: int,
    ay: int,
    aw: int,
    ah: int,
    bx: int,
    by: int,
    bw: int,
    bh: int,
) -> bool:
    """True if axis-aligned rectangles [a] and [b] share any interior pixels."""
    return not (
        ax + aw <= bx or bx + bw <= ax or ay + ah <= by or by + bh <= ay
    )


def virtual_screen_rect() -> dict[str, int]:
    """Virtual desktop bounds (mss monitor 0), else 1920×1080 at origin."""
    try:
        import mss

        with mss.mss() as sct:
            mon = sct.monitors[0]
            return {
                "x": int(mon.get("left", 0)),
                "y": int(mon.get("top", 0)),
                "width": int(mon["width"]),
                "height": int(mon["height"]),
            }
    except Exception:  # noqa: BLE001
        return {"x": 0, "y": 0, "width": 1920, "height": 1080}


def place_outside_rect(
    avoid: dict[str, int] | None,
    width: int,
    height: int,
    *,
    screen: dict[str, int] | None = None,
    gap: int = 24,
) -> tuple[int, int]:
    """Top-left for a WxH window that does not sit on ``avoid`` (FG).

    mss region-grabs the FG rectangle as composited on screen, so an OpenCV
    window stacked on FlightGear is captured inside balloon_camera.
    """
    screen = screen or {"x": 0, "y": 0, "width": 1920, "height": 1080}
    sx = int(screen["x"])
    sy = int(screen["y"])
    sw = int(screen["width"])
    sh = int(screen["height"])
    width = int(width)
    height = int(height)
    gap = int(gap)

    def clamp(x: int, y: int) -> tuple[int, int]:
        max_x = sx + max(sw - width, 0)
        max_y = sy + max(sh - height, 0)
        return (min(max(x, sx), max_x), min(max(y, sy), max_y))

    if avoid is None:
        return clamp(sx + sw - width - gap, sy + gap)

    ax = int(avoid["x"])
    ay = int(avoid["y"])
    aw = int(avoid["width"])
    ah = int(avoid["height"])
    candidates = (
        (ax + aw + gap, ay),
        (ax, ay + ah + gap),
        (ax - width - gap, ay),
        (ax, ay - height - gap),
    )
    for x, y in candidates:
        cx, cy = clamp(x, y)
        if not rects_overlap(cx, cy, width, height, ax, ay, aw, ah):
            return (cx, cy)
    return clamp(sx + sw - width - gap, sy + gap)


def fit_window_outside_rect(
    avoid: dict[str, int] | None,
    width: int,
    height: int,
    *,
    screen: dict[str, int] | None = None,
    gap: int = 24,
    min_width: int = 280,
    min_height: int = 200,
) -> tuple[int, int, int, int]:
    """Like ``place_outside_rect``, but shrink WxH to fit a leftover screen strip.

    A 640×480 HighGUI window cannot sit beside a nearly-fullscreen FlightGear
    window; the previous clamp still overlapped, so mss captured balloon_camera
    inside the FG grab.
    """
    width = int(width)
    height = int(height)
    x, y = place_outside_rect(avoid, width, height, screen=screen, gap=gap)
    if avoid is None:
        return (x, y, width, height)
    ax = int(avoid["x"])
    ay = int(avoid["y"])
    aw = int(avoid["width"])
    ah = int(avoid["height"])
    if not rects_overlap(x, y, width, height, ax, ay, aw, ah):
        return (x, y, width, height)

    screen = screen or {"x": 0, "y": 0, "width": 1920, "height": 1080}
    sx = int(screen["x"])
    sy = int(screen["y"])
    sw = int(screen["width"])
    sh = int(screen["height"])
    sr = sx + sw
    sb = sy + sh
    strips = (
        (ax + aw + gap, ay, sr - (ax + aw + gap), min(height, sh)),
        (ax, ay + ah + gap, min(width, sw), sb - (ay + ah + gap)),
        (sx, ay, ax - gap - sx, min(height, sh)),
        (ax, sy, min(width, sw), ay - gap - sy),
    )
    best: tuple[int, int, int, int] | None = None
    best_area = -1
    for x0, y0, w0, h0 in strips:
        w0 = min(int(w0), width)
        h0 = min(int(h0), height)
        if w0 < min_width or h0 < min_height:
            continue
        x1 = min(max(int(x0), sx), max(sr - w0, sx))
        y1 = min(max(int(y0), sy), max(sb - h0, sy))
        if rects_overlap(x1, y1, w0, h0, ax, ay, aw, ah):
            continue
        area = w0 * h0
        if area > best_area:
            best = (x1, y1, w0, h0)
            best_area = area
    if best is not None:
        return best
    return (x, y, width, height)


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
        if candidates:
            return _prefer_fg_geometry(candidates)

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
            if candidates:
                return _prefer_fg_geometry(candidates)

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
                size_m = _TREE_SIZE_RE.search(line)
                if size_m is not None:
                    tw, th = int(size_m.group(1)), int(size_m.group(2))
                    if not _is_plausible_fg_window(title, tw, th, line=line):
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
    tel.set_prop("/sim/rendering/draw-mask/clouds", 0)
    tel.set_prop("/sim/rendering/clouds3d-enable", 0)

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
    tel.set_prop("/sim/current-view/goal-field-of-view", fov)
    tel.set_prop("/sim/current-view/goal-fov", fov)
    tel.set_prop("/sim/view[0]/config/field-of-view", fov)
    tel.set_prop("/sim/view[0]/config/default-field-of-view", fov)
    tel.set_prop("/sim/view[0]/config/goal-field-of-view", fov)

    # Re-assert hide after view-number (some FG builds refresh draw defaults).
    if camera.fg_hide_aircraft:
        tel.set_prop("/sim/rendering/draw-mask/aircraft", 0)
    tel.set_prop("/sim/rendering/draw-mask/clouds", 0)
    tel.set_prop("/sim/rendering/clouds3d-enable", 0)

    tel.set_prop("/sim/hud/visibility[0]", 0)
    tel.set_prop("/sim/hud/enable", 0)


def _grab_bgr(sct: object, region: dict[str, int] | None) -> np.ndarray:
    """mss grab → BGR. ``region`` None means the virtual full-desktop monitor."""
    if region is None:
        mon = sct.monitors[0]  # type: ignore[attr-defined]
        shot = np.array(sct.grab(mon))  # type: ignore[attr-defined]
    else:
        shot = np.array(sct.grab(region))  # type: ignore[attr-defined]
    if shot.shape[2] == 4:
        return cv2.cvtColor(shot, cv2.COLOR_BGRA2BGR)
    return shot[:, :, :3].copy()


def capture_fg_frame(
    camera: CameraSpec,
    *,
    display: str | None = None,  # noqa: ARG001 — reserved for DISPLAY override
    window_pattern: str | None = None,
    geometry: dict[str, int] | None = None,
    sct: object | None = None,
    locate: bool = True,
) -> np.ndarray | None:
    """Grab FG *window* content; crop/resize to camera WxH.

    Primary: locate window by title/class regex (`camera.fg_window_pattern`,
    default ``FlightGear|fgfs``) via xdotool → wmctrl → xwininfo, then mss
    region grab (or xwd -id).

    Pass ``geometry`` (and a reused ``sct``) from the publisher loop so the
    X11 hunt and mss connect do not run every tick.

    Fallback: full X11 root (mss / xwd -root) if no matching window — last resort
    only; prefer installing xdotool or wmctrl on the host for reliable capture.
    """
    pattern = window_pattern or camera.fg_window_pattern or DEFAULT_FG_WINDOW_PATTERN
    if geometry is not None:
        geo = geometry
    elif locate:
        geo = find_fg_window_geometry(pattern)
    else:
        geo = None
    bgr: np.ndarray | None = None
    owned_sct = False
    grabber = sct
    if grabber is None:
        try:
            import mss

            grabber = mss.mss()
            owned_sct = True
        except Exception:  # noqa: BLE001
            grabber = None

    try:
        if geo is not None and grabber is not None:
            region = {
                "left": max(geo["x"], 0),
                "top": max(geo["y"], 0),
                "width": geo["width"],
                "height": geo["height"],
            }
            try:
                bgr = _grab_bgr(grabber, region)
            except Exception:  # noqa: BLE001
                bgr = None

        if bgr is None and grabber is not None:
            try:
                bgr = _grab_bgr(grabber, None)
                if geo is not None:
                    x0 = max(geo["x"], 0)
                    y0 = max(geo["y"], 0)
                    x1 = min(x0 + geo["width"], bgr.shape[1])
                    y1 = min(y0 + geo["height"], bgr.shape[0])
                    if x1 > x0 and y1 > y0:
                        bgr = bgr[y0:y1, x0:x1]
            except Exception:  # noqa: BLE001
                bgr = None
    finally:
        if owned_sct and grabber is not None:
            close = getattr(grabber, "close", None)
            if callable(close):
                close()

    if bgr is None:
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
    last_sync_s = 0.0
    last_geo_s = 0.0
    geo: dict[str, int] | None = None
    sct: object | None = None
    try:
        import mss

        sct = mss.mss()
    except Exception:  # noqa: BLE001
        sct = None
    print(
        f"FG capture publishing @ {setup.render_rate_hz} Hz → {setup.zmq.image} "
        f"(window /{pattern}/, mavlink UDP {udp_port})"
    )

    try:
        while True:
            while True:
                msg = master.recv_match(type="ATTITUDE", blocking=False)
                if msg is None:
                    break
                att = (float(msg.roll), float(msg.pitch), float(msg.yaw))
            poll_mavlink(master)
            now = time.time()
            if due_for_refresh(now, last_sync_s, FG_VIEW_SYNC_PERIOD_S):
                sync_camera_view(tel, setup.camera, att[0], att[1], att[2])
                last_sync_s = now
            if due_for_refresh(now, last_geo_s, FG_GEO_REFRESH_PERIOD_S):
                geo = find_fg_window_geometry(pattern)
                last_geo_s = now
            frame = capture_fg_frame(
                setup.camera,
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
