"""Color balloon tracker: threshold → largest blob → camera-frame LOS."""
from __future__ import annotations

from dataclasses import dataclass

import math

import cv2
import numpy as np

from fw_sitl.camera_model import CameraModel

# FG Zurich roofs are often the largest red HSV blob. Headless synth disks sit
# on the geometric projection; only trust a centroid within this many pixels.
TRACK_GEOM_MAX_PX = 80.0


@dataclass(frozen=True)
class BalloonTrack:
    """Tracker output for one frame."""

    in_view: bool
    dir_cam: tuple[float, float, float] | None
    centroid_uv: tuple[float, float] | None = None
    area_px: float = 0.0


def _rgb_to_hsv_target(rgb: tuple[int, int, int]) -> np.ndarray:
    """Convert a single RGB triple to OpenCV HSV (H,S,V uint8)."""
    bgr = np.uint8([[[int(rgb[2]), int(rgb[1]), int(rgb[0])]]])
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[0, 0]


def color_mask_hsv(
    image_rgb: np.ndarray,
    target_rgb: tuple[int, int, int],
    *,
    h_tol: int = 10,
    s_min: int = 50,
    v_min: int = 50,
) -> np.ndarray:
    """Binary mask of pixels near ``target_rgb`` in HSV.

    Hue wraps at 180 (OpenCV). Saturation/value floors avoid gray background.
    """
    if image_rgb.ndim != 3 or image_rgb.shape[2] != 3:
        raise ValueError("image_rgb must be HxWx3")
    bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = [int(x) for x in _rgb_to_hsv_target(target_rgb)]

    # Near-gray / low-chroma targets: fall back to RGB Euclidean gate.
    if s < 40:
        diff = np.abs(image_rgb.astype(np.int16) - np.array(target_rgb, dtype=np.int16))
        return (np.max(diff, axis=2) <= 40).astype(np.uint8) * 255

    lo_s = max(s_min, s - 80)
    lo_v = max(v_min, v - 80)

    lo_h = h - h_tol
    hi_h = h + h_tol
    if lo_h < 0 or hi_h > 179:
        h1 = lo_h % 180
        h2 = hi_h % 180
        m1 = cv2.inRange(hsv, (h1, lo_s, lo_v), (179, 255, 255))
        m2 = cv2.inRange(hsv, (0, lo_s, lo_v), (h2, 255, 255))
        return cv2.bitwise_or(m1, m2)
    return cv2.inRange(hsv, (lo_h, lo_s, lo_v), (hi_h, 255, 255))


def largest_blob_centroid(
    mask: np.ndarray, *, min_area_px: float = 16.0
) -> tuple[tuple[float, float], float] | None:
    """Return ((u, v), area) of the largest connected component, or None."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best: tuple[float, float] | None = None
    best_area = 0.0
    for cnt in contours:
        area = float(cv2.contourArea(cnt))
        if area < min_area_px or area <= best_area:
            continue
        moments = cv2.moments(cnt)
        if moments["m00"] <= 0.0:
            continue
        u = float(moments["m10"] / moments["m00"])
        v = float(moments["m01"] / moments["m00"])
        best = (u, v)
        best_area = area
    if best is None:
        return None
    return best, best_area


def track_centroid_near_expected(
    centroid_uv: tuple[float, float] | None,
    expected_uv: tuple[float, float] | None,
    max_px: float = TRACK_GEOM_MAX_PX,
    *,
    width_px: float | int | None = None,
    height_px: float | int | None = None,
) -> bool:
    """True when HSV centroid matches the geometric balloon projection.

    Missing centroid/projection, or a projection outside the image, means
    the blob is scenery (or the balloon is behind the camera) — reject.
    """
    if centroid_uv is None or expected_uv is None:
        return False
    u, v = float(expected_uv[0]), float(expected_uv[1])
    if width_px is not None and not (0.0 <= u < float(width_px)):
        return False
    if height_px is not None and not (0.0 <= v < float(height_px)):
        return False
    du = float(centroid_uv[0]) - u
    dv = float(centroid_uv[1]) - v
    return math.hypot(du, dv) <= float(max_px)


def track_balloon(
    image_rgb: np.ndarray,
    target_rgb: tuple[int, int, int],
    camera: CameraModel,
    *,
    min_area_px: float = 16.0,
    h_tol: int = 10,
) -> BalloonTrack:
    """Threshold commanded RGB → largest blob → camera-frame unit LOS.

    Returns ``in_view=False`` and ``dir_cam=None`` when no blob is found.
    """
    mask = color_mask_hsv(image_rgb, target_rgb, h_tol=h_tol)
    found = largest_blob_centroid(mask, min_area_px=min_area_px)
    if found is None:
        return BalloonTrack(in_view=False, dir_cam=None, centroid_uv=None, area_px=0.0)
    (u, v), area = found
    return BalloonTrack(
        in_view=True,
        dir_cam=camera.pixel_to_dir_cam(u, v),
        centroid_uv=(u, v),
        area_px=area,
    )
