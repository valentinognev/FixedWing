"""Pinhole camera model: FOV, body mount, pixel ↔ camera-frame LOS.

Camera optical frame (OpenCV-style)
-----------------------------------
- **+Z** forward along the optical boresight
- **+X** right
- **+Y** down

Body frame is FRD (+X forward, +Y right, +Z down).

Mount
-----
``azimuth_deg`` / ``elevation_deg`` are relative to body FRD.

- **Azimuth** positive toward body +Y (right), rotation about body +Z.
- **Elevation** positive upward (toward body −Z when level), rotation about body +Y
  after azimuth.

At **azimuth = 0, elevation = 0** the camera is aligned with body +X:

- camera +Z = body +X (boresight forward)
- camera +X = body +Y (right)
- camera +Y = body +Z (down)

Pixel projection uses a pinhole with principal point at the image center and
focal lengths from horizontal/vertical FOV.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from fw_sitl.flight_setup import CameraSpec


def _normalize(x: float, y: float, z: float) -> tuple[float, float, float]:
    n = math.sqrt(x * x + y * y + z * z)
    if n <= 0.0:
        raise ValueError("zero-length vector")
    return (x / n, y / n, z / n)


@dataclass(frozen=True)
class CameraModel:
    """Intrinsics + body-relative mount for balloon-race guidance."""

    hfov_deg: float
    vfov_deg: float
    width_px: int
    height_px: int
    azimuth_deg: float = 0.0
    elevation_deg: float = 0.0

    @classmethod
    def from_spec(cls, spec: CameraSpec) -> CameraModel:
        return cls(
            hfov_deg=float(spec.hfov_deg),
            vfov_deg=float(spec.vfov_deg),
            width_px=int(spec.width_px),
            height_px=int(spec.height_px),
            azimuth_deg=float(spec.azimuth_deg),
            elevation_deg=float(spec.elevation_deg),
        )

    @property
    def cx(self) -> float:
        return 0.5 * float(self.width_px)

    @property
    def cy(self) -> float:
        return 0.5 * float(self.height_px)

    @property
    def fx(self) -> float:
        """Horizontal focal length in pixels from hfov."""
        return (0.5 * float(self.width_px)) / math.tan(
            math.radians(self.hfov_deg) * 0.5
        )

    @property
    def fy(self) -> float:
        """Vertical focal length in pixels from vfov."""
        return (0.5 * float(self.height_px)) / math.tan(
            math.radians(self.vfov_deg) * 0.5
        )

    def pixel_to_dir_cam(self, u: float, v: float) -> tuple[float, float, float]:
        """Pixel (u right, v down) → unit LOS in camera frame (+Z forward)."""
        x = (float(u) - self.cx) / self.fx
        y = (float(v) - self.cy) / self.fy
        return _normalize(x, y, 1.0)

    def dir_cam_to_pixel(
        self, dir_cam: tuple[float, float, float]
    ) -> tuple[float, float] | None:
        """Unit (or any) camera-frame LOS → pixel. None if behind camera (z<=0)."""
        x, y, z = float(dir_cam[0]), float(dir_cam[1]), float(dir_cam[2])
        if z <= 0.0:
            return None
        u = self.cx + self.fx * (x / z)
        v = self.cy + self.fy * (y / z)
        return (u, v)

    def rotation_body_from_cam(self) -> tuple[tuple[float, float, float], ...]:
        """3x3 row-major R such that v_body = R @ v_cam.

        Base 0/0 map: cam X→body Y, cam Y→body Z, cam Z→body X, then mount
        R_z(az) @ R_y(el).
        """
        az = math.radians(self.azimuth_deg)
        el = math.radians(self.elevation_deg)
        caz, saz = math.cos(az), math.sin(az)
        cel, sel = math.cos(el), math.sin(el)

        # R0 columns = camera axes in body at 0/0:
        #   cam_x → (0,1,0), cam_y → (0,0,1), cam_z → (1,0,0)
        r0 = (
            (0.0, 0.0, 1.0),
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
        )

        # R_y(el): positive elevation pitches boresight toward body −Z.
        ry = (
            (cel, 0.0, sel),
            (0.0, 1.0, 0.0),
            (-sel, 0.0, cel),
        )
        # R_z(az): positive azimuth toward body +Y.
        rz = (
            (caz, -saz, 0.0),
            (saz, caz, 0.0),
            (0.0, 0.0, 1.0),
        )

        # R_mount = Rz @ Ry
        rm = _matmul3(rz, ry)
        return _matmul3(rm, r0)

    def dir_cam_to_body(
        self, dir_cam: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        """Rotate camera-frame unit LOS into body FRD."""
        r = self.rotation_body_from_cam()
        x, y, z = float(dir_cam[0]), float(dir_cam[1]), float(dir_cam[2])
        bx = r[0][0] * x + r[0][1] * y + r[0][2] * z
        by = r[1][0] * x + r[1][1] * y + r[1][2] * z
        bz = r[2][0] * x + r[2][1] * y + r[2][2] * z
        return _normalize(bx, by, bz)

    def dir_body_to_cam(
        self, dir_body: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        """Rotate body-FRD direction into camera frame (R^T)."""
        r = self.rotation_body_from_cam()
        x, y, z = float(dir_body[0]), float(dir_body[1]), float(dir_body[2])
        # R^T
        cx = r[0][0] * x + r[1][0] * y + r[2][0] * z
        cy = r[0][1] * x + r[1][1] * y + r[2][1] * z
        cz = r[0][2] * x + r[1][2] * y + r[2][2] * z
        return _normalize(cx, cy, cz)


def _matmul3(
    a: tuple[tuple[float, float, float], ...],
    b: tuple[tuple[float, float, float], ...],
) -> tuple[tuple[float, float, float], ...]:
    rows = []
    for i in range(3):
        rows.append(
            tuple(
                a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
                for j in range(3)
            )
        )
    return (rows[0], rows[1], rows[2])  # type: ignore[return-value]


def _transpose3(
    r: tuple[tuple[float, float, float], ...],
) -> tuple[tuple[float, float, float], ...]:
    return (
        (r[0][0], r[1][0], r[2][0]),
        (r[0][1], r[1][1], r[2][1]),
        (r[0][2], r[1][2], r[2][2]),
    )


def _matvec3(
    r: tuple[tuple[float, float, float], ...],
    v: tuple[float, float, float],
) -> tuple[float, float, float]:
    x, y, z = v
    return (
        r[0][0] * x + r[0][1] * y + r[0][2] * z,
        r[1][0] * x + r[1][1] * y + r[1][2] * z,
        r[2][0] * x + r[2][1] * y + r[2][2] * z,
    )


def ned_to_body_rotation(
    roll: float, pitch: float, yaw: float
) -> tuple[tuple[float, float, float], ...]:
    """3x3 row-major R: v_body = R @ v_ned (PX4 ATTITUDE Euler convention)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def vec_ned_to_cam(
    vec_ned: tuple[float, float, float],
    camera: CameraModel,
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float]:
    """NED-frame vector → camera optical frame (not normalized)."""
    rel_body = _matvec3(ned_to_body_rotation(roll, pitch, yaw), vec_ned)
    return _matvec3(_transpose3(camera.rotation_body_from_cam()), rel_body)


def project_ned_offset_to_pixel(
    offset_ned: tuple[float, float, float],
    camera: CameraModel,
    roll: float,
    pitch: float,
    yaw: float,
    *,
    min_cam_z: float = 0.5,
) -> tuple[float, float] | None:
    """Project a NED offset onto the image; None if behind the camera."""
    rel_cam = vec_ned_to_cam(offset_ned, camera, roll, pitch, yaw)
    if rel_cam[2] <= min_cam_z:
        return None
    return camera.dir_cam_to_pixel(rel_cam)


def dir_cam_to_ned(
    dir_cam: tuple[float, float, float],
    camera: CameraModel,
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float]:
    """Unit LOS in camera frame → unit direction in NED."""
    dir_body = camera.dir_cam_to_body(dir_cam)
    rb = ned_to_body_rotation(roll, pitch, yaw)
    return _normalize(*_matvec3(_transpose3(rb), dir_body))


def intrinsics_from_spec(spec: CameraSpec) -> CameraModel:
    """Alias for ``CameraModel.from_spec`` (plan / legacy name)."""
    return CameraModel.from_spec(spec)


def pixel_to_dir_cam(
    u: float,
    v: float,
    intr: CameraModel,
) -> tuple[float, float, float]:
    return intr.pixel_to_dir_cam(u, v)


def dir_cam_to_pixel(
    dir_cam: tuple[float, float, float],
    intr: CameraModel,
) -> tuple[float, float]:
    px = intr.dir_cam_to_pixel(dir_cam)
    if px is None:
        raise ValueError("direction behind camera (z <= 0)")
    return px


def ned_to_body_rotation(roll: float, pitch: float, yaw: float) -> tuple[tuple[float, float, float], ...]:
    """Aviation ZYX: NED → body FRD (row-major 3x3)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))
    ry = ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp))
    rx = ((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr))
    return _matmul3(rx, _matmul3(ry, rz))


def body_to_ned_rotation(roll: float, pitch: float, yaw: float) -> tuple[tuple[float, float, float], ...]:
    r = ned_to_body_rotation(roll, pitch, yaw)
    return (
        (r[0][0], r[1][0], r[2][0]),
        (r[0][1], r[1][1], r[2][1]),
        (r[0][2], r[1][2], r[2][2]),
    )


def camera_to_body_rotation(azimuth_deg: float, elevation_deg: float) -> tuple[tuple[float, float, float], ...]:
    return CameraModel(
        hfov_deg=90.0,
        vfov_deg=70.0,
        width_px=640,
        height_px=480,
        azimuth_deg=azimuth_deg,
        elevation_deg=elevation_deg,
    ).rotation_body_from_cam()


def dir_body_to_ned(
    dir_body: tuple[float, float, float],
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float]:
    r = body_to_ned_rotation(roll, pitch, yaw)
    x, y, z = float(dir_body[0]), float(dir_body[1]), float(dir_body[2])
    nx = r[0][0] * x + r[0][1] * y + r[0][2] * z
    ny = r[1][0] * x + r[1][1] * y + r[1][2] * z
    nz = r[2][0] * x + r[2][1] * y + r[2][2] * z
    return _normalize(nx, ny, nz)
