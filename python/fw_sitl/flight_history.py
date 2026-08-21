"""Record LOCAL_POSITION_NED + ATTITUDE during a flight and plot history."""

from __future__ import annotations

import csv
import math
import os
import pickle
import sys
import time
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

from pymavlink import mavutil

from fw_sitl.camera_model import CameraModel, dir_cam_az_el_deg, dir_cam_to_ned
from fw_sitl.path_geometry import wrap_pi
from fw_sitl.quat import Quat, from_rpy


def unwrap_deg_list(vals: list[float]) -> list[float]:
    """Unwrap a degree series so ±180 wraps do not draw as vertical jumps."""
    out: list[float] = []
    prev: float | None = None
    acc = 0.0
    for v in vals:
        if not math.isfinite(v):
            out.append(v)
            continue
        if prev is None:
            acc = float(v)
        else:
            acc += wrap_deg(float(v) - prev)
        out.append(acc)
        prev = float(v)
    return out


def wrap_deg(angle: float) -> float:
    """Wrap degrees to (-180, 180]."""
    a = float(angle)
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def slew_toward_ned(
    cur: tuple[float, float, float],
    tgt: tuple[float, float, float],
    max_step_m: float,
) -> tuple[float, float, float]:
    """Move ``cur`` toward ``tgt`` by at most ``max_step_m`` (no NED snaps)."""
    dn = float(tgt[0]) - float(cur[0])
    de = float(tgt[1]) - float(cur[1])
    dd = float(tgt[2]) - float(cur[2])
    dist = math.sqrt(dn * dn + de * de + dd * dd)
    if dist <= max_step_m or dist < 1e-9:
        return (float(tgt[0]), float(tgt[1]), float(tgt[2]))
    s = float(max_step_m) / dist
    return (float(cur[0]) + dn * s, float(cur[1]) + de * s, float(cur[2]) + dd * s)


def extrapolate_ned(
    samples: Sequence[tuple[float, tuple[float, float, float]]],
    now: float,
    *,
    max_age_s: float = 3.0,
    vel_ned: tuple[float, float, float] | None = None,
) -> tuple[float, float, float] | None:
    """FG pose at ``now`` from timestamped GT samples (no EKF position coast).

    Prefer live FG NED velocity when given (telnet is sparse). Else two samples
    give a finite-difference velocity. Cap coast at ``max_age_s``.
    """
    if not samples:
        return None
    t1, p1 = samples[-1]
    age = min(max(float(now) - float(t1), 0.0), float(max_age_s))
    if vel_ned is not None:
        return (
            float(p1[0]) + float(vel_ned[0]) * age,
            float(p1[1]) + float(vel_ned[1]) * age,
            float(p1[2]) + float(vel_ned[2]) * age,
        )
    if len(samples) == 1:
        return (float(p1[0]), float(p1[1]), float(p1[2]))
    t0, p0 = samples[-2]
    dt = float(t1) - float(t0)
    if dt < 0.05:
        return (float(p1[0]), float(p1[1]), float(p1[2]))
    k = age / dt
    return (
        float(p1[0]) + (float(p1[0]) - float(p0[0])) * k,
        float(p1[1]) + (float(p1[1]) - float(p0[1])) * k,
        float(p1[2]) + (float(p1[2]) - float(p0[2])) * k,
    )


def slew_toward_rpy(
    cur: tuple[float, float, float],
    tgt: tuple[float, float, float],
    max_step_rad: float,
) -> tuple[float, float, float]:
    """Slew roll/pitch/yaw (rad), wrapping each axis."""
    out: list[float] = []
    step = float(max_step_rad)
    for i in range(3):
        d = wrap_pi(float(tgt[i]) - float(cur[i]))
        if abs(d) <= step:
            out.append(wrap_pi(float(tgt[i])))
        else:
            out.append(wrap_pi(float(cur[i]) + math.copysign(step, d)))
    return (out[0], out[1], out[2])


def los_az_el_deg(
    pos_ned: tuple[float, float, float],
    target_ned: tuple[float, float, float],
) -> tuple[float, float]:
    """Geometric LOS azimuth/elevation (deg) from ``pos_ned`` to ``target_ned``.

    Azimuth is NED ``atan2(E, N)`` (0=north, 90=east). Elevation is positive up.
    """
    dx = float(target_ned[0]) - float(pos_ned[0])
    dy = float(target_ned[1]) - float(pos_ned[1])
    dz = float(target_ned[2]) - float(pos_ned[2])
    horiz = math.hypot(dx, dy)
    az = math.degrees(math.atan2(dy, dx)) if horiz > 1e-9 else 0.0
    el = math.degrees(math.atan2(-dz, horiz)) if horiz > 1e-9 else 0.0
    return az, el


def plot_png_paths(prefix: Path | str) -> tuple[Path, Path]:
    """``prefix_history.png`` and ``prefix_trajectory.png``."""
    path = Path(prefix)
    return (
        path.parent / f"{path.name}_history.png",
        path.parent / f"{path.name}_trajectory.png",
    )


def _use_gui_backend() -> None:
    """Prefer a zoom/pan GUI backend. No-op if one is already active."""
    import matplotlib

    if matplotlib.get_backend().lower() != "agg":
        return
    for name in ("TkAgg", "QtAgg", "Qt5Agg", "GTK3Agg"):
        try:
            matplotlib.use(name, force=True)
            return
        except Exception:
            continue


def first_unpassed_balloon(
    pos_ned: tuple[float, float, float],
    balloons: Sequence[tuple[float, float, float]],
) -> tuple[float, float, float] | None:
    """First balloon the plane has not yet gone abeam of (along the course).

    Chase retargets at ``pass_radius`` while still approaching, which hides the
    ±90° fly-past. LOS plots use this so azimuth grows through abeam.
    """
    if not balloons:
        return None
    prev: tuple[float, float, float] | None = None
    for balloon in balloons:
        if prev is None:
            ux, uy = 1.0, 0.0
        else:
            gx = float(balloon[0]) - float(prev[0])
            gy = float(balloon[1]) - float(prev[1])
            nrm = math.hypot(gx, gy)
            if nrm < 1e-9:
                ux, uy = 1.0, 0.0
            else:
                ux, uy = gx / nrm, gy / nrm
        along = (float(pos_ned[0]) - float(balloon[0])) * ux + (
            float(pos_ned[1]) - float(balloon[1])
        ) * uy
        if along <= 0.0:
            return (float(balloon[0]), float(balloon[1]), float(balloon[2]))
        prev = balloon
    last = balloons[-1]
    return (float(last[0]), float(last[1]), float(last[2]))


def ned_delta_m(
    pos_ned: tuple[float, float, float],
    target_ned: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Plane minus target in NED metres (ΔN, ΔE, ΔD)."""
    return (
        float(pos_ned[0]) - float(target_ned[0]),
        float(pos_ned[1]) - float(target_ned[1]),
        float(pos_ned[2]) - float(target_ned[2]),
    )


def frustum_corner_dirs_ned(
    camera: CameraModel,
    roll: float,
    pitch: float,
    yaw: float,
) -> list[tuple[float, float, float]]:
    """Unit NED (down+) directions for the camera's 4 image corners.

    Corner pixel coordinates only fix the ray *direction*; the ratio
    ``(u - cx) / fx`` is independent of ``width_px``/``height_px`` (both cancel:
    ``cx = 0.5*w``, ``fx = 0.5*w/tan(hfov/2)``), so any ``width_px``/``height_px``
    on ``camera`` gives the same 4 directions for its configured ``hfov``/``vfov``.
    """
    w, h = float(camera.width_px), float(camera.height_px)
    corners_px = ((0.0, 0.0), (w, 0.0), (w, h), (0.0, h))
    return [
        dir_cam_to_ned(camera.pixel_to_dir_cam(u, v), camera, roll, pitch, yaw)
        for u, v in corners_px
    ]


def _finite_or_none(val: object) -> float | None:
    try:
        num = float(val)
    except (TypeError, ValueError):
        return None
    return num if math.isfinite(num) else None


@dataclass
class FlightHistory:
    """Time series of NED position/velocity and attitude (deg)."""

    t0: float = field(default_factory=time.time)
    t: list[float] = field(default_factory=list)
    x: list[float] = field(default_factory=list)
    y: list[float] = field(default_factory=list)
    z: list[float] = field(default_factory=list)
    vx: list[float] = field(default_factory=list)
    vy: list[float] = field(default_factory=list)
    vz: list[float] = field(default_factory=list)
    roll_deg: list[float] = field(default_factory=list)
    pitch_deg: list[float] = field(default_factory=list)
    yaw_deg: list[float] = field(default_factory=list)
    roll_cmd_deg: list[float] = field(default_factory=list)
    pitch_cmd_deg: list[float] = field(default_factory=list)
    yaw_cmd_deg: list[float] = field(default_factory=list)
    main_mode: list[int | None] = field(default_factory=list)
    tgt_x: list[float] = field(default_factory=list)
    tgt_y: list[float] = field(default_factory=list)
    tgt_z: list[float] = field(default_factory=list)
    # Raw PX4 LOCAL_POSITION_NED (EKF frame). ``x/y/z`` may be EKF+offset.
    ekf_x: list[float] = field(default_factory=list)
    ekf_y: list[float] = field(default_factory=list)
    ekf_z: list[float] = field(default_factory=list)
    # Plant/sim NED (FG telnet or Gazebo world). Error plots use this.
    sim_x: list[float] = field(default_factory=list)
    sim_y: list[float] = field(default_factory=list)
    sim_z: list[float] = field(default_factory=list)
    cam_az_deg: list[float] = field(default_factory=list)
    cam_el_deg: list[float] = field(default_factory=list)

    # Onboard camera intrinsics/mount, for drawing the FOV frustum on the 3D
    # plot. Defaults match flight_setup.py's DEFAULT_*; getattr(..., default)
    # is used when reading these so pickles from before this field existed
    # still load (dataclass unpickling restores __dict__ verbatim, it does
    # not re-run field defaults).
    cam_hfov_deg: float = 90.0
    cam_vfov_deg: float = 70.0
    cam_mount_azimuth_deg: float = 0.0
    cam_mount_elevation_deg: float = 0.0

    # Optional locked path for along/cross-track plots (set by the runner).
    path_origin_xy: tuple[float, float] | None = field(default=None, repr=False)
    path_course_rad: float | None = field(default=None, repr=False)
    _target: tuple[float, float, float] | None = field(default=None, repr=False)
    _cam_los: tuple[float, float, float] | None = field(default=None, repr=False)
    _balloon_markers: list[
        tuple[tuple[float, float, float], tuple[int, int, int]]
    ] = field(default_factory=list, repr=False)

    _last_att_deg: tuple[float, float, float] | None = field(default=None, repr=False)
    last_att_rad: tuple[float, float, float] | None = field(default=None, repr=False)
    last_q: Quat | None = field(default=None, repr=False)
    last_pos: tuple[float, float, float] | None = field(default=None, repr=False)
    # Raw EKF from the last LOCAL_POSITION_NED. ``last_pos`` is overwritten
    # by FG/GZ offsets; locking ``gt − last_pos`` after that lock would
    # collapse the offset toward zero.
    last_ekf_pos: tuple[float, float, float] | None = field(default=None, repr=False)
    _boot0_ms: int | None = field(default=None, repr=False)
    last_armed: bool | None = field(default=None, repr=False)
    last_main_mode: int | None = field(default=None, repr=False)
    last_vz: float | None = field(default=None, repr=False)
    last_vx: float | None = field(default=None, repr=False)
    last_vy: float | None = field(default=None, repr=False)
    last_airspeed: float | None = field(default=None, repr=False)
    last_groundspeed: float | None = field(default=None, repr=False)
    last_throttle: float | None = field(default=None, repr=False)

    def clear_series(self) -> None:
        """Drop recorded samples (keep last EKF/att) and restart ``t0``."""
        self.t.clear()
        self.x.clear()
        self.y.clear()
        self.z.clear()
        self.vx.clear()
        self.vy.clear()
        self.vz.clear()
        self.roll_deg.clear()
        self.pitch_deg.clear()
        self.yaw_deg.clear()
        self.roll_cmd_deg.clear()
        self.pitch_cmd_deg.clear()
        self.yaw_cmd_deg.clear()
        self.main_mode.clear()
        self.tgt_x.clear()
        self.tgt_y.clear()
        self.tgt_z.clear()
        self.ekf_x.clear()
        self.ekf_y.clear()
        self.ekf_z.clear()
        self.sim_x.clear()
        self.sim_y.clear()
        self.sim_z.clear()
        self.cam_az_deg.clear()
        self.cam_el_deg.clear()
        self.t0 = time.time()
        self._boot0_ms = None

    def __len__(self) -> int:
        return len(self.t)

    @classmethod
    def from_race_csv(cls, path: Path | str) -> FlightHistory:
        """Rebuild a plottable history from race CSV sample/pass rows."""
        csv_path = Path(path)
        hist = cls()
        hist.t0 = 0.0
        balloons: dict[
            int, tuple[tuple[float, float, float], tuple[int, int, int]]
        ] = {}
        with csv_path.open("r", newline="", encoding="utf-8") as fh:
            for row in csv.DictReader(fh):
                event = (row.get("event") or "").strip()
                if event.startswith("end_") or event not in ("sample", "pass"):
                    continue
                hist.t.append(float(row["t_s"]))
                hist.x.append(float(row["pos_n"]))
                hist.y.append(float(row["pos_e"]))
                hist.z.append(float(row["pos_d"]))
                hist.vx.append(0.0)
                hist.vy.append(0.0)
                hist.vz.append(0.0)
                hist.roll_deg.append(float("nan"))
                hist.pitch_deg.append(float("nan"))
                hist.yaw_deg.append(float("nan"))
                hist.roll_cmd_deg.append(float("nan"))
                hist.pitch_cmd_deg.append(float("nan"))
                hist.yaw_cmd_deg.append(float("nan"))
                hist.main_mode.append(None)
                tgt = (
                    float(row["tgt_n"]),
                    float(row["tgt_e"]),
                    float(row["tgt_d"]),
                )
                hist.tgt_x.append(tgt[0])
                hist.tgt_y.append(tgt[1])
                hist.tgt_z.append(tgt[2])
                try:
                    idx = int(row.get("balloon_idx") or 0)
                except ValueError:
                    idx = 0
                color = (
                    int(float(row.get("color_r") or 0)),
                    int(float(row.get("color_g") or 0)),
                    int(float(row.get("color_b") or 0)),
                )
                balloons[idx] = (tgt, color)
        hist.set_balloon_markers([balloons[i] for i in sorted(balloons)])
        return hist

    def to_pickle(self, path: Path | str) -> Path:
        """Write this history for the host matplotlib waiter."""
        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_bytes(pickle.dumps(self, protocol=4))
        return out

    @classmethod
    def from_pickle(cls, path: Path | str) -> FlightHistory:
        obj = pickle.loads(Path(path).read_bytes())
        if not isinstance(obj, cls):
            raise TypeError(f"expected {cls.__name__}, got {type(obj)!r}")
        return obj

    def request_streams(self, master: mavutil.mavfile, hz: float = 20.0) -> None:
        interval_us = int(1e6 / max(hz, 1.0))
        for msg_id in (
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
        ):
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                msg_id,
                interval_us,
                0,
                0,
                0,
                0,
                0,
            )

    def poll(self, master: mavutil.mavfile) -> tuple[float, float, float] | None:
        """Drain position + attitude + heartbeat; append on each LOCAL_POSITION_NED."""
        while True:
            msg = master.recv_match(
                type=["LOCAL_POSITION_NED", "ATTITUDE", "HEARTBEAT", "VFR_HUD"],
                blocking=False,
            )
            if msg is None:
                break
            if msg.get_srcSystem() not in (0, master.target_system):
                continue
            mtype = msg.get_type()
            if mtype == "HEARTBEAT":
                if msg.get_srcSystem() != master.target_system:
                    continue
                self.last_armed = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                self.last_main_mode = (int(msg.custom_mode) >> 16) & 0xFF
                continue
            if mtype == "VFR_HUD":
                self.last_airspeed = _finite_or_none(getattr(msg, "airspeed", None))
                self.last_groundspeed = _finite_or_none(
                    getattr(msg, "groundspeed", None)
                )
                self.last_throttle = _finite_or_none(getattr(msg, "throttle", None))
                continue
            if mtype == "ATTITUDE":
                roll = float(msg.roll)
                pitch = float(msg.pitch)
                yaw = float(msg.yaw)
                self.last_att_rad = (roll, pitch, yaw)
                self.last_q = from_rpy(roll, pitch, yaw)
                self._last_att_deg = (
                    math.degrees(roll),
                    math.degrees(pitch),
                    math.degrees(yaw),
                )
                continue
            # LOCAL_POSITION_NED
            pos = (float(msg.x), float(msg.y), float(msg.z))
            self.last_pos = pos
            self.last_ekf_pos = pos
            roll_d, pitch_d, yaw_d = self._last_att_deg or (float("nan"),) * 3
            boot_ms = int(getattr(msg, "time_boot_ms", 0) or 0)
            if boot_ms > 0 and self._boot0_ms is None:
                self._boot0_ms = boot_ms
            # Wall clock, same origin as overlay / duration_s. PX4 boot time
            # runs ~1.3× slower than wall under JSBSim+FG.
            t_s = time.time() - self.t0
            if self.t and t_s <= self.t[-1]:
                t_s = self.t[-1] + 1e-3
            self.t.append(t_s)
            self.x.append(pos[0])
            self.y.append(pos[1])
            self.z.append(pos[2])
            self.ekf_x.append(pos[0])
            self.ekf_y.append(pos[1])
            self.ekf_z.append(pos[2])
            self.sim_x.append(float("nan"))
            self.sim_y.append(float("nan"))
            self.sim_z.append(float("nan"))
            self.vx.append(float(msg.vx))
            self.vy.append(float(msg.vy))
            self.last_vx = float(msg.vx)
            self.last_vy = float(msg.vy)
            self.last_vz = float(msg.vz)
            self.vz.append(self.last_vz)
            self.roll_deg.append(roll_d)
            self.pitch_deg.append(pitch_d)
            self.yaw_deg.append(yaw_d)
            self.roll_cmd_deg.append(float("nan"))
            self.pitch_cmd_deg.append(float("nan"))
            self.yaw_cmd_deg.append(float("nan"))
            self.main_mode.append(self.last_main_mode)
            tgt = self._target
            if tgt is None:
                self.tgt_x.append(float("nan"))
                self.tgt_y.append(float("nan"))
                self.tgt_z.append(float("nan"))
            else:
                self.tgt_x.append(tgt[0])
                self.tgt_y.append(tgt[1])
                self.tgt_z.append(tgt[2])
        return self.last_pos

    def note_target(self, ned: tuple[float, float, float]) -> None:
        """Remember the current chase target for the next / last position sample."""
        self._target = (float(ned[0]), float(ned[1]), float(ned[2]))

    def overwrite_positions_from(
        self, start_index: int, ned: tuple[float, float, float]
    ) -> None:
        """Replace x/y/z for every sample appended since ``start_index`` with ``ned``.

        ``poll()`` drains *all* queued LOCAL_POSITION_NED messages per call and
        appends one raw (EKF) sample each — the stream runs faster than a
        typical control loop, so a single ``poll()`` often appends more than
        one sample. A ground-truth source (e.g. Gazebo world pose) only gives
        one fresh value per control tick; patching only the last appended
        sample left the earlier ones in a burst at their raw EKF position,
        which is most wrong right after spawn (EKF still converging) — a
        dense position/LOS zigzag that shrinks to invisible once EKF error
        settles, but is very visible zoomed into the first few seconds.
        """
        n, e, d = float(ned[0]), float(ned[1]), float(ned[2])
        for i in range(max(start_index, 0), len(self.x)):
            self.x[i] = n
            self.y[i] = e
            self.z[i] = d

    def _series(self, name: str) -> list:
        val = self.__dict__.get(name)
        return val if isinstance(val, list) else []

    def _pad_sim_to_len(self) -> None:
        n = len(self.x)
        if "sim_x" not in self.__dict__:
            self.sim_x = []
            self.sim_y = []
            self.sim_z = []
        while len(self.sim_x) < n:
            self.sim_x.append(float("nan"))
            self.sim_y.append(float("nan"))
            self.sim_z.append(float("nan"))

    def set_sim_ned_from(
        self, start_index: int, ned: tuple[float, float, float]
    ) -> None:
        """Plant/sim NED for every sample appended since ``start_index``."""
        self._pad_sim_to_len()
        n, e, d = float(ned[0]), float(ned[1]), float(ned[2])
        for i in range(max(start_index, 0), len(self.x)):
            self.sim_x[i] = n
            self.sim_y[i] = e
            self.sim_z[i] = d

    def apply_sim_coast_from(
        self,
        start_index: int,
        gt_lock: tuple[float, float, float],
        ekf_lock: tuple[float, float, float],
    ) -> None:
        """Dense sim NED: last FG pose plus EKF Δ since that pose was taken."""
        self._pad_sim_to_len()
        ekf_x = self._series("ekf_x")
        ekf_y = self._series("ekf_y")
        ekf_z = self._series("ekf_z")
        for i in range(max(start_index, 0), len(self.x)):
            if i < len(ekf_x):
                ex, ey, ez = ekf_x[i], ekf_y[i], ekf_z[i]
            else:
                ex, ey, ez = self.x[i], self.y[i], self.z[i]
            self.sim_x[i] = gt_lock[0] + ex - ekf_lock[0]
            self.sim_y[i] = gt_lock[1] + ey - ekf_lock[1]
            self.sim_z[i] = gt_lock[2] + ez - ekf_lock[2]

    def _has_sim_series(self) -> bool:
        sx = self._series("sim_x")
        sy = self._series("sim_y")
        sz = self._series("sim_z")
        return (
            len(sx) == len(self.x) == len(sy) == len(sz)
            and len(sx) > 0
            and any(math.isfinite(v) for v in sx)
        )

    def plane_ned_at(self, i: int) -> tuple[float, float, float]:
        """Sim NED when recorded, else EKF/world ``x/y/z``."""
        if self._has_sim_series():
            sx = self.sim_x[i]
            if math.isfinite(sx):
                return (sx, self.sim_y[i], self.sim_z[i])
        return (self.x[i], self.y[i], self.z[i])

    def last_plane_ned(self) -> tuple[float, float, float] | None:
        if not self.x:
            return None
        return self.plane_ned_at(len(self.x) - 1)

    def overwrite_attitudes_from(
        self, start_index: int, att_rad: tuple[float, float, float]
    ) -> None:
        """Patch roll/pitch/yaw deg for samples appended since ``start_index``."""
        roll, pitch, yaw = (float(att_rad[0]), float(att_rad[1]), float(att_rad[2]))
        rd, pd, yd = math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
        self.last_att_rad = (roll, pitch, yaw)
        self.last_q = from_rpy(roll, pitch, yaw)
        self._last_att_deg = (rd, pd, yd)
        n = len(self.yaw_deg)
        for i in range(max(start_index, 0), n):
            if i < len(self.roll_deg):
                self.roll_deg[i] = rd
            if i < len(self.pitch_deg):
                self.pitch_deg[i] = pd
            self.yaw_deg[i] = yd

    def add_ned_offset_from(
        self, start_index: int, dned: tuple[float, float, float]
    ) -> None:
        """Add a locked NED offset to EKF samples since ``start_index``.

        Stamping every sample with the same slow FG pose made plots look like
        ~30 unique points/min. Offset = last_gt − ekf_at_lock; EKF increments
        keep unique samples at MAVLink rate between telnet updates.
        """
        dn, de, dd = float(dned[0]), float(dned[1]), float(dned[2])
        for i in range(max(start_index, 0), len(self.x)):
            self.x[i] += dn
            self.y[i] += de
            self.z[i] += dd
        if self.x:
            self.last_pos = (self.x[-1], self.y[-1], self.z[-1])

    def recompute_ned_velocity_from(
        self, start_index: int, min_dt: float = 0.1
    ) -> None:
        """Replace vx/vy/vz with ΔNED over at least ``min_dt`` seconds.

        Single-sample Δt at 50 Hz turns ~1 m EKF wobble into 20–100 m/s plot
        spikes (pickle 120853). A ~0.1 s baseline matches cruise (~18 m/s).
        """
        n = len(self.x)
        lo = max(int(start_index), 1)
        k = 0
        min_dt = float(min_dt)
        for i in range(lo, n):
            while k < i - 1 and (self.t[i] - self.t[k + 1]) >= min_dt:
                k += 1
            dt = float(self.t[i] - self.t[k])
            if dt < max(1e-3, 0.5 * min_dt):
                if i > 0:
                    if i < len(self.vx):
                        self.vx[i] = self.vx[i - 1]
                    if i < len(self.vy):
                        self.vy[i] = self.vy[i - 1]
                    if i < len(self.vz):
                        self.vz[i] = self.vz[i - 1]
                continue
            if i < len(self.vx):
                self.vx[i] = (self.x[i] - self.x[k]) / dt
            if i < len(self.vy):
                self.vy[i] = (self.y[i] - self.y[k]) / dt
            if i < len(self.vz):
                self.vz[i] = (self.z[i] - self.z[k]) / dt

    def absorb_vel_jumps_from(
        self, start_index: int, max_step_mps: float = 8.0
    ) -> None:
        """Hold plotted EKF vx/vy/vz across estimator glitches.

        Pickle 090533: |Δv| of 30–70 m/s in one sample while NED moved ~0.5 m.
        Finite-differencing offset NED at 50 Hz turned position noise into
        ~25–100 m/s velocity jitter (pickle 120853) — keep EKF v, drop spikes.
        """
        n = min(len(self.vx), len(self.vy), len(self.vz))
        lo = max(int(start_index), 1)
        for i in range(lo, n):
            d = math.hypot(self.vx[i] - self.vx[i - 1], self.vy[i] - self.vy[i - 1])
            if d <= max_step_mps:
                continue
            self.vx[i] = self.vx[i - 1]
            self.vy[i] = self.vy[i - 1]
            if i < len(self.vz):
                self.vz[i] = self.vz[i - 1]

    def absorb_yaw_jumps_from(
        self, start_index: int, max_step_deg: float = 10.0
    ) -> float:
        """Hold plotted yaw across EKF heading glitches; return extra yaw offset (rad).

        Same-tick LOCAL_POSITION bursts in pickle 090533 showed 30–107° yaw
        steps (roll unchanged) that persisted — EKF heading resets, not airframe
        motion. Shift later samples so the trace stays continuous and tell the
        caller to add the same delta to the locked FG att offset.
        """
        extra_deg = 0.0
        n = len(self.yaw_deg)
        lo = max(int(start_index), 1)
        for i in range(lo, n):
            if not math.isfinite(self.yaw_deg[i]) or not math.isfinite(
                self.yaw_deg[i - 1]
            ):
                continue
            dy = wrap_deg(self.yaw_deg[i] - self.yaw_deg[i - 1])
            if abs(dy) <= max_step_deg:
                continue
            extra_deg -= dy
            for j in range(i, n):
                if math.isfinite(self.yaw_deg[j]):
                    self.yaw_deg[j] = wrap_deg(self.yaw_deg[j] - dy)
        return math.radians(extra_deg)

    def add_rpy_offset_from(
        self, start_index: int, d_rad: tuple[float, float, float]
    ) -> None:
        """Add a locked roll/pitch/yaw offset (rad) to samples since ``start_index``."""
        dr = math.degrees(float(d_rad[0]))
        dp = math.degrees(float(d_rad[1]))
        dy = math.degrees(float(d_rad[2]))
        n = len(self.yaw_deg)
        for i in range(max(start_index, 0), n):
            if i < len(self.roll_deg) and math.isfinite(self.roll_deg[i]):
                self.roll_deg[i] += dr
            if i < len(self.pitch_deg) and math.isfinite(self.pitch_deg[i]):
                self.pitch_deg[i] += dp
            if math.isfinite(self.yaw_deg[i]):
                self.yaw_deg[i] = wrap_deg(self.yaw_deg[i] + dy)
        # Do not write offset Euler into ``_last_att_deg``: the next poll()
        # stamps new LOCAL_POSITION samples from that cache. If it already
        # held FG-frame att, adding the offset again flipped yaw by ~offset
        # every other sample (pickle 083609: ±125°).

    def apply_attitude_cmd_from(
        self, start_index: int, rpy_deg: tuple[float, float, float]
    ) -> None:
        """Write SET_ATTITUDE_TARGET Euler (deg) onto samples since ``start_index``."""
        rd, pd, yd = (float(rpy_deg[0]), float(rpy_deg[1]), float(rpy_deg[2]))
        n = len(self.t)
        while len(self.roll_cmd_deg) < n:
            self.roll_cmd_deg.append(float("nan"))
            self.pitch_cmd_deg.append(float("nan"))
            self.yaw_cmd_deg.append(float("nan"))
        for i in range(max(start_index, 0), n):
            self.roll_cmd_deg[i] = rd
            self.pitch_cmd_deg[i] = pd
            self.yaw_cmd_deg[i] = wrap_deg(yd)

    def _has_attitude_cmd_series(self) -> bool:
        cmds = getattr(self, "roll_cmd_deg", None)
        if not cmds or len(cmds) != len(self.t):
            return False
        return any(math.isfinite(v) for v in cmds)

    def set_balloon_markers(
        self,
        balloons: Sequence[tuple[tuple[float, float, float], tuple[int, int, int]]],
    ) -> None:
        """Store world balloon NED + RGB for the 3D trajectory figure."""
        self._balloon_markers = [
            (
                (float(ned[0]), float(ned[1]), float(ned[2])),
                (int(color[0]), int(color[1]), int(color[2])),
            )
            for ned, color in balloons
        ]

    def balloon_markers_neu(
        self,
    ) -> list[tuple[float, float, float, tuple[float, float, float]]]:
        """Balloon markers as (North, East, Up, rgb 0–1)."""
        markers: list[tuple[float, float, float, tuple[float, float, float]]] = []
        for (north, east, down), rgb in self._balloon_markers:
            r, g, b = rgb
            markers.append(
                (
                    north,
                    east,
                    -down,
                    (r / 255.0, g / 255.0, b / 255.0),
                )
            )
        return markers

    def scene_bounds_neu(self) -> tuple[tuple[float, float, float], float]:
        """Equal-aspect NEU box (mid, half-span) covering path + balloons."""
        xs = list(self.x)
        ys = list(self.y)
        zs = [-z for z in self.z]
        if self._has_sim_series():
            xs.extend(v for v in self.sim_x if math.isfinite(v))
            ys.extend(v for v in self.sim_y if math.isfinite(v))
            zs.extend(-v for v in self.sim_z if math.isfinite(v))
        for north, east, up, _rgb in self.balloon_markers_neu():
            xs.append(north)
            ys.append(east)
            zs.append(up)
        if not xs:
            return (0.0, 0.0, 0.0), 0.5
        mid = (
            0.5 * (min(xs) + max(xs)),
            0.5 * (min(ys) + max(ys)),
            0.5 * (min(zs) + max(zs)),
        )
        span = max(
            max(xs) - min(xs),
            max(ys) - min(ys),
            max(zs) - min(zs),
            1.0,
        )
        return mid, 0.5 * span

    def apply_target_to_last(self, start_index: int | None = None) -> None:
        """Write ``_target`` onto samples from ``start_index`` (default: last only, pad if needed).

        ``poll()`` can append more than one sample per call (LOCAL_POSITION_NED
        streams faster than a typical control loop). Callers that drive one
        ``poll()`` per tick must pass ``start_index`` (the length of ``x``
        *before* that ``poll()``) so every sample from this tick gets the
        current target, not just the last — otherwise the un-patched samples
        pad in as NaN, which ``los_deg_series`` and ``target_delta_series``
        interpret as "no target", flipping them onto a different (geometric)
        code path every other sample once cam-tracking starts.
        """
        if not self.x or self._target is None:
            return
        n = len(self.x)
        while len(self.tgt_x) < n:
            self.tgt_x.append(float("nan"))
            self.tgt_y.append(float("nan"))
            self.tgt_z.append(float("nan"))
        lo = n - 1 if start_index is None else max(start_index, 0)
        for i in range(lo, n):
            self.tgt_x[i] = self._target[0]
            self.tgt_y[i] = self._target[1]
            self.tgt_z[i] = self._target[2]

    def note_cam_los(
        self, dir_cam: tuple[float, float, float] | None
    ) -> None:
        """Remember tracker camera-frame LOS for the last position sample."""
        if dir_cam is None:
            self._cam_los = None
            return
        self._cam_los = (float(dir_cam[0]), float(dir_cam[1]), float(dir_cam[2]))

    def apply_cam_to_last(self, start_index: int | None = None) -> None:
        """Write camera-frame az/el onto samples from ``start_index`` (default: last only).

        Same burst hazard as ``apply_target_to_last``: patching only the last
        sample of a multi-sample ``poll()`` left the earlier sample(s) at NaN,
        which fell back to the geometric LOS instead of the tracked camera
        blob — every other sample alternating between two genuinely different
        LOS estimates once the tracker acquired the balloon (a real, dense
        LOS-elevation zigzag, not sensor noise).
        """
        if not self.x:
            return
        n = len(self.x)
        while len(self.cam_az_deg) < n:
            self.cam_az_deg.append(float("nan"))
            self.cam_el_deg.append(float("nan"))
        lo = n - 1 if start_index is None else max(start_index, 0)
        if self._cam_los is None:
            for i in range(lo, n):
                self.cam_az_deg[i] = float("nan")
                self.cam_el_deg[i] = float("nan")
            return
        az, el = dir_cam_az_el_deg(self._cam_los)
        for i in range(lo, n):
            self.cam_az_deg[i] = az
            self.cam_el_deg[i] = el

    def _has_target_series(self) -> bool:
        return (
            len(self.tgt_x) == len(self.x)
            and len(self.x) > 0
            and any(math.isfinite(v) for v in self.tgt_x)
        )

    def los_deg_series(self) -> tuple[list[float], list[float]] | None:
        """LOS azimuth / elevation (deg) vs the unpassed balloon, or None.

        Always body +X (bearing−yaw / elevation−pitch). Mixing camera-frame
        blob angles (0=image center, typically ±40°) with that geometric
        series jumped 85–140° whenever HSV lock flickered (pickle 090533).
        Camera az/el stay on ``cam_*_deg`` for pickle/debug. Yaw missing →
        ground track; pitch missing → horizon.
        """
        if not self._has_target_series():
            return None
        balloons = tuple(ned for ned, _rgb in self._balloon_markers)
        has_yaw = len(self.yaw_deg) == len(self.x)
        has_pitch = len(self.pitch_deg) == len(self.x)
        has_vel = len(self.vx) == len(self.x) and len(self.vy) == len(self.x)
        az: list[float] = []
        el: list[float] = []
        for i, (tx, ty, tz) in enumerate(zip(self.tgt_x, self.tgt_y, self.tgt_z)):
            px, py, pz = self.plane_ned_at(i)
            tgt = (tx, ty, tz)
            if balloons:
                sticky = first_unpassed_balloon((px, py, pz), balloons)
                if sticky is not None:
                    tgt = sticky
            a, e = los_az_el_deg((px, py, pz), tgt)
            heading: float | None = None
            if has_yaw and math.isfinite(self.yaw_deg[i]):
                heading = float(self.yaw_deg[i])
            elif has_vel:
                gs = math.hypot(float(self.vx[i]), float(self.vy[i]))
                if gs >= 5.0:
                    heading = math.degrees(
                        math.atan2(float(self.vy[i]), float(self.vx[i]))
                    )
            if heading is not None:
                a = wrap_deg(a - heading)
            if has_pitch and math.isfinite(self.pitch_deg[i]):
                e = wrap_deg(e - float(self.pitch_deg[i]))
            az.append(a)
            el.append(e)
        return az, el

    def target_delta_series(
        self,
    ) -> tuple[list[float], list[float], list[float]] | None:
        """Plane−target NED (m) from sim pose when recorded, else EKF/world."""
        if not self._has_target_series():
            return None
        dn: list[float] = []
        de: list[float] = []
        dd: list[float] = []
        for i, (tx, ty, tz) in enumerate(zip(self.tgt_x, self.tgt_y, self.tgt_z)):
            n, e, d = ned_delta_m(self.plane_ned_at(i), (tx, ty, tz))
            dn.append(n)
            de.append(e)
            dd.append(d)
        return dn, de, dd

    def path_series(self) -> tuple[list[float], list[float]] | None:
        """Along-track and signed cross-track (m) vs locked path, if configured."""
        if self.path_origin_xy is None or self.path_course_rad is None:
            return None
        ox, oy = self.path_origin_xy
        c = math.cos(self.path_course_rad)
        s = math.sin(self.path_course_rad)
        along: list[float] = []
        cross: list[float] = []
        for i in range(len(self.x)):
            x, y, _z = self.plane_ned_at(i)
            dx = x - ox
            dy = y - oy
            along.append(dx * c + dy * s)
            cross.append(-dx * s + dy * c)
        return along, cross

    def summarize_path(self) -> str | None:
        series = self.path_series()
        if series is None or not series[0]:
            return None
        along, cross = series
        i0 = max(0, len(along) // 10)
        a0, a1 = along[i0], along[-1]
        c_slice = cross[i0:]
        rms = math.sqrt(sum(v * v for v in c_slice) / max(1, len(c_slice)))
        flips = sum(
            1
            for i in range(1, len(self.main_mode))
            if self.main_mode[i] != self.main_mode[i - 1]
            and self.main_mode[i] is not None
            and self.main_mode[i - 1] is not None
        )
        max_dz = 0.0
        max_dxy = 0.0
        for i in range(1, len(self.z)):
            max_dz = max(max_dz, abs(self.z[i] - self.z[i - 1]))
            max_dxy = max(
                max_dxy,
                math.hypot(self.x[i] - self.x[i - 1], self.y[i] - self.y[i - 1]),
            )
        return (
            f"Path summary: along {a0:.0f}->{a1:.0f} m ({a1 - a0:.0f} m progress), "
            f"cross-track end={cross[-1]:.1f} m rms={rms:.1f} m, "
            f"mode flips={flips}, max|Δz|={max_dz:.1f} m max|Δxy|={max_dxy:.1f} m"
        )

    def make_figures(self, *, title: str = "Flight history"):
        """Build the time-series figure (shared x) and the 3D trajectory figure."""
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 — registers 3d projection

        path = self.path_series()
        los = self.los_deg_series()
        delta = self.target_delta_series()
        extra = 2 if los is not None and delta is not None else 0
        nrows = (4 if path is not None else 3) + extra
        fig, axes = plt.subplots(
            nrows,
            1,
            sharex=True,
            figsize=(10, 9 + 2.2 * extra if path else 8 + 2.2 * extra),
            constrained_layout=True,
        )
        fig.suptitle(title)

        ax = axes[0]
        ax.plot(
            self.t, self.x, color="C0", linestyle=":", alpha=0.7, label="EKF N"
        )
        ax.plot(
            self.t, self.y, color="C1", linestyle=":", alpha=0.7, label="EKF E"
        )
        ax.plot(
            self.t, self.z, color="C2", linestyle=":", alpha=0.7, label="EKF D"
        )
        if self._has_sim_series():
            ax.plot(self.t, self.sim_x, color="C0", label="sim N")
            ax.plot(self.t, self.sim_y, color="C1", label="sim E")
            ax.plot(self.t, self.sim_z, color="C2", label="sim D")
        if self._has_target_series():
            ax.plot(
                self.t, self.tgt_x, color="C0", linestyle="--", label="tgt x (N)"
            )
            ax.plot(
                self.t, self.tgt_y, color="C1", linestyle="--", label="tgt y (E)"
            )
            ax.plot(
                self.t, self.tgt_z, color="C2", linestyle="--", label="tgt z (D)"
            )
        ax.set_ylabel("NED position [m]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")
        ax.set_title(
            "Dotted = EKF (world). Solid = sim (FG/GZ). Dashed = target. "
            "ΔN/ΔE/ΔD uses sim when present."
        )

        row = 1
        if path is not None:
            along, cross = path
            ax = axes[row]
            ax.plot(self.t, along, label="along-track", color="C0")
            ax.plot(self.t, cross, label="cross-track", color="C1")
            ax.axhline(0.0, color="k", linewidth=0.8, alpha=0.4)
            ax.set_ylabel("Path frame [m]")
            ax.grid(True, alpha=0.3)
            ax.legend(loc="best")
            ax.set_title("Locked-line frame (this is the straight-flight metric)")
            row += 1

        ax = axes[row]
        ax.plot(self.t, self.vx, label="vx")
        ax.plot(self.t, self.vy, label="vy")
        ax.plot(self.t, self.vz, label="vz")
        ax.set_ylabel("Velocity [m/s]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")
        row += 1

        ax = axes[row]
        ax.plot(self.t, self.roll_deg, color="C0", label="roll")
        ax.plot(self.t, self.pitch_deg, color="C1", label="pitch")
        ax.plot(self.t, unwrap_deg_list(self.yaw_deg), color="C2", label="yaw")
        if self._has_attitude_cmd_series():
            ax.plot(
                self.t,
                self.roll_cmd_deg,
                color="C0",
                linestyle="--",
                label="roll cmd",
            )
            ax.plot(
                self.t,
                self.pitch_cmd_deg,
                color="C1",
                linestyle="--",
                label="pitch cmd",
            )
            ax.plot(
                self.t,
                unwrap_deg_list(self.yaw_cmd_deg),
                color="C2",
                linestyle="--",
                label="yaw cmd",
            )
        ax.set_ylabel("Attitude [deg]")
        ax.set_title("Solid = measured; dashed = SET_ATTITUDE_TARGET command")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")
        row += 1

        if los is not None and delta is not None:
            az, el = los
            ax = axes[row]
            ax.plot(self.t, unwrap_deg_list(az), label="LOS az")
            ax.plot(self.t, el, label="LOS el")
            ax.axhline(0.0, color="k", linewidth=0.8, alpha=0.4)
            ax.set_ylabel("LOS [deg]")
            ax.set_title(
                "LOS to balloon until abeam "
                "(body +X; 0=nose toward balloon)"
            )
            ax.grid(True, alpha=0.3)
            ax.legend(loc="best")
            row += 1
            dn, de, dd = delta
            ax = axes[row]
            ax.plot(self.t, dn, label="ΔN")
            ax.plot(self.t, de, label="ΔE")
            ax.plot(self.t, dd, label="ΔD")
            ax.set_ylabel("Plane − target [m]")
            ax.set_title(
                "NED distance of the plane from the current target (sim pose)"
            )
            ax.grid(True, alpha=0.3)
            ax.legend(loc="best")
            row += 1

        axes[-1].set_xlabel("Time [s]")

        up = [-z for z in self.z]
        fig3d = plt.figure(figsize=(8, 7))
        ax3d = fig3d.add_subplot(111, projection="3d")
        ax3d.plot(
            self.x,
            self.y,
            up,
            color="C0",
            linestyle=":",
            linewidth=1.2,
            alpha=0.7,
            label="EKF",
        )
        if self._has_sim_series():
            sim_up = [-z for z in self.sim_z]
            ax3d.plot(
                self.sim_x,
                self.sim_y,
                sim_up,
                color="C3",
                linewidth=1.6,
                label="sim",
            )
            s0 = self.plane_ned_at(0)
            s1 = self.plane_ned_at(len(self.x) - 1)
            ax3d.scatter(
                s0[0], s0[1], -s0[2], color="C2", s=40, label="start", depthshade=True
            )
            ax3d.scatter(
                s1[0], s1[1], -s1[2], color="C3", s=40, label="end", depthshade=True
            )
        else:
            ax3d.scatter(
                self.x[0],
                self.y[0],
                up[0],
                color="C2",
                s=40,
                label="start",
                depthshade=True,
            )
            ax3d.scatter(
                self.x[-1],
                self.y[-1],
                up[-1],
                color="C3",
                s=40,
                label="end",
                depthshade=True,
            )
        for i, (north, east, up_m, rgb) in enumerate(self.balloon_markers_neu()):
            ax3d.scatter(
                north,
                east,
                up_m,
                color=rgb,
                s=90,
                marker="o",
                edgecolors="k",
                linewidths=0.6,
                label=f"balloon {i}",
                depthshade=True,
            )
        mid, half = self.scene_bounds_neu()
        self._plot_fov_frustums(ax3d, max_frustum_len=max(1.3 * half, 50.0))

        ax3d.set_xlabel("North x [m]")
        ax3d.set_ylabel("East y [m]")
        ax3d.set_zlabel("Up [m]")
        ax3d.set_title(f"{title} — 3D trajectory (dotted EKF, solid sim)")
        fov_legend = getattr(ax3d, "_fov_legend", None)
        if fov_legend is not None:
            ax3d.legend(*fov_legend, loc="best")
        else:
            ax3d.legend(loc="best")
        ax3d.set_xlim(mid[0] - half, mid[0] + half)
        ax3d.set_ylim(mid[1] - half, mid[1] + half)
        ax3d.set_zlim(mid[2] - half, mid[2] + half)
        return fig, axes, fig3d

    def _plot_fov_frustums(
        self, ax3d, *, max_frustum_len: float, n_samples: int = 5
    ) -> None:
        """Draw the camera FOV as translucent pyramids at a few sampled poses.

        Static balloon dots + a path line give no sense of when a wide-FOV
        camera can actually see a balloon (this was the source of a real
        confusion: balloons visible on screen well after the plane's path
        looked "far away" from them in this same plot). Sampling the
        recorded attitude and drawing the true pinhole frustum answers that
        visually instead of requiring an offline az/el check.

        Each cone reaches to the *current target's* range (not a fixed/scene
        -wide length) — a 90°x70° FOV pyramid stretched to the far edge of
        the whole scene is wide enough at that range to blanket the entire
        plot in translucent color, hiding the very path/balloons it's meant
        to contextualize.
        """
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        valid = [
            i
            for i in range(len(self.x))
            if i < len(self.roll_deg)
            and i < len(self.pitch_deg)
            and i < len(self.yaw_deg)
            and math.isfinite(self.roll_deg[i])
            and math.isfinite(self.pitch_deg[i])
            and math.isfinite(self.yaw_deg[i])
        ]
        if not valid:
            return
        step = max(1, len(valid) // n_samples)
        sample_idx = valid[::step][:n_samples]
        camera = CameraModel(
            hfov_deg=getattr(self, "cam_hfov_deg", 90.0),
            vfov_deg=getattr(self, "cam_vfov_deg", 70.0),
            width_px=640,
            height_px=480,
            azimuth_deg=getattr(self, "cam_mount_azimuth_deg", 0.0),
            elevation_deg=getattr(self, "cam_mount_elevation_deg", 0.0),
        )
        has_tgt = len(self.tgt_x) == len(self.x)
        min_len = 0.05 * max_frustum_len
        import matplotlib.pyplot as plt

        cmap = plt.get_cmap("cool")
        tmin, tmax = self.t[sample_idx[0]], self.t[sample_idx[-1]]
        trange = max(tmax - tmin, 1e-6)
        legend_proxy = None
        for i in sample_idx:
            roll = math.radians(self.roll_deg[i])
            pitch = math.radians(self.pitch_deg[i])
            yaw = math.radians(self.yaw_deg[i])
            frustum_len = max_frustum_len
            pn, pe, pd = self.plane_ned_at(i)
            if has_tgt and math.isfinite(self.tgt_x[i]):
                dist = math.dist(
                    (pn, pe, pd),
                    (self.tgt_x[i], self.tgt_y[i], self.tgt_z[i]),
                )
                if math.isfinite(dist) and dist > 0.0:
                    frustum_len = min(max(dist, min_len), max_frustum_len)
            apex = (pn, pe, -pd)
            color = cmap((self.t[i] - tmin) / trange)
            corners = [
                (
                    pn + dn * frustum_len,
                    pe + de * frustum_len,
                    -(pd + dd * frustum_len),
                )
                for dn, de, dd in frustum_corner_dirs_ned(camera, roll, pitch, yaw)
            ]
            faces = [
                [apex, corners[k], corners[(k + 1) % 4]] for k in range(4)
            ]
            poly = Poly3DCollection(
                faces, facecolor=color, edgecolor=color, alpha=0.10, linewidth=0.5
            )
            ax3d.add_collection3d(poly)
            if legend_proxy is None:
                legend_proxy = plt.Line2D(
                    [], [], color=color, linewidth=1.4,
                    label="camera FOV (sampled, color=time)",
                )
        if legend_proxy is not None:
            handles, labels = ax3d.get_legend_handles_labels()
            handles.append(legend_proxy)
            labels.append(legend_proxy.get_label())
            ax3d._fov_legend = (handles, labels)

    def plot(
        self,
        *,
        title: str = "Flight history",
        save_prefix: Path | str | None = None,
        show: bool | None = None,
    ) -> list[Path]:
        """Draw history figures. ``show=True`` opens an interactive matplotlib window.

        Time-series subplots share the x axis so zoom/pan stays in sync.
        ``show=False`` (tmux savefig) uses Agg and does not need a display.
        """
        if not self.t:
            print("No flight samples to plot.", file=sys.stderr)
            return []
        interactive = show is True
        try:
            import matplotlib

            if interactive:
                _use_gui_backend()
            else:
                matplotlib.use("Agg", force=True)
            import matplotlib.pyplot as plt
        except ImportError as exc:
            print(f"Plot skipped (matplotlib missing): {exc}", file=sys.stderr)
            return []

        fig, _axes, fig3d = self.make_figures(title=title)

        summary = self.summarize_path()
        if summary:
            print(summary)
        prefix: Path | None
        if save_prefix is not None:
            prefix = Path(save_prefix)
        elif show is not False:
            prefix = Path("/tmp") / f"flight_history_{os.getpid()}"
        else:
            prefix = None
        written: list[Path] = []
        if prefix is not None:
            prefix.parent.mkdir(parents=True, exist_ok=True)
            hist_png, traj_png = plot_png_paths(prefix)
            fig.savefig(hist_png, dpi=120)
            fig3d.savefig(traj_png, dpi=120)
            written = [hist_png, traj_png]
            print(f"Saved plots: {hist_png} {traj_png}", flush=True)
        if interactive:
            print("Close the matplotlib windows to continue.", flush=True)
            plt.show(block=True)
        plt.close("all")
        return written
