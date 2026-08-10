"""Record LOCAL_POSITION_NED + ATTITUDE during a flight and plot history."""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass, field

from pymavlink import mavutil


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
    main_mode: list[int | None] = field(default_factory=list)

    # Optional locked path for along/cross-track plots (set by the runner).
    path_origin_xy: tuple[float, float] | None = field(default=None, repr=False)
    path_course_rad: float | None = field(default=None, repr=False)

    _last_att_deg: tuple[float, float, float] | None = field(default=None, repr=False)
    last_att_rad: tuple[float, float, float] | None = field(default=None, repr=False)
    last_pos: tuple[float, float, float] | None = field(default=None, repr=False)
    last_armed: bool | None = field(default=None, repr=False)
    last_main_mode: int | None = field(default=None, repr=False)

    def __len__(self) -> int:
        return len(self.t)

    def request_streams(self, master: mavutil.mavfile, hz: float = 20.0) -> None:
        interval_us = int(1e6 / max(hz, 1.0))
        for msg_id in (
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
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
                type=["LOCAL_POSITION_NED", "ATTITUDE", "HEARTBEAT"],
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
            if mtype == "ATTITUDE":
                roll = float(msg.roll)
                pitch = float(msg.pitch)
                yaw = float(msg.yaw)
                self.last_att_rad = (roll, pitch, yaw)
                self._last_att_deg = (
                    math.degrees(roll),
                    math.degrees(pitch),
                    math.degrees(yaw),
                )
                continue
            # LOCAL_POSITION_NED
            pos = (float(msg.x), float(msg.y), float(msg.z))
            self.last_pos = pos
            roll_d, pitch_d, yaw_d = self._last_att_deg or (float("nan"),) * 3
            self.t.append(time.time() - self.t0)
            self.x.append(pos[0])
            self.y.append(pos[1])
            self.z.append(pos[2])
            self.vx.append(float(msg.vx))
            self.vy.append(float(msg.vy))
            self.vz.append(float(msg.vz))
            self.roll_deg.append(roll_d)
            self.pitch_deg.append(pitch_d)
            self.yaw_deg.append(yaw_d)
            self.main_mode.append(self.last_main_mode)
        return self.last_pos

    def path_series(self) -> tuple[list[float], list[float]] | None:
        """Along-track and signed cross-track (m) vs locked path, if configured."""
        if self.path_origin_xy is None or self.path_course_rad is None:
            return None
        ox, oy = self.path_origin_xy
        c = math.cos(self.path_course_rad)
        s = math.sin(self.path_course_rad)
        along: list[float] = []
        cross: list[float] = []
        for x, y in zip(self.x, self.y):
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

    def plot(self, *, title: str = "Flight history") -> None:
        if not self.t:
            print("No flight samples to plot.", file=sys.stderr)
            return
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 — registers 3d projection
        except ImportError as exc:
            print(f"Plot skipped (matplotlib missing): {exc}", file=sys.stderr)
            return

        path = self.path_series()
        nrows = 4 if path is not None else 3
        fig, axes = plt.subplots(
            nrows, 1, sharex=True, figsize=(10, 9 if path else 8), constrained_layout=True
        )
        fig.suptitle(title)

        ax = axes[0]
        ax.plot(self.t, self.x, label="x (N)")
        ax.plot(self.t, self.y, label="y (E)")
        ax.plot(self.t, self.z, label="z (D)")
        ax.set_ylabel("NED position [m]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")
        ax.set_title(
            "Raw NED: x=North is NOT along-track unless course≈0°. "
            "Sawtooth often = brief OFFBOARD→ALTCTL turns + EKF jumps."
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
        ax.plot(self.t, self.roll_deg, label="roll")
        ax.plot(self.t, self.pitch_deg, label="pitch")
        ax.plot(self.t, self.yaw_deg, label="yaw")
        ax.set_ylabel("Attitude [deg]")
        ax.set_xlabel("Time [s]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")

        # Second figure: 3D path in North-East-Up (Up = -NED z).
        up = [-z for z in self.z]
        fig3d = plt.figure(figsize=(8, 7))
        ax3d = fig3d.add_subplot(111, projection="3d")
        ax3d.plot(self.x, self.y, up, color="C0", linewidth=1.5, label="path")
        ax3d.scatter(
            self.x[0], self.y[0], up[0], color="C2", s=40, label="start", depthshade=True
        )
        ax3d.scatter(
            self.x[-1], self.y[-1], up[-1], color="C3", s=40, label="end", depthshade=True
        )
        ax3d.set_xlabel("North x [m]")
        ax3d.set_ylabel("East y [m]")
        ax3d.set_zlabel("Up [m]")
        ax3d.set_title(f"{title} — 3D trajectory")
        ax3d.legend(loc="best")
        xs, ys, zs = self.x, self.y, up
        mid = (0.5 * (min(xs) + max(xs)), 0.5 * (min(ys) + max(ys)), 0.5 * (min(zs) + max(zs)))
        span = max(
            max(xs) - min(xs),
            max(ys) - min(ys),
            max(zs) - min(zs),
            1.0,
        )
        half = 0.5 * span
        ax3d.set_xlim(mid[0] - half, mid[0] + half)
        ax3d.set_ylim(mid[1] - half, mid[1] + half)
        ax3d.set_zlim(mid[2] - half, mid[2] + half)

        summary = self.summarize_path()
        if summary:
            print(summary)
        print(
            f"Showing flight history ({len(self.t)} samples) + 3D trajectory. "
            "Close the windows to exit."
        )
        plt.show()
