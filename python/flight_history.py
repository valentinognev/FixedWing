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

    _last_att_deg: tuple[float, float, float] | None = field(default=None, repr=False)
    last_pos: tuple[float, float, float] | None = field(default=None, repr=False)

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
        """Drain position + attitude; append samples on each LOCAL_POSITION_NED."""
        while True:
            msg = master.recv_match(
                type=["LOCAL_POSITION_NED", "ATTITUDE"],
                blocking=False,
            )
            if msg is None:
                break
            if msg.get_srcSystem() not in (0, master.target_system):
                continue
            mtype = msg.get_type()
            if mtype == "ATTITUDE":
                self._last_att_deg = (
                    math.degrees(float(msg.roll)),
                    math.degrees(float(msg.pitch)),
                    math.degrees(float(msg.yaw)),
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
        return self.last_pos

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

        fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8), constrained_layout=True)
        fig.suptitle(title)

        ax = axes[0]
        ax.plot(self.t, self.x, label="x (N)")
        ax.plot(self.t, self.y, label="y (E)")
        ax.plot(self.t, self.z, label="z (D)")
        ax.set_ylabel("Position [m]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")

        ax = axes[1]
        ax.plot(self.t, self.vx, label="vx")
        ax.plot(self.t, self.vy, label="vy")
        ax.plot(self.t, self.vz, label="vz")
        ax.set_ylabel("Velocity [m/s]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")

        ax = axes[2]
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
        # Equal-ish aspect so straight legs are not visually skewed.
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

        print(
            f"Showing flight history ({len(self.t)} samples) + 3D trajectory. "
            "Close the windows to exit."
        )
        plt.show()
