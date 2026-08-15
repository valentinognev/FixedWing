"""Map chase LOS / body velocity to LOCAL_NED path setpoints."""
from __future__ import annotations

import math
from dataclasses import dataclass

from pymavlink import mavutil

from fw_sitl.mavlink_io import send_path_setpoint
from fw_sitl.path_geometry import ned_velocity_from_course, wrap_pi

# Cap |z_hold − last command| so a long LOS lookahead cannot jump hundreds of metres.
DEFAULT_MAX_ALT_STEP_M = 40.0
# |heading error| at/above this → hold current altitude (level turn); below → blend toward aim Z.
DEFAULT_ALT_PRESERVE_HEADING_ERR_RAD = math.radians(20.0)


@dataclass
class BodyCmdBridge:
    lookahead_m: float
    speed_mps: float
    max_alt_step_m: float = DEFAULT_MAX_ALT_STEP_M
    alt_preserve_heading_err_rad: float = DEFAULT_ALT_PRESERVE_HEADING_ERR_RAD
    _alt_hold_z: float | None = None

    def aim_point_ned(
        self,
        pos_ned: tuple[float, float, float],
        dir_ned: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        """Point along LOS at lookahead_m from current position."""
        return (
            pos_ned[0] + dir_ned[0] * self.lookahead_m,
            pos_ned[1] + dir_ned[1] * self.lookahead_m,
            pos_ned[2] + dir_ned[2] * self.lookahead_m,
        )

    def chase_geometry(
        self,
        pos_ned: tuple[float, float, float],
        dir_ned: tuple[float, float, float],
        yaw_rad: float | None = None,
    ) -> tuple[tuple[float, float, float], float, float]:
        """Horizontal course from aim XY; altitude-preserving turn when heading error is large.

        Course always comes from the XY component of the 3D aim. When ``yaw_rad`` is
        set and |course − yaw| is large, ``z_hold`` stays at the last command so the
        turn does not couple balloon-climb ΔZ into a spiral. When nearly aligned or
        ``yaw_rad`` is None, ``z_hold`` tracks clamped aim Z.

        Altitude clamp is versus the last command, not current ``pos_z``. Rebasing
        to ``pos_z`` every tick ratchets a sink so TECS never sees a growing climb
        error. If the aircraft is below the command, ``z_hold`` must not rise toward
        it.

        Returns (aim_ned, course_rad, z_hold).
        """
        aim = self.aim_point_ned(pos_ned, dir_ned)
        course = math.atan2(aim[1] - pos_ned[1], aim[0] - pos_ned[0])
        pos_z = float(pos_ned[2])
        z_aim = float(aim[2])
        if self._alt_hold_z is None:
            self._alt_hold_z = pos_z
        ref_z = float(self._alt_hold_z)
        max_dz = float(self.max_alt_step_m)
        if max_dz > 0.0:
            z_aim = max(ref_z - max_dz, min(ref_z + max_dz, z_aim))

        z_hold = z_aim
        if yaw_rad is not None:
            heading_err = abs(wrap_pi(course - float(yaw_rad)))
            thresh = float(self.alt_preserve_heading_err_rad)
            if thresh <= 0.0 or heading_err >= thresh:
                z_hold = ref_z
            else:
                # 1 = aligned (full aim Z), 0 = at threshold (preserve altitude).
                alpha = 1.0 - (heading_err / thresh)
                z_hold = ref_z + alpha * (z_aim - ref_z)

        # Below the command (NED pos_z > ref): do not raise z_hold toward the sink.
        if z_hold > ref_z and pos_z > ref_z:
            z_hold = ref_z

        self._alt_hold_z = z_hold
        return aim, course, z_hold

    def send_chase_setpoint(
        self,
        master: mavutil.mavfile,
        pos_ned: tuple[float, float, float],
        dir_ned: tuple[float, float, float],
        frame: int,
        yaw_rad: float | None = None,
    ) -> tuple[float, float, float]:
        """Stream FW path setpoint: course from aim XY, altitude via chase_geometry."""
        _aim, course, z_hold = self.chase_geometry(pos_ned, dir_ned, yaw_rad=yaw_rad)
        origin_xy = (pos_ned[0], pos_ned[1])
        vx, vy, vz = ned_velocity_from_course(self.speed_mps, course)
        send_path_setpoint(
            master,
            (pos_ned[0], pos_ned[1]),
            z_hold,
            origin_xy,
            course,
            self.lookahead_m,
            vx,
            vy,
            vz,
            frame,
        )
        return (vx, vy, vz)
