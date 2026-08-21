"""Race / assisted LOS guidance and balloon pass detection."""
from __future__ import annotations

import math
import time
from collections.abc import Sequence
from dataclasses import dataclass, field

from fw_sitl.flight_setup import BalloonSpec, GuidanceSpec
from fw_sitl.path_geometry import wrap_pi

_G_MPS2 = 9.81

ASSISTED_OVERLAY_TEXT = "assisted guidance"
# After a lock, range rising by this much within 4× pass_radius counts as a fly-by.
PASS_CLOSEST_HYST_M = 10.0
PASS_MISS_MULT = 4.0
# Hold last HSV LOS briefly when the blob flickers off (pickle 122330: ~100 ms
# cam_az nan↔finite flipped roll ±max).
VISUAL_HOLD_S = 0.35


def format_ned_pos_line(
    t_s: float,
    pos_ned: tuple[float, float, float],
    ekf_err_h: float | None = None,
) -> str:
    """One-line race pose for stdout and balloon_camera overlay."""
    line = (
        f"t={float(t_s):.1f}s "
        f"x={float(pos_ned[0]):.1f} "
        f"y={float(pos_ned[1]):.1f} "
        f"z={float(pos_ned[2]):.1f}"
    )
    if ekf_err_h is None:
        return line
    if math.isnan(ekf_err_h):
        return f"{line} ekf_err_h=nan"
    return f"{line} ekf_err_h={float(ekf_err_h):.1f}m"


def show_assisted_overlay(*, assisted: bool, in_view: bool) -> bool:
    """True when camera should overlay assisted-guidance indication.

    Follows the control ``assisted`` flag only. A painted balloon with a
    missed HSV blob must not show assisted (``in_view`` is unused).
    """
    return bool(assisted)


def chase_uses_lookat(*, tracker_in_view: bool, on_screen: bool) -> bool:
    """Bank onto chase LOS whenever we have a target (blob or geometric).

    Off-screen used to freeze a path line. If engage already passed the
    balloon, that line never puts the blob in the camera. Always close the
    LOS angle; tracker dir still preferred at the call site when in view.
    """
    return True


def rebase_balloons_to_local_z(
    balloons: tuple[BalloonSpec, ...] | list[BalloonSpec],
    local_z: float,
    *,
    config_ref_z: float | None = None,
) -> tuple[BalloonSpec, ...]:
    """Map config balloon NED into PX4 LOCAL_NED at the current flight altitude.

    Config files historically used ground-relative Z (≈ -80 at cruise AGL). PX4
    local home is near the aircraft (z≈0), so chasing raw config Z commands a
    huge climb and the plane stalls/falls. Preserve relative balloon altitudes
    vs ``config_ref_z`` (default: first balloon Z) and place that cluster at
    ``local_z``.
    """
    specs = tuple(balloons)
    if not specs:
        return specs
    ref = float(specs[0].ned[2] if config_ref_z is None else config_ref_z)
    z0 = float(local_z)
    return tuple(
        BalloonSpec(
            ned=(float(b.ned[0]), float(b.ned[1]), z0 + (float(b.ned[2]) - ref)),
            color=b.color,
            diameter_m=float(b.diameter_m),
        )
        for b in specs
    )


def offset_balloons_ned(
    balloons: tuple[BalloonSpec, ...] | list[BalloonSpec],
    spawn_ned: tuple[float, float, float],
) -> tuple[BalloonSpec, ...]:
    """Express balloon NED relative to the aircraft spawn (PX4 home ≈ spawn)."""
    sn, se, sd = (float(spawn_ned[0]), float(spawn_ned[1]), float(spawn_ned[2]))
    return tuple(
        BalloonSpec(
            ned=(float(b.ned[0]) - sn, float(b.ned[1]) - se, float(b.ned[2]) - sd),
            color=b.color,
            diameter_m=float(b.diameter_m),
        )
        for b in balloons
    )


def translate_balloons_ned(
    balloons: tuple[BalloonSpec, ...] | list[BalloonSpec],
    delta_ned: tuple[float, float, float],
) -> tuple[BalloonSpec, ...]:
    """Add a NED offset to every balloon (live aircraft origin → chase frame)."""
    dn, de, dd = (float(delta_ned[0]), float(delta_ned[1]), float(delta_ned[2]))
    return tuple(
        BalloonSpec(
            ned=(float(b.ned[0]) + dn, float(b.ned[1]) + de, float(b.ned[2]) + dd),
            color=b.color,
            diameter_m=float(b.diameter_m),
        )
        for b in balloons
    )


def balloons_with_xy(
    balloons: tuple[BalloonSpec, ...] | list[BalloonSpec],
    xy: Sequence[tuple[float, float] | None],
) -> tuple[BalloonSpec, ...]:
    """Replace balloon north/east from FG model geodetic; keep existing Z."""
    out: list[BalloonSpec] = []
    for i, b in enumerate(balloons):
        pair = xy[i] if i < len(xy) else None
        if pair is None:
            out.append(b)
            continue
        out.append(
            BalloonSpec(
                ned=(float(pair[0]), float(pair[1]), float(b.ned[2])),
                color=b.color,
                diameter_m=float(b.diameter_m),
            )
        )
    return tuple(out)


def _normalize3(v: tuple[float, float, float]) -> tuple[float, float, float]:
    n = math.hypot(v[0], v[1], math.hypot(v[2], 0.0))
    if n < 1e-9:
        return (1.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def _horizontal_unit(v: tuple[float, float, float]) -> tuple[float, float, float]:
    n = math.hypot(v[0], v[1])
    if n < 1e-9:
        return (1.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, 0.0)


def coordinated_turn_radius_m(speed_mps: float, max_roll_rad: float) -> float:
    """R = v^2 / (g tan φ). φ clamped to >= 1e-3 rad."""
    phi = max(abs(float(max_roll_rad)), 1e-3)
    return float(speed_mps) ** 2 / (_G_MPS2 * math.tan(phi))


def flyby_closing_ahead(
    pos_xy: tuple[float, float],
    balloon_xy: tuple[float, float],
    approach_xy: tuple[float, float],
    *,
    min_cos: float = 0.5,
) -> bool:
    """True if horizontal motion is toward the balloon within ``acos(min_cos)``.

    Fly-by ``d_turn`` uses inbound = LOS to the current balloon. Abeam of it
    (heading away) that LOS vs the next balloon is a ~180° corner, so ``d_turn``
    explodes and chase skips the balloon you can still see. Require closing.
    """
    rel_n = float(balloon_xy[0]) - float(pos_xy[0])
    rel_e = float(balloon_xy[1]) - float(pos_xy[1])
    rel_h = math.hypot(rel_n, rel_e)
    if rel_h < 1e-9:
        return True
    an, ae = float(approach_xy[0]), float(approach_xy[1])
    speed = math.hypot(an, ae)
    if speed < 1e-9:
        return False
    return (rel_n * an + rel_e * ae) >= float(min_cos) * rel_h * speed


def flyby_turn_distance_m(
    pos_xy: tuple[float, float],
    current_xy: tuple[float, float],
    next_xy: tuple[float, float],
    turn_radius_m: float,
) -> float:
    """Standard fly-by: R * tan(θ/2) with θ = |wrap_pi(outbound_az − inbound_az)|.

    Cap θ at π − 1e-3 so tan does not explode on a 180° U-turn; then d_turn
    is large and chase switches to next (still finite).
    """
    inbound_az = math.atan2(current_xy[1] - pos_xy[1], current_xy[0] - pos_xy[0])
    outbound_az = math.atan2(next_xy[1] - current_xy[1], next_xy[0] - current_xy[0])
    theta = abs(wrap_pi(outbound_az - inbound_az))
    theta = min(theta, math.pi - 1e-3)
    return float(turn_radius_m) * math.tan(0.5 * theta)


def _los_to_balloon(
    pos_ned: tuple[float, float, float],
    balloon_ned: tuple[float, float, float],
) -> tuple[float, float, float]:
    return _normalize3(
        (
            balloon_ned[0] - pos_ned[0],
            balloon_ned[1] - pos_ned[1],
            balloon_ned[2] - pos_ned[2],
        )
    )


def race_end_reason(
    *,
    laps_completed: int,
    laps_target: int,
    elapsed_s: float,
    duration_s: float,
    interrupted: bool = False,
) -> str | None:
    """First-wins end reason: ``interrupt`` | ``laps`` | ``duration``, else None."""
    if interrupted:
        return "interrupt"
    if laps_target > 0 and laps_completed >= laps_target:
        return "laps"
    if duration_s > 0.0 and elapsed_s >= duration_s:
        return "duration"
    return None


@dataclass
class RaceGuidance:
    balloons: tuple[BalloonSpec, ...]
    guidance: GuidanceSpec
    target_idx: int = 0
    turn_radius_m: float = 0.0
    last_dir_ned: tuple[float, float, float] = (1.0, 0.0, 0.0)
    last_in_view: bool = False
    assisted: bool = False
    stale_locked: bool = False
    pass_count: int = 0
    laps_completed: int = 0
    last_passed_idx: int | None = None
    last_passed_color: tuple[int, int, int] | None = None
    _seen_track: bool = False
    _last_track_time_s: float | None = None
    _last_assisted_print: float = field(default_factory=lambda: -1e9)
    _last_stale_warn: float = field(default_factory=lambda: -1e9)
    _prev_gate_dot: float | None = None
    _min_dist: float | None = None
    _saw_in_view: bool = False
    _visual_hold_until_s: float = 0.0

    @property
    def active_balloon(self) -> BalloonSpec:
        return self.balloons[self.target_idx]

    @property
    def active_color(self) -> tuple[int, int, int]:
        return self.active_balloon.color

    def balloon_ned(self, idx: int | None = None) -> tuple[float, float, float]:
        i = idx if idx is not None else self.target_idx
        return tuple(self.balloons[i].ned)

    def geometric_los(
        self, pos_ned: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        return _los_to_balloon(pos_ned, self.balloon_ned())

    def mark_track_received(self, now_s: float) -> None:
        """Record that a TrackMessage arrived at ``now_s`` (does not clear stale lock)."""
        self._last_track_time_s = float(now_s)
        self._seen_track = True

    def tick_stale(self, now_s: float, stale_age_s: float) -> None:
        """If no track for ``stale_age_s`` after first receipt → lock assisted forever.

        Stale age is typically ``2.0 / camera.rate_hz``. Warn period uses
        ``guidance.stale_track_warn_s`` (warn cadence only, not the age threshold).
        """
        now = float(now_s)
        if not self.stale_locked:
            if (
                self._seen_track
                and self._last_track_time_s is not None
                and (now - self._last_track_time_s) > float(stale_age_s)
            ):
                self.stale_locked = True
                self.assisted = True

        if not self.stale_locked:
            return

        self.assisted = True
        if now - self._last_stale_warn >= self.guidance.stale_track_warn_s:
            age = (
                now - self._last_track_time_s
                if self._last_track_time_s is not None
                else float("nan")
            )
            print(
                f"WARNING: stale track (age={age:.2f}s > {stale_age_s:.2f}s) — "
                "staying in assisted forever"
            )
            self._last_stale_warn = now

    def update_track(
        self,
        in_view: bool,
        dir_ned: tuple[float, float, float],
        *,
        now_s: float | None = None,
    ) -> None:
        now = float(now_s) if now_s is not None else time.time()
        self._seen_track = True
        if in_view:
            self.last_in_view = True
            self.last_dir_ned = _normalize3(dir_ned)
            self._visual_hold_until_s = now + VISUAL_HOLD_S
            self.assisted = bool(self.stale_locked)
            return
        if now < self._visual_hold_until_s and self.last_in_view:
            # Brief HSV miss: keep camera LOS; do not snap to geometric.
            return
        self.last_in_view = False
        self.last_dir_ned = _normalize3(dir_ned)
        self.assisted = True

    def chase_dir_ned(
        self,
        pos_ned: tuple[float, float, float],
        *,
        sim_time_s: float | None = None,
        approach_xy: tuple[float, float] | None = None,
    ) -> tuple[float, float, float]:
        """Pure 3D LOS: in-view holds last dir; else geometric to balloon.

        When ``turn_radius_m > 0``, switch chase to the *next* balloon once
        horizontal range to the current one is within the fly-by distance
        *and* we are closing on it (approach within 60° of LOS). Pass
        detection still uses the current balloon.
        """
        now = sim_time_s if sim_time_s is not None else time.time()

        if self.turn_radius_m > 0.0 and self.balloons:
            current = self.balloon_ned()
            next_idx = (self.target_idx + 1) % len(self.balloons)
            nxt = self.balloon_ned(next_idx)
            horiz = math.hypot(pos_ned[0] - current[0], pos_ned[1] - current[1])
            d_turn = flyby_turn_distance_m(
                (pos_ned[0], pos_ned[1]),
                (current[0], current[1]),
                (nxt[0], nxt[1]),
                self.turn_radius_m,
            )
            closing = approach_xy is None or flyby_closing_ahead(
                (pos_ned[0], pos_ned[1]),
                (current[0], current[1]),
                approach_xy,
            )
            if horiz <= d_turn and closing:
                return _los_to_balloon(pos_ned, nxt)

        # No camera track yet: geometric LOS to the *active* balloon (not hard-coded
        # 0). After a radius/gate pass without ever receiving track, target_idx advances
        # but we must chase the new balloon — balloon_ned(0) caused orbiting the old one.
        if not self._seen_track:
            return self._assisted_los(pos_ned, self.balloon_ned(), now)

        if self.stale_locked:
            return self._assisted_los(pos_ned, self.balloon_ned(), now)

        if self.last_in_view:
            self.assisted = False
            return self.last_dir_ned

        # Assisted: full 3D geometric LOS to active balloon.
        return self._assisted_los(pos_ned, self.balloon_ned(), now)

    def _assisted_los(
        self,
        pos_ned: tuple[float, float, float],
        balloon: tuple[float, float, float],
        now: float,
    ) -> tuple[float, float, float]:
        self.assisted = True
        if now - self._last_assisted_print >= self.guidance.assisted_print_period_s:
            print(
                f"assisted guidance → balloon {self.target_idx} "
                f"NED=({balloon[0]:.0f},{balloon[1]:.0f},{balloon[2]:.0f})"
            )
            self._last_assisted_print = now
        return _los_to_balloon(pos_ned, balloon)

    def check_pass(
        self,
        pos_ned: tuple[float, float, float],
        *,
        approach_dir_ned: tuple[float, float, float] | None = None,
    ) -> bool:
        """True if within pass_radius, closest-approach fly-by, or gate crossing.

        ``approach_dir_ned`` must be ground track (velocity), not camera LOS:
        LOS·(pos−balloon) is identically −range so a gate never fires.
        """
        balloon = self.balloon_ned()
        dx = pos_ned[0] - balloon[0]
        dy = pos_ned[1] - balloon[1]
        dz = pos_ned[2] - balloon[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if self.last_in_view:
            self._saw_in_view = True
        if self._min_dist is None or dist < self._min_dist:
            self._min_dist = dist
        miss_lim = PASS_MISS_MULT * self.guidance.pass_radius_m
        reason: str | None = None
        if dist <= self.guidance.pass_radius_m:
            reason = "radius"
        elif (
            self._saw_in_view
            and self._min_dist is not None
            and self._min_dist <= miss_lim
            and dist >= self._min_dist + PASS_CLOSEST_HYST_M
        ):
            reason = "closest"
        approach = approach_dir_ned or self.last_dir_ned
        normal = _horizontal_unit(approach)
        gate_dot = dx * normal[0] + dy * normal[1]
        horiz = math.hypot(dx, dy)
        near_gate = horiz <= miss_lim
        if (
            reason is None
            and near_gate
            and self._prev_gate_dot is not None
            and self._prev_gate_dot < 0.0
            and gate_dot >= 0.0
        ):
            reason = "gate"
        self._prev_gate_dot = gate_dot
        if reason is not None:
            self._advance_target(reason)
            return True
        return False

    def _advance_target(self, reason: str) -> None:
        old = self.target_idx
        self.last_passed_idx = old
        self.last_passed_color = self.active_color
        self.target_idx = (self.target_idx + 1) % len(self.balloons)
        self.pass_count += 1
        # Full cycle through all balloons (wrap last → 0) completes one lap.
        if self.target_idx == 0:
            self.laps_completed += 1
        self._prev_gate_dot = None
        self._min_dist = None
        self._saw_in_view = False
        self.last_in_view = False
        self._visual_hold_until_s = 0.0
        print(
            f"Passed balloon {old} ({reason}) → targeting balloon {self.target_idx} "
            f"color=RGB{self.active_color} "
            f"(laps={self.laps_completed}/{self.guidance.laps or '∞'})"
        )

    def horizontal_course_rad(self, dir_ned: tuple[float, float, float]) -> float:
        return math.atan2(dir_ned[1], dir_ned[0])
