"""The calibration tasks: registry, gain seeds and reference profiles.

One row per Ben Dickinson flight-control lesson the bench flies, in spec order.
``TASKS`` is the single source of that metadata — family, lesson folder,
duration, the loops a task closes (by gain name) and the signal it tracks — and
``SEEDS`` is the single source of the seed gains. Angles in the lessons are
degrees; every value here is radians and SI.

The profiles (``step``, ``pulse``, ``doublet``) are pure functions of time so a
run can sample them on any grid, and ``step``/``pulse``/``doublet`` onsets land
on the 0.02 s sample grid.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable

import numpy as np

from flightbench.adapters.base import PlaneAdapter
from flightbench.common import FlightbenchError, TrimPoint

if TYPE_CHECKING:  # the run's controller protocol; imported only for the type
    from flightbench.engine import Controller


@dataclass(frozen=True)
class GainSpec:
    """One editable gain: the API name, the UI label, the unit and the seed."""

    name: str
    label: str
    unit: str
    seed: float
    log: bool = False


# The spec's seed list, verbatim: channel-normalized SI values.
SEEDS: dict[str, float] = {
    "kp_theta": 2.0,
    "ki_theta": 0.5,
    "kq": 0.5,
    "lead_zero": 1.0,
    "lead_pole": 1.0,
    "kp_v": 0.05,
    "ki_v": 0.005,
    "kp_gamma": 1.0,
    "ki_gamma": 0.2,
    "kp_nz": 0.05,
    "ki_nz": 0.2,
    "kr": 0.5,
    "tau_w": 1.0,
    "kp_phi": 2.0,
    "ki_phi": 0.1,
    "kp_p": 0.5,
    "k_beta": 1.0,
    "ki_beta": 0.2,
    "k_betadot": 0.5,
    "k_ari": 0.0,
    "kp_psi": 0.5,
    "kr_track": 1.0,
}

# Gains the tuner moves in log space (they are times and break frequencies).
LOG_GAINS = {"tau_w", "lead_zero", "lead_pole"}

# name -> (label, unit). The unit is the channel per unit of the signal the gain
# multiplies, so a pitch-rate weight is seconds and a washout is seconds.
_GAIN_META: dict[str, tuple[str, str]] = {
    "kp_theta": ("Pitch theta proportional", "rad/rad"),
    "ki_theta": ("Pitch theta integral", "rad/(rad s)"),
    "kq": ("Pitch-rate feedback", "s"),
    "lead_zero": ("Lead zero", "rad/s"),
    "lead_pole": ("Lead pole", "rad/s"),
    "kp_v": ("Throttle P (airspeed)", "1/(m/s)"),
    "ki_v": ("Throttle I (airspeed)", "1/m"),
    "kp_gamma": ("Flight-path P (gamma)", "rad/rad"),
    "ki_gamma": ("Flight-path I (gamma)", "rad/(rad s)"),
    "kp_nz": ("Load-factor P (nz)", "1/g"),
    "ki_nz": ("Load-factor I (nz)", "1/(g s)"),
    "kr": ("Yaw-rate feedback", "s"),
    "tau_w": ("Washout time constant", "s"),
    "kp_phi": ("Roll P (phi)", "rad/rad"),
    "ki_phi": ("Roll I (phi)", "rad/(rad s)"),
    "kp_p": ("Roll-rate feedback", "s"),
    "k_beta": ("Sideslip feedback", "rad/rad"),
    "ki_beta": ("Sideslip integral", "rad/(rad s)"),
    "k_betadot": ("Sideslip-rate feedback", "s"),
    "k_ari": ("Aileron-rudder interconnect", "rad/rad"),
    "kp_psi": ("Heading P (psi)", "rad/rad"),
    "kr_track": ("Yaw-rate tracking", "s"),
}


def _spec(name: str) -> GainSpec:
    """The gain spec of a seeded name; a name without a seed is a registry bug."""
    try:
        label, unit = _GAIN_META[name]
    except KeyError:
        raise FlightbenchError(
            f"gain {name!r} has no label/unit or seed; seeded gains: {sorted(SEEDS)}"
        ) from None
    return GainSpec(name, label, unit, SEEDS[name], name in LOG_GAINS)


@dataclass(frozen=True)
class TaskInfo:
    """One calibration task: what it flies, how long, and the loops it closes."""

    id: str
    label: str
    family: str
    lesson: str
    duration_s: float
    description: str
    gains: tuple[GainSpec, ...]
    reference_signal: str | None
    second_trim_factor: float | None = None


def _task(
    id: str,
    label: str,
    family: str,
    lesson: str,
    duration_s: float,
    description: str,
    gain_names: tuple[str, ...],
    reference_signal: str | None,
    second_trim_factor: float | None = None,
) -> TaskInfo:
    """One registry row; ``gain_names`` are expanded into ``GainSpec``s here."""
    return TaskInfo(
        id=id,
        label=label,
        family=family,
        lesson=lesson,
        duration_s=float(duration_s),
        description=description,
        gains=tuple(_spec(name) for name in gain_names),
        reference_signal=reference_signal,
        second_trim_factor=second_trim_factor,
    )


# The eleven tasks, in the design spec's order: seven longitudinal, four lateral.
TASKS: dict[str, TaskInfo] = {
    info.id: info
    for info in (
        _task(
            "pitch_disturbance",
            "Pitch disturbance",
            "longitudinal",
            "2023-06-05_lead-compensation-for-aircraft-pitch-control",
            10.0,
            "Pitch-channel +1 deg step at 1 s with theta_ref held at trim; "
            "how far theta sits from its trim value at T is the measurement.",
            ("kp_theta", "ki_theta", "kq"),
            "theta",
        ),
        _task(
            "short_period_phugoid",
            "Short period and phugoid",
            "longitudinal",
            "2023-06-05_lead-compensation-for-aircraft-pitch-control",
            60.0,
            "Part A of the lead-compensation lesson: an open-loop doublet on the "
            "pitch channel (+2 deg on [1, 2), -2 deg on [2, 3)) with only the "
            "pitch-rate damper closed. kq = 0 is the bare airframe.",
            ("kq",),
            None,
        ),
        _task(
            "lead_pitch",
            "Lead-compensated pitch",
            "longitudinal",
            "2023-06-19_lead-compensated-aircraft-pitch-control-w-codes",
            10.0,
            "theta_ref +5 deg step at 1 s through the lead-compensated pitch loop.",
            ("kp_theta", "ki_theta", "kq", "lead_zero", "lead_pole"),
            "theta",
        ),
        _task(
            "trim_cruise",
            "Trim to cruise",
            "longitudinal",
            "2023-04-24_trim-to-cruise-example",
            40.0,
            "A second wings-level trim at 1.2 vt0 and the same altitude "
            "(2023-04-23_trim-flight-control-fundamentals-section-1-7), commanded "
            "at 1 s as a theta / V / command feed-forward step.",
            ("kp_theta", "ki_theta", "kq", "kp_v", "ki_v"),
            "vt",
            second_trim_factor=1.2,
        ),
        _task(
            "airspeed",
            "Airspeed control",
            "longitudinal",
            "2023-07-10_airspeed-control-now-available",
            40.0,
            "V_ref +10% vt0 step at 1 s with the speed loop on the throttle and "
            "the pitch loop holding theta.",
            ("kp_theta", "ki_theta", "kq", "kp_v", "ki_v"),
            "vt",
        ),
        _task(
            "acceleration",
            "Acceleration control",
            "longitudinal",
            "2023-02-13_acceleration-control-fcf-section-1-5",
            10.0,
            "nz_ref +0.5 g pulse on [1, 4) with the speed loop holding vt0.",
            ("kp_nz", "ki_nz", "kq", "kp_v", "ki_v"),
            "nz",
        ),
        _task(
            "steady_descent",
            "Steady descent",
            "longitudinal",
            "2023-09-09_full-lesson-and-code-now-available",
            30.0,
            "gamma_ref -3 deg step at 1 s on the path loop (lesson section 1.6.4), "
            "with the speed loop holding vt0.",
            ("kp_gamma", "ki_gamma", "kp_theta", "ki_theta", "kq", "kp_v", "ki_v"),
            "gamma",
        ),
        _task(
            "dutch_roll",
            "Dutch-roll control",
            "lateral",
            "2024-12-18_dutch-roll-control-section-1-1",
            15.0,
            "Yaw-channel +2 deg pulse on [1, 1.5) with the yaw damper closed.",
            ("kr", "tau_w"),
            None,
        ),
        _task(
            "turn_coordination",
            "Automatic turn coordination",
            "lateral",
            "2025-03-01_lateral-flight-control-automatic-turn-coordination-section-1",
            20.0,
            "phi_ref +20 deg step at 1 s through the bank loop, the yaw damper "
            "and sideslip feedback.",
            ("kp_phi", "ki_phi", "kp_p", "kr", "tau_w", "k_beta", "k_ari"),
            "phi",
        ),
        _task(
            "sideslip_turn",
            "Sideslip turn",
            "lateral",
            "2025-05-09_new-lesson-sideslip-sideslip-rate-feedback-for-improved-turn",
            20.0,
            "phi_ref +20 deg step at 1 s through the bank loop, the yaw damper and "
            "sideslip feedback with its integral and rate.",
            ("kp_phi", "ki_phi", "kp_p", "kr", "tau_w", "k_beta", "ki_beta", "k_betadot"),
            "phi",
        ),
        _task(
            "yaw_orientation",
            "Yaw orientation control",
            "lateral",
            "2025-12-21_now-available-yaw-orientation-control-multivariable-robustne",
            30.0,
            "psi_ref +30 deg step at 1 s; the bank loop tracks r_cmd = "
            "kp_psi e_psi and the yaw channel tracks r_cmd with kr_track.",
            ("kp_phi", "ki_phi", "kp_p", "k_beta", "k_betadot", "kp_psi", "kr_track"),
            "psi",
        ),
    )
}


def task_ids() -> tuple[str, ...]:
    """The legal task ids, in spec order."""
    return tuple(TASKS)


def get_task(task_id: str) -> TaskInfo:
    """The registry row for a task. Unknown id -> FlightbenchError naming the set."""
    try:
        return TASKS[task_id]
    except KeyError:
        raise FlightbenchError(
            f"unknown task {task_id!r}; legal tasks: {task_ids()}"
        ) from None


def step(t: float, t0: float, amp: float) -> float:
    """``amp`` from ``t0`` on, zero before it."""
    return amp if t >= t0 else 0.0


def pulse(t: float, t0: float, t1: float, amp: float) -> float:
    """``amp`` on the half-open window ``[t0, t1)``, zero outside it."""
    return amp if t0 <= t < t1 else 0.0


def doublet(t: float, t0: float, width: float, amp: float) -> float:
    """``+amp`` on ``[t0, t0 + width)``, ``-amp`` on ``[t0 + width, t0 + 2 width)``."""
    if t < t0:
        return 0.0
    if t < t0 + width:
        return amp
    if t < t0 + 2.0 * width:
        return -amp
    return 0.0


@dataclass(frozen=True)
class TaskContext:
    """The run request a task builder reads: the plane and its trim point(s)."""

    adapter: PlaneAdapter
    trim: TrimPoint
    trim2: TrimPoint | None = None


@dataclass
class TaskSetup:
    """What a task builder hands the engine: controller, disturbance, reference.

    ``reference`` returns the absolute value of the task's reference signal (for
    ``nz`` the load-factor increment in g), so the API can chart the commanded
    trace against the plant's own absolute signals.
    """

    controller: Controller
    disturbance: Callable[[float], np.ndarray] | None = None
    reference: Callable[[float], float] | None = None


__all__ = [
    "GainSpec",
    "LOG_GAINS",
    "SEEDS",
    "TASKS",
    "TaskContext",
    "TaskInfo",
    "TaskSetup",
    "doublet",
    "get_task",
    "pulse",
    "step",
    "task_ids",
]
