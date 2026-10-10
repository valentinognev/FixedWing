"""The four lateral ``linear``-law tasks of the bench.

Each entry of ``BUILDERS`` turns (gains, context) into the controller, disturbance
and reference the engine integrates; each entry of ``ACCEPT`` turns a finished run
into the list of acceptance-row failures. They share one classical controller,
:class:`LateralLaw`, whose topology is fixed by the task: the same object flies the
plane's nonlinear plant and its linearization, and
``engine.closed_loop_matrix`` adds its states to the augmented Jacobian (R10).

The loops, verbatim from the design spec, on the channels ``(throttle, pitch, roll,
yaw)`` (only roll and yaw move on a lateral task):

- **yaw damper** (``dutch_roll``, ``turn_coordination``, ``sideslip_turn``):
  ``yaw = -kr * Washout(tau_w)[r]``.
- **bank** (every task but ``dutch_roll``):
  ``roll = kp_phi*e_phi + ki_phi*integral(e_phi) - kp_p*p`` on ``e_phi = phi_ref - phi``.
- **sideslip** (``turn_coordination``: ``k_beta``; ``sideslip_turn``: ``k_beta``,
  ``ki_beta``; ``yaw_orientation``: ``k_beta``, ``k_betadot``):
  ``yaw += k_beta*beta + ki_beta*integral(beta) + k_betadot*beta_dot_est`` (positive
  yaw reduces positive beta).
- **ARI** (``turn_coordination`` only): ``yaw += k_ari*roll``.
- **yaw tracking** (``yaw_orientation``): ``r_cmd = kp_psi*e_psi``,
  ``phi_ref = clip((vt0/G)*r_cmd, +-PHI_CLIP)`` feeds the bank loop, and
  ``yaw += kr_track*(r_cmd - r)``.

The proportional ``k_beta`` and rate ``k_betadot`` terms are stateless scalars; the
integral ``ki_beta`` rides a ``PI(0, ki_beta)`` block (its ``kp`` is zero, so it is a
pure integrator) and its integral state appears in the controller state only where
the task closes it. ``last_phi_ref`` is a debug attribute holding the bank reference the
controller last commanded, so a caller can read the clipped ``phi_ref`` of the
yaw-orientation loop.

Disturbance and reference follow the spec's task table: ``dutch_roll`` pulses the
yaw channel +2 deg on [1, 1.5); the turn tasks step ``phi_ref`` +20 deg at 1 s; the
yaw task steps ``psi_ref`` +30 deg at 1 s. Channel commands are trim + deviation;
the engine clips to the plane's limits in both runs. Angles are radians; the
acceptance messages report degrees.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

import numpy as np
from f16.units import G_MPS2 as G

from flightbench.common import PHI, PSI, ROLL, YAW
from flightbench.blocks import PI, Washout
from flightbench.tasks.base import SEEDS, TaskSetup, pulse, step

# Command amplitudes and the yaw-orientation bank-reference clip (radians).
TURN_PHI_CMD = math.radians(20.0)
YAW_PSI_CMD = math.radians(30.0)
DUTCH_PULSE = math.radians(2.0)
PHI_CLIP = math.radians(30.0)


@dataclass(frozen=True)
class _Tracking:
    """The yaw-orientation outer loop: heading P and yaw-rate tracking."""

    kp_psi: float
    kr_track: float
    psi_ref: Callable[[float], float]


class LateralLaw:
    """The ``linear`` lateral controller shared by the four lateral tasks.

    A block is present only where its task closes it: ``roll_pi`` (one state) for
    the bank loop, ``washout`` (one state) for the yaw damper, and ``beta_integ``
    (one state) for the sideslip integral. The controller state vector ``xc`` lists
    those blocks' states in that order; ``k_beta`` / ``k_betadot`` / ``k_ari`` are
    stateless scalars, active when their value is not ``None``.
    """

    def __init__(self, *, vt0: float, roll_pi=None, kp_p: float = 0.0,
                 washout=None, kr: float = 0.0, k_beta=None, beta_integ=None,
                 k_betadot=None, k_ari=None, phi_ref=None,
                 tracking: _Tracking | None = None) -> None:
        self.vt0 = float(vt0)
        self.roll_pi = roll_pi
        self.kp_p = float(kp_p)
        self.washout = washout
        self.kr = float(kr)
        self.k_beta = None if k_beta is None else float(k_beta)
        self.beta_integ = beta_integ
        self.k_betadot = None if k_betadot is None else float(k_betadot)
        self.k_ari = None if k_ari is None else float(k_ari)
        self._phi_ref = phi_ref
        self._tracking = tracking
        self._blocks = []
        if roll_pi is not None:
            self._blocks.append(("roll", roll_pi))
        if washout is not None:
            self._blocks.append(("wash", washout))
        if beta_integ is not None:
            self._blocks.append(("beta", beta_integ))
        self.n_states = int(sum(block.n_states for _, block in self._blocks))
        self.needs_nz = False
        # Debug attribute: the bank reference last commanded (clipped when tracking).
        self.last_phi_ref = 0.0
        self._last_r_cmd = 0.0

    def _split(self, xc: np.ndarray) -> dict:
        """The controller state, sliced per block, keyed by the block's name."""
        xc = np.asarray(xc, dtype=float)
        parts: dict = {}
        offset = 0
        for name, block in self._blocks:
            parts[name] = xc[offset:offset + block.n_states]
            offset += block.n_states
        return parts

    def _bank_reference(self, t: float, y: dict) -> float:
        """The bank loop's reference angle; sets ``last_phi_ref`` (clip when tracking)."""
        if self._tracking is not None:
            e_psi = self._tracking.psi_ref(t) - y["psi"]
            r_cmd = self._tracking.kp_psi * e_psi
            self._last_r_cmd = r_cmd
            phi_ref = float(np.clip((self.vt0 / G) * r_cmd, -PHI_CLIP, PHI_CLIP))
        elif self._phi_ref is not None:
            phi_ref = float(self._phi_ref(t, y))
        else:
            phi_ref = 0.0
        self.last_phi_ref = phi_ref
        return phi_ref

    def derivative(self, t: float, xc: np.ndarray, y: dict) -> np.ndarray:
        """Controller-state derivative: integrate the bank error, ``r`` and ``beta``."""
        parts = self._split(xc)
        pieces = []
        if self.roll_pi is not None:
            e_phi = self._bank_reference(t, y) - y["phi"]
            pieces.append(self.roll_pi.derivative(parts["roll"], e_phi))
        if self.washout is not None:
            pieces.append(self.washout.derivative(parts["wash"], y["r"]))
        if self.beta_integ is not None:
            pieces.append(self.beta_integ.derivative(parts["beta"], y["beta"]))
        return np.zeros(0) if not pieces else np.concatenate(pieces)

    def output(self, t: float, xc: np.ndarray, y: dict) -> np.ndarray:
        """Four channel deviations: roll from the bank loop, yaw from the rest."""
        parts = self._split(xc)
        phi_ref = self._bank_reference(t, y)
        roll = 0.0
        if self.roll_pi is not None:
            e_phi = phi_ref - y["phi"]
            roll = self.roll_pi.output(parts["roll"], e_phi) - self.kp_p * y["p"]
        yaw = 0.0
        if self.washout is not None:
            yaw += -self.kr * self.washout.output(parts["wash"], y["r"])
        if self.k_beta is not None:
            yaw += self.k_beta * y["beta"]
        if self.beta_integ is not None:
            yaw += self.beta_integ.output(parts["beta"], y["beta"])
        if self.k_betadot is not None:
            yaw += self.k_betadot * y["beta_dot_est"]
        if self.k_ari is not None:
            yaw += self.k_ari * roll
        if self._tracking is not None:
            yaw += self._tracking.kr_track * (self._last_r_cmd - y["r"])
        u = np.zeros(4)
        u[ROLL] = roll
        u[YAW] = yaw
        return u


def _g(gains: dict, name: str) -> float:
    """A gain's value, the seed as the fallback for a name the caller omitted."""
    return float(gains.get(name, SEEDS[name]))


def _yaw_pulse(t: float) -> np.ndarray:
    """+2 deg on the yaw channel over [1, 1.5) s, nothing else."""
    u = np.zeros(4)
    u[YAW] = pulse(t, 1.0, 1.5, DUTCH_PULSE)
    return u


def _phi_reference(ctx, command: float) -> Callable[[float], float]:
    """Absolute phi reference: the trim bank angle plus the commanded step."""
    phi0 = float(ctx.trim.x[PHI])
    return lambda t: phi0 + step(t, 1.0, command)


def _build_dutch_roll(gains: dict, ctx) -> TaskSetup:
    """The yaw damper only: ``yaw = -kr * Washout(tau_w)[r]``, a +2 deg yaw pulse."""
    law = LateralLaw(vt0=ctx.trim.vt_mps, washout=Washout(_g(gains, "tau_w")),
                     kr=_g(gains, "kr"))
    return TaskSetup(controller=law, disturbance=_yaw_pulse, reference=None)


def _build_turn_coordination(gains: dict, ctx) -> TaskSetup:
    """Bank + yaw damper + sideslip (``k_beta``) + ARI, tracking a +20 deg phi step."""
    law = LateralLaw(
        vt0=ctx.trim.vt_mps,
        roll_pi=PI(_g(gains, "kp_phi"), _g(gains, "ki_phi")),
        kp_p=_g(gains, "kp_p"),
        washout=Washout(_g(gains, "tau_w")), kr=_g(gains, "kr"),
        k_beta=_g(gains, "k_beta"), k_ari=_g(gains, "k_ari"),
        phi_ref=lambda t, y: step(t, 1.0, TURN_PHI_CMD),
    )
    return TaskSetup(controller=law, reference=_phi_reference(ctx, TURN_PHI_CMD))


def _build_sideslip_turn(gains: dict, ctx) -> TaskSetup:
    """Bank + yaw damper + sideslip (``k_beta``, ``ki_beta``, ``k_betadot``)."""
    law = LateralLaw(
        vt0=ctx.trim.vt_mps,
        roll_pi=PI(_g(gains, "kp_phi"), _g(gains, "ki_phi")),
        kp_p=_g(gains, "kp_p"),
        washout=Washout(_g(gains, "tau_w")), kr=_g(gains, "kr"),
        k_beta=_g(gains, "k_beta"), beta_integ=PI(0.0, _g(gains, "ki_beta")),
        k_betadot=_g(gains, "k_betadot"),
        phi_ref=lambda t, y: step(t, 1.0, TURN_PHI_CMD),
    )
    return TaskSetup(controller=law, reference=_phi_reference(ctx, TURN_PHI_CMD))


def _build_yaw_orientation(gains: dict, ctx) -> TaskSetup:
    """Bank + sideslip (``k_beta``, ``k_betadot``) + yaw tracking, on a +30 deg psi step."""
    tracking = _Tracking(kp_psi=_g(gains, "kp_psi"), kr_track=_g(gains, "kr_track"),
                         psi_ref=lambda t: step(t, 1.0, YAW_PSI_CMD))
    law = LateralLaw(
        vt0=ctx.trim.vt_mps,
        roll_pi=PI(_g(gains, "kp_phi"), _g(gains, "ki_phi")),
        kp_p=_g(gains, "kp_p"),
        k_beta=_g(gains, "k_beta"), k_betadot=_g(gains, "k_betadot"),
        tracking=tracking,
    )
    return TaskSetup(controller=law,
                     reference=(lambda t: float(ctx.trim.x[PSI])
                                + step(t, 1.0, YAW_PSI_CMD)))


BUILDERS = {
    "dutch_roll": _build_dutch_roll,
    "turn_coordination": _build_turn_coordination,
    "sideslip_turn": _build_sideslip_turn,
    "yaw_orientation": _build_yaw_orientation,
}


def _final(trace, name: str) -> float:
    """The last sample of a series (absolute value)."""
    return float(trace.series[name][-1])


def _max_beta_degrees(trace) -> float:
    """``max|beta|`` over the run, in degrees."""
    beta = np.asarray(trace.series["beta"], dtype=float)
    return math.degrees(float(np.max(np.abs(beta)))) if beta.size else 0.0


def _accept_dutch_roll(trace, ctx, closed_loop_modes) -> list[str]:
    """Closed-loop Dutch-roll zeta >= 0.3 (linear model) and ``|beta(T)| <= 0.2 deg``."""
    failures: list[str] = []
    dutch = next((m for m in closed_loop_modes if m.name == "dutch_roll"), None)
    if dutch is None:
        failures.append("no closed-loop Dutch-roll mode on the linear model")
    elif dutch.zeta < 0.3:
        failures.append(f"closed-loop Dutch-roll zeta = {dutch.zeta:.3f} below 0.300")
    beta_T = math.degrees(abs(_final(trace, "beta")))
    if beta_T > 0.2:
        failures.append(f"|beta(T)| = {beta_T:.3f} deg exceeds 0.200 deg")
    return failures


def _accept_bank_turn(trace, phi_tol_deg: float, beta_limit_deg: float) -> list[str]:
    """``|phi(T) - 20 deg|`` within tolerance and ``max|beta|`` within its limit."""
    failures: list[str] = []
    phi_T = math.degrees(_final(trace, "phi"))
    want = math.degrees(TURN_PHI_CMD)
    if abs(phi_T - want) > phi_tol_deg:
        failures.append(
            f"phi(T) = {phi_T:.3f} deg, want {want:.3f} deg within {phi_tol_deg:.3f} deg")
    max_beta = _max_beta_degrees(trace)
    if max_beta > beta_limit_deg:
        failures.append(
            f"max|beta| = {max_beta:.3f} deg exceeds {beta_limit_deg:.3f} deg")
    return failures


def _accept_turn_coordination(trace, ctx, closed_loop_modes) -> list[str]:
    """``|phi(T) - 20 deg| <= 1 deg`` and ``max|beta| <= 2 deg``."""
    return _accept_bank_turn(trace, 1.0, 2.0)


def _accept_sideslip_turn(trace, ctx, closed_loop_modes) -> list[str]:
    """``|phi(T) - 20 deg| <= 1 deg`` and ``max|beta| <= 1 deg``."""
    return _accept_bank_turn(trace, 1.0, 1.0)


def _accept_yaw_orientation(trace, ctx, closed_loop_modes) -> list[str]:
    """``|psi(T) - 30 deg| <= 1.5 deg`` and ``max|beta| <= 2 deg``."""
    failures: list[str] = []
    psi_T = math.degrees(_final(trace, "psi"))
    want = math.degrees(YAW_PSI_CMD)
    if abs(psi_T - want) > 1.5:
        failures.append(
            f"psi(T) = {psi_T:.3f} deg, want {want:.3f} deg within 1.500 deg")
    max_beta = _max_beta_degrees(trace)
    if max_beta > 2.0:
        failures.append(f"max|beta| = {max_beta:.3f} deg exceeds 2.000 deg")
    return failures


ACCEPT = {
    "dutch_roll": _accept_dutch_roll,
    "turn_coordination": _accept_turn_coordination,
    "sideslip_turn": _accept_sideslip_turn,
    "yaw_orientation": _accept_yaw_orientation,
}


__all__ = ["ACCEPT", "BUILDERS", "LateralLaw"]
