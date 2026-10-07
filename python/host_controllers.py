"""Explicit plane-to-controller dispatch for the host-only runners.

The F-16 and the X-31 do not share a controller, and they are not expected to.
The F-16's is a linear LQR inner loop over four channels (throttle, elevator,
aileron, rudder); the X-31's is a nonlinear gain schedule over seven
(aileron, canard, flap, rudder, thrust, thrust pitch, thrust yaw), with
nonlinear dynamic inversion as a second peer. Nothing is inferred from a file
name or a convention: a plane is asked for by name, the controller it is
allowed to run is named here, and asking a plane for a controller that is not
its own raises rather than quietly substituting one.

A controller and an input mode are different things and live in different
tables, because `CONTROLLERS` is the one place a user looks to see what
controllers exist. `INPUT_MODES` holds what is NOT a controller:

* `open_loop` (X-31 only) is the port's Simulink Manual Switch: a constant
  open-loop input to the plant, carried through `simulate._plant_rhs` as
  `simulate._PLANT_COMMAND`. It has no measurement, no feedback and no
  integrator, the scenario demand never reaches it, and it is therefore not a
  controller under any honest name. It used to be listed as one here, under
  the name `plant`, which was a list of controllers containing something that
  closes no loop. It is still runnable -- `resolve_mode` reaches it, and
  `run_x31.py --open-loop` selects it -- and it is now called what it is.

`CONTROL_CHANNELS` is the one place a plane's channel order is written down.
There is no second copy and no list of (command, output) pairs: whether a
column is commanded or flown is the `command` flag of `x31_sim.column_name`,
not an entry in an order. What the order means per plane, and why it is a CSV
order for `x31` but not for `f16`, is in `control_channels`.

`run_f16.py` predates this table and keeps its own `--maneuver` dispatch; the
F-16 row is here so the pair is stated in one place and so a caller can read
off which controller drives which plane.
"""
from __future__ import annotations

CONTROLLERS: dict[str, tuple[str, ...]] = {
    "f16": ("lqr",),
    "x31": ("gain_schedule", "ndi"),
}

DEFAULT_CONTROLLER: dict[str, str] = {
    "f16": "lqr",
    "x31": "gain_schedule",
}

INPUT_MODES: dict[str, tuple[str, ...]] = {
    # `open_loop` is the port's diagram Manual Switch: a constant open-loop
    # input, not a controller. See the module docstring.
    "f16": (),
    "x31": ("open_loop",)
}

CONTROL_CHANNELS: dict[str, tuple[str, ...]] = {
    # The LQR four-vector `f16/llc.py`'s `get_u` returns, which is throttle
    # first: `_U_IMP = [0.1395, -0.7496, 0, 0]` is a throttle fraction, an
    # elevator angle and two neutrals, `u[0]` is the only index driven from the
    # throttle reference `u_ref4[3]`, and `u[1:4]` carries the three rows of
    # the (3, 8) `K_lqr`. NOT a CSV column order -- `run_f16.py`'s header stops
    # at `mode` and writes none of these four names at all. See
    # `control_channels` for what the order means per plane.
    "f16": ("throttle", "elevator", "aileron", "rudder"),
    # `run_x31.py`'s CSV control column order, and it IS a CSV column order:
    # the runner builds its header from the same list, seven commanded columns
    # then seven flown, split by the `command` flag in `column_name`.
    "x31": (
        "aileron",
        "canard",
        "flap",
        "rudder",
        "thrust",
        "thrust_pitch",
        "thrust_yaw",
    ),
}


def planes() -> tuple[str, ...]:
    """Every plane in the dispatch table, in a stable order."""
    return tuple(CONTROLLERS)


def controllers_for(plane: str) -> tuple[str, ...]:
    """The controllers `plane` may run, in a stable order."""
    try:
        return CONTROLLERS[plane]
    except KeyError:
        raise ValueError(
            f"unknown plane {plane!r}; known planes are {', '.join(planes())}"
        ) from None


def control_channels(plane: str) -> tuple[str, ...]:
    """The control channels `plane` exposes, in this table's order.

    What that order means is not the same for both planes, and saying
    otherwise is wrong:

    * `x31` -- it IS the CSV control column order. `run_x31.py` builds its
      header from this list, seven commanded columns then the seven flown,
      with the command/output split carried by the `command` flag of
      `x31_sim.column_name` rather than by a second table of pairs.
    * `f16` -- it is NOT a CSV column order, because `run_f16.py` writes no
      control column at all: its header stops at `mode`. The F-16's real row
      order is the four-vector `f16/llc.py`'s `get_u` returns, which is
      throttle first. That gap is recorded here rather than papered over, and
      `tests/test_x31_integration.py` asserts it from `run_f16._HEADER`.
    """
    controllers_for(plane)
    return CONTROL_CHANNELS[plane]


def input_modes_for(plane: str) -> tuple[str, ...]:
    """The open-loop input modes `plane` offers, in a stable order.

    These are NOT controllers and are deliberately kept out of `CONTROLLERS`.
    """
    controllers_for(plane)
    return INPUT_MODES[plane]


def resolve_mode(plane: str, controller: str | None = None, open_loop: bool = False) -> str:
    """Resolve what actually flies `plane`: a controller name or an input mode.

    Exactly one of `controller` and `open_loop` may be given. `open_loop` names
    the mode rather than a controller, so `--controller open_loop` is a
    refusal and `--open-loop` is the way to ask for it.
    """
    allowed = controllers_for(plane)
    modes = input_modes_for(plane)
    if controller is not None and open_loop:
        raise ValueError(
            f"plane {plane!r} cannot run controller {controller!r} and the "
            f"open_loop input mode at once; pick one"
        )
    if open_loop:
        if "open_loop" not in modes:
            raise ValueError(
                f"plane {plane!r} has no open-loop input mode"
            )
        return "open_loop"
    if controller is None:
        return DEFAULT_CONTROLLER[plane]
    if controller not in allowed:
        raise ValueError(
            f"plane {plane!r} has no controller {controller!r}; "
            f"it runs {', '.join(allowed)}"
            + (f", and its open-loop input modes are {', '.join(modes)}" if modes else "")
        )
    return controller
