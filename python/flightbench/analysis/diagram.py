"""Controller diagrams for the eleven linear-law tasks.

Each task is the loop list in the linear-analysis diagram table. Expressions use
``format_gain`` (six significant figures). The lead, when the task closes one,
sits on the command ahead of the error sum. Rate feedback (``kq``, ``kp_p``,
washout/``kr``) is a wire signed ``-``. ``k_ari`` sums onto yaw on ``+``, the
same sign as ``yaw += k_ari * roll``. Neither path is a loop's ``feedback``.
This module does not import a plant.
"""

from __future__ import annotations

from flightbench.common import FlightbenchError

_TASKS = (
    "pitch_disturbance",
    "short_period_phugoid",
    "lead_pitch",
    "trim_cruise",
    "airspeed",
    "acceleration",
    "steady_descent",
    "dutch_roll",
    "turn_coordination",
    "sideslip_turn",
    "yaw_orientation",
)

def format_gain(value: float) -> str:
    """Six significant figures, shared by every block expression."""
    return format(value, ".6g")


def _pi(kp: float, ki: float) -> str:
    return f"{format_gain(kp)} + {format_gain(ki)}/s"


def _lead(zero: float, pole: float) -> str:
    return (
        f"({format_gain(pole)}/{format_gain(zero)})"
        f"*(s+{format_gain(zero)})/(s+{format_gain(pole)})"
    )


def _washout(tau: float) -> str:
    return f"s/(s+1/{format_gain(tau)})"


def _node(node_id: str, kind: str, label: str, text: str = "") -> dict:
    return {"id": node_id, "kind": kind, "label": label, "text": text}


def _wire(src: str, dst: str, sign: str = "") -> dict:
    return {"from": src, "to": dst, "sign": sign}


def _plant() -> dict:
    return _node("plant", "plant", "airframe")


def _loop(loop_input: str, output: str, feedback: str,
          nodes: list[dict], wires: list[dict]) -> dict:
    return {
        "input": loop_input,
        "output": output,
        "feedback": feedback,
        "nodes": nodes,
        "wires": wires,
    }


def _servo(
    *,
    loop_input: str,
    output: str,
    feedback: str,
    ref_id: str,
    ref_label: str,
    kp: float,
    ki: float,
    rate: tuple[str, float] | None = None,
    lead: tuple[float, float] | None = None,
    extras: list[tuple[dict, str]] | None = None,
    drive_plant: bool = True,
) -> dict:
    """Reference, optional lead, error sum, PI, optional rate, then the plant.

    ``rate`` is ``(gain name, value)`` taken from the plant and subtracted at the
    sum after the PI. ``extras`` are ``(node, sign)`` summed onto the channel
    that drives the plant. ``drive_plant`` is false for a loop whose output is a
    reference for another loop (the path loop); the plant still supplies
    ``feedback``.
    """
    nodes: list[dict] = [_node(ref_id, "input", ref_label)]
    wires: list[dict] = []
    src = ref_id
    if lead is not None:
        zero, pole = lead
        nodes.append(_node("lead", "block", "lead", _lead(zero, pole)))
        wires.append(_wire(src, "lead"))
        src = "lead"
    nodes.append(_node("sum_e", "sum", "sum"))
    wires.append(_wire(src, "sum_e", "+"))
    wires.append(_wire("plant", "sum_e", "-"))
    nodes.append(_node("pi", "block", "PI", _pi(kp, ki)))
    wires.append(_wire("sum_e", "pi"))
    prev = "pi"
    if rate is not None:
        name, gain = rate
        nodes.append(_node(name, "block", name, format_gain(gain)))
        nodes.append(_node("sum_rate", "sum", "sum"))
        wires.append(_wire(prev, "sum_rate", "+"))
        wires.append(_wire("plant", name))
        wires.append(_wire(name, "sum_rate", "-"))
        prev = "sum_rate"
    for node, _sign in extras or []:
        nodes.append(node)
    if extras:
        nodes.append(_node("sum_out", "sum", "sum"))
        wires.append(_wire(prev, "sum_out", "+"))
        for node, sign in extras:
            wires.append(_wire(node["id"], "sum_out", sign))
        prev = "sum_out"
    nodes.append(_plant())
    if drive_plant:
        wires.append(_wire(prev, "plant"))
    return _loop(loop_input, output, feedback, nodes, wires)


def _pitch(gains: dict[str, float], *, lead: bool = False,
           disturbance: str | None = None,
           pitch_ff: float | None = None) -> dict:
    extras: list[tuple[dict, str]] = []
    if disturbance is not None:
        extras.append((_node("disturbance", "block", "disturbance", disturbance), "+"))
    if pitch_ff is not None:
        extras.append((
            _node("pitch_ff", "block", "Δpitch", format_gain(pitch_ff)), "+",
        ))
    return _servo(
        loop_input="theta", output="pitch", feedback="theta",
        ref_id="theta_ref", ref_label="theta_ref",
        kp=gains["kp_theta"], ki=gains["ki_theta"],
        rate=("kq", gains["kq"]),
        lead=(gains["lead_zero"], gains["lead_pole"]) if lead else None,
        extras=extras,
    )


def _speed(gains: dict[str, float], *, throttle_ff: float | None = None) -> dict:
    extras: list[tuple[dict, str]] = []
    if throttle_ff is not None:
        extras.append((
            _node("throttle_ff", "block", "Δthrottle", format_gain(throttle_ff)), "+",
        ))
    return _servo(
        loop_input="vt", output="throttle", feedback="vt",
        ref_id="vt_ref", ref_label="vt_ref",
        kp=gains["kp_v"], ki=gains["ki_v"],
        extras=extras,
    )


def _path(gains: dict[str, float]) -> dict:
    """gamma PI. Its output is theta_ref for the pitch loop, not a channel."""
    return _servo(
        loop_input="gamma", output="theta", feedback="gamma",
        ref_id="gamma_ref", ref_label="gamma_ref",
        kp=gains["kp_gamma"], ki=gains["ki_gamma"],
        drive_plant=False,
    )


def _nz(gains: dict[str, float]) -> dict:
    return _servo(
        loop_input="nz", output="pitch", feedback="nz",
        ref_id="nz_ref", ref_label="nz_ref",
        kp=gains["kp_nz"], ki=gains["ki_nz"],
        rate=("kq", gains["kq"]),
    )


def _bank(gains: dict[str, float]) -> dict:
    return _servo(
        loop_input="phi", output="roll", feedback="phi",
        ref_id="phi_ref", ref_label="phi_ref",
        kp=gains["kp_phi"], ki=gains["ki_phi"],
        rate=("kp_p", gains["kp_p"]),
    )


def _damper(feedback: str, output: str, series: list[dict],
            disturbance: str | None,
            branches: list[tuple[str, list[dict], str]]) -> dict:
    """A measured signal through ``series``, minus, into ``output``.

    ``branches`` are further paths into the same output sum:
    ``(source id, blocks, sign)``. Source ``"plant"`` is the airframe; any other
    source is an input node of that id (a cross-feed or an external command).
    """
    nodes: list[dict] = []
    wires: list[dict] = []
    prev = "plant"
    for block in series:
        nodes.append(block)
        wires.append(_wire(prev, block["id"]))
        prev = block["id"]
    nodes.append(_node("sum_out", "sum", "sum"))
    wires.append(_wire(prev, "sum_out", "-"))
    if disturbance is not None:
        nodes.append(_node("disturbance", "block", "disturbance", disturbance))
        wires.append(_wire("disturbance", "sum_out", "+"))
    for source, blocks, sign in branches:
        if source != "plant":
            nodes.append(_node(source, "input", source))
        cursor = source
        for block in blocks:
            nodes.append(block)
            wires.append(_wire(cursor, block["id"]))
            cursor = block["id"]
        wires.append(_wire(cursor, "sum_out", sign))
    nodes.append(_plant())
    wires.append(_wire("sum_out", "plant"))
    return _loop(feedback, output, feedback, nodes, wires)


def _wash_kr(gains: dict[str, float]) -> list[dict]:
    return [
        _node("washout", "block", "washout", _washout(gains["tau_w"])),
        _node("kr", "block", "kr", format_gain(gains["kr"])),
    ]


def _beta_branch(gains: dict[str, float]) -> tuple[str, list[dict], str]:
    return ("plant", [_node("k_beta", "block", "k_beta", format_gain(gains["k_beta"]))], "+")


def _pitch_disturbance(gains: dict[str, float], **_kwargs) -> list[dict]:
    return [_pitch(gains, disturbance="+1 deg")]


def _short_period(gains: dict[str, float], **_kwargs) -> list[dict]:
    series = [_node("kq", "block", "kq", format_gain(gains["kq"]))]
    return [_damper("q", "pitch", series, "doublet ±2 deg", [])]


def _lead_pitch(gains: dict[str, float], **_kwargs) -> list[dict]:
    return [_pitch(gains, lead=True)]


def _trim_cruise(gains: dict[str, float], *, pitch_ff: float | None = None,
                 throttle_ff: float | None = None, **_kwargs) -> list[dict]:
    return [
        _pitch(gains, pitch_ff=pitch_ff),
        _speed(gains, throttle_ff=throttle_ff),
    ]


def _airspeed(gains: dict[str, float], **_kwargs) -> list[dict]:
    return [_pitch(gains), _speed(gains)]


def _acceleration(gains: dict[str, float], **_kwargs) -> list[dict]:
    return [_nz(gains), _speed(gains)]


def _steady_descent(gains: dict[str, float], **_kwargs) -> list[dict]:
    return [_path(gains), _pitch(gains), _speed(gains)]


def _dutch_roll(gains: dict[str, float], **_kwargs) -> list[dict]:
    return [_damper("r", "yaw", _wash_kr(gains), "+2 deg pulse", [])]


def _turn_coordination(gains: dict[str, float], **_kwargs) -> list[dict]:
    ari = _node("k_ari", "block", "k_ari", format_gain(gains["k_ari"]))
    return [
        _bank(gains),
        _damper("r", "yaw", _wash_kr(gains), None, [
            _beta_branch(gains),
            ("roll", [ari], "+"),
        ]),
    ]


def _sideslip_turn(gains: dict[str, float], **_kwargs) -> list[dict]:
    integ = _node("ki_beta", "block", "ki_beta", f"{format_gain(gains['ki_beta'])}/s")
    rate = _node("k_betadot", "block", "k_betadot", format_gain(gains["k_betadot"]))
    return [
        _bank(gains),
        _damper("r", "yaw", _wash_kr(gains), None, [
            _beta_branch(gains),
            ("plant", [integ], "+"),
            ("plant", [rate], "+"),
        ]),
    ]


def _heading(gains: dict[str, float]) -> dict:
    """kp_psi on the heading error. Its output is r_cmd for the inner loops."""
    nodes = [
        _node("psi_ref", "input", "psi_ref"),
        _node("sum_e", "sum", "sum"),
        _node("kp_psi", "block", "kp_psi", format_gain(gains["kp_psi"])),
        _plant(),
    ]
    wires = [
        _wire("psi_ref", "sum_e", "+"),
        _wire("plant", "sum_e", "-"),
        _wire("sum_e", "kp_psi"),
    ]
    return _loop("r_cmd", "psi", "psi", nodes, wires)


def _bank_from_rcmd(gains: dict[str, float], vt0: float | None) -> dict:
    """phi_ref = clip((vt0/g)·r_cmd, ±30 deg), then the bank PI minus kp_p·p."""
    coord = f"{format_gain(vt0)}/g" if vt0 is not None else "vt0/g"
    nodes = [
        _node("r_cmd", "input", "r_cmd"),
        _node("coord", "block", "vt0/g", coord),
        _node("clip", "block", "clip", "±30 deg"),
        _node("sum_e", "sum", "sum"),
        _node("pi", "block", "PI", _pi(gains["kp_phi"], gains["ki_phi"])),
        _node("kp_p", "block", "kp_p", format_gain(gains["kp_p"])),
        _node("sum_rate", "sum", "sum"),
        _plant(),
    ]
    wires = [
        _wire("r_cmd", "coord"),
        _wire("coord", "clip"),
        _wire("clip", "sum_e", "+"),
        _wire("plant", "sum_e", "-"),
        _wire("sum_e", "pi"),
        _wire("pi", "sum_rate", "+"),
        _wire("plant", "kp_p"),
        _wire("kp_p", "sum_rate", "-"),
        _wire("sum_rate", "plant"),
    ]
    return _loop("phi", "roll", "phi", nodes, wires)


def _yaw_track(gains: dict[str, float]) -> dict:
    """kr_track on (r_cmd − r), plus k_beta·beta and k_betadot·beta_dot_est."""
    nodes = [
        _node("r_cmd", "input", "r_cmd"),
        _node("sum_r", "sum", "sum"),
        _node("kr_track", "block", "kr_track", format_gain(gains["kr_track"])),
        _node("k_beta", "block", "k_beta", format_gain(gains["k_beta"])),
        _node("k_betadot", "block", "k_betadot", format_gain(gains["k_betadot"])),
        _node("sum_yaw", "sum", "sum"),
        _plant(),
    ]
    wires = [
        _wire("r_cmd", "sum_r", "+"),
        _wire("plant", "sum_r", "-"),
        _wire("sum_r", "kr_track"),
        _wire("kr_track", "sum_yaw", "+"),
        _wire("plant", "k_beta"),
        _wire("k_beta", "sum_yaw", "+"),
        _wire("plant", "k_betadot"),
        _wire("k_betadot", "sum_yaw", "+"),
        _wire("sum_yaw", "plant"),
    ]
    return _loop("r", "yaw", "r", nodes, wires)


def _yaw_orientation(gains: dict[str, float], *, vt0: float | None = None,
                     **_kwargs) -> list[dict]:
    return [_heading(gains), _bank_from_rcmd(gains, vt0), _yaw_track(gains)]


_BUILDERS = {
    "pitch_disturbance": _pitch_disturbance,
    "short_period_phugoid": _short_period,
    "lead_pitch": _lead_pitch,
    "trim_cruise": _trim_cruise,
    "airspeed": _airspeed,
    "acceleration": _acceleration,
    "steady_descent": _steady_descent,
    "dutch_roll": _dutch_roll,
    "turn_coordination": _turn_coordination,
    "sideslip_turn": _sideslip_turn,
    "yaw_orientation": _yaw_orientation,
}


def controller_diagram(
    task: str,
    gains: dict[str, float],
    *,
    pitch_ff: float | None = None,
    throttle_ff: float | None = None,
    vt0: float | None = None,
) -> list[dict]:
    """The task's controller loops, with ``gains`` written into the blocks.

    ``pitch_ff`` and ``throttle_ff`` are the trim-cruise channel steps. ``vt0``
    is the yaw-orientation coordination speed in the ``vt0/g`` block. A task
    omits a block its gain list does not close. Unknown ``task`` raises
    ``FlightbenchError``.
    """
    try:
        build = _BUILDERS[task]
    except KeyError:
        raise FlightbenchError(
            f"unknown task {task!r}; legal tasks: {_TASKS}"
        ) from None
    return build(gains, pitch_ff=pitch_ff, throttle_ff=throttle_ff, vt0=vt0)
