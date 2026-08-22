"""JSONC plant files → ``PlantGains``.

Comments (``//``, ``/* */``) are stripped outside of strings, then
``json.loads``. Top-level shared keys merge with ``controllers.<id>``.
PP-only aero/governor fields are required under ``pure_pursuit_quat``;
``race_quat``/``race_euler`` may omit them and inherit those keys from
the sibling ``controllers.pure_pursuit_quat`` block in the same file.
"""
from __future__ import annotations

import json
from dataclasses import fields
from pathlib import Path
from typing import Any

from fw_sitl.flight_setup import DEFAULT_CONTROLLER, KNOWN_CONTROLLER_IDS
from fw_sitl.plant_gains import PlantGains

_ATTITUDE_FROM_ACCEL = frozenset({"polar", "geometric"})

# Shared across plants + controllers (not duplicated under controllers.*).
_TOP_LEVEL_KEYS = frozenset(
    {
        "plant_id",
        "px4_inner",
        "fw_airspd_min",
        "fw_airspd_trim",
        "fw_airspd_max",
        "lookahead_m",
    }
)

# PP aero / governor / attitude_from_accel — required for pure_pursuit_quat;
# optional for race_quat (filled from sibling pure_pursuit_quat block).
_PP_ONLY_KEYS = frozenset(
    {
        "mass_kg",
        "wing_area_m2",
        "cd0",
        "k_induced",
        "cl_alpha",
        "rho_kg_m3",
        "t_max_n",
        "v_stall_mps",
        "pp_gain",
        "thrust_target_frac",
        "v_min_mult",
        "v_recover_mult",
        "v_up_mps_s",
        "attitude_from_accel",
        "alpha_small_rad",
    }
)

_ALLOWED_ROOT_KEYS = _TOP_LEVEL_KEYS | frozenset({"controllers"})


def strip_jsonc(text: str) -> str:
    """Remove ``//`` line and ``/* */`` block comments outside of strings."""
    out: list[str] = []
    i = 0
    n = len(text)
    in_string = False
    escape = False
    while i < n:
        c = text[i]
        if in_string:
            out.append(c)
            if escape:
                escape = False
            elif c == "\\":
                escape = True
            elif c == '"':
                in_string = False
            i += 1
            continue
        if c == '"':
            in_string = True
            out.append(c)
            i += 1
            continue
        if c == "/" and i + 1 < n:
            nxt = text[i + 1]
            if nxt == "/":
                i += 2
                while i < n and text[i] not in "\n\r":
                    i += 1
                continue
            if nxt == "*":
                i += 2
                while i + 1 < n and not (text[i] == "*" and text[i + 1] == "/"):
                    i += 1
                i += 2
                continue
        out.append(c)
        i += 1
    return "".join(out)


def load_plant_jsonc(path: Path) -> dict:
    raw = Path(path).read_text(encoding="utf-8")
    data = json.loads(strip_jsonc(raw))
    if not isinstance(data, dict):
        raise ValueError(f"plant JSONC root must be an object, got {type(data).__name__}")
    return data


def merge_plant_controller(
    data: dict, controller: str = DEFAULT_CONTROLLER
) -> dict:
    """Flatten top-level shared keys + ``controllers[controller]``.

    Missing controller block → ``KeyError``. Unknown controller id →
    ``ValueError``. Controller blocks must not redefine top-level keys.
    For ``race_quat``/``race_euler``, omitted PP-only keys are taken from
    the sibling ``controllers.pure_pursuit_quat`` block (hard error if
    that sibling is missing a required PP key). For ``pure_pursuit_quat``,
    PP-only keys are required in-block.
    """
    cid = str(controller).strip()
    if cid not in KNOWN_CONTROLLER_IDS:
        raise ValueError(
            f"unknown controller {cid!r}; expected one of "
            + "|".join(sorted(KNOWN_CONTROLLER_IDS))
        )

    unexpected = sorted(set(data) - _ALLOWED_ROOT_KEYS)
    if unexpected:
        raise ValueError(
            "plant JSONC has unexpected top-level key(s): "
            + ", ".join(repr(k) for k in unexpected)
            + "; expected only "
            + ", ".join(sorted(_ALLOWED_ROOT_KEYS))
        )

    controllers = data.get("controllers")
    if not isinstance(controllers, dict):
        raise KeyError("plant JSONC missing controllers object")
    if cid not in controllers:
        raise KeyError(f"plant JSONC missing controllers.{cid}")
    block = controllers[cid]
    if not isinstance(block, dict):
        raise ValueError(f"controllers.{cid} must be an object")

    collisions = sorted(set(block) & _TOP_LEVEL_KEYS)
    if collisions:
        raise ValueError(
            f"controllers.{cid} must not redefine top-level key(s): "
            + ", ".join(repr(k) for k in collisions)
        )

    merged: dict[str, Any] = {}
    for key in _TOP_LEVEL_KEYS:
        if key not in data:
            raise KeyError(f"plant JSONC missing top-level {key!r}")
        merged[key] = data[key]
    merged.update(block)

    if cid in ("race_quat", "race_euler"):
        sibling = controllers.get("pure_pursuit_quat")
        if not isinstance(sibling, dict):
            raise KeyError(
                "plant JSONC missing controllers.pure_pursuit_quat "
                f"(required to fill {cid} PP-only fields)"
            )
        for key in _PP_ONLY_KEYS:
            if key in merged:
                continue
            if key not in sibling:
                raise KeyError(
                    "controllers.pure_pursuit_quat missing required PP field "
                    f"{key!r} (needed to fill {cid})"
                )
            merged[key] = sibling[key]
    else:
        for key in _PP_ONLY_KEYS:
            if key not in merged:
                raise KeyError(
                    f"controllers.{cid} missing required PP field {key!r}"
                )
    return merged


def plant_gains_from_dict(data: dict) -> PlantGains:
    """Build ``PlantGains`` from an already-merged flat dict."""
    mode = data["attitude_from_accel"]
    if mode not in _ATTITUDE_FROM_ACCEL:
        raise ValueError(
            f"attitude_from_accel must be 'polar' or 'geometric', got {mode!r}"
        )
    kwargs: dict = {}
    for field in fields(PlantGains):
        kwargs[field.name] = data[field.name]
    kwargs["px4_inner"] = tuple(
        (str(name), float(value)) for name, value in kwargs["px4_inner"]
    )
    kwargs["attitude_from_accel"] = str(mode)
    return PlantGains(**kwargs)
