"""Load a host airplane's mass, inertia, and thrust from <model>.json(c)."""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from f16.aero_data import _MODELS, load_aero_coefficients, resolve_model_file, strip_jsonc_comments

_AIRCRAFT_KEYS = (
    "mass_kg", "s_m2", "b_m", "cbar_m", "xcg", "xcg_ref",
    "ixx", "iyy", "izz", "ixz", "he", "t_max_n", "tau_s", "v_ref_mps",
)
_POSITIVE_KEYS = (
    "mass_kg", "s_m2", "b_m", "cbar_m", "ixx", "iyy", "izz", "tau_s",
)
_STATE_KEYS = (
    "vt_mps", "alpha_rad", "beta_rad", "phi_rad", "theta_rad", "psi_rad",
    "p_rad_s", "q_rad_s", "r_rad_s", "pn_m", "pe_m", "alt_m", "power",
)
_CONTROL_KEYS = ("throttle", "elevator_rad", "aileron_rad", "rudder_rad")


@dataclass(frozen=True)
class Aircraft:
    mass_kg: float
    s_m2: float
    b_m: float
    cbar_m: float
    xcg: float
    xcg_ref: float
    ixx: float
    iyy: float
    izz: float
    ixz: float
    he: float
    t_max_n: float
    tau_s: float
    v_ref_mps: float
    model: str
    c1: float
    c2: float
    c3: float
    c4: float
    c5: float
    c6: float
    c7: float
    c8: float
    c9: float
    coeff_dir: Path
    initial: dict
    controls: dict


def load_aircraft(plane: str, model: str = "morelli", root: Path | None = None) -> Aircraft:
    if model not in _MODELS:
        raise ValueError(f"model {model!r} not implemented")
    directory = _plane_directory(plane, root)
    payload = json.loads(strip_jsonc_comments(resolve_model_file(directory, model).read_text()))
    if not isinstance(payload, dict):
        raise ValueError(f"{model}.json must be an object")
    body = _require_object(payload, "aircraft")
    values = _require_keys(body, _AIRCRAFT_KEYS)
    for name in _POSITIVE_KEYS:
        if values[name] <= 0.0:
            raise ValueError(f"{name} must be > 0")
    if values["t_max_n"] < 0.0:
        raise ValueError("t_max_n must be >= 0")
    initial = _require_object(payload, "initial")
    controls = _require_object(payload, "controls")
    _require_keys(initial, _STATE_KEYS)
    _require_keys(controls, _CONTROL_KEYS)
    load_aero_coefficients(model, root=directory)
    c1, c2, c3, c4, c5, c6, c7, c8, c9 = _inertia_coefficients(
        values["ixx"], values["iyy"], values["izz"], values["ixz"],
    )
    return Aircraft(
        mass_kg=values["mass_kg"],
        s_m2=values["s_m2"],
        b_m=values["b_m"],
        cbar_m=values["cbar_m"],
        xcg=values["xcg"],
        xcg_ref=values["xcg_ref"],
        ixx=values["ixx"],
        iyy=values["iyy"],
        izz=values["izz"],
        ixz=values["ixz"],
        he=values["he"],
        t_max_n=values["t_max_n"],
        tau_s=values["tau_s"],
        v_ref_mps=values["v_ref_mps"],
        model=model,
        c1=c1,
        c2=c2,
        c3=c3,
        c4=c4,
        c5=c5,
        c6=c6,
        c7=c7,
        c8=c8,
        c9=c9,
        coeff_dir=directory,
        initial=initial,
        controls=controls,
    )


def thrust_n(power: float, vt: float, aircraft: Aircraft) -> float:
    scale = max(0.0, 1.0 - vt / aircraft.v_ref_mps)
    return float(power) * aircraft.t_max_n * scale


def state_vector(payload_initial: dict) -> np.ndarray:
    return np.array([float(payload_initial[key]) for key in _STATE_KEYS], dtype=float)


def control_vector(payload_controls: dict) -> np.ndarray:
    return np.array([float(payload_controls[key]) for key in _CONTROL_KEYS], dtype=float)


def _plane_directory(plane: str, root: Path | None) -> Path:
    if root is None:
        return Path(__file__).resolve().parents[2] / "data" / "planes" / plane
    return Path(root) / plane


def _require_object(payload: dict, name: str) -> dict:
    value = payload.get(name)
    if not isinstance(value, dict):
        raise ValueError(f"missing {name}")
    return value


def _require_keys(body: dict, keys: tuple[str, ...]) -> dict[str, float]:
    values: dict[str, float] = {}
    for key in keys:
        if key not in body:
            raise ValueError(f"missing {key}")
        values[key] = float(body[key])
    return values


def _inertia_coefficients(
    ixx: float, iyy: float, izz: float, ixz: float,
) -> tuple[float, float, float, float, float, float, float, float, float]:
    gamma = ixx * izz - ixz * ixz
    if gamma <= 0.0:
        raise ValueError(f"gamma must be > 0, got {gamma}")
    c1 = ((iyy - izz) * izz - ixz * ixz) / gamma
    c2 = (ixx - iyy + izz) * ixz / gamma
    c3 = izz / gamma
    c4 = ixz / gamma
    c5 = (izz - ixx) / iyy
    c6 = ixz / iyy
    c7 = 1.0 / iyy
    c8 = (ixx * (ixx - iyy) + ixz * ixz) / gamma
    c9 = ixx / gamma
    return c1, c2, c3, c4, c5, c6, c7, c8, c9
