"""Wings-level trim grids for the SI F-16."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from f16.model import _adc, _tgear, subf16_derivative
from f16.trim import trim_wings_level
from f16.units import deg_to_rad, ft_to_m, fts_to_ms

VT_MPS = (
    fts_to_ms(120.0 / 0.3048),
    fts_to_ms(140.0 / 0.3048),
    fts_to_ms(502.0),
    fts_to_ms(540.0),
    fts_to_ms(180.0 / 0.3048),
    fts_to_ms(200.0 / 0.3048),
)
ALTITUDE_M = tuple(ft_to_m(ft) for ft in (1000, 1500, 5000, 10000, 20000, 30000, 40000, 50000))
TRIM_DIR = Path(__file__).resolve().parents[2] / "data" / "planes" / "f16"
_MODELS = ("morelli", "stevens")
_GUESSES = (
    np.array([0.2, 0.0, 0.0], dtype=float),
    np.array([0.8, -5.0, 0.15], dtype=float),
)
_MACH_MAX = 0.6
_ALPHA_MIN = deg_to_rad(-10.0)
_ALPHA_MAX = deg_to_rad(45.0)
_ELEV_MIN = deg_to_rad(-25.0)
_ELEV_MAX = deg_to_rad(25.0)
_GCAS = ("gcas_upright", "gcas_inverted", "gcas_long")


def maneuver_vt_mps(maneuver: str) -> float:
    if maneuver == "straight_level":
        return fts_to_ms(502.0)
    if maneuver in _GCAS:
        return fts_to_ms(540.0)
    raise ValueError(f"unknown maneuver {maneuver!r}")


def _corner(table: dict, vt: float, alt: float) -> dict | None:
    for point in table["points"]:
        if abs(float(point["vt_mps"]) - vt) <= 1e-6 and abs(float(point["altitude_m"]) - alt) <= 1e-4:
            return point
    return None


def _pack(alpha: float, throttle: float, elevator: float, vt: float, alt: float):
    x = np.zeros(13, dtype=float)
    u = np.zeros(4, dtype=float)
    x[0] = float(vt)
    x[1] = float(alpha)
    x[4] = float(alpha)
    x[11] = float(alt)
    x[12] = _tgear(float(throttle))
    u[0] = float(throttle)
    u[1] = float(elevator)
    return x, u


def lookup_table(table: dict, vt_mps: float, altitude_m: float):
    vt = float(vt_mps)
    alt = float(altitude_m)
    vts = [float(v) for v in table["vt_mps"]]
    alts = [float(v) for v in table["altitude_m"]]
    if vt < vts[0] - 1e-6 or vt > vts[-1] + 1e-6 or alt < alts[0] - 1e-4 or alt > alts[-1] + 1e-4:
        raise ValueError(f"trim lookup outside axes vt={vt} altitude={alt}")
    hit = _corner(table, vt, alt)
    if hit is not None:
        return _pack(hit["alpha_rad"], hit["throttle"], hit["elevator_rad"], vt, alt)
    i = max(i for i in range(len(vts) - 1) if vts[i] <= vt + 1e-6)
    j = max(j for j in range(len(alts) - 1) if alts[j] <= alt + 1e-4)
    corners = (
        _corner(table, vts[i], alts[j]),
        _corner(table, vts[i + 1], alts[j]),
        _corner(table, vts[i], alts[j + 1]),
        _corner(table, vts[i + 1], alts[j + 1]),
    )
    if any(point is None for point in corners):
        raise ValueError(f"trim lookup missing corner vt={vt} altitude={alt}")
    tx = 0.0 if vts[i + 1] == vts[i] else (vt - vts[i]) / (vts[i + 1] - vts[i])
    ty = 0.0 if alts[j + 1] == alts[j] else (alt - alts[j]) / (alts[j + 1] - alts[j])
    weights = ((1 - tx) * (1 - ty), tx * (1 - ty), (1 - tx) * ty, tx * ty)
    alpha = throttle = elevator = 0.0
    for weight, point in zip(weights, corners, strict=True):
        alpha += weight * float(point["alpha_rad"])
        throttle += weight * float(point["throttle"])
        elevator += weight * float(point["elevator_rad"])
    return _pack(alpha, throttle, elevator, vt, alt)


def load_trim_table(model: str, root: Path | None = None) -> dict:
    if model not in _MODELS:
        raise ValueError(f"model {model!r} is not implemented")
    directory = TRIM_DIR if root is None else Path(root)
    path = directory / f"{model}.json"
    return json.loads(path.read_text())


def lookup_trim(model: str, vt_mps: float, altitude_m: float, root: Path | None = None):
    return lookup_table(load_trim_table(model, root=root), vt_mps, altitude_m)


def trimmed_initial(maneuver: str, aero: str = "morelli", height_m: float | None = None, root: Path | None = None):
    if height_m is None:
        height_m = ft_to_m(1000.0)
    return lookup_trim(aero, maneuver_vt_mps(maneuver), float(height_m), root=root)


def _solve_point(model: str, vt: float, alt: float) -> dict | None:
    mach, _ = _adc(vt, alt)
    if mach > _MACH_MAX:
        return None
    for guess in _GUESSES:
        try:
            x, u, _ = trim_wings_level(vt, alt, model=model, s0=guess)
        except RuntimeError:
            continue
        alpha = float(x[1])
        throttle = float(u[0])
        elevator = float(u[1])
        if not (_ALPHA_MIN <= alpha <= _ALPHA_MAX and 0.0 <= throttle <= 1.0 and _ELEV_MIN <= elevator <= _ELEV_MAX):
            continue
        _, nz, _ = subf16_derivative(x, u, model=model)
        return {
            "vt_mps": vt,
            "altitude_m": alt,
            "mach": mach,
            "alpha_rad": alpha,
            "theta_rad": float(x[4]),
            "power": float(x[12]),
            "throttle": throttle,
            "elevator_rad": elevator,
            "nz": nz,
        }
    return None


def _existing_coefficients(path: Path) -> dict | None:
    if not path.is_file():
        return None
    try:
        data = json.loads(path.read_text())
    except (OSError, UnicodeError, json.JSONDecodeError):
        return None
    if not isinstance(data, dict):
        return None
    coefficients = data.get("coefficients")
    if not isinstance(coefficients, dict):
        return None
    return coefficients


def write_trim_survey(plane: str = "f16", model: str = "morelli", root: Path | None = None) -> Path:
    """Solve one wings-level grid and write data/planes/<plane>/<model>.json."""
    if plane != "f16":
        raise ValueError(f"plane {plane!r} is not implemented")
    if model not in _MODELS:
        raise ValueError(f"model {model!r} is not implemented")
    if root is None:
        directory = Path(__file__).resolve().parents[2] / "data" / "planes" / plane
    else:
        directory = Path(root)
    directory.mkdir(parents=True, exist_ok=True)
    points = []
    for alt in ALTITUDE_M:
        for vt in VT_MPS:
            point = _solve_point(model, float(vt), float(alt))
            if point is not None:
                points.append(point)
    payload = {
        "model": model,
        "constraints": {
            "beta_rad": 0.0,
            "phi_rad": 0.0,
            "gamma_rad": 0.0,
            "p_rad_s": 0.0,
            "q_rad_s": 0.0,
            "r_rad_s": 0.0,
            "aileron_rad": 0.0,
            "rudder_rad": 0.0,
        },
        "vt_mps": list(VT_MPS),
        "altitude_m": list(ALTITUDE_M),
        "points": points,
    }
    path = directory / f"{model}.json"
    coefficients = _existing_coefficients(path)
    if coefficients is not None:
        payload["coefficients"] = coefficients
    path.write_text(json.dumps(payload, indent=2) + "\n")
    return path


def write_trim_tables(root: Path | None = None) -> None:
    directory = TRIM_DIR if root is None else Path(root)
    for model in _MODELS:
        write_trim_survey("f16", model, root=directory)
