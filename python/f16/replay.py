"""CSV replay: anim3d frames + native-FDM dicts (replay only)."""
from __future__ import annotations

import math

from f16.units import m_to_ft, ms_to_fts, rad_to_deg

_EARTH_RADIUS_M = 6378137.0
_MPS_TO_KT = 1.9438444924406048
_FG_NET_FDM_VERSION = 24
# Host-order FGNetFDM v24 (408 bytes). Same field order as FlightGear net_fdm.hxx.
_FDM_FMT = (
    "=II"       # version, padding
    "3d"        # longitude, latitude (rad), altitude (m)
    "6f"        # agl, phi, theta, psi, alpha, beta
    "5f"        # phidot, thetadot, psidot, vcas (kt), climb_rate (ft/s)
    "6f"        # v_north, v_east, v_down, v_body_u, v_body_v, v_body_w (ft/s)
    "3f"        # A_X, A_Y, A_Z
    "2f"        # stall_warning, slip_deg
    "I4I"       # num_engines, eng_state[4]
    "36f"       # nine engine float[4] channels
    "I4f"       # num_tanks, fuel_quantity[4]
    "I3I"       # num_wheels, wow[3]
    "9f"        # gear_pos, gear_steer, gear_compression
    "Iif"       # cur_time, warp, visibility
    "10f"       # elevator through spoilers
)


def csv_row_to_fdm(row: dict) -> dict:
    alt_m = -float(row["d_m"])
    out = {
        "alt_m": alt_m,
        "vt_mps": float(row["vt_mps"]),
        "phi_deg": rad_to_deg(float(row["phi"])),
        "theta_deg": rad_to_deg(float(row["theta"])),
        "psi_deg": rad_to_deg(float(row["psi"])),
    }
    assert math.isfinite(out["alt_m"]) and math.isfinite(out["vt_mps"])
    return out


def csv_columns_to_frames(rows: list[dict]) -> dict:
    """Build an AeroBench anim3d result dict from CSV columns.

    Pure: no matplotlib, no window, no socket. ``play_anim`` is what shows it.
    """
    times: list[float] = []
    states: list[list[float]] = []
    modes: list[str] = []
    ps_list: list[float] = []
    nz_list: list[float] = []
    for index, row in enumerate(rows):
        t = _row_time(row)
        times.append(float(index) if t is None else t)
        mode = row.get("mode", "")
        modes.append(str(mode) if mode not in ("", None) else "replay")
        # vt, alpha, beta, phi, theta, psi, P, Q, R, pn, pe, h, pow
        states.append([
            ms_to_fts(float(row["vt_mps"])),
            _opt_float(row, "alpha"),
            _opt_float(row, "beta"),
            float(row["phi"]),
            float(row["theta"]),
            float(row["psi"]),
            0.0,
            0.0,
            0.0,
            m_to_ft(float(row["n_m"])),
            m_to_ft(float(row["e_m"])),
            m_to_ft(-float(row["d_m"])),
            0.0,
        ])
        ps_list.append(0.0)
        nz_list.append(0.0)
    return {
        "times": times,
        "states": states,
        "modes": modes,
        "ps_list": ps_list,
        "Nz_list": nz_list,
    }


def play_anim(csv_path: str, filename: str = "") -> int:
    """Replay CSV columns with AeroBench ``anim3d.make_anim``.

    Imports matplotlib only here, when a window or a saved animation is requested.
    ``filename`` empty plots on screen; a path ending in ``.gif`` or ``.mp4`` saves.
    ``aerobench`` must already import, ``AEROBENCH_CODE`` must point at its code
    directory, or ``compare.PYTHON_REF`` must be that checkout.
    """
    rows = _read_csv(csv_path)
    res = csv_columns_to_frames(rows)
    anim3d = _import_anim3d()
    anim3d.make_anim(res, filename)
    return len(rows)


def send_to_fg(csv_path: str, host: str = "127.0.0.1", port: int = 5500) -> int:
    """Pack native-FDM UDP from CSV rows; returns rows sent. fgfs must already run --fdm=null."""
    import socket as _socket
    import time as _time

    rows = _read_csv(csv_path)
    sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
    sent = 0
    prev = None
    origin_t = None
    origin_wall = None
    try:
        # Connected UDP: a later send raises ConnectionRefusedError after ICMP unreachable.
        sock.connect((host, int(port)))
        for row in rows:
            t = _row_time(row)
            if t is not None and origin_t is not None and origin_wall is not None:
                delay = origin_wall + (t - origin_t) - _time.perf_counter()
                if delay > 0.0:
                    _time.sleep(delay)
            if t is not None and origin_t is None:
                origin_t = t
                origin_wall = _time.perf_counter()
            dt = 0.0
            if prev is not None and t is not None:
                prev_t = _row_time(prev)
                if prev_t is not None:
                    dt = t - prev_t
            packet = _pack_native_fdm(row, prev, dt)
            sock.send(packet)
            sent += 1
            prev = row
    finally:
        sock.close()
    return sent


def _read_csv(csv_path: str) -> list[dict]:
    import csv as _csv

    with open(csv_path, newline="") as handle:
        return list(_csv.DictReader(handle))


def _row_time(row: dict) -> float | None:
    if "t" not in row:
        return None
    raw = row["t"]
    if raw is None or raw == "":
        return None
    return float(raw)


def _opt_float(row: dict, key: str, default: float = 0.0) -> float:
    if key not in row:
        return default
    raw = row[key]
    if raw is None or raw == "":
        return default
    return float(raw)


def _pack_native_fdm(row: dict, prev: dict | None, dt: float) -> bytes:
    """Position from n_m/e_m/alt. Euler and Euler rates in the native packet.

    Geodetic fields stay off the dict ``csv_row_to_fdm`` returns.
    """
    import struct as _struct
    import time as _time

    fdm = csv_row_to_fdm(row)
    alt_m = fdm["alt_m"]
    n_m = float(row["n_m"])
    e_m = float(row["e_m"])
    # NED origin (0, 0) → geodetic (0, 0). lat = n/R, lon = e/R.
    lat = n_m / _EARTH_RADIUS_M
    lon = e_m / _EARTH_RADIUS_M
    phi = float(row["phi"])
    theta = float(row["theta"])
    psi = float(row["psi"])
    alpha = _opt_float(row, "alpha")
    beta = _opt_float(row, "beta")
    phidot = thetadot = psidot = 0.0
    v_north = v_east = v_down = 0.0
    if prev is not None and dt > 0.0:
        phidot = (phi - float(prev["phi"])) / dt
        thetadot = (theta - float(prev["theta"])) / dt
        psidot = (psi - float(prev["psi"])) / dt
        v_north = ms_to_fts((n_m - float(prev["n_m"])) / dt)
        v_east = ms_to_fts((e_m - float(prev["e_m"])) / dt)
        v_down = ms_to_fts((float(row["d_m"]) - float(prev["d_m"])) / dt)
    vt_fps = ms_to_fts(fdm["vt_mps"])
    cos_a = math.cos(alpha)
    cos_b = math.cos(beta)
    v_body_u = vt_fps * cos_a * cos_b
    v_body_v = vt_fps * math.sin(beta)
    v_body_w = vt_fps * math.sin(alpha) * cos_b
    values = [
        _FG_NET_FDM_VERSION, 0,
        lon, lat, alt_m,
        alt_m, phi, theta, psi, alpha, beta,
        phidot, thetadot, psidot, fdm["vt_mps"] * _MPS_TO_KT, -v_down,
        v_north, v_east, v_down, v_body_u, v_body_v, v_body_w,
        0.0, 0.0, 0.0,
        0.0, 0.0,
        1,
        2, 0, 0, 0,
        *([0.0] * 36),
        1,
        0.0, 0.0, 0.0, 0.0,
        3,
        0, 0, 0,
        *([0.0] * 9),
        int(_time.time()), 0, 25000.0,
        *([0.0] * 10),
    ]
    packet = _struct.pack(_FDM_FMT, *values)
    if len(packet) != 408:
        raise RuntimeError(f"FGNetFDM size {len(packet)} != 408")
    return packet


def _import_anim3d():
    """Same import as the AeroBench examples.

    ``AEROBENCH_CODE`` overrides ``compare.PYTHON_REF``. That directory is not left on ``sys.path``.
    """
    try:
        from aerobench.visualize import anim3d
        return anim3d
    except ImportError:
        pass
    import os
    import sys

    code = os.environ.get("AEROBENCH_CODE", "").strip()
    if not code:
        from f16.compare import PYTHON_REF

        code = str(PYTHON_REF)
    if not code or not os.path.isdir(code):
        raise ImportError(
            "No module named 'aerobench'. Set AEROBENCH_CODE to the AeroBench code directory."
        )
    inserted = code not in sys.path
    if inserted:
        sys.path.insert(0, code)
    try:
        from aerobench.visualize import anim3d
    finally:
        if inserted:
            sys.path.remove(code)
    return anim3d
