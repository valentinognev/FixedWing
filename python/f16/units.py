"""SI wall for the in-tree F-16. Only converter in the package."""
from __future__ import annotations

import math

import numpy as np

M_PER_FT = 0.3048
FT_PER_M = 1.0 / M_PER_FT
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi
AEROBENCH_RTOD = 57.29578
LBF_TO_N = 4.4482216152605
SLUG_TO_KG = LBF_TO_N / M_PER_FT
G_MPS2 = 32.17 * M_PER_FT
S_M2 = 300.0 * M_PER_FT * M_PER_FT
B_M = 30.0 * M_PER_FT
CBAR_M = 11.32 * M_PER_FT
XA_M = 15.0 * M_PER_FT
HE_KGM2 = 160.0 * SLUG_TO_KG * M_PER_FT ** 2
RM_PER_KG = 1.57e-3 / SLUG_TO_KG
_I_KGM2_PER_SLUGFT2 = SLUG_TO_KG * M_PER_FT ** 2
C3 = 1.055e-4 / _I_KGM2_PER_SLUGFT2
C4 = 1.642e-6 / _I_KGM2_PER_SLUGFT2
C7 = 1.792e-5 / _I_KGM2_PER_SLUGFT2
C9 = 1.587e-5 / _I_KGM2_PER_SLUGFT2
RHO0_KG_M3 = 2.377e-3 * SLUG_TO_KG / M_PER_FT ** 3
LAPSE_PER_M = 0.703e-5 / M_PER_FT
T0_K = 519.0 * 5.0 / 9.0
T_STRAT_K = 390.0 * 5.0 / 9.0
H_STRAT_M = 35000.0 * M_PER_FT
R_AIR = 1716.3 * M_PER_FT ** 2 * 9.0 / 5.0
H_THRUST_STEP_M = 10000.0 * M_PER_FT
ALT_FLOOR_M = 0.01 * M_PER_FT
MIN_H_DIFF_M = 50.0 * M_PER_FT
FINAL_RMS_LIMIT = 5.0
GCAS_FLOOR_M = 1000.0 * M_PER_FT
VT_GCAS_MPS = 540.0 * M_PER_FT
H_GCAS_M = 1000.0 * M_PER_FT
K_ALT_PER_M = 0.01 / M_PER_FT
K_VT_PER_MPS = 0.5 / M_PER_FT
K_GAMMA_PER_RAD = 15.0
SL_H_BAND_M = 15.0 * M_PER_FT
SL_VT_BAND_MPS = 8.0 * M_PER_FT
SL_MIN_H_M = 900.0 * M_PER_FT
AERO_STOP_MPS = 200.0 * M_PER_FT

_LENGTH_INDEX = (0, 9, 10, 11)


def m_to_ft(m: float) -> float:
    return float(m) * FT_PER_M


def ft_to_m(ft: float) -> float:
    return float(ft) * M_PER_FT


def ms_to_fts(ms: float) -> float:
    return float(ms) * FT_PER_M


def fts_to_ms(fts: float) -> float:
    return float(fts) * M_PER_FT


def deg_to_rad(deg: float) -> float:
    return float(deg) * DEG_TO_RAD


def rad_to_deg(rad: float) -> float:
    return float(rad) * RAD_TO_DEG


def aerobench_deg(angle_rad: float) -> float:
    return float(angle_rad) * AEROBENCH_RTOD


def aerobench_poly_rad(angle_rad: float) -> float:
    return float(angle_rad) * AEROBENCH_RTOD * DEG_TO_RAD


def lbf_to_n(lbf: float) -> float:
    return float(lbf) * LBF_TO_N


def ned_m_to_f16_ft(n_m: float, e_m: float, d_m: float) -> tuple[float, float, float]:
    return (float(n_m) * FT_PER_M, float(e_m) * FT_PER_M, -float(d_m) * FT_PER_M)


def f16_ft_to_ned_m(pn_ft: float, pe_ft: float, h_ft: float) -> tuple[float, float, float]:
    return (float(pn_ft) * M_PER_FT, float(pe_ft) * M_PER_FT, -float(h_ft) * M_PER_FT)


def ned_m_to_f16_m(n_m: float, e_m: float, d_m: float) -> tuple[float, float, float]:
    return (float(n_m), float(e_m), -float(d_m))


def f16_m_to_ned_m(pn_m: float, pe_m: float, h_m: float) -> tuple[float, float, float]:
    return (float(pn_m), float(pe_m), -float(h_m))


def state_imp_to_si(x) -> np.ndarray:
    y = np.asarray(x, dtype=float).copy()
    for index in _LENGTH_INDEX:
        if index < y.size:
            y[index] *= M_PER_FT
    return y


def state_si_to_imp(x) -> np.ndarray:
    y = np.asarray(x, dtype=float).copy()
    for index in _LENGTH_INDEX:
        if index < y.size:
            y[index] /= M_PER_FT
    return y


def u_imp_to_si(u) -> np.ndarray:
    y = np.asarray(u, dtype=float).copy()
    y[1:] *= DEG_TO_RAD
    return y


def u_si_to_imp(u) -> np.ndarray:
    y = np.asarray(u, dtype=float).copy()
    y[1:] *= RAD_TO_DEG
    return y
