"""Metre↔feet wall. Only converter in the package."""
from __future__ import annotations

M_PER_FT = 0.3048
FT_PER_M = 1 / M_PER_FT


def m_to_ft(m: float) -> float:
    return float(m) * FT_PER_M


def ft_to_m(ft: float) -> float:
    return float(ft) * M_PER_FT


def ms_to_fts(ms: float) -> float:
    return float(ms) * FT_PER_M


def fts_to_ms(fts: float) -> float:
    return float(fts) * M_PER_FT


def ned_m_to_f16_ft(n_m: float, e_m: float, d_m: float) -> tuple[float, float, float]:
    return (float(n_m) * FT_PER_M, float(e_m) * FT_PER_M, -float(d_m) * FT_PER_M)


def f16_ft_to_ned_m(pn_ft: float, pe_ft: float, h_ft: float) -> tuple[float, float, float]:
    return (float(pn_ft) * M_PER_FT, float(pe_ft) * M_PER_FT, -float(h_ft) * M_PER_FT)
