import sys
import unittest

import numpy as np

REF = "/home/valentin/Projects/FlightSimulation/F16/AeroBenchVVPython/code"
sys.path.insert(0, REF)

from aerobench.lowlevel.low_level_controller import (  # noqa: E402
    LowLevelController as RefLlc,
)
from aerobench.lowlevel.subf16_model import subf16_model as ref_subf16  # noqa: E402

from f16.llc import CtrlLimits, F16Llc  # noqa: E402
from f16.model import subf16_derivative  # noqa: E402
from f16.units import state_imp_to_si, state_si_to_imp, u_imp_to_si, u_si_to_imp  # noqa: E402

_F16DYNAMICS_ATOL = 1e-6
_F16DYNAMICS_RTOL = 1e-6


def _f16dynamics_identity(mod) -> str:
    parts = [
        str(getattr(mod, "__version__", "") or ""),
        str(getattr(mod, "__file__", "") or ""),
    ]
    identity = " ".join(part for part in parts if part).strip()
    return identity or "f16dynamics"


class TestModelLlc(unittest.TestCase):
    def test_derivative_matches_reference_at_trim(self) -> None:
        ref = RefLlc()
        x_imp = np.asarray(ref.xequil, dtype=float).copy()
        u_imp = np.asarray(ref.uequil, dtype=float).copy()
        xd, nz, ny = subf16_derivative(state_imp_to_si(x_imp), u_imp_to_si(u_imp))
        rxd, rnz, rny, _, _ = ref_subf16(x_imp, u_imp, "morelli")
        np.testing.assert_allclose(state_si_to_imp(xd), rxd, rtol=1e-8, atol=1e-8)
        self.assertAlmostEqual(nz, float(rnz), places=6)
        self.assertAlmostEqual(ny, float(rny), places=6)

    def test_llc_matches_reference(self) -> None:
        llc = F16Llc()
        ref = RefLlc()
        x_imp = np.concatenate([np.asarray(ref.xequil, dtype=float), np.zeros(3)])
        u_ref = np.array([0.0, 0.0, 0.0, 0.1395])
        _, u_si = llc.get_u(u_ref, state_imp_to_si(x_imp))
        _, ru_deg = ref.get_u_deg(u_ref, x_imp)
        np.testing.assert_allclose(u_si_to_imp(u_si), ru_deg, rtol=1e-8, atol=1e-8)
        self.assertEqual(llc.get_num_integrators(), 3)
        self.assertEqual(CtrlLimits().NzMax, 6)

    def test_cpp_trim_derivatives_and_u(self) -> None:
        try:
            import f16dynamics
        except ImportError:
            self.skipTest("f16dynamics not built")
        identity = _f16dynamics_identity(f16dynamics)
        llc = F16Llc()
        x_si = llc.xequil.copy()
        u_si = llc.uequil.copy()
        xd, nz, ny = subf16_derivative(x_si, u_si)
        plant = f16dynamics.F16Plant()
        full = np.asarray(
            plant.f16model(
                np.ascontiguousarray(state_si_to_imp(x_si), dtype=np.float64),
                np.ascontiguousarray(u_si_to_imp(u_si), dtype=np.float64),
            ),
            dtype=np.float64,
        ).reshape(-1)
        self._match_cpp(
            "trim_derivative",
            np.concatenate([state_si_to_imp(xd), [nz, ny]]),
            full[:15],
            identity,
        )
        u_ref = np.array([0.0, 0.0, 0.0, 0.1395])
        _, u_cmd = llc.get_u(u_ref, np.concatenate([x_si, np.zeros(3)]))
        f16_full = np.zeros(17, dtype=np.float64)
        f16_full[:13] = state_si_to_imp(x_si)
        llc_in = np.concatenate([f16_full, u_ref])
        u_cpp = np.asarray(
            f16dynamics.LowLevelController().output(
                np.ascontiguousarray(np.zeros(3), dtype=np.float64),
                np.ascontiguousarray(llc_in, dtype=np.float64),
            ),
            dtype=np.float64,
        ).reshape(-1)
        self._match_cpp("get_u", u_si_to_imp(u_cmd), u_cpp, identity)

    def _match_cpp(self, metric: str, ours, ref, identity: str) -> None:
        ours_a = np.asarray(ours, dtype=float).reshape(-1)
        ref_a = np.asarray(ref, dtype=float).reshape(-1)
        if ours_a.shape != ref_a.shape or not np.allclose(
            ours_a, ref_a, rtol=_F16DYNAMICS_RTOL, atol=_F16DYNAMICS_ATOL
        ):
            if ours_a.shape == ref_a.shape:
                value = float(np.max(np.abs(ours_a - ref_a)))
            else:
                value = float("inf")
            self.fail(
                f"metric={metric} value={value} ours={ours_a.tolist()} "
                f"ref={ref_a.tolist()} ref_version={identity} "
                f"atol={_F16DYNAMICS_ATOL} rtol={_F16DYNAMICS_RTOL}"
            )
