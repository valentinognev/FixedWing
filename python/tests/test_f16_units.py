import unittest

from f16.units import (
    FT_PER_M,
    f16_ft_to_ned_m,
    ft_to_m,
    fts_to_ms,
    m_to_ft,
    ms_to_fts,
    ned_m_to_f16_ft,
)


class TestUnits(unittest.TestCase):
    def test_trim_spot_values(self) -> None:
        self.assertAlmostEqual(m_to_ft(1.0), 3.28084, places=4)
        self.assertAlmostEqual(ft_to_m(502.0), 153.0096, places=3)
        self.assertAlmostEqual(ms_to_fts(153.0096), 502.0, places=1)
        self.assertAlmostEqual(fts_to_ms(502.0), 153.0096, places=3)

    def test_ned_round_trip(self) -> None:
        pn, pe, h = ned_m_to_f16_ft(100.0, 50.0, -20.0)
        self.assertAlmostEqual(h, 65.6168, places=3)
        n, e, d = f16_ft_to_ned_m(pn, pe, h)
        self.assertAlmostEqual(n, 100.0, places=6)
        self.assertAlmostEqual(e, 50.0, places=6)
        self.assertAlmostEqual(d, -20.0, places=6)

    def test_ft_per_m_constant(self) -> None:
        self.assertAlmostEqual(FT_PER_M, 1 / 0.3048, places=9)

    def test_angle_and_force_factors(self) -> None:
        import math

        from f16.units import (
            AEROBENCH_RTOD,
            DEG_TO_RAD,
            LBF_TO_N,
            aerobench_deg,
            aerobench_poly_rad,
            deg_to_rad,
            lbf_to_n,
            rad_to_deg,
        )

        self.assertAlmostEqual(deg_to_rad(180.0), math.pi, places=12)
        self.assertAlmostEqual(rad_to_deg(math.pi), 180.0, places=12)
        self.assertAlmostEqual(DEG_TO_RAD, math.pi / 180.0, places=15)
        self.assertAlmostEqual(LBF_TO_N, 4.4482216152605, places=12)
        self.assertAlmostEqual(lbf_to_n(1.0), 4.4482216152605, places=12)
        self.assertAlmostEqual(AEROBENCH_RTOD, 57.29578, places=9)
        self.assertAlmostEqual(aerobench_deg(1.0), 57.29578, places=9)
        self.assertAlmostEqual(aerobench_poly_rad(1.0), 57.29578 * DEG_TO_RAD, places=12)

    def test_scaled_plant_constants(self) -> None:
        from f16.units import (
            AERO_STOP_MPS,
            C3,
            C4,
            C7,
            C9,
            CBAR_M,
            FINAL_RMS_LIMIT,
            G_MPS2,
            GCAS_FLOOR_M,
            H_GCAS_M,
            H_STRAT_M,
            H_THRUST_STEP_M,
            HE_KGM2,
            K_ALT_PER_M,
            K_VT_PER_MPS,
            MIN_H_DIFF_M,
            RM_PER_KG,
            S_M2,
            SL_H_BAND_M,
            SL_MIN_H_M,
            SL_VT_BAND_MPS,
            VT_GCAS_MPS,
            XA_M,
        )

        self.assertAlmostEqual(G_MPS2, 32.17 * 0.3048, places=9)
        self.assertAlmostEqual(S_M2, 300.0 * 0.3048 * 0.3048, places=9)
        self.assertAlmostEqual(CBAR_M, 11.32 * 0.3048, places=9)
        self.assertAlmostEqual(XA_M, 15.0 * 0.3048, places=9)
        self.assertAlmostEqual(HE_KGM2, 216.93087173302405, places=6)
        self.assertAlmostEqual(RM_PER_KG, 1.0757917239516308e-4, places=12)
        self.assertAlmostEqual(C3, 7.78128067487515e-05, places=12)
        self.assertAlmostEqual(C4, 1.2110770491132697e-06, places=15)
        self.assertAlmostEqual(C7, 1.3217113715048597e-05, places=15)
        self.assertAlmostEqual(C9, 1.17051113090302e-05, places=15)
        self.assertAlmostEqual(H_STRAT_M, 10668.0, places=6)
        self.assertAlmostEqual(H_THRUST_STEP_M, 3048.0, places=6)
        self.assertAlmostEqual(MIN_H_DIFF_M, 15.24, places=9)
        self.assertEqual(FINAL_RMS_LIMIT, 5.0)
        self.assertAlmostEqual(GCAS_FLOOR_M, 304.8, places=6)
        self.assertAlmostEqual(VT_GCAS_MPS, 164.592, places=6)
        self.assertAlmostEqual(H_GCAS_M, 304.8, places=6)
        self.assertAlmostEqual(K_ALT_PER_M, 0.01 / 0.3048, places=12)
        self.assertAlmostEqual(K_VT_PER_MPS, 0.5 / 0.3048, places=12)
        self.assertAlmostEqual(SL_H_BAND_M, 4.572, places=9)
        self.assertAlmostEqual(SL_VT_BAND_MPS, 2.4384, places=9)
        self.assertAlmostEqual(SL_MIN_H_M, 274.32, places=6)
        self.assertAlmostEqual(AERO_STOP_MPS, 60.96, places=6)

    def test_state_si_to_imp_exact_length_round_trip(self) -> None:
        import numpy as np

        from f16.units import H_GCAS_M, VT_GCAS_MPS, state_si_to_imp

        h = np.zeros(13, dtype=float)
        h[11] = H_GCAS_M
        self.assertEqual(float(state_si_to_imp(h)[11]), 1000.0)
        vt = np.zeros(13, dtype=float)
        vt[0] = 153.0096
        self.assertEqual(float(state_si_to_imp(vt)[0]), 502.0)
        gcas = np.zeros(13, dtype=float)
        gcas[0] = VT_GCAS_MPS
        self.assertEqual(float(state_si_to_imp(gcas)[0]), 540.0)

    def test_state_and_control_round_trip(self) -> None:
        import numpy as np

        from f16.units import state_imp_to_si, state_si_to_imp, u_imp_to_si, u_si_to_imp

        x_imp = np.array(
            [502.0, 0.0389, 0.0, 0.0, 0.0389, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000.0, 9.0567],
            dtype=float,
        )
        x_si = state_imp_to_si(x_imp)
        self.assertAlmostEqual(float(x_si[0]), 153.0096, places=6)
        self.assertAlmostEqual(float(x_si[1]), 0.0389, places=9)
        self.assertAlmostEqual(float(x_si[11]), 304.8, places=6)
        self.assertAlmostEqual(float(x_si[12]), 9.0567, places=6)
        np.testing.assert_allclose(state_si_to_imp(x_si), x_imp, rtol=0, atol=1e-9)
        u_imp = np.array([0.1395, -0.7496, 2.0, -3.0], dtype=float)
        u_si = u_imp_to_si(u_imp)
        self.assertAlmostEqual(float(u_si[0]), 0.1395, places=9)
        self.assertAlmostEqual(float(u_si[1]), -0.7496 * np.pi / 180.0, places=12)
        np.testing.assert_allclose(u_si_to_imp(u_si), u_imp, rtol=0, atol=1e-9)

    def test_ned_sign_only(self) -> None:
        from f16.units import f16_m_to_ned_m, ned_m_to_f16_m

        pn, pe, h = ned_m_to_f16_m(10.0, -4.0, -304.8)
        self.assertEqual((pn, pe, h), (10.0, -4.0, 304.8))
        n, e, d = f16_m_to_ned_m(pn, pe, h)
        self.assertEqual((n, e, d), (10.0, -4.0, -304.8))
