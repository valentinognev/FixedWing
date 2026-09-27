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
