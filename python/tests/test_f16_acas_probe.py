import unittest

from f16.acas_probe import acas_available, acas_reference_version, run_acas_headon_reference


class TestAcasProbe(unittest.TestCase):
    def test_probe_skips_cleanly_without_csaf(self) -> None:
        if not acas_available():
            with self.assertRaises(unittest.SkipTest):
                acas_reference_version()
            with self.assertRaises(unittest.SkipTest):
                run_acas_headon_reference(t_end=10.0)
            self.skipTest("csaf_f16 or f16dynamics not installed")
        version = acas_reference_version()
        self.assertIn("f16dynamics @", version)
        out = run_acas_headon_reference(t_end=10.0)
        self.assertGreater(len(out["times"]), 10)
