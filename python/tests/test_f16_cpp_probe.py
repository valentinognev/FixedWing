import unittest

from f16.cpp_probe import cpp_available, cpp_version


class TestCppProbe(unittest.TestCase):
    def test_probe_reports_availability(self) -> None:
        if not cpp_available():
            self.skipTest("f16dynamics not built")
        version = cpp_version()
        self.assertIn("f16dynamics @", version)
        self.assertGreater(len(version.split()[0]), 4)
