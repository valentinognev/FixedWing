"""Model-name selection and JSONC comment stripping for the host airplane."""
from __future__ import annotations

import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path

MORELLI_LENGTHS = {
    "cx": 7, "cxq": 5, "cy": 3, "cyp": 4, "cyr": 4,
    "cz": 6, "czq": 5, "cl": 8, "clp": 4, "clr": 5,
    "clda": 7, "cldr": 7, "cm": 8, "cmq": 6, "cn": 7,
    "cnp": 5, "cnr": 3, "cnda": 10, "cndr": 6,
}
STEVENS_GROUPS = (
    "cx", "cz", "cl", "cm", "cn", "dlda", "dldr", "dnda", "dndr", "damp",
    "cy_beta", "cy_aileron", "cy_rudder", "aileron_norm_deg", "rudder_norm_deg",
    "beta_norm_deg", "cz_elevator", "elevator_norm_deg",
)
AIRCRAFT = {
    "mass_kg": 2.0, "s_m2": 1.0, "b_m": 2.0, "cbar_m": 1.0,
    "xcg": 0.25, "xcg_ref": 0.25, "ixx": 2.0, "iyy": 3.0, "izz": 4.0,
    "ixz": 0.0, "he": 0.0, "t_max_n": 100.0, "tau_s": 2.0, "v_ref_mps": 50.0,
}
INITIAL = {
    "vt_mps": 40.0, "alpha_rad": 0.02, "beta_rad": 0.0, "phi_rad": 0.0,
    "theta_rad": 0.02, "psi_rad": 0.0, "p_rad_s": 0.0, "q_rad_s": 0.0,
    "r_rad_s": 0.0, "pn_m": 0.0, "pe_m": 0.0, "alt_m": 500.0, "power": 0.4,
}
CONTROLS = {
    "throttle": 0.4, "elevator_rad": 0.0, "aileron_rad": 0.0, "rudder_rad": 0.0,
}


def _morelli_coefficients(cx0: float = -0.5) -> dict:
    coefficients = {name: [0.0] * length for name, length in MORELLI_LENGTHS.items()}
    coefficients["cx"][0] = cx0
    return coefficients


def _stevens_coefficients() -> dict:
    coefficients: dict = {name: [0.0] for name in STEVENS_GROUPS}
    for name in ("cx", "cz", "cl", "cm", "cn"):
        coefficients[name] = [0.0] * 12
    for name in ("dlda", "dldr", "dnda", "dndr"):
        coefficients[name] = [0.0] * 12
    coefficients["damp"] = [[0.0] * 9 for _ in range(12)]
    for name in ("cy_beta", "cy_aileron", "cy_rudder", "cz_elevator"):
        coefficients[name] = 0.0
    for name in ("aileron_norm_deg", "rudder_norm_deg", "beta_norm_deg", "elevator_norm_deg"):
        coefficients[name] = 20.0
    return coefficients


def _write_jsonc(path: Path, payload: dict) -> None:
    lines = json.dumps(payload, indent=2).splitlines()
    lines.insert(1, '  // JSONC comment mentioning "quotes" and a trailing comma,')
    path.write_text("\n".join(lines) + "\n")


class TestModelSelectAeroData(unittest.TestCase):
    def test_tornado_json_file_is_read(self) -> None:
        from f16.aero_data import load_aero_coefficients

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            payload = {"model": "tornado", "coefficients": _morelli_coefficients(-0.25)}
            (root / "tornado.json").write_text(json.dumps(payload))
            coefficients = load_aero_coefficients("tornado", root=root)
        self.assertEqual(coefficients["cx"][0], -0.25)

    def test_tornado_jsonc_file_wins_over_json_and_drops_comments(self) -> None:
        from f16.aero_data import load_aero_coefficients, strip_jsonc_comments

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            payload = {
                "model": "tornado",
                "source": "http://example.invalid/a//b",
                "coefficients": _morelli_coefficients(-0.75),
            }
            _write_jsonc(root / "tornado.jsonc", payload)
            coefficients = load_aero_coefficients("tornado", root=root)
            raw = (root / "tornado.jsonc").read_text()
        self.assertEqual(coefficients["cx"][0], -0.75)
        self.assertEqual(json.loads(strip_jsonc_comments(raw))["source"], "http://example.invalid/a//b")

    def test_stevens_keeps_its_own_group_set(self) -> None:
        from f16.aero_data import load_aero_coefficients

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "stevens.json").write_text(json.dumps({
                "model": "stevens", "coefficients": _morelli_coefficients(),
            }))
            with self.assertRaises(ValueError) as caught:
                load_aero_coefficients("stevens", root=root)
            self.assertIn("dlda", str(caught.exception))
            (root / "stevens.json").unlink()
            (root / "stevens.jsonc").write_text(json.dumps({
                "model": "stevens", "coefficients": _stevens_coefficients(),
            }))
            coefficients = load_aero_coefficients("stevens", root=root)
        self.assertIn("damp", coefficients)

    def test_unknown_model_still_raises_not_implemented(self) -> None:
        from f16.aero_data import load_aero_coefficients

        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError) as caught:
                load_aero_coefficients("nonesuch", root=Path(directory))
        self.assertIn("not implemented", str(caught.exception))


class TestModelSelectAircraft(unittest.TestCase):
    def test_linear_default_model_is_morelli(self) -> None:
        from plane.aircraft import load_aircraft

        aircraft = load_aircraft("linear")
        self.assertEqual(aircraft.model, "morelli")
        self.assertEqual(aircraft.mass_kg, 1000)

    def test_plane_directory_tornado_jsonc(self) -> None:
        from plane.aircraft import load_aircraft

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plane = root / "cessna172"
            plane.mkdir()
            _write_jsonc(plane / "tornado.jsonc", {
                "model": "tornado",
                "aircraft": AIRCRAFT,
                "initial": INITIAL,
                "controls": CONTROLS,
                "coefficients": _morelli_coefficients(-0.125),
            })
            aircraft = load_aircraft("cessna172", model="tornado", root=root)
        self.assertEqual(aircraft.model, "tornado")
        self.assertEqual(aircraft.mass_kg, 2.0)


class TestModelSelectMorelli(unittest.TestCase):
    def test_morelli_coefficients_model_keyword(self) -> None:
        from f16.aero_morelli import morelli_coefficients

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "morelli.json").write_text(json.dumps({
                "model": "morelli", "coefficients": _morelli_coefficients(-0.5),
            }))
            (root / "tornado.jsonc").write_text(json.dumps({
                "model": "tornado", "coefficients": _morelli_coefficients(-0.9),
            }))
            default = morelli_coefficients(
                0, 0, 0, 0, 0, 0, 0, 0, 1.0, 1.0, 100.0, 0.35, 0.35, root=root,
            )
            tornado = morelli_coefficients(
                0, 0, 0, 0, 0, 0, 0, 0, 1.0, 1.0, 100.0, 0.35, 0.35,
                model="tornado", root=root,
            )
        self.assertAlmostEqual(default[0], -0.5, delta=1e-12)
        self.assertAlmostEqual(tornado[0], -0.9, delta=1e-12)


class TestModelSelectRunner(unittest.TestCase):
    def test_unknown_model_exits_2_with_stderr(self) -> None:
        from run_plane import main

        with tempfile.TemporaryDirectory() as directory:
            csv_path = Path(directory) / "out.csv"
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                code = main([
                    "--plane", "linear", "--model", "nonesuch",
                    "--duration", "0.1", "--csv", str(csv_path),
                ])
        self.assertEqual(code, 2)
        self.assertIn("missing nonesuch.json", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()