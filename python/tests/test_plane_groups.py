import json
import tempfile
import unittest
from pathlib import Path

from f16.aero_data import _MORELLI_GROUPS
from f16.aero_morelli import morelli_coefficients


class TestPlaneGroups(unittest.TestCase):
    def test_lengths_cover_the_f16_groups(self) -> None:
        from plane.groups import MORELLI_LENGTHS, LINEAR_INDEX, zero_coefficients

        self.assertEqual(tuple(MORELLI_LENGTHS), _MORELLI_GROUPS)
        coeff = zero_coefficients()
        for name, length in MORELLI_LENGTHS.items():
            self.assertEqual(len(coeff[name]), length)
            self.assertTrue(set(LINEAR_INDEX[name]).issubset(range(length)))

    def test_all_zero_coefficients_are_zero_when_the_state_is_not(self) -> None:
        from plane.groups import zero_coefficients

        root = _write(zero_coefficients())
        got = _eval(root, alpha=0.4, beta=0.3, de=0.2, da=-0.1, dr=0.15, p=0.5, q=-0.4, r=0.3)
        self.assertEqual(got, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    def test_elevator_square_is_absent_until_its_slot_is_set(self) -> None:
        from plane.groups import zero_coefficients

        coeff = zero_coefficients()
        coeff["cx"][0] = -0.2
        coeff["cx"][3] = 0.5
        root = _write(coeff)
        base = _eval(root, de=0.0)[0]
        moved = _eval(root, de=0.2)[0]
        self.assertAlmostEqual(moved - base, 0.5 * 0.2, delta=1e-12)
        coeff["cx"][2] = 4.0
        root = _write(coeff)
        squared = _eval(root, de=0.2)[0]
        self.assertAlmostEqual(squared - base, 0.5 * 0.2 + 4.0 * 0.2 ** 2, delta=1e-12)

    def test_cz_keeps_the_beta_square_factor_when_higher_slots_are_zero(self) -> None:
        from plane.groups import zero_coefficients

        coeff = zero_coefficients()
        coeff["cz"][0] = -1.0
        root = _write(coeff)
        cz = _eval(root, beta=0.2)[2]
        self.assertAlmostEqual(cz, -1.0 * (1.0 - 0.2 ** 2), delta=1e-12)

    def test_roll_keeps_only_the_beta_term(self) -> None:
        from plane.groups import zero_coefficients, nonlinear_index

        self.assertIn(1, nonlinear_index("cl"))
        coeff = zero_coefficients()
        coeff["cl"][0] = 0.5
        root = _write(coeff)
        cl = _eval(root, alpha=0.3, beta=0.2)[3]
        self.assertAlmostEqual(cl, 0.5 * 0.2, delta=1e-12)


def _write(coefficients: dict) -> Path:
    directory = Path(tempfile.mkdtemp())
    payload = {"model": "morelli", "coefficients": coefficients}
    (directory / "morelli.json").write_text(json.dumps(payload))
    return directory


def _eval(root: Path, **kw) -> tuple[float, ...]:
    args = dict(alpha=0.0, beta=0.0, de=0.0, da=0.0, dr=0.0, p=0.0, q=0.0, r=0.0)
    args.update(kw)
    return tuple(
        float(v)
        for v in morelli_coefficients(
            args["alpha"], args["beta"], args["de"], args["da"], args["dr"],
            args["p"], args["q"], args["r"], 1.0, 1.0, 10.0, 0.25, 0.25, root=root,
        )
    )
