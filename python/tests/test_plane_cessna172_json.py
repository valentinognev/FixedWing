import csv
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from f16.aero_data import load_aero_coefficients, strip_jsonc_comments
from plane.aircraft import control_vector, load_aircraft, state_vector
from plane.atmosphere import air
from plane.dynamics import plane_derivative
from plane.groups import MORELLI_LENGTHS, nonlinear_index

MODELS = ("tornado", "datcom", "avl", "flow5")
DATA = Path(__file__).resolve().parents[2] / "data" / "planes" / "cessna172"
PYTHON = Path(__file__).resolve().parents[1]

INVARIANTS = (
    ("cz", 0, -0.6, -0.05),
    ("cz", 1, -6.5, -4.0),
    ("cm", 1, -1.5, -0.3),
    ("czq", 0, -25.0, -3.0),
    ("cmq", 0, -60.0, -5.0),
    ("cl", 0, -0.35, -0.03),
    ("cy", 0, -1.2, -0.15),
    ("cn", 0, 0.02, 0.40),
    ("clp", 0, -0.9, -0.15),
    ("clr", 0, 0.005, 0.25),
    ("cyp", 0, -1.2, -0.02),
    ("cyr", 0, 0.05, 1.2),
    ("cnp", 0, -0.35, -0.005),
    ("cnr", 0, -0.9, -0.05),
    ("cx", 0, -0.15, -0.005),
    ("cxq", 0, 0.0, 2.0),
    ("clda", 0, -0.6, -0.02),
    ("cnda", 0, -0.25, -0.005),
    ("cy", 1, 0.005, 0.30),
    ("cldr", 0, 0.002, 0.20),
    ("cndr", 0, -0.90, -0.02),
    ("cy", 2, 0.02, 0.60),
    ("cz", 5, -2.0, -0.05),
    ("cm", 2, -4.0, -0.30),
)


def _payload(model: str) -> dict:
    return json.loads(strip_jsonc_comments((DATA / f"{model}.jsonc").read_text()))


def _missing_slots(payload: dict) -> set[str]:
    slots = set()
    for item in payload["provenance"]["missing"]:
        slots.add(item["slot"])
    return slots


def _slot(name: str, index: int) -> str:
    return f"{name}[{index}]"


def _read_slot(coefficients: dict, slot: str) -> float:
    name, raw = slot[:-1].split("[")
    return float(coefficients[name][int(raw)])


def _present(coefficients: dict, missing: set[str], name: str, index: int) -> bool:
    slot = _slot(name, index)
    if slot in missing:
        return False
    return coefficients[name][index] != 0.0


class TestPlaneCessna172Json(unittest.TestCase):
    def test_schema(self) -> None:
        for model in MODELS:
            payload = _payload(model)
            self.assertEqual(payload["model"], model)
            aircraft = load_aircraft("cessna172", model)
            self.assertEqual(aircraft.model, model)
            coefficients = load_aero_coefficients(model, root=aircraft.coeff_dir)
            self.assertEqual(set(coefficients), set(MORELLI_LENGTHS))
            for name, length in MORELLI_LENGTHS.items():
                row = coefficients[name]
                self.assertEqual(len(row), length, name)
                for value in row:
                    self.assertIsInstance(value, float)
                    self.assertTrue(math.isfinite(value), f"{model} {name}")
                for index in nonlinear_index(name):
                    self.assertEqual(row[index], 0.0)

    def test_invariants(self) -> None:
        self.assertNotIn(("cm", 0), tuple((name, index) for name, index, _, _ in INVARIANTS))
        for model in MODELS:
            payload = _payload(model)
            self.assertIn("cm0", payload["provenance"]["not_normalised"])
            coefficients = load_aero_coefficients(model, root=DATA)
            missing = _missing_slots(payload)
            for name, index, lo, hi in INVARIANTS:
                if not _present(coefficients, missing, name, index):
                    continue
                value = coefficients[name][index]
                if model == "tornado":
                    self.assertGreaterEqual(value, lo, f"tornado {_slot(name, index)}")
                    self.assertLessEqual(value, hi, f"tornado {_slot(name, index)}")
                elif lo < 0.0 and hi <= 0.0:
                    self.assertLess(value, 0.0, f"{model} {_slot(name, index)}")
                else:
                    self.assertGreater(value, 0.0, f"{model} {_slot(name, index)}")
                if model != "tornado" and not (lo <= value <= hi):
                    print(
                        f"{model} {_slot(name, index)}={value:.6g} outside tornado "
                        f"magnitude band [{lo}, {hi}]"
                    )

    def test_static_margin(self) -> None:
        for model in MODELS:
            coefficients = load_aero_coefficients(model, root=DATA)
            margin = coefficients["cm"][1] / coefficients["cz"][1]
            print(f"{model} static margin cbar={margin:.6f}")
            self.assertGreaterEqual(margin, 0.05, model)
            self.assertLessEqual(margin, 0.45, model)

    def test_tornado_trim(self) -> None:
        coefficients = load_aero_coefficients("tornado", root=DATA)
        cm0 = coefficients["cm"][0]
        cm1 = coefficients["cm"][1]
        alpha = -cm0 / cm1
        alpha_deg = math.degrees(alpha)
        cz_sum = coefficients["cz"][0] + coefficients["cz"][1] * alpha
        cruise_cl = -cz_sum
        print(
            f"tornado alpha_trim_deg={alpha_deg:.6f} "
            f"cz_sum={cz_sum:.6f} cruise_CL={cruise_cl:.6f}"
        )
        self.assertAlmostEqual(cm0 + cm1 * alpha, 0.0, places=12)
        self.assertGreaterEqual(alpha_deg, -2.0)
        self.assertLessEqual(alpha_deg, 8.0)
        self.assertGreaterEqual(cruise_cl, 0.2)
        self.assertLessEqual(cruise_cl, 1.2)

    def test_modes(self) -> None:
        aircraft = load_aircraft("cessna172", "tornado")
        coefficients = load_aero_coefficients("tornado", root=DATA)
        state = state_vector(aircraft.initial)
        control = control_vector(aircraft.controls)
        longitudinal = _jacobian(aircraft, state, control, (0, 1, 4, 7))
        ordered = sorted(np.linalg.eigvals(longitudinal), key=lambda pole: abs(pole), reverse=True)
        short_period = ordered[:2]
        phugoid = ordered[2:]
        analytic = _short_period(aircraft, coefficients, state)
        roll, dutch = _lateral(aircraft, coefficients, state)
        print(f"tornado short-period poles: {_fmt(short_period)}")
        print(f"tornado short-period 2x2: {_fmt(np.linalg.eigvals(analytic))}")
        print(f"tornado phugoid poles: {_fmt(phugoid)}")
        print(f"tornado roll pole: {roll:.6g}")
        print(f"tornado dutch-roll poles: {_fmt(np.linalg.eigvals(dutch))}")
        for pole in (*short_period, *phugoid, *np.linalg.eigvals(analytic), roll, *np.linalg.eigvals(dutch)):
            self.assertLess(pole.real, 0.0, pole)

    def test_cross_model_agreement(self) -> None:
        loaded = {
            model: (
                load_aero_coefficients(model, root=DATA),
                _missing_slots(_payload(model)),
            )
            for model in MODELS
        }
        slopes = [abs(loaded[model][0]["cz"][1]) for model in MODELS]
        ratio = max(slopes) / min(slopes)
        print(f"cl_alpha magnitudes={slopes} ratio={ratio:.6f}")
        self.assertLessEqual(ratio, 1.6)
        for name, index, lo, hi in INVARIANTS:
            signed = []
            for model, (coefficients, missing) in loaded.items():
                if _present(coefficients, missing, name, index):
                    signed.append((model, coefficients[name][index]))
            if len(signed) < 2:
                continue
            reference = math.copysign(1.0, signed[0][1])
            for model, value in signed[1:]:
                self.assertEqual(math.copysign(1.0, value), reference, f"{model} {_slot(name, index)}")
            if lo < 0.0 and hi <= 0.0:
                self.assertLess(reference, 0.0)
            elif lo > 0.0:
                self.assertGreater(reference, 0.0)

    def test_declared_gaps(self) -> None:
        for model in MODELS:
            payload = _payload(model)
            coefficients = load_aero_coefficients(model, root=DATA)
            missing = _missing_slots(payload)
            for item in payload["provenance"]["missing"]:
                value = _read_slot(coefficients, item["slot"])
                self.assertEqual(value, 0.0, f"{model} {item['slot']} declared missing but non-zero")
            for name, index, _, _ in INVARIANTS:
                slot = _slot(name, index)
                if coefficients[name][index] == 0.0:
                    self.assertIn(slot, missing, f"{model} {slot} is zero and undeclared")

    def test_runner_smoke(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            csv_path = Path(temporary) / "c172.csv"
            proc = subprocess.run(
                [
                    sys.executable, "run_plane.py",
                    "--plane", "cessna172", "--model", "tornado",
                    "--duration", "5", "--csv", str(csv_path),
                ],
                cwd=PYTHON, check=False, capture_output=True, text=True,
            )
            self.assertEqual(proc.returncode, 0, proc.stderr)
            rows = list(csv.reader(csv_path.read_text().splitlines()))
            self.assertGreaterEqual(len(rows), 2)
            for row in rows[1:]:
                for cell in row:
                    self.assertTrue(math.isfinite(float(cell)))


def _jacobian(aircraft, state, control, indexes: tuple[int, ...]) -> np.ndarray:
    step = 1e-6
    matrix = np.zeros((len(indexes), len(indexes)))
    for column, index in enumerate(indexes):
        plus = state.copy()
        minus = state.copy()
        plus[index] += step
        minus[index] -= step
        high = plane_derivative(plus, control, aircraft)
        low = plane_derivative(minus, control, aircraft)
        for row, out in enumerate(indexes):
            matrix[row, column] = (high[out] - low[out]) / (2.0 * step)
    return matrix


def _short_period(aircraft, coefficients, state) -> np.ndarray:
    speed = float(state[0])
    _, qbar, _ = air(speed, float(state[11]))
    mass = aircraft.mass_kg
    area = aircraft.s_m2
    chord = aircraft.cbar_m
    force = qbar * area / mass
    moment = qbar * area * chord / aircraft.iyy
    za = force * coefficients["cz"][1]
    zq = force * (chord / (2.0 * speed)) * coefficients["czq"][0]
    ma = moment * coefficients["cm"][1]
    mq = moment * (chord / (2.0 * speed)) * coefficients["cmq"][0]
    return np.array([[za / speed, 1.0 + zq / speed], [ma, mq]])


def _lateral(aircraft, coefficients, state) -> tuple[float, np.ndarray]:
    speed = float(state[0])
    _, qbar, _ = air(speed, float(state[11]))
    mass = aircraft.mass_kg
    area = aircraft.s_m2
    span = aircraft.b_m
    force = qbar * area / mass
    roll = qbar * area * span / aircraft.ixx
    yaw = qbar * area * span / aircraft.izz
    rate = span / (2.0 * speed)
    lp = roll * rate * coefficients["clp"][0]
    yb = force * coefficients["cy"][0]
    yr = force * rate * coefficients["cyr"][0]
    nb = yaw * coefficients["cn"][0]
    nr = yaw * rate * coefficients["cnr"][0]
    return lp, np.array([[yb / speed, yr / speed - 1.0], [nb, nr]])


def _fmt(poles) -> str:
    return " ".join(f"{pole.real:.6f}{pole.imag:+.6f}j" for pole in poles)
