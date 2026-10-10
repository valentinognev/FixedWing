"""Flightbench HTTP API: planes, tasks, gains, and one run.

The four routes are the bench's only HTTP surface. Listing routes answer from
the registries; a run answers the spec's RunResponse, including a plant stop
as 200. A law the plane does not offer, an unknown task, a non-finite gain,
and a trim outside the spec bounds are 400 before any plant runs. TrimError
is 422. Shape checks stand in for ``laws.run`` with a tiny RunResult; one
Cessna dutch-roll request flies the real law.
"""
from __future__ import annotations

import unittest
from unittest import mock

import numpy as np
from fastapi.testclient import TestClient

from flightbench.api import app, main
from flightbench.common import (
    ALPHA,
    CHANNELS,
    SERIES,
    THETA,
    FlightbenchError,
    Mode,
    RunResult,
    Trace,
    TrimError,
    TrimPoint,
)

_RUN = {
    "plane": "cessna172",
    "law": "linear",
    "task": "dutch_roll",
}

# 1e400 is the JSON spelling of +inf. ``json.dumps`` would emit ``Infinity``,
# which is not JSON, so the non-finite gain is sent as raw bytes.
_INFINITE_GAIN = (
    b'{"plane":"cessna172","law":"linear","task":"pitch_disturbance",'
    b'"gains":{"kq":1e400}}'
)


def _series(length: int, scale: float) -> dict[str, np.ndarray]:
    """One trace: every SERIES key, identical length, plain finite samples."""
    return {
        key: np.arange(length, dtype=float) * scale for key in SERIES
    }


def _trim(vt_mps: float, altitude_m: float) -> TrimPoint:
    x = np.zeros(13)
    x[ALPHA] = 0.04
    x[THETA] = 0.05
    return TrimPoint(
        x=x,
        u=np.array([0.3, -0.02, 0.0, 0.01]),
        vt_mps=vt_mps,
        altitude_m=altitude_m,
    )


def _result(stopped_at: float | None = None, stop_reason: str | None = None) -> RunResult:
    """A linear-law result small enough to assert the response shape."""
    nonlinear = Trace(
        series=_series(4, 0.02),
        stopped_at=stopped_at,
        stop_reason=stop_reason,
    )
    linear = Trace(series=_series(4, 0.01))
    mode = Mode("dutch_roll", 1.5, 0.2, -0.3, 1.4)
    return RunResult(
        plane="cessna172",
        law="linear",
        task="dutch_roll",
        aero="tornado",
        trim=_trim(23.0, 100.0),
        runs={"linear": linear, "nonlinear": nonlinear},
        reference=("beta", np.array([0.0, 0.02]), np.array([0.0, 0.01])),
        metrics={
            "open_loop_modes": [mode],
            "closed_loop_modes": [mode],
            "trims": [_trim(23.0, 100.0), _trim(27.6, 100.0)],
        },
    )


class TestFlightbenchApi(unittest.TestCase):
    def setUp(self) -> None:
        self.client = TestClient(app)

    def test_planes_lists_three_ids_with_laws_and_channel_labels(self) -> None:
        # Drops a plane, a law, or renames the X-31 pitch channel.
        response = self.client.get("/api/planes")
        self.assertEqual(response.status_code, 200)
        planes = response.json()
        self.assertEqual([plane["id"] for plane in planes], ["f16", "x31", "cessna172"])
        by_id = {plane["id"]: plane for plane in planes}
        self.assertEqual(by_id["f16"]["laws"], ["linear", "lqr"])
        self.assertEqual(by_id["x31"]["laws"], ["linear", "ndi"])
        self.assertEqual(by_id["cessna172"]["laws"], ["linear"])
        self.assertEqual(by_id["x31"]["channel_labels"]["pitch"], "canard")
        for plane in planes:
            self.assertEqual(
                set(plane),
                {"id", "label", "laws", "aero_models", "default_trim", "channel_labels"},
            )
            self.assertEqual(set(plane["default_trim"]), {"vt_mps", "altitude_m"})
            self.assertEqual(
                set(plane["channel_labels"]),
                {"throttle", "pitch", "roll", "yaw"},
            )

    def test_tasks_returns_eleven_items(self) -> None:
        # Registry row dropped, or a field the task list is specified to carry.
        response = self.client.get("/api/tasks")
        self.assertEqual(response.status_code, 200)
        tasks = response.json()
        self.assertEqual(len(tasks), 11)
        self.assertEqual(
            [task["id"] for task in tasks],
            [
                "pitch_disturbance",
                "short_period_phugoid",
                "lead_pitch",
                "trim_cruise",
                "airspeed",
                "acceleration",
                "steady_descent",
                "dutch_roll",
                "turn_coordination",
                "sideslip_turn",
                "yaw_orientation",
            ],
        )
        for task in tasks:
            self.assertEqual(
                set(task),
                {"id", "label", "family", "lesson", "duration_s", "description"},
            )

    def test_gains_for_f16_linear_dutch_roll_are_kr_and_tau_w(self) -> None:
        # Wrong task's gains, or the port-law table instead of the linear one.
        response = self.client.get(
            "/api/gains",
            params={"plane": "f16", "law": "linear", "task": "dutch_roll"},
        )
        self.assertEqual(response.status_code, 200)
        rows = response.json()
        self.assertEqual([row["name"] for row in rows], ["kr", "tau_w"])
        for row in rows:
            self.assertEqual(set(row), {"name", "label", "value", "unit"})

    def test_unknown_task_on_port_law_gains_is_400(self) -> None:
        # lqr/ndi gain tables answer 200 for a task the registry does not have.
        cases = (
            {"plane": "f16", "law": "lqr", "task": "not_a_task"},
            {"plane": "x31", "law": "ndi", "task": "not_a_task"},
        )
        for params in cases:
            with self.subTest(law=params["law"]):
                response = self.client.get("/api/gains", params=params)
                self.assertEqual(response.status_code, 400)
                detail = response.json()["detail"]
                self.assertIn("not_a_task", detail)
                self.assertIn("legal tasks", detail)
                self.assertIn("pitch_disturbance", detail)
                self.assertIn("yaw_orientation", detail)

    def test_lqr_on_cessna_is_400_naming_linear(self) -> None:
        # A law the plane lacks accepted, or the legal set omitted from detail.
        response = self.client.post("/api/runs", json={
            "plane": "cessna172",
            "law": "lqr",
            "task": "dutch_roll",
        })
        self.assertEqual(response.status_code, 400)
        self.assertIn("linear", response.json()["detail"])

    def test_unknown_task_is_400(self) -> None:
        response = self.client.post("/api/runs", json={
            "plane": "cessna172",
            "law": "linear",
            "task": "not_a_task",
        })
        self.assertEqual(response.status_code, 400)
        self.assertIn("not_a_task", response.json()["detail"])

    def test_nonfinite_gain_is_400_before_run(self) -> None:
        # 1e400 accepted, or the plant started before the gain was refused.
        with mock.patch("flightbench.api.run", side_effect=AssertionError("ran")) as run:
            response = self.client.post(
                "/api/runs",
                content=_INFINITE_GAIN,
                headers={"Content-Type": "application/json"},
            )
        self.assertEqual(response.status_code, 400)
        self.assertIn("kq", response.json()["detail"])
        run.assert_not_called()

    def test_trim_vt_zero_is_400_before_run(self) -> None:
        # vt_mps 0 accepted, or the bound checked only after trim solved.
        with mock.patch("flightbench.api.run", side_effect=AssertionError("ran")) as run:
            response = self.client.post("/api/runs", json={
                **_RUN,
                "trim": {"vt_mps": 0, "altitude_m": 100},
            })
        self.assertEqual(response.status_code, 400)
        detail = response.json()["detail"]
        self.assertIn("vt_mps", detail)
        self.assertIn("400", detail)
        run.assert_not_called()

    def test_trim_outside_spec_bounds_is_400_before_run(self) -> None:
        # A bound loosened (vt above 400, altitude below 0 or above 15000).
        cases = (
            {"vt_mps": 400.1, "altitude_m": 100},
            {"vt_mps": 50, "altitude_m": -1},
            {"vt_mps": 50, "altitude_m": 15001},
        )
        for trim in cases:
            with self.subTest(trim=trim):
                with mock.patch(
                    "flightbench.api.run", side_effect=AssertionError("ran")
                ) as run:
                    response = self.client.post("/api/runs", json={**_RUN, "trim": trim})
                self.assertEqual(response.status_code, 400)
                run.assert_not_called()

    def test_trim_on_spec_bounds_reaches_run(self) -> None:
        # Inclusive ends (vt 400, altitude 0 and 15000) refused.
        with mock.patch("flightbench.api.run", return_value=_result()) as run:
            response = self.client.post("/api/runs", json={
                **_RUN,
                "trim": {"vt_mps": 400, "altitude_m": 0},
            })
            high = self.client.post("/api/runs", json={
                **_RUN,
                "trim": {"vt_mps": 1, "altitude_m": 15000},
            })
        self.assertEqual(response.status_code, 200)
        self.assertEqual(high.status_code, 200)
        self.assertEqual(run.call_args_list[0].kwargs["trim"], (400, 0))
        self.assertEqual(run.call_args_list[1].kwargs["trim"], (1, 15000))

    def test_trim_error_is_422(self) -> None:
        # TrimError collapsed into the FlightbenchError 400 handler.
        with mock.patch(
            "flightbench.api.run", side_effect=TrimError("solver did not converge")
        ):
            response = self.client.post("/api/runs", json=_RUN)
        self.assertEqual(response.status_code, 422)
        self.assertEqual(response.json(), {"detail": "solver did not converge"})

    def test_flightbench_error_is_400(self) -> None:
        with mock.patch(
            "flightbench.api.run", side_effect=FlightbenchError("unknown plane 'b747'")
        ):
            response = self.client.post("/api/runs", json=_RUN)
        self.assertEqual(response.status_code, 400)
        self.assertEqual(response.json(), {"detail": "unknown plane 'b747'"})

    def test_stopped_nonlinear_trace_is_200_with_stopped_at(self) -> None:
        # A plant stop turned into an error, or stopped_at taken from nowhere.
        with mock.patch(
            "flightbench.api.run",
            return_value=_result(stopped_at=1.5, stop_reason="ground contact"),
        ):
            response = self.client.post("/api/runs", json=_RUN)
        self.assertEqual(response.status_code, 200)
        body = response.json()
        self.assertEqual(body["stopped_at"], 1.5)
        self.assertEqual(body["stop_reason"], "ground contact")

    def test_run_response_shape(self) -> None:
        # A key dropped, a numpy array left in place, or a series missing a signal.
        with mock.patch("flightbench.api.run", return_value=_result()) as run:
            response = self.client.post("/api/runs", json={
                **_RUN,
                "aero": "tornado",
                "gains": {"kr": 0.4},
                "trim": {"vt_mps": 30, "altitude_m": 200},
            })
        self.assertEqual(response.status_code, 200)
        run.assert_called_once_with(
            "cessna172",
            "linear",
            "dutch_roll",
            gains={"kr": 0.4},
            aero="tornado",
            trim=(30.0, 200.0),
        )
        body = response.json()
        self.assertEqual(
            set(body),
            {
                "plane", "law", "task", "aero", "trim", "runs",
                "reference", "metrics", "stopped_at", "stop_reason",
            },
        )
        self.assertIsNone(body["stopped_at"])
        self.assertIsNone(body["stop_reason"])
        self.assertEqual(
            set(body["trim"]),
            {"vt_mps", "altitude_m", "alpha_rad", "theta_rad", "controls"},
        )
        self.assertEqual(set(body["trim"]["controls"]), set(CHANNELS))
        self.assertEqual(body["trim"]["alpha_rad"], 0.04)
        self.assertEqual(body["trim"]["theta_rad"], 0.05)
        self.assertEqual(set(body["runs"]), {"linear", "nonlinear"})
        for series in body["runs"].values():
            self.assertEqual(set(series), set(SERIES))
            lengths = {len(series[key]) for key in SERIES}
            self.assertEqual(lengths, {4})
        self.assertEqual(
            set(body["reference"]),
            {"signal", "time", "values"},
        )
        self.assertEqual(body["reference"]["signal"], "beta")
        mode = body["metrics"]["open_loop_modes"][0]
        self.assertEqual(set(mode), {"name", "wn", "zeta", "real", "imag"})
        self.assertEqual(mode["name"], "dutch_roll")
        self.assertEqual(len(body["metrics"]["trims"]), 2)
        self.assertEqual(set(body["metrics"]["trims"][0]["controls"]), set(CHANNELS))

    def test_real_cessna_dutch_roll_returns_both_runs(self) -> None:
        # Dispatch patched, or the linear trace omitted for the linear law.
        response = self.client.post("/api/runs", json=_RUN)
        self.assertEqual(response.status_code, 200, response.text)
        body = response.json()
        self.assertEqual(body["plane"], "cessna172")
        self.assertEqual(body["law"], "linear")
        self.assertEqual(body["task"], "dutch_roll")
        self.assertEqual(set(body["runs"]), {"linear", "nonlinear"})
        for series in body["runs"].values():
            self.assertEqual(set(series), set(SERIES))
            lengths = {len(series[key]) for key in SERIES}
            self.assertEqual(len(lengths), 1)
            self.assertGreater(lengths.pop(), 1)


class TestMain(unittest.TestCase):
    def test_main_binds_loopback_8765(self) -> None:
        # Host or port drifted from the spec.
        with mock.patch("flightbench.api.uvicorn.run") as run:
            main()
        run.assert_called_once_with(app, host="127.0.0.1", port=8765)


if __name__ == "__main__":
    unittest.main()
