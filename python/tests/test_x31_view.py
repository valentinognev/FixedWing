"""The X-31 picture is a path-fixed frame, not a camera welded to the aircraft."""
import sys
import unittest
from pathlib import Path

import numpy as np

_PY = Path(__file__).resolve().parents[1]
if str(_PY) not in sys.path:
    sys.path.insert(0, str(_PY))

import x31_numpy_compat  # noqa: E402,F401

from x31.quaternion import body_321_to_q  # noqa: E402
from x31.params import physical  # noqa: E402
from x31_view import (  # noqa: E402
    apply_follow,
    apply_limits,
    draw_follow,
    follow_limits,
    follow_span,
    frame_limits,
    place,
    retarget,
    trail,
)


class TestFrame(unittest.TestCase):
    def test_limits_span_the_path_rather_than_one_airframe(self) -> None:
        points = np.array([
            [0.0, 0.0, -100.0],
            [800.0, 40.0, -90.0],
        ])
        north, east, down = frame_limits(points, margin=30.0)
        self.assertLessEqual(north[0], 0.0)
        self.assertGreaterEqual(north[1], 800.0)
        self.assertGreater(north[1] - north[0], 200.0)
        self.assertGreater(east[0], east[1])
        self.assertGreater(down[0], down[1])

    def test_moving_the_airframe_does_not_move_the_axes(self) -> None:
        import matplotlib

        matplotlib.use("Agg", force=True)
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        points = np.array([
            [0.0, 0.0, -100.0],
            [800.0, 40.0, -90.0],
        ])
        limits = frame_limits(points, margin=40.0)
        fig = plt.figure()
        try:
            ax = fig.add_subplot(111, projection="3d")
            q0 = body_321_to_q(0.0, 0.1, 0.0)
            q1 = body_321_to_q(0.4, -0.3, 1.0)
            artists = []
            for faces in place(q0, points[0]):
                collection = Poly3DCollection(faces)
                ax.add_collection3d(collection)
                artists.append(collection)
            apply_limits(ax, limits)
            before = (ax.get_xlim(), ax.get_ylim(), ax.get_zlim())
            retarget(artists, q1, points[1])
            after = (ax.get_xlim(), ax.get_ylim(), ax.get_zlim())
            self.assertEqual(before, after)
            nose = place(q1, points[1])[0][0][0]
            self.assertGreater(float(nose[0]), 700.0)
        finally:
            plt.close(fig)

    def test_a_translated_origin_translates_every_vertex(self) -> None:
        q = body_321_to_q(0.2, -0.1, 0.4)
        origin = np.array([0.0, 0.0, -50.0])
        shift = np.array([120.0, -30.0, 10.0])
        stayed = np.vstack([vertex for faces in place(q, origin) for face in faces for vertex in face])
        moved = np.vstack([vertex for faces in place(q, origin + shift) for face in faces for vertex in face])
        np.testing.assert_allclose(moved - stayed, np.broadcast_to(shift, moved.shape), atol=1e-9)


class TestFollow(unittest.TestCase):
    def test_the_window_stays_on_the_aircraft_and_drops_the_rest_of_the_path(self) -> None:
        span = follow_span()
        length = float(physical()["size"]["fuselage"]["length"])
        self.assertGreater(span, 2.0 * length)
        self.assertLess(span, 8.0 * length)
        here = np.array([800.0, 40.0, -90.0])
        north, east, down = follow_limits(here, span)
        self.assertAlmostEqual(0.5 * (north[0] + north[1]), 800.0)
        self.assertAlmostEqual(north[1] - north[0], span)
        self.assertLess(north[0], 800.0)
        self.assertGreater(north[1], 800.0)
        self.assertGreater(north[0], 0.0)
        self.assertGreater(east[0], east[1])
        self.assertGreater(down[0], down[1])
        self.assertAlmostEqual(0.5 * (east[0] + east[1]), 40.0)
        self.assertAlmostEqual(0.5 * (down[0] + down[1]), -90.0)

    def test_following_moves_the_axes_and_holds_the_third_person_angle(self) -> None:
        import matplotlib

        matplotlib.use("Agg", force=True)
        import matplotlib.pyplot as plt

        fig = plt.figure()
        try:
            ax = fig.add_subplot(111, projection="3d")
            apply_follow(ax, np.array([0.0, 0.0, -100.0]), follow_span())
            apply_follow(ax, np.array([800.0, 40.0, -90.0]), follow_span())
            north, east, down = (ax.get_xlim(), ax.get_ylim(), ax.get_zlim())
            self.assertAlmostEqual(0.5 * (north[0] + north[1]), 800.0, places=6)
            self.assertGreater(north[0], 0.0)
            self.assertGreater(east[0], east[1])
            self.assertGreater(down[0], down[1])
            self.assertAlmostEqual(ax.elev, 30.0)
            self.assertAlmostEqual(ax.azim, 45.0)
        finally:
            plt.close(fig)

    def test_the_trail_is_the_recent_points_behind_the_aircraft(self) -> None:
        points = np.array([[float(i), 0.0, -100.0] for i in range(10)])
        recent = trail(points, frame=6, count=4)
        np.testing.assert_allclose(recent[:, 0], [3.0, 4.0, 5.0, 6.0])
        early = trail(points, frame=1, count=4)
        np.testing.assert_allclose(early[:, 0], [0.0, 1.0])

    def test_the_drawn_frame_follows_the_sample_and_keeps_a_trail(self) -> None:
        import matplotlib

        matplotlib.use("Agg", force=True)
        import matplotlib.pyplot as plt

        out = {
            "n_m": np.array([0.0, 400.0, 800.0]),
            "e_m": np.array([0.0, 10.0, 40.0]),
            "d_m": np.array([-100.0, -95.0, -90.0]),
            "phi": np.array([0.0, 0.1, 0.2]),
            "theta": np.array([0.1, 0.0, -0.1]),
            "psi": np.array([0.0, 0.4, 0.8]),
        }
        fig = plt.figure()
        try:
            ax = fig.add_subplot(111, projection="3d")
            update = draw_follow(out, ax)
            update(2)
            north = ax.get_xlim()
            self.assertAlmostEqual(0.5 * (north[0] + north[1]), 800.0, places=6)
            self.assertGreater(north[0], 400.0)
            self.assertAlmostEqual(ax.elev, 30.0)
            self.assertAlmostEqual(ax.azim, 45.0)
            trail_line = ax.lines[0]
            xs = np.asarray(trail_line.get_data_3d()[0], dtype=float)
            np.testing.assert_allclose(xs, [0.0, 400.0, 800.0])
        finally:
            plt.close(fig)


if __name__ == "__main__":
    unittest.main()
