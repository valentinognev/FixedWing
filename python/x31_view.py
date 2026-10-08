"""World-fixed matplotlib view of an X-31 trajectory.

The upstream port animation rotates every vertex into earth axes and then
re-aims the limits on the aircraft each frame, with a window the size of the
airframe. The camera is welded to the airplane, so only the attitude appears
to change. This view sets the limits once, from the whole path, and the
airframe is translated through that frame. It is the host-side picture, the
same role as the F-16 ``--anim`` replay, and it does not start a simulator.
"""
from __future__ import annotations

import math

import numpy as np

from x31.params import physical
from x31.quaternion import body_321_to_q, rotate_body_to_earth

_COLORS = ("#cccccc", "#ffcccc", "#ccffcc", "#ccccff")


def frame_limits(points, margin: float) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
    """Axis limits that contain every point, padded by ``margin``.

    North runs low to high. East and down run high to low, which puts east to
    the right of a north-up plot and altitude up on the screen. The limits
    come from the path, not from the current aircraft position.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    if pts.size == 0:
        raise ValueError("frame_limits needs at least one point")
    if not np.all(np.isfinite(pts)):
        raise ValueError("frame_limits points must be finite")
    if not math.isfinite(margin) or margin < 0.0:
        raise ValueError(f"frame_limits margin {margin!r} is not a finite non-negative number")
    lo = pts.min(axis=0) - margin
    hi = pts.max(axis=0) + margin
    return (
        (float(lo[0]), float(hi[0])),
        (float(hi[1]), float(lo[1])),
        (float(hi[2]), float(lo[2])),
    )


def apply_limits(ax, limits) -> None:
    ax.set_autoscale_on(False)
    ax.set_xlim(limits[0])
    ax.set_ylim(limits[1])
    ax.set_zlim(limits[2])


def retarget(artists, q, origin) -> None:
    """Move an existing airframe. Does not touch the axes limits."""
    for collection, faces in zip(artists, place(q, origin)):
        collection.set_verts(faces)


def body_faces() -> list[list[np.ndarray]]:
    """Fuselage, wing, canard and fin, in body axes, from the port's size table."""
    size = physical()["size"]
    return [
        _fuselage(size["fuselage"]),
        _panel(size["mainwing"]),
        _panel(size["canard"]),
        _fin(size["fin"]),
    ]


def place(q, origin) -> list[list[np.ndarray]]:
    """Body faces rotated into earth and translated to ``origin``."""
    origin = np.asarray(origin, dtype=float)
    placed = []
    for faces in body_faces():
        placed.append([_place_face(q, face, origin) for face in faces])
    return placed


def _place_face(q, face, origin) -> np.ndarray:
    earth = np.vstack([rotate_body_to_earth(q, vertex) for vertex in face])
    return earth + origin


def show(out: dict):
    """Animate ``out`` in a fixed frame. Returns the animation object."""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    points = np.column_stack([out["n_m"], out["e_m"], out["d_m"]])
    attitudes = [
        body_321_to_q(float(phi), float(theta), float(psi))
        for phi, theta, psi in zip(out["phi"], out["theta"], out["psi"], strict=True)
    ]
    faces0 = place(attitudes[0], points[0])
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection="3d")
    artists = []
    for faces, color in zip(faces0, _COLORS):
        collection = Poly3DCollection(
            faces, facecolors=color, edgecolors="k", linewidths=0.3, alpha=0.95
        )
        ax.add_collection3d(collection)
        artists.append(collection)
    apply_limits(ax, frame_limits(points, margin=_margin(points)))
    ax.set_xlabel("north")
    ax.set_ylabel("east")
    ax.set_zlabel("down")

    def update(frame: int):
        retarget(artists, attitudes[frame], points[frame])
        return artists

    animation = FuncAnimation(
        fig, update, frames=points.shape[0], interval=50, blit=False
    )
    plt.show()
    return animation


def _margin(points: np.ndarray) -> float:
    span = float(np.max(points.max(axis=0) - points.min(axis=0)))
    return max(40.0, 0.05 * span)


def _fuselage(spec: dict) -> list[np.ndarray]:
    length = float(spec["length"])
    radius = float(spec["radius"]) / 2.0
    origin_x = float(spec["offset_x"])
    count = 12
    nose = _ring(origin_x + length / 2.0, radius, count)
    tail = _ring(origin_x - length / 2.0, radius, count)
    faces = []
    for index in range(count):
        nxt = (index + 1) % count
        faces.append(np.vstack([nose[index], nose[nxt], tail[nxt], tail[index]]))
    faces.append(nose)
    faces.append(tail)
    return faces


def _ring(x_station: float, radius: float, count: int) -> np.ndarray:
    theta = np.linspace(0.0, 2.0 * math.pi, count, endpoint=False)
    return np.column_stack([
        np.full(count, x_station),
        radius * np.cos(theta),
        radius * np.sin(theta),
    ])


def _panel(spec: dict) -> list[np.ndarray]:
    origin_x = float(spec["offset_x"])
    origin_z = float(spec["offset_z"])
    half_chord = float(spec["center_chord"]) / 2.0
    half_span = float(spec["span"]) / 2.0
    triangle = np.array([
        [origin_x - half_chord, -half_span, origin_z],
        [origin_x + half_chord, 0.0, origin_z],
        [origin_x - half_chord, half_span, origin_z],
    ])
    return [triangle, triangle[::-1]]


def _fin(spec: dict) -> list[np.ndarray]:
    origin_x = float(spec["offset_x"])
    origin_z = float(spec["offset_z"])
    half_base = float(spec["base_width"]) / 2.0
    tip = float(spec["tip_width"])
    tip_z = origin_z - float(spec["height"])
    aft = origin_x - half_base - 1.0
    quad = np.array([
        [origin_x + half_base, 0.0, origin_z],
        [origin_x - half_base, 0.0, origin_z],
        [aft, 0.0, tip_z],
        [aft + tip, 0.0, tip_z],
    ])
    return [quad, quad[::-1]]
