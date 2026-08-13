"""One-shot spawn velocity for in-air gz plane (loaded by PythonSystemLoader)."""
from __future__ import annotations

import os
from collections.abc import Mapping

def velocity_from_env(env: Mapping[str, str] | None = None) -> tuple[float, float, float]:
    src = os.environ if env is None else env
    return (
        float(src.get("FW_GZ_SPAWN_VX", "0")),
        float(src.get("FW_GZ_SPAWN_VY", "0")),
        float(src.get("FW_GZ_SPAWN_VZ", "0")),
    )


try:
    from gz.math import Vector3d
    from gz.sim import Link, Model
except ImportError:
    Vector3d = None  # type: ignore[misc, assignment]
    Link = None  # type: ignore[misc, assignment]
    Model = None  # type: ignore[misc, assignment]


class RaceSpawnVelocity(object):
    def __init__(self) -> None:
        self._done = False
        self.link = None

    def configure(self, entity, sdf, ecm, event_mgr) -> None:
        if Model is None or Link is None:
            return
        model = Model(entity)
        self.link = Link(model.canonical_link(ecm))

    def pre_update(self, info, ecm) -> None:
        if self._done or self.link is None:
            return
        if info.paused:
            return
        if Vector3d is None:
            return
        vx, vy, vz = velocity_from_env()
        self.link.enable_velocity_checks(ecm, True)
        self.link.set_linear_velocity(ecm, Vector3d(vx, vy, vz))
        self._done = True


def get_system():
    return RaceSpawnVelocity()
