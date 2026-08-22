"""Selectable chase attitude controllers."""
from __future__ import annotations

from typing import Any

from fw_sitl.attitude_pid import AttitudePid
from fw_sitl.body_cmd_bridge import BodyCmdBridge
from fw_sitl.controllers.pure_pursuit_quat import PurePursuitQuatController

AttitudeChaseController = PurePursuitQuatController  # default / compat alias
from fw_sitl.controllers.race_euler import RaceEulerController
from fw_sitl.controllers.race_quat import RaceQuatController
from fw_sitl.flight_setup import (
    DEFAULT_ATTITUDE_FORMAT,
    DEFAULT_CONTROLLER,
    KNOWN_CONTROLLER_IDS,
)
from fw_sitl.plant_gains import PlantGains
from fw_sitl.px4_att_cascade import Px4FwAttCascade

CONTROLLER_REGISTRY: dict[str, type] = {
    "pure_pursuit_quat": PurePursuitQuatController,
    "race_quat": RaceQuatController,
    "race_euler": RaceEulerController,
}


def build_controller(
    name: str,
    bridge: BodyCmdBridge,
    *,
    speed_mps: float,
    plant: PlantGains | None = None,
    pid: AttitudePid | None = None,
    cascade: Px4FwAttCascade | None = None,
    cmd_mode: str = "attitude",
    attitude_format: str = DEFAULT_ATTITUDE_FORMAT,
) -> Any:
    """Construct a chase attitude controller by registry name."""
    key = str(name).strip()
    if key not in CONTROLLER_REGISTRY:
        known = ", ".join(sorted(KNOWN_CONTROLLER_IDS))
        raise ValueError(
            f"unknown controller {name!r}; expected one of: {known}"
        )
    cls = CONTROLLER_REGISTRY[key]
    return cls(
        bridge,
        speed_mps=speed_mps,
        plant=plant,
        pid=pid,
        cascade=cascade,
        cmd_mode=cmd_mode,
        attitude_format=attitude_format,
    )


__all__ = [
    "CONTROLLER_REGISTRY",
    "DEFAULT_CONTROLLER",
    "AttitudeChaseController",
    "PurePursuitQuatController",
    "RaceQuatController",
    "RaceEulerController",
    "build_controller",
]
