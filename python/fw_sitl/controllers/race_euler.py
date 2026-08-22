"""Race LOS chase that closes Euler roll/pitch error every tick."""
from __future__ import annotations

from fw_sitl.controllers.race_quat import RaceQuatController


class RaceEulerController(RaceQuatController):
    """Same LOS/path as race_quat; in-view keeps cascade I-state and commands q_cmd."""

    _close_in_view_euler = True
