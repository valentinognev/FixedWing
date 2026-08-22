"""Resolve a calibration sim (plant + launcher script) from flightSetup.json
plus CLI plant flags, mirroring how the race launchers pick a SITL plant.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from fw_sitl.flight_setup import load_flight_setup, resolve_race_sim
from fw_sitl.plant_gains import plant_id_from_flags
from fw_sitl.sim_lifecycle import SCRIPTS_DIR


@dataclass(frozen=True)
class CalibrationSim:
    plant_id: str
    sim_script: Path
    extra_args: tuple[str, ...]
    kill_target: str


def resolve_calibration_sim(
    *,
    setup_path: Path,
    platform: str | None,
    gz_model: str | None,
) -> CalibrationSim:
    """CLI ``platform``/``gz_model`` override ``setup_path``'s ``sim`` block."""
    setup = load_flight_setup(setup_path)
    plat, model, _duration_s = resolve_race_sim(
        setup, platform=platform, gz_model=gz_model
    )

    if plat == "jsbsim":
        return CalibrationSim(
            plant_id=plant_id_from_flags(),
            sim_script=SCRIPTS_DIR / "runSimJsbsimRascal.sh",
            extra_args=(),
            kill_target="--jsbsim",
        )
    if plat == "viz":
        return CalibrationSim(
            plant_id=plant_id_from_flags(viz=True),
            sim_script=SCRIPTS_DIR / "runSimJsbsimRascal.sh",
            extra_args=("--viz",),
            kill_target="--jsbsim",
        )
    if plat == "yasim":
        return CalibrationSim(
            plant_id=plant_id_from_flags(yasim=True),
            sim_script=SCRIPTS_DIR / "runSimYasimRascal.sh",
            extra_args=(),
            kill_target="--fg",
        )
    if plat == "gz":
        extra: tuple[str, ...] = () if model == "rc_cessna" else ("--model", model)
        return CalibrationSim(
            plant_id=plant_id_from_flags(gz=True, gz_model=model),
            sim_script=SCRIPTS_DIR / "runSimGzPlane.sh",
            extra_args=extra,
            kill_target="--gz",
        )
    raise ValueError(f"unsupported calibration sim platform: {plat!r}")
