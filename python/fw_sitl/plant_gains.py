"""Controller constants keyed by plant + airframe.

Tables live in ``plants/{plant_id}.jsonc``. JSBSim headless is
``jsbsim_rascal``; JSBSim+FG ``--viz`` is ``jsbsim_rascal_viz`` (same
FDM, HSV keeps the altitude mix). Changing ``--model`` / YASim /
Gazebo / ``--xplane`` selects a different file. Unknown ids fail.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from fw_sitl.attitude_pid import AttitudePid


KNOWN_PLANT_IDS = (
    "jsbsim_rascal",
    "jsbsim_rascal_viz",
    "yasim_rascal",
    "gz_rc_cessna",
    "gz_advanced_plane",
    "xplane_cessna172",
)


@dataclass(frozen=True)
class PlantGains:
    plant_id: str
    pid_kp: float
    pid_ki: float
    pid_kd: float
    bank_kp_heading: float
    bank_kp_cross_track: float
    bank_xt_lookahead_m: float
    bank_max_roll_rad: float
    bank_kp_alt: float
    bank_max_pitch_rad: float
    att_max_pitch_rad: float
    att_los_max_pitch_rad: float
    cruise_thrust: float
    climb_thrust_per_m: float
    min_thrust: float
    max_thrust: float
    speed_mps: float
    approach_speed_mps: float
    slow_range_m: float
    speed_thrust_per_mps: float
    lookahead_m: float
    fw_airspd_min: float
    fw_airspd_trim: float
    fw_airspd_max: float
    visual_lock_kp_alt: float
    px4_inner: tuple[tuple[str, float], ...]
    mass_kg: float
    wing_area_m2: float
    cd0: float
    k_induced: float
    cl_alpha: float
    rho_kg_m3: float
    t_max_n: float
    v_stall_mps: float
    pp_gain: float
    thrust_target_frac: float
    v_min_mult: float
    v_recover_mult: float
    v_up_mps_s: float
    attitude_from_accel: str
    alpha_small_rad: float

    def fingerprint(self) -> tuple:
        return (
            self.pid_kp,
            self.pid_ki,
            self.pid_kd,
            self.bank_kp_heading,
            self.bank_kp_cross_track,
            self.bank_xt_lookahead_m,
            self.bank_max_roll_rad,
            self.bank_kp_alt,
            self.bank_max_pitch_rad,
            self.att_max_pitch_rad,
            self.att_los_max_pitch_rad,
            self.cruise_thrust,
            self.climb_thrust_per_m,
            self.min_thrust,
            self.max_thrust,
            self.speed_mps,
            self.approach_speed_mps,
            self.slow_range_m,
            self.speed_thrust_per_mps,
            self.lookahead_m,
            self.fw_airspd_min,
            self.fw_airspd_trim,
            self.fw_airspd_max,
            self.visual_lock_kp_alt,
            self.px4_inner,
            self.mass_kg,
            self.wing_area_m2,
            self.cd0,
            self.k_induced,
            self.cl_alpha,
            self.rho_kg_m3,
            self.t_max_n,
            self.v_stall_mps,
            self.pp_gain,
            self.thrust_target_frac,
            self.v_min_mult,
            self.v_recover_mult,
            self.v_up_mps_s,
            self.attitude_from_accel,
            self.alpha_small_rad,
        )

    def make_pid(self) -> AttitudePid:
        return AttitudePid(kp=self.pid_kp, ki=self.pid_ki, kd=self.pid_kd)

    def path_kwargs(self) -> dict[str, float]:
        return {
            "kp_heading": self.bank_kp_heading,
            "kp_cross_track": self.bank_kp_cross_track,
            "max_roll": self.bank_max_roll_rad,
            "kp_alt": self.bank_kp_alt,
            "max_pitch": self.att_max_pitch_rad,
            "xt_lookahead_m": self.bank_xt_lookahead_m,
        }

    def los_kwargs(self) -> dict[str, float]:
        return {
            "kp_heading": self.bank_kp_heading,
            "max_roll": self.bank_max_roll_rad,
            "max_pitch": self.att_los_max_pitch_rad,
            "kp_alt": self.bank_kp_alt,
        }

    def thrust_kwargs(self) -> dict[str, float]:
        return {
            "cruise": self.cruise_thrust,
            "climb_gain": self.climb_thrust_per_m,
            "min_t": self.min_thrust,
            "max_t": self.max_thrust,
            "speed_gain": self.speed_thrust_per_mps,
        }

    def px4_overlay(self) -> tuple[tuple[str, float], ...]:
        airspeed = (
            ("FW_AIRSPD_MIN", self.fw_airspd_min),
            ("FW_AIRSPD_TRIM", self.fw_airspd_trim),
            ("FW_AIRSPD_MAX", self.fw_airspd_max),
        )
        return airspeed + self.px4_inner


def plant_id_from_flags(
    *,
    gz: bool = False,
    yasim: bool = False,
    viz: bool = False,
    xplane: bool = False,
    gz_model: str = "rc_cessna",
) -> str:
    nflags = int(bool(gz)) + int(bool(yasim)) + int(bool(viz)) + int(bool(xplane))
    if nflags > 1:
        raise ValueError("gz, yasim, viz, and xplane are mutually exclusive")
    if gz:
        model = str(gz_model).strip().lower()
        if model == "rc_cessna":
            return "gz_rc_cessna"
        if model == "advanced_plane":
            return "gz_advanced_plane"
        raise ValueError(f"unknown gz model {gz_model!r}")
    if yasim:
        return "yasim_rascal"
    if viz:
        return "jsbsim_rascal_viz"
    if xplane:
        return "xplane_cessna172"
    return "jsbsim_rascal"


def load_plant_gains(plant_id: str) -> PlantGains:
    from fw_sitl.plant_loader import load_plant_jsonc, plant_gains_from_dict

    pid = str(plant_id)
    if pid not in KNOWN_PLANT_IDS:
        raise KeyError(f"unknown plant {pid!r}; expected one of {KNOWN_PLANT_IDS}")
    path = Path(__file__).resolve().parent / "plants" / f"{pid}.jsonc"
    if not path.is_file():
        raise FileNotFoundError(f"plant file missing: {path}")
    gains = plant_gains_from_dict(load_plant_jsonc(path))
    if str(gains.plant_id) != pid:
        raise ValueError(
            f"plant file {path.name} has plant_id {gains.plant_id!r}, expected {pid!r}"
        )
    return gains
