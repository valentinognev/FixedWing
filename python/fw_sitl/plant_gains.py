"""Controller constants keyed by plant + airframe.

JSBSim headless is ``jsbsim_rascal``; JSBSim+FG ``--viz`` is
``jsbsim_rascal_viz`` (same FDM, HSV keeps the altitude mix). Changing
``--model`` / YASim / Gazebo selects a different table. Unknown ids fail.
"""
from __future__ import annotations

from dataclasses import dataclass

from fw_sitl.attitude_pid import AttitudePid


KNOWN_PLANT_IDS = (
    "jsbsim_rascal",
    "jsbsim_rascal_viz",
    "yasim_rascal",
    "gz_rc_cessna",
    "gz_advanced_plane",
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
    gz_model: str = "rc_cessna",
) -> str:
    nflags = int(bool(gz)) + int(bool(yasim)) + int(bool(viz))
    if nflags > 1:
        raise ValueError("gz, yasim, and viz are mutually exclusive")
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
    return "jsbsim_rascal"


def load_plant_gains(plant_id: str) -> PlantGains:
    try:
        return _PLANTS[str(plant_id)]
    except KeyError as exc:
        raise KeyError(
            f"unknown plant {plant_id!r}; expected one of {KNOWN_PLANT_IDS}"
        ) from exc


_JSB_INNER: tuple[tuple[str, float], ...] = (
    ("FW_PR_FF", 0.40),
    ("FW_PR_I", 0.05),
    ("FW_PR_P", 0.05),
    ("FW_R_TC", 0.45),
    ("FW_RR_FF", 0.50),
    ("FW_RR_I", 0.18),
    ("FW_RR_P", 0.15),
    ("FW_THR_TRIM", 0.62),
)

# Same Rascal airframe, slower YASim FDM: less P, more I/D, slightly more thrust.
_YAS_INNER: tuple[tuple[str, float], ...] = (
    ("FW_PR_FF", 0.35),
    ("FW_PR_I", 0.08),
    ("FW_PR_P", 0.04),
    ("FW_R_TC", 0.50),
    ("FW_RR_FF", 0.48),
    ("FW_RR_I", 0.18),
    ("FW_RR_P", 0.14),
    ("FW_THR_TRIM", 0.68),
)

# PX4 v1.17 4003_gz_rc_cessna rate/TECS snapshot.
_CESSNA_INNER: tuple[tuple[str, float], ...] = (
    ("FW_PR_P", 0.9),
    ("FW_PR_FF", 0.2),
    ("FW_PR_I", 0.5),
    ("FW_RR_FF", 0.5),
    ("FW_RR_P", 0.4),
    ("FW_RR_I", 0.7),
    ("FW_R_RMAX", 56.15),
    ("FW_YR_FF", 0.3),
    ("FW_YR_P", 1.3),
    ("FW_YR_I", 0.7),
    ("FW_PSP_OFF", 0.0),
    ("FW_P_LIM_MIN", -15.0),
    ("FW_T_CLMB_MAX", 5.0),
    ("FW_T_SINK_MAX", 3.5),
    ("FW_T_SINK_MIN", 3.0),
    ("FW_THR_TRIM", 0.55),
)

# PX4 v1.17 4008_gz_advanced_plane rate/TECS/throttle snapshot.
_ADV_INNER: tuple[tuple[str, float], ...] = (
    ("FW_PR_FF", 0.08),
    ("FW_PR_I", 0.3),
    ("FW_PR_P", 0.08),
    ("FW_RR_FF", 0.05),
    ("FW_RR_I", 0.2),
    ("FW_RR_P", 0.03),
    ("FW_YR_FF", 0.3),
    ("FW_YR_I", 0.4),
    ("FW_YR_P", 0.2),
    ("FW_PSP_OFF", 2.0),
    ("FW_P_LIM_MAX", 32.0),
    ("FW_P_LIM_MIN", -15.0),
    ("FW_THR_MIN", 0.05),
    ("FW_THR_TRIM", 0.25),
    ("FW_THR_MAX", 0.6),
    ("FW_T_CLMB_R_SP", 5.0),
    ("FW_T_CLMB_MAX", 6.0),
    ("FW_T_SINK_MAX", 2.7),
    ("FW_T_SINK_MIN", 2.2),
)


_PLANTS: dict[str, PlantGains] = {
    "jsbsim_rascal": PlantGains(
        plant_id="jsbsim_rascal",
        pid_kp=0.8,
        pid_ki=0.12,
        pid_kd=0.04,
        bank_kp_heading=1.0,
        bank_kp_cross_track=0.003,
        bank_xt_lookahead_m=180.0,
        bank_max_roll_rad=0.62,
        bank_kp_alt=0.028,
        bank_max_pitch_rad=0.12,
        att_max_pitch_rad=0.35,
        att_los_max_pitch_rad=0.70,
        cruise_thrust=0.62,
        climb_thrust_per_m=0.020,
        min_thrust=0.22,
        max_thrust=1.0,
        speed_mps=18.0,
        approach_speed_mps=15.0,
        slow_range_m=180.0,
        speed_thrust_per_mps=0.05,
        lookahead_m=500.0,
        fw_airspd_min=10.0,
        fw_airspd_trim=18.0,
        fw_airspd_max=40.0,
        visual_lock_kp_alt=0.020,
        px4_inner=_JSB_INNER,
    ),
    "jsbsim_rascal_viz": PlantGains(
        plant_id="jsbsim_rascal_viz",
        pid_kp=0.8,
        pid_ki=0.12,
        pid_kd=0.04,
        bank_kp_heading=1.0,
        bank_kp_cross_track=0.003,
        bank_xt_lookahead_m=180.0,
        bank_max_roll_rad=0.62,
        bank_kp_alt=0.028,
        bank_max_pitch_rad=0.12,
        att_max_pitch_rad=0.35,
        att_los_max_pitch_rad=0.70,
        cruise_thrust=0.62,
        climb_thrust_per_m=0.020,
        min_thrust=0.22,
        max_thrust=1.0,
        speed_mps=18.0,
        approach_speed_mps=15.0,
        slow_range_m=180.0,
        speed_thrust_per_mps=0.05,
        lookahead_m=500.0,
        fw_airspd_min=10.0,
        fw_airspd_trim=18.0,
        fw_airspd_max=40.0,
        visual_lock_kp_alt=0.028,
        px4_inner=_JSB_INNER,
    ),
    "yasim_rascal": PlantGains(
        plant_id="yasim_rascal",
        pid_kp=0.65,
        pid_ki=0.10,
        pid_kd=0.06,
        bank_kp_heading=1.2,
        bank_kp_cross_track=0.003,
        bank_xt_lookahead_m=180.0,
        bank_max_roll_rad=0.40,
        bank_kp_alt=0.028,
        bank_max_pitch_rad=0.12,
        att_max_pitch_rad=0.35,
        att_los_max_pitch_rad=0.70,
        cruise_thrust=0.68,
        climb_thrust_per_m=0.025,
        min_thrust=0.18,
        max_thrust=1.0,
        speed_mps=28.0,
        approach_speed_mps=20.0,
        slow_range_m=280.0,
        speed_thrust_per_mps=0.06,
        lookahead_m=500.0,
        fw_airspd_min=5.0,
        fw_airspd_trim=28.0,
        fw_airspd_max=50.0,
        visual_lock_kp_alt=0.028,
        px4_inner=_YAS_INNER,
    ),
    "gz_rc_cessna": PlantGains(
        plant_id="gz_rc_cessna",
        pid_kp=1.0,
        pid_ki=0.10,
        pid_kd=0.03,
        bank_kp_heading=1.4,
        bank_kp_cross_track=0.003,
        bank_xt_lookahead_m=150.0,
        bank_max_roll_rad=0.55,
        bank_kp_alt=0.030,
        bank_max_pitch_rad=0.12,
        att_max_pitch_rad=0.35,
        att_los_max_pitch_rad=0.70,
        cruise_thrust=0.62,
        climb_thrust_per_m=0.018,
        min_thrust=0.22,
        max_thrust=1.0,
        speed_mps=16.0,
        approach_speed_mps=12.0,
        slow_range_m=180.0,
        speed_thrust_per_mps=0.07,
        lookahead_m=500.0,
        fw_airspd_min=8.0,
        fw_airspd_trim=16.0,
        fw_airspd_max=25.0,
        visual_lock_kp_alt=0.030,
        px4_inner=_CESSNA_INNER,
    ),
    "gz_advanced_plane": PlantGains(
        plant_id="gz_advanced_plane",
        pid_kp=0.7,
        pid_ki=0.10,
        pid_kd=0.05,
        bank_kp_heading=1.3,
        bank_kp_cross_track=0.003,
        bank_xt_lookahead_m=200.0,
        bank_max_roll_rad=0.45,
        bank_kp_alt=0.022,
        bank_max_pitch_rad=0.12,
        att_max_pitch_rad=0.35,
        att_los_max_pitch_rad=0.70,
        cruise_thrust=0.50,
        climb_thrust_per_m=0.010,
        min_thrust=0.20,
        max_thrust=1.0,
        speed_mps=20.0,
        approach_speed_mps=14.0,
        slow_range_m=160.0,
        speed_thrust_per_mps=0.04,
        lookahead_m=500.0,
        fw_airspd_min=10.0,
        fw_airspd_trim=20.0,
        fw_airspd_max=35.0,
        visual_lock_kp_alt=0.022,
        px4_inner=_ADV_INNER,
    ),
}
