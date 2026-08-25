"""Load and validate balloon-race flightSetup.json."""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

# Defaults match python/flightSetup.json / plan schema.
DEFAULT_ZMQ_IMAGE = "tcp://127.0.0.1:5555"
DEFAULT_ZMQ_COLOR = "tcp://127.0.0.1:5556"
DEFAULT_ZMQ_TRACK = "tcp://127.0.0.1:5557"
DEFAULT_ZMQ_POSE = "tcp://127.0.0.1:5558"

DEFAULT_HFOV_DEG = 90.0
DEFAULT_VFOV_DEG = 70.0
DEFAULT_AZIMUTH_DEG = 0.0
DEFAULT_ELEVATION_DEG = 0.0
DEFAULT_WIDTH_PX = 640
DEFAULT_HEIGHT_PX = 480
DEFAULT_CAMERA_RATE_HZ = 10.0
DEFAULT_FG_WINDOW_PATTERN = "FlightGear|fgfs"
DEFAULT_XP_WINDOW_PATTERN = "X-Plane|X-System"
# FG cockpit eyepoint sits behind canopy/struts; push eye forward (body +X / -Z view).
# Past Rascal nose/canopy (stock cockpit z≈+0.9 aft); pair with draw-mask hide.
DEFAULT_FG_EYE_FORWARD_M = 5.0
DEFAULT_FG_HIDE_AIRCRAFT = True
DEFAULT_RENDER_RATE_HZ = 20.0

DEFAULT_CONTROL_RATE_HZ = 20.0
DEFAULT_SPEED_MPS = 30.0
DEFAULT_PASS_RADIUS_M = 50.0
DEFAULT_LOOKAHEAD_M = 500.0
DEFAULT_ASSISTED_PRINT_PERIOD_S = 5.0
DEFAULT_STALE_TRACK_WARN_S = 10.0
DEFAULT_LAPS = 1
DEFAULT_DURATION_S = 60.0
DEFAULT_CMD_MODE = "velocity"
DEFAULT_ATTITUDE_FORMAT = "euler"
# Chase attitude law selected via guidance.controller (plant JSONC controllers.*).
DEFAULT_CONTROLLER = "pure_pursuit_quat"
# In-view HSV seeker principal (guidance.homing_law). FW_HOMING_LAW overrides.
# Keep names in sync with fw_sitl.controllers.cam_homing.KNOWN_HOMING_LAWS.
DEFAULT_HOMING_LAW = "lookat"
KNOWN_HOMING_LAWS = frozenset(
    {
        "lookat",
        "pd_lead",
        "pn",
        "bias",
        "el_first",
        "bang",
        "area_slow",
        "fpa_thrust",
        "filter",
        "apn",
    }
)

# During large heading error, hold altitude instead of chasing aim Z (level turn).
DEFAULT_ALT_PRESERVE_HEADING_ERR_DEG = 20.0
DEFAULT_BALLOON_DIAMETER_M = 10.0

DEFAULT_PIXEL_RMS_MAX_PX = 15.0
DEFAULT_PASS_TIME_TOL_S = 5.0
DEFAULT_PATH_RMS_MAX_M = 30.0

DEFAULT_SPAWN_NED = (0.0, 0.0, 0.0)
DEFAULT_SPAWN_HEADING_DEG = 0.0

_ALLOWED_CMD_MODES = frozenset({"velocity", "attitude", "rates"})
_ALLOWED_ATTITUDE_FORMATS = frozenset({"quat", "euler"})
KNOWN_CONTROLLER_IDS = frozenset({"race_quat", "pure_pursuit_quat", "race_euler"})
# Race launcher plant flags (CLI --viz/--yasim/--gz/--xplane; default headless jsbsim).
KNOWN_SIM_PLATFORMS = frozenset({"jsbsim", "viz", "yasim", "gz"})
DEFAULT_SIM_PLATFORM = "jsbsim"
KNOWN_GZ_MODELS = frozenset({"rc_cessna", "advanced_plane"})
DEFAULT_GZ_MODEL = "rc_cessna"
# Only Gazebo exposes selectable airframes; other platforms ignore sim.gz_model.
GZ_MODELS_BY_PLATFORM: dict[str, frozenset[str]] = {
    "gz": KNOWN_GZ_MODELS,
}


def gz_models_for_platform(platform: str) -> frozenset[str]:
    """Airframe ids selectable for ``platform`` (empty if the plant has no --model)."""
    return GZ_MODELS_BY_PLATFORM.get(str(platform).strip().lower(), frozenset())


def validate_gz_model_for_platform(platform: str, gz_model: str) -> None:
    """Raise ``ValueError`` with a clear message if ``gz_model`` is not usable."""
    plat = str(platform).strip().lower()
    model = str(gz_model).strip().lower()
    available = gz_models_for_platform(plat)
    if model in available:
        return
    if not available:
        raise ValueError(
            f"gz_model {model!r} is not available for platform {plat!r}; "
            "only platform 'gz' supports sim.gz_model / --model "
            f"(available for gz: {'|'.join(sorted(KNOWN_GZ_MODELS))})"
        )
    raise ValueError(
        f"gz_model {model!r} is not available for platform {plat!r}; "
        f"available: {'|'.join(sorted(available))}"
    )


@dataclass(frozen=True)
class ZmqEndpoints:
    image: str = DEFAULT_ZMQ_IMAGE
    color: str = DEFAULT_ZMQ_COLOR
    track: str = DEFAULT_ZMQ_TRACK
    # Gazebo model world pose (ENU), streamed continuously by gz_pose_bridge
    # from the physics-rate dynamic_pose/info topic. --gz only.
    pose: str = DEFAULT_ZMQ_POSE


@dataclass(frozen=True)
class BalloonSpec:
    ned: tuple[float, float, float]
    color: tuple[int, int, int]
    diameter_m: float = DEFAULT_BALLOON_DIAMETER_M


@dataclass(frozen=True)
class CameraSpec:
    hfov_deg: float = DEFAULT_HFOV_DEG
    vfov_deg: float = DEFAULT_VFOV_DEG
    # Seeker mount vs body +X (future gimbal). +azimuth right, +elevation up.
    azimuth_deg: float = DEFAULT_AZIMUTH_DEG
    elevation_deg: float = DEFAULT_ELEVATION_DEG
    width_px: int = DEFAULT_WIDTH_PX
    height_px: int = DEFAULT_HEIGHT_PX
    rate_hz: float = DEFAULT_CAMERA_RATE_HZ
    fg_window_pattern: str = DEFAULT_FG_WINDOW_PATTERN
    # X-Plane mss grab title/class regex (--mode xp).
    xp_window_pattern: str = DEFAULT_XP_WINDOW_PATTERN
    # FG viz: meters forward of model origin for eyepoint (clears fuselage/canopy).
    fg_eye_forward_m: float = DEFAULT_FG_EYE_FORWARD_M
    # Hide ownship mesh so capture matches fictional body camera (no cockpit frame).
    fg_hide_aircraft: bool = DEFAULT_FG_HIDE_AIRCRAFT


@dataclass(frozen=True)
class GuidanceSpec:
    control_rate_hz: float = DEFAULT_CONTROL_RATE_HZ
    speed_mps: float = DEFAULT_SPEED_MPS
    pass_radius_m: float = DEFAULT_PASS_RADIUS_M
    lookahead_m: float = DEFAULT_LOOKAHEAD_M
    assisted_print_period_s: float = DEFAULT_ASSISTED_PRINT_PERIOD_S
    stale_track_warn_s: float = DEFAULT_STALE_TRACK_WARN_S
    laps: int = DEFAULT_LAPS
    duration_s: float = DEFAULT_DURATION_S
    cmd_mode: str = DEFAULT_CMD_MODE
    # SET_ATTITUDE_TARGET packing when cmd_mode=attitude (quat | euler).
    attitude_format: str = DEFAULT_ATTITUDE_FORMAT
    # Attitude chase law id (race_quat | pure_pursuit_quat | race_euler).
    controller: str = DEFAULT_CONTROLLER
    # In-view camera seeker (lookat | pd_lead | pn | bias | el_first | bang |
    # area_slow | fpa_thrust | filter | apn). HSV dir_cam only.
    homing_law: str = DEFAULT_HOMING_LAW
    # |course−yaw| ≥ this (deg) → z_hold = current altitude during the turn.
    alt_preserve_heading_err_deg: float = DEFAULT_ALT_PRESERVE_HEADING_ERR_DEG


@dataclass(frozen=True)
class SpawnSpec:
    """Aircraft start in the same home-relative NED frame as balloons.

    heading_deg is compass / PX4 yaw: 0=north, 90=east. ned z=0 is the
    in-air cruise used by jsb_spawn / fg_spawn / GZ 500 m AGL; +z is down.
    """

    ned: tuple[float, float, float] = DEFAULT_SPAWN_NED
    heading_deg: float = DEFAULT_SPAWN_HEADING_DEG


@dataclass(frozen=True)
class SimSpec:
    """Which SITL plant the race launcher starts (overridable by CLI flags)."""

    platform: str = DEFAULT_SIM_PLATFORM
    gz_model: str = DEFAULT_GZ_MODEL
    # Race wall-clock length; CLI --duration overrides. 0 = no time limit.
    duration_s: float = DEFAULT_DURATION_S


@dataclass(frozen=True)
class VerificationSpec:
    pixel_rms_max_px: float = DEFAULT_PIXEL_RMS_MAX_PX
    pass_time_tol_s: float = DEFAULT_PASS_TIME_TOL_S
    path_rms_max_m: float = DEFAULT_PATH_RMS_MAX_M


@dataclass(frozen=True)
class FlightSetup:
    zmq: ZmqEndpoints = field(default_factory=ZmqEndpoints)
    balloons: tuple[BalloonSpec, ...] = ()
    camera: CameraSpec = field(default_factory=CameraSpec)
    render_rate_hz: float = DEFAULT_RENDER_RATE_HZ
    guidance: GuidanceSpec = field(default_factory=GuidanceSpec)
    spawn: SpawnSpec = field(default_factory=SpawnSpec)
    sim: SimSpec = field(default_factory=SimSpec)
    verification: VerificationSpec = field(default_factory=VerificationSpec)
    source_path: Path | None = None

    def balloon(self, index: int) -> BalloonSpec:
        if not self.balloons:
            raise IndexError("no balloons in flight setup")
        return self.balloons[index % len(self.balloons)]


def _require_mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be an object")
    return value


def _as_float(value: Any, name: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a number") from exc


def _as_int(value: Any, name: str) -> int:
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be an integer") from exc


def _as_str(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value.strip()


def _as_rgb(value: Any, name: str) -> tuple[int, int, int]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 3:
        raise ValueError(f"{name} must be an RGB triple [r, g, b]")
    rgb = tuple(_as_int(c, f"{name}[{i}]") for i, c in enumerate(value))
    for i, c in enumerate(rgb):
        if c < 0 or c > 255:
            raise ValueError(f"{name}[{i}] must be in 0..255")
    return int(rgb[0]), int(rgb[1]), int(rgb[2])


def _as_ned(value: Any, name: str) -> tuple[float, float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 3:
        raise ValueError(f"{name} must be an NED triple [n, e, d]")
    return (
        _as_float(value[0], f"{name}[0]"),
        _as_float(value[1], f"{name}[1]"),
        _as_float(value[2], f"{name}[2]"),
    )


def _parse_zmq(raw: Any) -> ZmqEndpoints:
    data = _require_mapping(raw if raw is not None else {}, "zmq")
    return ZmqEndpoints(
        image=_as_str(data.get("image", DEFAULT_ZMQ_IMAGE), "zmq.image"),
        color=_as_str(data.get("color", DEFAULT_ZMQ_COLOR), "zmq.color"),
        track=_as_str(data.get("track", DEFAULT_ZMQ_TRACK), "zmq.track"),
        pose=_as_str(data.get("pose", DEFAULT_ZMQ_POSE), "zmq.pose"),
    )


def _parse_balloon(raw: Any, index: int) -> BalloonSpec:
    data = _require_mapping(raw, f"balloons[{index}]")
    if "ned" not in data:
        raise ValueError(f"balloons[{index}].ned is required")
    if "color" not in data:
        raise ValueError(f"balloons[{index}].color is required")
    diameter_m = _as_float(
        data.get("diameter_m", DEFAULT_BALLOON_DIAMETER_M),
        f"balloons[{index}].diameter_m",
    )
    if diameter_m <= 0.0:
        raise ValueError(f"balloons[{index}].diameter_m must be > 0")
    return BalloonSpec(
        ned=_as_ned(data["ned"], f"balloons[{index}].ned"),
        color=_as_rgb(data["color"], f"balloons[{index}].color"),
        diameter_m=diameter_m,
    )


def _parse_camera(raw: Any) -> CameraSpec:
    data = _require_mapping(raw if raw is not None else {}, "camera")
    width_px = _as_int(data.get("width_px", DEFAULT_WIDTH_PX), "camera.width_px")
    height_px = _as_int(data.get("height_px", DEFAULT_HEIGHT_PX), "camera.height_px")
    if width_px <= 0 or height_px <= 0:
        raise ValueError("camera.width_px and camera.height_px must be > 0")
    hfov = _as_float(data.get("hfov_deg", DEFAULT_HFOV_DEG), "camera.hfov_deg")
    vfov = _as_float(data.get("vfov_deg", DEFAULT_VFOV_DEG), "camera.vfov_deg")
    if hfov <= 0.0 or vfov <= 0.0 or hfov >= 180.0 or vfov >= 180.0:
        raise ValueError("camera FOV degrees must be in (0, 180)")
    rate = _as_float(data.get("rate_hz", DEFAULT_CAMERA_RATE_HZ), "camera.rate_hz")
    if rate <= 0.0:
        raise ValueError("camera.rate_hz must be > 0")
    eye_fwd = _as_float(
        data.get("fg_eye_forward_m", DEFAULT_FG_EYE_FORWARD_M),
        "camera.fg_eye_forward_m",
    )
    if eye_fwd < 0.0:
        raise ValueError("camera.fg_eye_forward_m must be >= 0")
    hide_raw = data.get("fg_hide_aircraft", DEFAULT_FG_HIDE_AIRCRAFT)
    if isinstance(hide_raw, bool):
        hide_ac = hide_raw
    elif isinstance(hide_raw, (int, float)):
        hide_ac = bool(hide_raw)
    elif isinstance(hide_raw, str):
        hide_ac = hide_raw.strip().lower() in {"1", "true", "yes", "on"}
    else:
        raise ValueError("camera.fg_hide_aircraft must be a boolean")
    return CameraSpec(
        hfov_deg=hfov,
        vfov_deg=vfov,
        azimuth_deg=_as_float(
            data.get("azimuth_deg", DEFAULT_AZIMUTH_DEG), "camera.azimuth_deg"
        ),
        elevation_deg=_as_float(
            data.get("elevation_deg", DEFAULT_ELEVATION_DEG), "camera.elevation_deg"
        ),
        width_px=width_px,
        height_px=height_px,
        rate_hz=rate,
        fg_window_pattern=_as_str(
            data.get("fg_window_pattern", DEFAULT_FG_WINDOW_PATTERN),
            "camera.fg_window_pattern",
        ),
        xp_window_pattern=_as_str(
            data.get("xp_window_pattern", DEFAULT_XP_WINDOW_PATTERN),
            "camera.xp_window_pattern",
        ),
        fg_eye_forward_m=eye_fwd,
        fg_hide_aircraft=hide_ac,
    )


def _parse_guidance(raw: Any) -> GuidanceSpec:
    data = _require_mapping(raw if raw is not None else {}, "guidance")
    control_rate = _as_float(
        data.get("control_rate_hz", DEFAULT_CONTROL_RATE_HZ), "guidance.control_rate_hz"
    )
    speed = _as_float(data.get("speed_mps", DEFAULT_SPEED_MPS), "guidance.speed_mps")
    pass_radius = _as_float(
        data.get("pass_radius_m", DEFAULT_PASS_RADIUS_M), "guidance.pass_radius_m"
    )
    lookahead = _as_float(
        data.get("lookahead_m", DEFAULT_LOOKAHEAD_M), "guidance.lookahead_m"
    )
    assisted = _as_float(
        data.get("assisted_print_period_s", DEFAULT_ASSISTED_PRINT_PERIOD_S),
        "guidance.assisted_print_period_s",
    )
    stale = _as_float(
        data.get("stale_track_warn_s", DEFAULT_STALE_TRACK_WARN_S),
        "guidance.stale_track_warn_s",
    )
    laps = _as_int(data.get("laps", DEFAULT_LAPS), "guidance.laps")
    duration = _as_float(
        data.get("duration_s", DEFAULT_DURATION_S), "guidance.duration_s"
    )
    cmd_mode = _as_str(
        data.get("cmd_mode", DEFAULT_CMD_MODE), "guidance.cmd_mode"
    ).lower()
    attitude_format = _as_str(
        data.get("attitude_format", DEFAULT_ATTITUDE_FORMAT),
        "guidance.attitude_format",
    ).lower()
    # Absent key → default; present but unknown → hard error.
    controller = _as_str(
        data.get("controller", DEFAULT_CONTROLLER), "guidance.controller"
    ).strip()
    homing_law = _as_str(
        data.get("homing_law", DEFAULT_HOMING_LAW), "guidance.homing_law"
    ).strip().lower()
    alt_preserve_err_deg = _as_float(
        data.get(
            "alt_preserve_heading_err_deg", DEFAULT_ALT_PRESERVE_HEADING_ERR_DEG
        ),
        "guidance.alt_preserve_heading_err_deg",
    )
    if control_rate <= 0.0:
        raise ValueError("guidance.control_rate_hz must be > 0")
    if speed <= 0.0:
        raise ValueError("guidance.speed_mps must be > 0")
    if pass_radius <= 0.0:
        raise ValueError("guidance.pass_radius_m must be > 0")
    if lookahead <= 0.0:
        raise ValueError("guidance.lookahead_m must be > 0")
    if assisted <= 0.0:
        raise ValueError("guidance.assisted_print_period_s must be > 0")
    if stale <= 0.0:
        raise ValueError("guidance.stale_track_warn_s must be > 0")
    if laps < 0:
        raise ValueError(
            "guidance.laps must be >= 0 (0 = cycle until duration_s; "
            "duration_s=0 = no time limit)"
        )
    if duration < 0.0:
        raise ValueError("guidance.duration_s must be >= 0 (0 = no time limit)")
    if cmd_mode not in _ALLOWED_CMD_MODES:
        raise ValueError(
            "guidance.cmd_mode must be one of velocity|attitude|rates"
        )
    if attitude_format not in _ALLOWED_ATTITUDE_FORMATS:
        raise ValueError(
            "guidance.attitude_format must be one of quat|euler"
        )
    if controller not in KNOWN_CONTROLLER_IDS:
        raise ValueError(
            "guidance.controller must be one of "
            + "|".join(sorted(KNOWN_CONTROLLER_IDS))
            + f", got {controller!r}"
        )
    if homing_law not in KNOWN_HOMING_LAWS:
        raise ValueError(
            "guidance.homing_law must be one of "
            + "|".join(sorted(KNOWN_HOMING_LAWS))
            + f", got {homing_law!r}"
        )
    if alt_preserve_err_deg < 0.0:
        raise ValueError("guidance.alt_preserve_heading_err_deg must be >= 0")
    return GuidanceSpec(
        control_rate_hz=control_rate,
        speed_mps=speed,
        pass_radius_m=pass_radius,
        lookahead_m=lookahead,
        assisted_print_period_s=assisted,
        stale_track_warn_s=stale,
        laps=laps,
        duration_s=duration,
        cmd_mode=cmd_mode,
        attitude_format=attitude_format,
        controller=controller,
        homing_law=homing_law,
        alt_preserve_heading_err_deg=alt_preserve_err_deg,
    )


def _parse_spawn(raw: Any) -> SpawnSpec:
    data = _require_mapping(raw if raw is not None else {}, "spawn")
    ned = (
        _as_ned(data["ned"], "spawn.ned")
        if "ned" in data
        else DEFAULT_SPAWN_NED
    )
    heading = _as_float(
        data.get("heading_deg", DEFAULT_SPAWN_HEADING_DEG), "spawn.heading_deg"
    )
    return SpawnSpec(ned=ned, heading_deg=heading)


def _parse_sim(raw: Any) -> SimSpec:
    data = _require_mapping(raw if raw is not None else {}, "sim")
    platform = _as_str(
        data.get("platform", DEFAULT_SIM_PLATFORM), "sim.platform"
    ).lower()
    gz_model = _as_str(
        data.get("gz_model", DEFAULT_GZ_MODEL), "sim.gz_model"
    ).lower()
    duration = _as_float(
        data.get("duration_s", DEFAULT_DURATION_S), "sim.duration_s"
    )
    if platform not in KNOWN_SIM_PLATFORMS:
        raise ValueError(
            "sim.platform must be one of "
            + "|".join(sorted(KNOWN_SIM_PLATFORMS))
            + f", got {platform!r}"
        )
    # Non-gz platforms may still store a placeholder gz_model for when the
    # user switches platform; only enforce the menu when platform is gz (or
    # when resolve_race_sim sees an explicit --model for a non-gz plant).
    if platform == "gz":
        validate_gz_model_for_platform(platform, gz_model)
    elif gz_model not in KNOWN_GZ_MODELS:
        raise ValueError(
            "sim.gz_model must be one of "
            + "|".join(sorted(KNOWN_GZ_MODELS))
            + f", got {gz_model!r}"
        )
    if duration < 0.0:
        raise ValueError("sim.duration_s must be >= 0 (0 = no time limit)")
    return SimSpec(platform=platform, gz_model=gz_model, duration_s=duration)


def resolve_race_sim(
    setup: FlightSetup,
    *,
    platform: str | None = None,
    gz_model: str | None = None,
    duration_s: float | None = None,
) -> tuple[str, str, float]:
    """CLI overrides for platform / gz_model / duration; None → setup values."""
    plat = (
        setup.sim.platform
        if platform is None
        else _as_str(platform, "platform").lower()
    )
    model_explicit = gz_model is not None
    model = (
        setup.sim.gz_model
        if gz_model is None
        else _as_str(gz_model, "gz_model").lower()
    )
    if plat not in KNOWN_SIM_PLATFORMS:
        raise ValueError(
            "platform must be one of "
            + "|".join(sorted(KNOWN_SIM_PLATFORMS))
            + f", got {plat!r}"
        )
    if plat == "gz" or model_explicit:
        validate_gz_model_for_platform(plat, model)
    dur = (
        float(setup.sim.duration_s)
        if duration_s is None
        else float(duration_s)
    )
    if dur < 0.0:
        raise ValueError("duration_s must be >= 0 (0 = no time limit)")
    return plat, model, dur


def _parse_verification(raw: Any) -> VerificationSpec:
    data = _require_mapping(raw if raw is not None else {}, "verification")
    pixel_rms = _as_float(
        data.get("pixel_rms_max_px", DEFAULT_PIXEL_RMS_MAX_PX),
        "verification.pixel_rms_max_px",
    )
    pass_tol = _as_float(
        data.get("pass_time_tol_s", DEFAULT_PASS_TIME_TOL_S),
        "verification.pass_time_tol_s",
    )
    path_rms = _as_float(
        data.get("path_rms_max_m", DEFAULT_PATH_RMS_MAX_M),
        "verification.path_rms_max_m",
    )
    if pixel_rms <= 0.0:
        raise ValueError("verification.pixel_rms_max_px must be > 0")
    if pass_tol <= 0.0:
        raise ValueError("verification.pass_time_tol_s must be > 0")
    if path_rms <= 0.0:
        raise ValueError("verification.path_rms_max_m must be > 0")
    return VerificationSpec(
        pixel_rms_max_px=pixel_rms,
        pass_time_tol_s=pass_tol,
        path_rms_max_m=path_rms,
    )


def flight_setup_from_dict(
    raw: dict[str, Any], *, source_path: Path | None = None
) -> FlightSetup:
    """Validate a raw dict into FlightSetup (missing keys → plan defaults)."""
    from dataclasses import replace

    if not isinstance(raw, dict):
        raise ValueError("flight setup root must be an object")

    balloons_raw = raw.get("balloons", [])
    if not isinstance(balloons_raw, list):
        raise ValueError("balloons must be a list")
    balloons = tuple(_parse_balloon(item, i) for i, item in enumerate(balloons_raw))

    render_rate = _as_float(
        raw.get("render_rate_hz", DEFAULT_RENDER_RATE_HZ), "render_rate_hz"
    )
    if render_rate <= 0.0:
        raise ValueError("render_rate_hz must be > 0")

    # Race length lives under sim.duration_s; accept legacy guidance.duration_s.
    sim_raw = raw.get("sim")
    sim_data: dict[str, Any] = dict(sim_raw) if isinstance(sim_raw, dict) else {}
    guidance_raw = raw.get("guidance")
    if (
        "duration_s" not in sim_data
        and isinstance(guidance_raw, dict)
        and "duration_s" in guidance_raw
    ):
        sim_data["duration_s"] = guidance_raw["duration_s"]
    sim = _parse_sim(sim_data if (sim_raw is not None or sim_data) else None)
    guidance = _parse_guidance(guidance_raw)
    guidance = replace(guidance, duration_s=sim.duration_s)

    return FlightSetup(
        zmq=_parse_zmq(raw.get("zmq")),
        balloons=balloons,
        camera=_parse_camera(raw.get("camera")),
        render_rate_hz=render_rate,
        guidance=guidance,
        spawn=_parse_spawn(raw.get("spawn")),
        sim=sim,
        verification=_parse_verification(raw.get("verification")),
        source_path=source_path,
    )


def load_flight_setup(path: str | Path) -> FlightSetup:
    """Load flightSetup.json from disk (JSONC: ``//`` and ``/* */`` allowed)."""
    from fw_sitl.plant_loader import strip_jsonc

    setup_path = Path(path).expanduser().resolve()
    raw_text = setup_path.read_text(encoding="utf-8")
    raw = json.loads(strip_jsonc(raw_text))
    if not isinstance(raw, dict):
        raise ValueError(f"{setup_path}: root must be a JSON object")
    return flight_setup_from_dict(raw, source_path=setup_path)
