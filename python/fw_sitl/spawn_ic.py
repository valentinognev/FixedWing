"""Materialize aircraft spawn from flightSetup.json for JSBSim, YASim, and Gazebo."""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from fw_sitl.balloon_scene import (
    DEFAULT_AIRCRAFT_MSL_M,
    DEFAULT_GROUND_ALT_M,
    DEFAULT_ORIGIN_LAT_DEG,
    DEFAULT_ORIGIN_LON_DEG,
    ned_to_geodetic,
)
from fw_sitl.flight_setup import FlightSetup, SpawnSpec, load_flight_setup
from fw_sitl.gz_pose import DEFAULT_GZ_ORIGIN_ENU, ned_to_gz_enu, world_velocity_enu

# Match python/assets/jsb_spawn.xml / fg_spawn.env in-air IC (~30 m/s).
_JSB_VT_FT_S = 98.4
_FG_VC_KN = 58.3


def gz_yaw_rad(heading_deg: float) -> float:
    """Compass heading (0=north, 90=east) → Gazebo ENU yaw (0=east, π/2=north)."""
    return math.radians(90.0 - float(heading_deg))


def gz_pose_csv(
    setup: FlightSetup,
    *,
    origin_enu: tuple[float, float, float] = DEFAULT_GZ_ORIGIN_ENU,
) -> str:
    """PX4_GZ_MODEL_POSE: x,y,z,roll,pitch,yaw (ENU metres / rad)."""
    x, y, z = ned_to_gz_enu(setup.spawn.ned, origin_enu)
    yaw = gz_yaw_rad(setup.spawn.heading_deg)
    return f"{x:g},{y:g},{z:g},0,0,{yaw:.6f}"


def gz_spawn_velocity_enu(setup: FlightSetup) -> tuple[float, float, float]:
    return world_velocity_enu(
        setup.guidance.speed_mps, gz_yaw_rad(setup.spawn.heading_deg)
    )


def _geodetic(spawn: SpawnSpec) -> tuple[float, float, float]:
    return ned_to_geodetic(
        spawn.ned[0],
        spawn.ned[1],
        spawn.ned[2],
        DEFAULT_ORIGIN_LAT_DEG,
        DEFAULT_ORIGIN_LON_DEG,
        DEFAULT_AIRCRAFT_MSL_M,
    )


def jsb_spawn_xml(spawn: SpawnSpec) -> str:
    lat, lon, alt = _geodetic(spawn)
    return (
        '<?xml version="1.0"?>\n'
        "<!-- Generated from flightSetup.json spawn (LSZH origin + NED). -->\n"
        '<initialize name="Zurich-inair">\n'
        f'  <latitude type="geodetic" unit="DEG"> {lat:.8f} </latitude>\n'
        f'  <longitude unit="DEG"> {lon:.8f} </longitude>\n'
        f'  <altitude unit="M"> {alt:.3f} </altitude>\n'
        f'  <elevation unit="M"> {DEFAULT_GROUND_ALT_M} </elevation>\n'
        f'  <vt unit="FT/SEC"> {_JSB_VT_FT_S} </vt>\n'
        '  <gamma unit="DEG"> 0.0 </gamma>\n'
        '  <phi unit="DEG"> 0.0 </phi>\n'
        '  <theta unit="DEG"> 0.0 </theta>\n'
        f'  <psi unit="DEG"> {spawn.heading_deg:g} </psi>\n'
        "</initialize>\n"
    )


def fg_spawn_env_text(spawn: SpawnSpec) -> str:
    lat, lon, alt = _geodetic(spawn)
    return (
        "# Generated from flightSetup.json spawn (LSZH origin + NED).\n"
        "FG_ARGS_EX="
        f'"--disable-terrasync --in-air --units-meters '
        f"--lat={lat:.8f} --lon={lon:.8f} --altitude={alt:.3f} "
        f'--heading={spawn.heading_deg:g} --vc={_FG_VC_KN}"\n'
    )


def write_jsb_spawn_xml(path: str | Path, spawn: SpawnSpec) -> None:
    Path(path).write_text(jsb_spawn_xml(spawn), encoding="utf-8")


def write_fg_spawn_env(path: str | Path, spawn: SpawnSpec) -> None:
    Path(path).write_text(fg_spawn_env_text(spawn), encoding="utf-8")


def _main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Write plant IC from flightSetup.json spawn"
    )
    parser.add_argument("--setup", required=True, type=Path)
    parser.add_argument("--jsb-xml", type=Path, default=None)
    parser.add_argument("--fg-env", type=Path, default=None)
    parser.add_argument("--gz-pose", action="store_true")
    args = parser.parse_args(argv)
    setup = load_flight_setup(args.setup)
    if args.jsb_xml is not None:
        write_jsb_spawn_xml(args.jsb_xml, setup.spawn)
    if args.fg_env is not None:
        write_fg_spawn_env(args.fg_env, setup.spawn)
    if args.gz_pose:
        print(gz_pose_csv(setup))
    if args.jsb_xml is None and args.fg_env is None and not args.gz_pose:
        parser.error("pass --jsb-xml, --fg-env, and/or --gz-pose")
    return 0


if __name__ == "__main__":
    sys.exit(_main())
