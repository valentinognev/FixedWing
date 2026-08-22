"""X-Plane 12 demo geodetic origin (Salzburg LOWS, ~500 m AGL)."""
from __future__ import annotations

from fw_sitl.balloon_scene import geodetic_to_ned, ned_to_geodetic
from fw_sitl.flight_setup import SpawnSpec

# Salzburg Airport / LOWS — X-Plane 12 demo scenery tiles.
XP_ORIGIN_LAT_DEG = 47.7933
XP_ORIGIN_LON_DEG = 13.0044
XP_GROUND_ALT_M = 430.0
XP_AIRCRAFT_MSL_M = 930.0  # ground + 500 m AGL
XP_ORIGIN_ALT_MSL_M = XP_AIRCRAFT_MSL_M


def xp_geodetic(spawn: SpawnSpec) -> tuple[float, float, float]:
    """Home-relative NED → (lat_deg, lon_deg, alt_msl_m) at LOWS cruise origin."""
    return ned_to_geodetic(
        spawn.ned[0],
        spawn.ned[1],
        spawn.ned[2],
        XP_ORIGIN_LAT_DEG,
        XP_ORIGIN_LON_DEG,
        XP_AIRCRAFT_MSL_M,
    )


def xp_geodetic_csv(spawn: SpawnSpec) -> str:
    """lat,lon,alt_msl_m,heading_deg for runSimXplaneCessna.sh."""
    lat, lon, alt = xp_geodetic(spawn)
    return f"{lat:.8f},{lon:.8f},{alt:.3f},{spawn.heading_deg:g}"


def geodetic_to_ned_xp(
    lat_deg: float,
    lon_deg: float,
    alt_msl_m: float,
    *,
    origin_lat_deg: float = XP_ORIGIN_LAT_DEG,
    origin_lon_deg: float = XP_ORIGIN_LON_DEG,
    origin_alt_msl_m: float = XP_ORIGIN_ALT_MSL_M,
) -> tuple[float, float, float]:
    """Geodetic → NED relative to LOWS race origin (m)."""
    return geodetic_to_ned(
        lat_deg,
        lon_deg,
        alt_msl_m,
        origin_lat_deg,
        origin_lon_deg,
        origin_alt_msl_m,
    )
