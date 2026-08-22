"""NED ↔ geodetic helpers and FlightGear AI balloon placement."""
from __future__ import annotations

import math
import os
import socket
import time
from collections.abc import Callable, Sequence
from pathlib import Path

from fw_sitl.flight_setup import BalloonSpec, FlightSetup
from fw_sitl.platforms.gz.gz_pose import DEFAULT_GZ_ORIGIN_ENU, ned_to_gz_enu
from fw_sitl.race_guidance import rebase_balloons_to_local_z

# WGS84
_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563
_WGS84_E2 = 2.0 * _WGS84_F - _WGS84_F * _WGS84_F

# Default spawn from python/assets/jsb_spawn.xml (LSZH in-air IC).
DEFAULT_ORIGIN_LAT_DEG = 47.458159
DEFAULT_ORIGIN_LON_DEG = 8.548004
DEFAULT_GROUND_ALT_M = 419.2  # LSZH scene elevation (m MSL)
# Aircraft spawn MSL (= ground + ~500 m AGL). Balloon NED Z is home/aircraft-relative
# (z≈0 at cruise); FG telnet placement must use this MSL as NED origin altitude.
DEFAULT_AIRCRAFT_MSL_M = 919.2
DEFAULT_ORIGIN_ALT_M = DEFAULT_AIRCRAFT_MSL_M
# FGModelMgr::add_model reads elevation-ft only (not elevation-m).
_M_TO_FT = 1.0 / 0.3048

ASSETS_BALLOONS = Path(__file__).resolve().parents[1] / "assets" / "balloons"
# Bind-mount target in runSimJsbsimRascal.sh --viz (host assets → container).
CONTAINER_BALLOONS_DIR = Path("/opt/fixedwing/balloons")
# runSim --viz copies color XML wrappers here so geo.put_model resolves FG_ROOT-relative.
FG_ROOT_BALLOONS_MODEL_DIR = "Models/FixedWing"


def balloons_assets_dir() -> Path:
    """Host or override dir for balloon .ac files."""
    env = os.environ.get("FG_BALLOONS_DIR")
    if env:
        return Path(env)
    return ASSETS_BALLOONS


def fg_elevation_ft_from_msl_m(alt_m: float) -> float:
    """Metres AMSL → feet for FG ``/models/model/elevation-ft``.

    ``geo.put_model`` writes ``elevation-m``. FGModelMgr::add_model only
    reads ``elevation-ft`` (default 0) so 919.2 m MSL balloons appeared at
    sea level while the JSBSim aircraft stayed at cruise.
    """
    return float(alt_m) * _M_TO_FT


def fg_msl_m_from_altitude_ft(alt_ft: float) -> float:
    """Feet AMSL → metres (inverse of ``fg_elevation_ft_from_msl_m``)."""
    return float(alt_ft) / _M_TO_FT


def parse_fg_telnet_float(raw: str) -> float | None:
    """Parse FG ``get /path``: ``/path = '3977.03' (double)``."""
    if not raw or "=" not in raw:
        return None
    rhs = raw.split("=", 1)[1]
    for quote in ("'", '"'):
        if quote in rhs:
            try:
                return float(rhs.split(quote)[1])
            except (IndexError, ValueError):
                return None
    try:
        return float(rhs.split()[0].strip())
    except (ValueError, IndexError):
        return None


def origin_alt_m_from_fg_altitude_ft(
    alt_ft: float | None,
    *,
    fallback_m: float = DEFAULT_ORIGIN_ALT_M,
) -> float:
    """NED origin MSL from live FG ``/position/altitude-ft``.

    Hardcoded 919.2 m left models ~293 m under a visual aircraft at 3977 ft;
    EKF chase still rebased ΔD≈0 so the plot hid the gap.
    """
    if alt_ft is None or not math.isfinite(float(alt_ft)) or float(alt_ft) < 100.0:
        return float(fallback_m)
    return fg_msl_m_from_altitude_ft(float(alt_ft))


def origin_latlon_from_fg(
    lat_deg: float | None,
    lon_deg: float | None,
    *,
    fallback_lat_deg: float = DEFAULT_ORIGIN_LAT_DEG,
    fallback_lon_deg: float = DEFAULT_ORIGIN_LON_DEG,
) -> tuple[float, float]:
    """NED origin lat/lon from live FG ``/position/latitude-deg`` / ``longitude-deg``.

    Live --viz 20260821: models used DEFAULT_ORIGIN while the aircraft had
    already drifted (NED ≈ 326 N, 127 E at spawn; 411 m from balloon 0 at
    race t=0, heading error 134°). HSV locked only when the camera swept
    past them. Use the live aircraft as XY origin (same idea as live alt).
    """
    if (
        lat_deg is None
        or lon_deg is None
        or not math.isfinite(float(lat_deg))
        or not math.isfinite(float(lon_deg))
        or abs(float(lat_deg)) > 90.0
        or abs(float(lon_deg)) > 180.0
    ):
        return float(fallback_lat_deg), float(fallback_lon_deg)
    return float(lat_deg), float(lon_deg)


def balloons_fg_model_dir() -> Path:
    """Path FG inside Docker should use for model-path (bind-mount).

    Override with FG_BALLOONS_CONTAINER_DIR; set FG_BALLOONS_REWRITE=0 to pass
    host paths through unchanged (native FG on host).
    """
    env = os.environ.get("FG_BALLOONS_CONTAINER_DIR")
    if env:
        return Path(env)
    return CONTAINER_BALLOONS_DIR


def geodetic_to_ned(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    origin_lat_deg: float,
    origin_lon_deg: float,
    origin_alt_m: float,
) -> tuple[float, float, float]:
    """Geodetic → local NED relative to origin (m)."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    lat0 = math.radians(origin_lat_deg)
    lon0 = math.radians(origin_lon_deg)

    sin_lat = math.sin(lat0)
    cos_lat = math.cos(lat0)
    n0 = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * sin_lat * sin_lat)
    m0 = _WGS84_A * (1.0 - _WGS84_E2) / (1.0 - _WGS84_E2 * sin_lat * sin_lat) ** 1.5

    d_lat = lat - lat0
    d_lon = lon - lon0
    north = d_lat * m0
    east = d_lon * n0 * cos_lat
    down = origin_alt_m - alt_m
    return (north, east, down)


def ned_to_geodetic(
    north: float,
    east: float,
    down: float,
    origin_lat_deg: float,
    origin_lon_deg: float,
    origin_alt_m: float,
) -> tuple[float, float, float]:
    """Local NED relative to origin → geodetic (deg, deg, m MSL)."""
    lat0 = math.radians(origin_lat_deg)
    lon0 = math.radians(origin_lon_deg)
    sin_lat = math.sin(lat0)
    cos_lat = math.cos(lat0)
    n0 = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * sin_lat * sin_lat)
    m0 = _WGS84_A * (1.0 - _WGS84_E2) / (1.0 - _WGS84_E2 * sin_lat * sin_lat) ** 1.5

    d_lat = north / m0
    d_lon = east / (n0 * cos_lat)
    lat = lat0 + d_lat
    lon = lon0 + d_lon
    alt = origin_alt_m - down
    return (math.degrees(lat), math.degrees(lon), alt)


def balloon_ned_positions(balloons: tuple[BalloonSpec, ...]) -> list[tuple[float, float, float]]:
    return [tuple(b.ned) for b in balloons]


class FgTelnet:
    """Minimal FlightGear telnet client (line-oriented)."""

    def __init__(self, host: str = "127.0.0.1", port: int = 5501, timeout: float = 2.0) -> None:
        self._host = host
        self._port = port
        self._timeout = timeout
        self._sock: socket.socket | None = None

    def connect(self, retries: int = 10, delay_s: float = 0.5) -> None:
        last_exc: BaseException | None = None
        for i in range(retries):
            try:
                s = socket.create_connection((self._host, self._port), timeout=self._timeout)
                s.settimeout(self._timeout)
                # Banner is optional — some FG builds send nothing until first command.
                try:
                    s.recv(4096)
                except TimeoutError:
                    pass
                self._sock = s
                # Probe so we fail fast if the port is open but not FG props.
                probe = self.command("get /sim/version/flightgear")
                if "unknown command" in probe.lower() and "flightgear" not in probe.lower():
                    raise OSError(f"unexpected telnet probe: {probe[:120]!r}")
                return
            except (OSError, TimeoutError) as exc:
                last_exc = exc
                try:
                    if self._sock is not None:
                        self._sock.close()
                except OSError:
                    pass
                self._sock = None
                if i + 1 >= retries:
                    raise TimeoutError(
                        f"FG telnet {self._host}:{self._port} timed out "
                        f"after {retries} tries ({exc})"
                    ) from exc
                time.sleep(delay_s)
        if last_exc is not None:
            raise last_exc

    def command(self, line: str) -> str:
        if self._sock is None:
            raise RuntimeError("not connected")
        payload = (line.strip() + "\r\n").encode("utf-8")
        self._sock.sendall(payload)
        chunks: list[bytes] = []
        while True:
            try:
                chunk = self._sock.recv(4096)
            except socket.timeout:
                break
            if not chunk:
                break
            chunks.append(chunk)
            joined = b"".join(chunks)
            # FG often uses `\r` and/or `/>` with no `\n`. Do not treat `=` as
            # end-of-reply: a truncated packet parsed as lat/lon and sent
            # guidance to x≈−3.7e6 m (live 20260821_081926).
            if b"\n" in joined or b"\r" in joined or b"/>" in joined:
                break
        # Trailing `/>` after `\r` must not become the next get's whole reply.
        try:
            self._sock.settimeout(0.0)
            extra = self._sock.recv(4096)
            if extra:
                chunks.append(extra)
        except (BlockingIOError, TimeoutError, OSError, socket.timeout, AttributeError):
            pass
        try:
            self._sock.settimeout(self._timeout)
        except AttributeError:
            pass
        return b"".join(chunks).decode("utf-8", errors="replace")

    def set_prop(self, path: str, value: str | float | int) -> None:
        """Fire-and-forget property write.

        FG ``set`` often sends no line. ``command()`` then waits the full socket
        timeout (2s) per write — ~26 view props per capture tick froze --viz.
        """
        if self._sock is None:
            raise RuntimeError("not connected")
        payload = (f"set {path} {value}".strip() + "\r\n").encode("utf-8")
        self._sock.sendall(payload)

    def nasal(self, code: str, *, timeout_s: float = 5.0) -> str:
        """Run Nasal via telnet (requires FG ``--allow-nasal-from-sockets``).

        Protocol (FG wiki): ``nasal`` / code lines / ``##EOF##``.
        """
        if self._sock is None:
            raise RuntimeError("not connected")
        marker = "##EOF##"
        # Drain any pending banner/prompt bytes.
        try:
            self._sock.settimeout(0.05)
            while True:
                chunk = self._sock.recv(4096)
                if not chunk:
                    break
        except (TimeoutError, OSError, socket.timeout):
            pass
        self._sock.settimeout(timeout_s)
        # Newline before and after code (FG docs); default EOF marker ##EOF##.
        payload = "nasal\r\n" + code.strip() + "\r\n" + marker + "\r\n"
        self._sock.sendall(payload.encode("utf-8"))
        chunks: list[bytes] = []
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            try:
                chunk = self._sock.recv(4096)
            except (TimeoutError, OSError, socket.timeout):
                break
            if not chunk:
                break
            chunks.append(chunk)
            text = b"".join(chunks)
            # Prompt returns after Nasal finishes.
            if b"/>" in text:
                try:
                    self._sock.settimeout(0.15)
                    extra = self._sock.recv(4096)
                    if extra:
                        chunks.append(extra)
                except (TimeoutError, OSError, socket.timeout):
                    pass
                break
        self._sock.settimeout(self._timeout)
        return b"".join(chunks).decode("utf-8", errors="replace")

    def read_pose_deg(self) -> tuple[
        float | None, float | None, float | None, float | None, float | None, float | None
    ]:
        """lat, lon, alt-ft, roll, pitch, heading (deg) via six ``get``s.

        A Nasal snapshot still needed a following ``get``, and FG's ``/>``
        prompt without ``\\n`` made that ``get`` wait the socket timeout
        (~0.8 s/tick, 16 plot samples / 60 s). ``command()`` now returns on
        ``/>``; six gets are then milliseconds on a live socket.
        """
        paths = (
            "/position/latitude-deg",
            "/position/longitude-deg",
            "/position/altitude-ft",
            "/orientation/roll-deg",
            "/orientation/pitch-deg",
            "/orientation/heading-deg",
        )
        out: list[float | None] = []
        for path in paths:
            out.append(parse_fg_telnet_float(self.command(f"get {path}")))
        return (out[0], out[1], out[2], out[3], out[4], out[5])

    def read_pose_snapshot(
        self,
    ) -> tuple[float | None, float | None, float | None, float | None, float | None, float | None, float, float, float] | None:
        """lat, lon, alt-ft, roll, pitch, heading, vn/ve/vd fps via one Nasal dump."""
        self.nasal(_DUMP_POSE_NASAL, timeout_s=2.0)
        vals = parse_fg_csv_prop(self.command("get /tmp/fw_pose"))
        if len(vals) < 6:
            return None
        while len(vals) < 9:
            vals.append(0.0)
        pose = tuple(vals[i] if math.isfinite(vals[i]) else None for i in range(6))
        vn, ve, vd = float(vals[6]), float(vals[7]), float(vals[8])
        return (pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], vn, ve, vd)

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None


def _color_model_filename(rgb: tuple[int, int, int]) -> str:
    """Prefer XML model wrapper (FG loads .ac via PropertyList path)."""
    r, g, b = rgb
    host_dir = balloons_assets_dir()
    for name in (f"balloon_{r}_{g}_{b}.xml", f"balloon_{r}_{g}_{b}.ac"):
        if (host_dir / name).is_file():
            return name
    for name in ("balloon_sphere.xml", "balloon_sphere.ac"):
        if (host_dir / name).is_file():
            return name
    return "balloon_sphere.ac"


def _color_model_path(rgb: tuple[int, int, int]) -> str:
    """FG ``geo.put_model`` path.

    Default (Docker viz): ``Models/FixedWing/balloon_R_G_B.xml`` under FG_ROOT
    (installed by ``runSimJsbsimRascal.sh --viz``). Absolute bind-mount paths
    break PropertyList ``<path>`` resolution for the stock mesh.
    Set ``FG_BALLOONS_REWRITE=0`` for host-native FG with asset-dir paths.
    """
    name = _color_model_filename(rgb)
    if os.environ.get("FG_BALLOONS_REWRITE", "1") != "0":
        return f"{FG_ROOT_BALLOONS_MODEL_DIR}/{Path(name).name}"
    return str(balloons_assets_dir() / name)


# Remove prior race balloons so re-spawn is idempotent (geo.put_model only adds;
# without this, each race/probe piles up leftovers — mostly stock-red balloon4 /
# older FixedWing reds — mixed with a few correctly colored instances).
_CLEAR_FIXEDWING_BALLOONS_NASAL = r"""
var cleared = 0;
var root = props.globals.getNode("/models", 1);
var doomed = [];
foreach (var c; root.getChildren("model")) {
  var p = c.getNode("path");
  if (p == nil) { continue; }
  var path = p.getValue();
  if (path == nil) { continue; }
  if (find("FixedWing/balloon_", path) >= 0 or find("balloon4.ac", path) >= 0
      or find("Aircraft/balloon/", path) >= 0) {
    append(doomed, c);
  }
}
foreach (var c; doomed) { c.remove(); cleared += 1; }
# Old stub path (/ai/models/model[i]) never drew, but clear leftovers anyway.
var ai = props.globals.getNode("/ai/models", 0);
if (ai != nil) {
  var ai_doomed = [];
  foreach (var c; ai.getChildren("model")) {
    var p = c.getNode("path");
    var idn = c.getNode("id");
    var path = (p != nil) ? p.getValue() : nil;
    var id = (idn != nil) ? idn.getValue() : nil;
    if ((path != nil and (find("balloon", path) >= 0))
        or (id != nil and find("balloon", id) >= 0)) {
      append(ai_doomed, c);
    }
  }
  foreach (var c; ai_doomed) { c.remove(); cleared += 1; }
}
setprop("/tmp/fw_balloons_cleared", cleared);
"""

_LIST_FIXEDWING_BALLOONS_NASAL = r"""
var paths = [];
var root = props.globals.getNode("/models", 1);
foreach (var c; root.getChildren("model")) {
  var p = c.getNode("path");
  if (p == nil) { continue; }
  var path = p.getValue();
  if (path != nil and find("FixedWing/balloon_", path) >= 0) {
    append(paths, path);
  }
}
setprop("/tmp/fw_balloon_model_count", size(paths));
setprop("/tmp/fw_balloon_model_paths", string.join("|", paths));
"""

# One Nasal dump of live model lat/lon so chase NED matches the visual mesh
# (same WGS84 conversion as the aircraft GT), not settle+config offset.
_DUMP_FIXEDWING_BALLOON_LL_NASAL = r"""
var s = "";
var root = props.globals.getNode("/models", 1);
foreach (var c; root.getChildren("model")) {
  var p = c.getNode("path");
  if (p == nil) { continue; }
  var path = p.getValue();
  if (path == nil or find("FixedWing/balloon_", path) < 0) { continue; }
  var latn = c.getNode("latitude-deg");
  var lonn = c.getNode("longitude-deg");
  var lat = (latn != nil) ? latn.getValue() : 0;
  var lon = (lonn != nil) ? lonn.getValue() : 0;
  s = s ~ lat ~ "," ~ lon ~ ";";
}
setprop("/tmp/fw_balloon_ll", s);
"""

# One Nasal dump of aircraft pose + NED fps so the GT thread is not 6 serial
# ``get``s plus a balloon-model walk (~4 s/cycle in pickle 143601).
_DUMP_POSE_NASAL = r"""
var g = func (p) {
  var v = getprop(p);
  return (v == nil) ? 0 : v;
};
var s = g("/position/latitude-deg") ~ "," ~
  g("/position/longitude-deg") ~ "," ~
  g("/position/altitude-ft") ~ "," ~
  g("/orientation/roll-deg") ~ "," ~
  g("/orientation/pitch-deg") ~ "," ~
  g("/orientation/heading-deg") ~ "," ~
  g("/velocities/speed-north-fps") ~ "," ~
  g("/velocities/speed-east-fps") ~ "," ~
  g("/velocities/speed-down-fps");
setprop("/tmp/fw_pose", s);
"""


def clear_fixedwing_balloons_fg(tel: FgTelnet) -> int:
    """Remove previously placed ``Models/FixedWing/balloon_*`` models. Returns count."""
    tel.nasal(_CLEAR_FIXEDWING_BALLOONS_NASAL, timeout_s=8.0)
    status = tel.command("get /tmp/fw_balloons_cleared")
    # Interactive: "/tmp/fw_balloons_cleared = '3' (double)"
    try:
        return int(float(status.split("=")[-1].split("'")[1]))
    except (IndexError, ValueError):
        digits = "".join(ch for ch in status.split("=")[-1] if ch.isdigit() or ch == ".")
        try:
            return int(float(digits)) if digits else 0
        except ValueError:
            return 0


def list_fixedwing_balloon_paths_fg(tel: FgTelnet) -> list[str]:
    """Return path strings for currently placed FixedWing balloon models."""
    tel.nasal(_LIST_FIXEDWING_BALLOONS_NASAL, timeout_s=5.0)
    raw = tel.command("get /tmp/fw_balloon_model_paths")
    # "/tmp/fw_balloon_model_paths = 'a|b' (string)" or empty
    try:
        val = raw.split("=")[-1].split("'")[1]
    except IndexError:
        return []
    if not val:
        return []
    return [p for p in val.split("|") if p]


def parse_fg_csv_prop(raw: str) -> list[float]:
    """Parse FG ``get /tmp/fw_pose``: ``/tmp/fw_pose = '1,2,3' (string)``."""
    val = ""
    if raw and "=" in raw:
        rhs = raw.split("=", 1)[1]
        for quote in ("'", '"'):
            if quote in rhs:
                try:
                    val = rhs.split(quote)[1]
                    break
                except IndexError:
                    val = ""
        else:
            tok = rhs.split()
            val = tok[0].strip() if tok else ""
    out: list[float] = []
    for part in val.split(","):
        part = part.strip()
        if not part:
            continue
        try:
            out.append(float(part))
        except ValueError:
            continue
    return out


def parse_fg_balloon_ll_dump(raw: str) -> list[tuple[float, float]]:
    """Parse Nasal ``lat,lon;lat,lon;`` dump from ``/tmp/fw_balloon_ll``."""
    try:
        val = raw.split("=")[-1].split("'")[1]
    except IndexError:
        val = ""
        rhs = raw.split("=")[-1] if "=" in raw else raw
        val = rhs.strip().strip('"')
    out: list[tuple[float, float]] = []
    for part in val.split(";"):
        part = part.strip()
        if not part or "," not in part:
            continue
        a, b = part.split(",", 1)
        try:
            out.append((float(a), float(b)))
        except ValueError:
            continue
    return out


def fg_balloons_ned_from_models(
    tel: FgTelnet,
    *,
    origin_lat_deg: float = DEFAULT_ORIGIN_LAT_DEG,
    origin_lon_deg: float = DEFAULT_ORIGIN_LON_DEG,
) -> list[tuple[float, float]]:
    """Live FixedWing model lat/lon → NED XY in the aircraft GT frame."""
    tel.nasal(_DUMP_FIXEDWING_BALLOON_LL_NASAL, timeout_s=5.0)
    pairs = parse_fg_balloon_ll_dump(tel.command("get /tmp/fw_balloon_ll"))
    neds: list[tuple[float, float]] = []
    for lat, lon in pairs:
        n, e, _ = geodetic_to_ned(
            lat, lon, 0.0, origin_lat_deg, origin_lon_deg, 0.0
        )
        neds.append((n, e))
    return neds


def spawn_balloons_fg(
    balloons: tuple[BalloonSpec, ...],
    *,
    origin_lat_deg: float = DEFAULT_ORIGIN_LAT_DEG,
    origin_lon_deg: float = DEFAULT_ORIGIN_LON_DEG,
    origin_alt_m: float = DEFAULT_ORIGIN_ALT_M,
    telnet_host: str = "127.0.0.1",
    telnet_port: int = 5501,
    connect_retries: int = 60,
    connect_delay_s: float = 1.0,
    timeout_s: float | None = None,
    clear_existing: bool = True,
) -> tuple[float, float, float]:
    """Place visible static models at balloon NED via ``fgcommand("add-model")``.

    Returns the live FG origin ``(lat_deg, lon_deg, alt_m)`` used for placement.
    Chase NED must use this same origin (not a lat/lon sampled seconds earlier).

    ``geo.put_model`` passes ``elevation-m``. FG 2024 ``FGModelMgr::add_model``
    only reads ``elevation-ft`` (metres are ignored → 0 ft MSL). We set
    ``elevation-ft`` from MSL metres. Requires FG ``--allow-nasal-from-sockets``.

    By default clears prior ``Models/FixedWing/balloon_*`` instances first so
    repeated race starts / probes do not accumulate leftover models.
    ``timeout_s`` overrides ``connect_retries`` as ceil(timeout / delay).
    """
    if timeout_s is not None:
        connect_retries = max(
            1, int(math.ceil(float(timeout_s) / float(connect_delay_s)))
        )
    tel = FgTelnet(host=telnet_host, port=telnet_port)
    tel.connect(retries=connect_retries, delay_s=connect_delay_s)
    # Probe Nasal-from-sockets via a property (print() does not echo on telnet).
    tel.nasal('setprop("/tmp/fw_nasal_ok", 0); setprop("/tmp/fw_nasal_ok", 1);', timeout_s=3.0)
    probe = tel.command("get /tmp/fw_nasal_ok")
    if " = '1'" not in probe and ' = "1"' not in probe and "= '1'" not in probe:
        # FG may report bool/double.
        if "1" not in probe.split("=")[-1]:
            tel.close()
            raise RuntimeError(
                "FG Nasal-from-sockets disabled or failed; relaunch viz with "
                "--allow-nasal-from-sockets (patch V6+). "
                f"probe={probe.strip()[:160]!r}"
            )
    default_origin_m = float(origin_alt_m)
    live_ft = parse_fg_telnet_float(tel.command("get /position/altitude-ft"))
    origin_alt_m = origin_alt_m_from_fg_altitude_ft(
        live_ft, fallback_m=default_origin_m
    )
    live_lat = parse_fg_telnet_float(tel.command("get /position/latitude-deg"))
    live_lon = parse_fg_telnet_float(tel.command("get /position/longitude-deg"))
    origin_lat_deg, origin_lon_deg = origin_latlon_from_fg(
        live_lat,
        live_lon,
        fallback_lat_deg=origin_lat_deg,
        fallback_lon_deg=origin_lon_deg,
    )
    print(
        f"FG balloon origin MSL {origin_alt_m:.1f} m "
        f"lat={origin_lat_deg:.6f} lon={origin_lon_deg:.6f} "
        f"(live altitude-ft={live_ft!r} lat={live_lat!r} lon={live_lon!r}, "
        f"default alt {default_origin_m:.1f} m)"
    )
    if clear_existing:
        cleared = clear_fixedwing_balloons_fg(tel)
        if cleared:
            print(f"Cleared {cleared} stale FixedWing balloon model(s)")
    placed = 0
    for i, spec in enumerate(balloons):
        lat, lon, alt = ned_to_geodetic(
            spec.ned[0],
            spec.ned[1],
            spec.ned[2],
            origin_lat_deg,
            origin_lon_deg,
            origin_alt_m,
        )
        model_path = _color_model_path(spec.color).replace("\\", "/")
        # Escape for Nasal string literal.
        path_lit = model_path.replace("\\", "\\\\").replace('"', '\\"')
        status_prop = f"/tmp/fw_balloon_{i}"
        elev_ft = fg_elevation_ft_from_msl_m(alt)
        # elevation-ft: FGModelMgr ignores elevation-m (geo.put_model's field).
        # enable-hot: 0 at load time clears SG_NODEMASK_TERRAIN_BIT (YASim
        # collision). Setting the prop after add-model is too late.
        code = (
            f'setprop("{status_prop}", "pending");\n'
            f'fgcommand("add-model", var req = props.Node.new({{\n'
            f'  "path": "{path_lit}",\n'
            f'  "latitude-deg": {lat:.8f},\n'
            f'  "longitude-deg": {lon:.8f},\n'
            f'  "elevation-ft": {elev_ft:.3f},\n'
            f'  "heading-deg": 0,\n'
            f'  "enable-hot": 0\n'
            f'}}));\n'
            f'var n = (req.getNode("property") == nil) ? nil '
            f': props.globals.getNode(req.getNode("property").getValue());\n'
            f'if (n != nil) {{\n'
            f'  n.getNode("elevation-ft", 1).setDoubleValue({elev_ft:.3f});\n'
            f'  n.getNode("solid", 1).setBoolValue(0);\n'
            f'  n.getNode("enable-hot", 1).setBoolValue(0);\n'
            f'}}\n'
            f'setprop("{status_prop}", (n == nil) ? "nil" : n.getPath());\n'
        )
        print(
            f"FG balloon {i} color={spec.color} path={model_path} "
            f"lat={lat:.6f} lon={lon:.6f} alt_m={alt:.1f} elev_ft={elev_ft:.1f}"
        )
        tel.nasal(code, timeout_s=8.0)
        status = tel.command(f"get {status_prop}")
        if "nil" in status or "pending" in status or "unknown" in status.lower():
            print(f"FG balloon {i} put_model warning: {status.strip()[:200]}")
        else:
            placed += 1
            node = status.split("=")[-1].split("'")[1] if "'" in status else status.strip()
            print(f"FG balloon {i} placed → {node}")
    live_paths = list_fixedwing_balloon_paths_fg(tel)
    tel.close()
    print(
        f"Spawned {placed}/{len(balloons)} FG balloons via add-model elevation-ft "
        f"telnet {telnet_host}:{telnet_port}; "
        f"live FixedWing balloons={len(live_paths)} paths={live_paths}"
    )
    if len(live_paths) != len(balloons):
        print(
            f"FG balloon count mismatch: expected {len(balloons)}, "
            f"live {len(live_paths)}"
        )
    return (float(origin_lat_deg), float(origin_lon_deg), float(origin_alt_m))


def spawn_fg_from_setup(
    setup: FlightSetup,
    *,
    timeout_s: float = 90.0,
    host: str = "127.0.0.1",
    port: int = 5501,
) -> tuple[float, float, float]:
    """Place FG balloons at live-aircraft MSL (fallback cruise 919.2 m).

    Returns the FG origin used for ``ned_to_geodetic``.
    """
    return spawn_balloons_fg(
        rebase_balloons_to_local_z(setup.balloons, local_z=0.0),
        telnet_host=host,
        telnet_port=port,
        timeout_s=timeout_s,
    )


GZ_CONTAINER_MODELS = Path("/opt/fixedwing/gz/models")
DEFAULT_GZ_CONTAINER = "px4-noble-gz-plane"
DEFAULT_GZ_WORLD = "default"


def gz_balloon_model_name(color: tuple[int, int, int]) -> str:
    r, g, b = (int(color[0]), int(color[1]), int(color[2]))
    return f"balloon_{r}_{g}_{b}"


def gz_model_list_argv() -> list[str]:
    return ["gz", "model", "--list"]


def parse_gz_model_list(text: str) -> list[str]:
    names: list[str] = []
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith("- "):
            names.append(line[2:].strip().strip('"'))
    return names


def balloon_names_to_remove(
    requested: Sequence[str], world_models: Sequence[str]
) -> list[str]:
    out: list[str] = []
    seen: set[str] = set()
    for name in world_models:
        if name.startswith("balloon_") and name not in seen:
            seen.add(name)
            out.append(name)
    for name in requested:
        if name not in seen:
            seen.add(name)
            out.append(name)
    return out


def gz_remove_argv(world: str, name: str) -> list[str]:
    return [
        "gz",
        "service",
        "-s",
        f"/world/{world}/remove",
        "--reqtype",
        "gz.msgs.Entity",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        "3000",
        "--req",
        f'name: "{name}" type: MODEL',
    ]


def gz_create_argv(
    world: str,
    name: str,
    sdf_filename: str,
    pose_enu: tuple[float, float, float],
) -> list[str]:
    x, y, z = pose_enu
    req = (
        f'name: "{name}" sdf_filename: "{sdf_filename}" '
        f"pose {{ position {{ x: {x} y: {y} z: {z} }} }}"
    )
    return [
        "gz",
        "service",
        "-s",
        f"/world/{world}/create",
        "--reqtype",
        "gz.msgs.EntityFactory",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        "3000",
        "--req",
        req,
    ]


def _docker_exec_runner(container: str) -> Callable[[list[str]], None]:
    def run(argv: list[str]) -> None:
        import subprocess

        cmd = ["docker", "exec", container, *argv]
        subprocess.run(cmd, check=True)
    return run


def _list_models_docker(container: str) -> list[str]:
    import subprocess

    try:
        proc = subprocess.run(
            ["docker", "exec", container, *gz_model_list_argv()],
            check=False,
            capture_output=True,
            text=True,
        )
    except Exception:
        return []
    if proc.returncode != 0:
        return []
    return parse_gz_model_list(proc.stdout or "")


def spawn_balloons_gz(
    balloons: Sequence[BalloonSpec],
    *,
    origin_enu: tuple[float, float, float] = DEFAULT_GZ_ORIGIN_ENU,
    world: str = DEFAULT_GZ_WORLD,
    container: str = DEFAULT_GZ_CONTAINER,
    models_dir: Path = GZ_CONTAINER_MODELS,
    runner: Callable[[list[str]], None] | None = None,
    list_models: Callable[[], Sequence[str]] | None = None,
    clear_existing: bool = True,
) -> None:
    """Clear then place colored sphere models at NED converted to Gazebo ENU."""
    exec_fn = runner if runner is not None else _docker_exec_runner(container)
    names = [gz_balloon_model_name(b.color) for b in balloons]
    if clear_existing:
        world_models: Sequence[str] = []
        if list_models is not None:
            world_models = list_models()
        elif runner is None:
            world_models = _list_models_docker(container)
        for name in balloon_names_to_remove(names, world_models):
            try:
                exec_fn(gz_remove_argv(world, name))
            except Exception as exc:  # noqa: BLE001
                print(f"GZ balloon clear {name}: {exc}")
    for spec, name in zip(balloons, names):
        pose = ned_to_gz_enu(spec.ned, origin_enu)
        sdf_path = str(models_dir / name / "model.sdf")
        print(f"GZ balloon {name} ENU={pose} sdf={sdf_path}")
        exec_fn(gz_create_argv(world, name, sdf_path, pose))


def spawn_gz_from_setup(
    setup: FlightSetup,
    *,
    timeout_s: float = 90.0,
    container: str = DEFAULT_GZ_CONTAINER,
    world: str = DEFAULT_GZ_WORLD,
) -> None:
    """Place Gazebo balloons at cruise-relative NED; retry until gz create works."""
    balloons = rebase_balloons_to_local_z(setup.balloons, local_z=0.0)
    deadline = time.time() + float(timeout_s)
    last_exc: BaseException | None = None
    while time.time() < deadline:
        try:
            spawn_balloons_gz(balloons, container=container, world=world)
            return
        except Exception as exc:  # noqa: BLE001
            last_exc = exc
            time.sleep(1.0)
    raise TimeoutError(
        f"GZ balloon spawn timed out after {timeout_s:.0f}s ({last_exc})"
    ) from last_exc


def main(argv: Sequence[str] | None = None) -> int:
    import argparse

    from fw_sitl.flight_setup import load_flight_setup
    from fw_sitl.platforms.xplane.xp_balloon import spawn_xp_from_setup

    parser = argparse.ArgumentParser(
        description="Place race balloons in FlightGear, Gazebo, or X-Plane before PX4 is used"
    )
    parser.add_argument("--setup", required=True, help="flightSetup.json")
    renderer = parser.add_mutually_exclusive_group(required=True)
    renderer.add_argument("--fg", action="store_true", help="FlightGear geo.put_model")
    renderer.add_argument("--gz", action="store_true", help="Gazebo model create")
    renderer.add_argument("--xplane", action="store_true", help="X-Plane plugin UDP")
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5501)
    parser.add_argument("--container", default=DEFAULT_GZ_CONTAINER)
    args = parser.parse_args(argv)
    setup = load_flight_setup(Path(args.setup).resolve())
    if args.fg:
        spawn_fg_from_setup(
            setup, timeout_s=args.timeout, host=args.host, port=args.port
        )
        return 0
    if args.xplane:
        return spawn_xp_from_setup(
            setup, host=args.host, timeout_s=args.timeout
        )
    spawn_gz_from_setup(
        setup, timeout_s=args.timeout, container=args.container
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
