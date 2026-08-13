"""NED ↔ geodetic helpers and FlightGear AI balloon placement."""
from __future__ import annotations

import math
import os
import socket
import time
from collections.abc import Callable, Sequence
from pathlib import Path

from fw_sitl.flight_setup import BalloonSpec
from fw_sitl.gz_pose import DEFAULT_GZ_ORIGIN_ENU, ned_to_gz_enu

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
            if b"\n" in chunk:
                break
        return b"".join(chunks).decode("utf-8", errors="replace")

    def set_prop(self, path: str, value: str | float | int) -> None:
        self.command(f"set {path} {value}")

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
    clear_existing: bool = True,
) -> None:
    """Place visible static models at balloon NED via ``geo.put_model`` (Nasal).

    Setting ``/ai/models/model[i]/*`` alone only creates property stubs — FG does
    not instantiate drawable geometry from those. Runtime placement needs
    ``fgcommand("add-model", ...)`` (``geo.put_model``), which requires FG
    launched with ``--allow-nasal-from-sockets``.

    By default clears prior ``Models/FixedWing/balloon_*`` instances first so
    repeated race starts / probes do not accumulate leftover models.
    """
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
        code = (
            f'setprop("{status_prop}", "pending");\n'
            f'var n = geo.put_model("{path_lit}", {lat:.8f}, {lon:.8f}, {alt:.3f}, 0);\n'
            f'setprop("{status_prop}", (n == nil) ? "nil" : n.getPath());\n'
        )
        print(
            f"FG balloon {i} color={spec.color} path={model_path} "
            f"lat={lat:.6f} lon={lon:.6f} alt={alt:.1f}"
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
        f"Spawned {placed}/{len(balloons)} FG balloons via geo.put_model "
        f"telnet {telnet_host}:{telnet_port}; "
        f"live FixedWing balloons={len(live_paths)} paths={live_paths}"
    )
    if len(live_paths) != len(balloons):
        print(
            f"FG balloon count mismatch: expected {len(balloons)}, "
            f"live {len(live_paths)}"
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
