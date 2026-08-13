"""In-container Gazebo camera → ZMQ ImageFrame (RGB8)."""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

# Allow `python3 /opt/fixedwing/python/fw_sitl/gz_camera.py` with PYTHONPATH=.../python
_PYTHON_ROOT = Path(__file__).resolve().parent.parent
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.zmq_bus import ImageFrame, ImagePublisher

# gz.msgs.Image.pixel_format_type (Jetty protobuf enum numbers).
_GZ_PIXEL_FORMAT_INT = {
    3: "RGB_INT8",
    4: "RGBA_INT8",
    5: "BGRA_INT8",
    8: "BGR_INT8",
}


def find_race_cam_topic(topic_names: list[str], sensor: str = "race_cam") -> str:
    hits = [t for t in topic_names if sensor in t and "image" in t and "info" not in t]
    if not hits:
        raise RuntimeError(f"no gz topic for sensor {sensor!r} in {topic_names[:12]}")
    hits.sort(key=len)
    return hits[0]


def gz_image_to_rgb(
    width: int, height: int, step: int, data: bytes, pixel_format: str
) -> bytes:
    fmt = (pixel_format or "RGB_INT8").upper()
    if fmt.isdigit():
        fmt = _GZ_PIXEL_FORMAT_INT.get(int(fmt), fmt)
    arr = np.frombuffer(data, dtype=np.uint8)
    if fmt in {"RGB_INT8", "R8G8B8", "RGB8"}:
        if arr.size < width * height * 3:
            raise ValueError("short RGB buffer")
        return bytes(arr[: width * height * 3])
    if fmt in {"BGR_INT8", "B8G8R8", "BGR8"}:
        img = arr[: width * height * 3].reshape((height, width, 3))
        return np.ascontiguousarray(img[:, :, ::-1]).tobytes()
    if fmt in {"RGBA_INT8", "R8G8B8A8"}:
        img = arr[: width * height * 4].reshape((height, width, 4))
        return np.ascontiguousarray(img[:, :, :3]).tobytes()
    if fmt in {"BGRA_INT8", "B8G8R8A8"}:
        img = arr[: width * height * 4].reshape((height, width, 4))
        return np.ascontiguousarray(img[:, :, 2::-1]).tobytes()
    raise ValueError(f"unsupported gz pixel format {pixel_format!r}")


def _import_gz():
    try:
        from gz.transport import Node
        return Node
    except ImportError:
        pass
    for mod in ("gz.transport14", "gz.transport13", "gz.transport12"):
        try:
            return __import__(mod, fromlist=["Node"]).Node
        except ImportError:
            continue
    raise ImportError(
        "gz.transport Node not found; add python3-gz-transport* to Noble image"
    )


def _pixel_format_name(msg) -> str:
    raw = getattr(msg, "pixel_format_type", None)
    if raw is None or raw == "":
        raw = getattr(msg, "pixel_format", "RGB_INT8")
    if isinstance(raw, int):
        if raw == 0:
            return "RGB_INT8"
        return _GZ_PIXEL_FORMAT_INT.get(raw, str(raw))
    name = getattr(raw, "name", None)
    if isinstance(name, str) and name:
        return name
    text = str(raw or "RGB_INT8")
    return text


def _subscribe_image(node, Image, topic, callback) -> bool:
    """Jetty ``subscribe(msg_type, topic, callback)`` returns None on success."""
    attempts = (
        lambda: node.subscribe(Image, topic, callback),
        lambda: node.subscribe(topic, Image, callback),
        lambda: node.subscribe(topic, callback),
    )
    last: Exception | None = None
    for fn in attempts:
        try:
            result = fn()
        except TypeError as exc:
            last = exc
            continue
        return result is not False
    if last is not None:
        raise last
    return False


def run_bridge(*, endpoint: str, sensor: str, timeout_s: float) -> int:
    Node = _import_gz()
    try:
        from gz.msgs.image_pb2 import Image
    except ImportError:
        from gz.msgs11.image_pb2 import Image  # type: ignore

    node = Node()
    deadline = time.time() + timeout_s
    topic = None
    while time.time() < deadline and topic is None:
        try:
            names = list(node.topic_list())
        except Exception:
            names = []
        try:
            topic = find_race_cam_topic(names, sensor)
        except RuntimeError:
            time.sleep(0.5)
    if topic is None:
        print(f"Error: no gz camera topic for {sensor} within {timeout_s:.0f}s", file=sys.stderr)
        return 1

    pub = ImagePublisher(endpoint)
    got = {"n": 0}

    def _cb(msg: Image) -> None:
        rgb = gz_image_to_rgb(
            int(msg.width),
            int(msg.height),
            int(getattr(msg, "step", 0) or 0),
            bytes(msg.data),
            _pixel_format_name(msg),
        )
        frame = ImageFrame(
            stamp=time.time(),
            width=int(msg.width),
            height=int(msg.height),
            rgb=rgb,
        )
        pub.publish(frame)
        got["n"] += 1

    if not _subscribe_image(node, Image, topic, _cb):
        print(f"Error: subscribe failed {topic}", file=sys.stderr)
        return 1
    print(f"gz_camera subscribed {topic} → {endpoint}")
    while True:
        time.sleep(0.05)
        if got["n"] == 0 and time.time() >= deadline:
            print(f"Error: no camera image within {timeout_s:.0f}s", file=sys.stderr)
            return 1


def run_gz_publisher_via_docker(
    setup,
    *,
    container: str = "px4-noble-gz-plane",
    timeout_s: float = 30.0,
) -> int:
    import subprocess

    cmd = [
        "docker",
        "exec",
        "-i",
        container,
        "env",
        "PYTHONPATH=/opt/fixedwing/python",
        "python3",
        "/opt/fixedwing/python/fw_sitl/gz_camera.py",
        "--endpoint",
        setup.zmq.image,
        "--sensor",
        "race_cam",
        "--timeout-s",
        str(timeout_s),
    ]
    print("Starting gz camera bridge:", " ".join(cmd))
    return int(subprocess.call(cmd))


def main() -> int:
    p = argparse.ArgumentParser(description="gz race_cam → ZMQ image")
    p.add_argument("--endpoint", required=True)
    p.add_argument("--sensor", default="race_cam")
    p.add_argument("--timeout-s", type=float, default=30.0)
    args = p.parse_args()
    return run_bridge(
        endpoint=args.endpoint, sensor=args.sensor, timeout_s=args.timeout_s
    )


if __name__ == "__main__":
    raise SystemExit(main())
