"""In-container Gazebo dynamic pose -> ZMQ PoseSample (ENU metres).

Root cause this replaces: sampling world pose via one-shot
``docker exec gz model -m <name> --pose`` from the host has ~0.4-0.5s
subprocess/docker latency per call. Polling that on a timer produces a
staircase (many identical samples, then a jump) with irregular step widths
set by docker scheduling jitter, not by the simulation — that is what showed
up as "jitter" in the position plots.

Gazebo already publishes every model's pose continuously on
``/world/<world>/dynamic_pose/info`` (``gz.msgs.Pose_V``) at the physics/render
rate (~40+ Hz measured against gz-sim Harmonic). Subscribing once inside the
container and forwarding samples over ZMQ (same pattern as ``gz_camera.py``)
gives a genuinely continuous, low-latency position stream instead.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_PYTHON_ROOT = Path(__file__).resolve().parent.parent
if str(_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(_PYTHON_ROOT))

from fw_sitl.gz_camera import _import_gz
from fw_sitl.zmq_bus import PosePublisher, PoseSample

DYNAMIC_POSE_TOPIC = "/world/default/dynamic_pose/info"


def model_name_candidates(model: str) -> tuple[str, ...]:
    """Spawn naming varies; ``<model>_0`` is what runSimGzPlane.sh actually uses."""
    return (f"{model}_0", model, f"gz_{model}")


def extract_named_pose(
    msg, names: str | tuple[str, ...]
) -> tuple[float, float, float] | None:
    """Pull ENU (x, y, z) for the first matching name out of a ``Pose_V`` message."""
    wanted = (names,) if isinstance(names, str) else names
    wanted_set = set(wanted)
    for pose in msg.pose:
        if pose.name in wanted_set:
            return (float(pose.position.x), float(pose.position.y), float(pose.position.z))
    return None


def run_bridge(*, endpoint: str, model: str, timeout_s: float) -> int:
    Node = _import_gz()
    try:
        from gz.msgs.pose_v_pb2 import Pose_V
    except ImportError:
        from gz.msgs11.pose_v_pb2 import Pose_V  # type: ignore

    names = model_name_candidates(model)
    node = Node()
    pub = PosePublisher(endpoint)
    got = {"n": 0}
    deadline = time.time() + timeout_s

    def _cb(msg: Pose_V) -> None:
        xyz = extract_named_pose(msg, names)
        if xyz is None:
            return
        pub.publish(PoseSample(stamp=time.time(), x=xyz[0], y=xyz[1], z=xyz[2]))
        got["n"] += 1

    if not node.subscribe(Pose_V, DYNAMIC_POSE_TOPIC, _cb):
        print(f"Error: subscribe failed {DYNAMIC_POSE_TOPIC}", file=sys.stderr)
        return 1
    print(f"gz_pose_bridge subscribed {DYNAMIC_POSE_TOPIC} model={names[0]!r} -> {endpoint}")
    while True:
        time.sleep(0.1)
        if got["n"] == 0 and time.time() >= deadline:
            print(
                f"Error: no pose for model in {names} within {timeout_s:.0f}s",
                file=sys.stderr,
            )
            return 1


def run_gz_pose_publisher_via_docker(
    setup,
    *,
    model: str,
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
        "PYTHONUNBUFFERED=1",
        "python3",
        "-u",
        "/opt/fixedwing/python/fw_sitl/gz_pose_bridge.py",
        "--endpoint",
        setup.zmq.pose,
        "--model",
        model,
        "--timeout-s",
        str(timeout_s),
    ]
    print("Starting gz pose bridge:", " ".join(cmd))
    return int(subprocess.call(cmd))


def main() -> int:
    p = argparse.ArgumentParser(description="gz model dynamic pose -> ZMQ (in-container)")
    p.add_argument("--endpoint", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--timeout-s", type=float, default=30.0)
    args = p.parse_args()
    return run_bridge(endpoint=args.endpoint, model=args.model, timeout_s=args.timeout_s)


if __name__ == "__main__":
    raise SystemExit(main())
