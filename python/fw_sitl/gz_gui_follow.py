"""Gazebo GUI chase camera: look at in-air spawn and follow the plane."""
from __future__ import annotations

import argparse
import math
import os
import re
import subprocess
import sys
import time
from pathlib import Path

DEFAULT_GUI_CONFIG = Path("/usr/share/gz/gz-sim/gui/gui.config")
DEFAULT_BACK_M = 10.0
DEFAULT_UP_M = 3.0
DEFAULT_PITCH_RAD = 0.35
STOCK_CAMERA_POSE = "-6 0 6 0 0.5 0"


def parse_pose_csv(pose: str) -> tuple[float, float, float, float, float, float]:
    parts = [float(p.strip()) for p in pose.split(",") if p.strip() != ""]
    while len(parts) < 6:
        parts.append(0.0)
    x, y, z, roll, pitch, yaw = parts[:6]
    return x, y, z, roll, pitch, yaw


def chase_camera_pose(
    pose_csv: str,
    *,
    back_m: float = DEFAULT_BACK_M,
    up_m: float = DEFAULT_UP_M,
    pitch_rad: float = DEFAULT_PITCH_RAD,
) -> str:
    """World camera pose behind/above the spawn, looking along heading."""
    x, y, z, _r, _p, yaw = parse_pose_csv(pose_csv)
    cx = x - float(back_m) * math.cos(yaw)
    cy = y - float(back_m) * math.sin(yaw)
    cz = z + float(up_m)
    return f"{cx:.6g} {cy:.6g} {cz:.6g} 0 {float(pitch_rad):.6g} {yaw:.6g}"


def listed_model_names(text: str) -> list[str]:
    """Parse `gz model --list` stdout into exact model names (not substrings)."""
    names: list[str] = []
    for raw in text.splitlines():
        line = raw.strip().strip('"')
        if line.startswith("- "):
            line = line[2:].strip().strip('"')
        low = line.lower()
        if not line or low.startswith("request") or low.startswith("available"):
            continue
        names.append(line)
    return names


def resolve_follow_model(listed: list[str], model: str) -> str | None:
    """Pick the world model to chase. Prefer `model_0` over substring `model`."""
    listed_set = set(listed)
    for name in sorted(model_name_candidates(model), key=len, reverse=True):
        if name in listed_set:
            return name
    prefix = f"{model}_"
    for name in listed:
        if name.startswith(prefix):
            return name
    if model in listed_set:
        return model
    return None


def patch_gui_config(
    text: str,
    camera_pose: str,
    *,
    follow_model: str | None = None,
    back_m: float = DEFAULT_BACK_M,
    up_m: float = DEFAULT_UP_M,
) -> str:
    """Move MinimalScene camera to the chase pose and extend far clip."""
    patched = re.sub(
        r"<camera_pose>[^<]*</camera_pose>",
        f"<camera_pose>{camera_pose}</camera_pose>",
        text,
        count=1,
    )
    if patched == text and "<camera_pose>" not in text:
        raise ValueError("gui.config missing <camera_pose>")
    if "<camera_clip>" not in patched:
        patched = patched.replace(
            f"<camera_pose>{camera_pose}</camera_pose>",
            (
                f"<camera_pose>{camera_pose}</camera_pose>\n"
                "  <camera_clip>\n"
                "    <near>0.25</near>\n"
                "    <far>25000</far>\n"
                "  </camera_clip>"
            ),
            1,
        )
    if follow_model and "<plugin filename=\"CameraTracking\"" in patched:
        follow_name = f"{follow_model}_0"
        offset = f"{-float(back_m):g} 0 {float(up_m):g}"
        if "<follow_target>" not in patched:
            patched = re.sub(
                r'(<plugin filename="CameraTracking"[^>]*>)',
                (
                    rf"\1\n  <follow_target>{follow_name}</follow_target>\n"
                    rf"  <follow_offset>{offset}</follow_offset>"
                ),
                patched,
                count=1,
            )
    return patched


def camera_track_protobuf(
    model: str,
    *,
    back_m: float = DEFAULT_BACK_M,
    up_m: float = DEFAULT_UP_M,
) -> str:
    """gz.msgs.CameraTrack text: FOLLOW (2) with body-frame chase offset."""
    return (
        "track_mode: 2, "
        f'follow_target: {{name: "{model}"}}, '
        f"follow_offset: {{x: {-float(back_m):g}, y: 0, z: {float(up_m):g}}}, "
        "follow_pgain: 1.0, track_pgain: 1.0"
    )


def model_name_candidates(model: str) -> list[str]:
    names = [model]
    if not model.startswith("gz_"):
        names.append(f"gz_{model}")
    names.append(f"{model}_0")
    return names


def write_follow_gui_config(
    dest: Path,
    pose_csv: str,
    *,
    stock: Path = DEFAULT_GUI_CONFIG,
    back_m: float = DEFAULT_BACK_M,
    up_m: float = DEFAULT_UP_M,
    model: str = "rc_cessna",
) -> Path:
    if not stock.is_file():
        raise FileNotFoundError(stock)
    cam = chase_camera_pose(pose_csv, back_m=back_m, up_m=up_m)
    dest.write_text(
        patch_gui_config(
            stock.read_text(encoding="utf-8"),
            cam,
            follow_model=model,
            back_m=back_m,
            up_m=up_m,
        ),
        encoding="utf-8",
    )
    return dest


def _gz_model_list_stdout() -> str:
    try:
        proc = subprocess.run(
            ["gz", "model", "--list"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return ""
    return proc.stdout or ""


def publish_follow(model: str, protobuf: str) -> int:
    try:
        return subprocess.run(
            [
                "gz",
                "topic",
                "-t",
                "/gui/track",
                "-m",
                "gz.msgs.CameraTrack",
                "-p",
                protobuf,
            ],
            check=False,
            timeout=5,
        ).returncode
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return 1


def follow_loop(
    model: str,
    *,
    period_s: float = 2.0,
    timeout_s: float = 0.0,
    back_m: float = DEFAULT_BACK_M,
    up_m: float = DEFAULT_UP_M,
) -> int:
    """Publish FOLLOW until timeout (0 = forever). Wait for the model name."""
    deadline = None if timeout_s <= 0 else time.time() + timeout_s
    locked: str | None = None
    protobuf = ""
    while deadline is None or time.time() < deadline:
        if locked is None:
            locked = resolve_follow_model(
                listed_model_names(_gz_model_list_stdout()), model
            )
            if locked is not None:
                protobuf = camera_track_protobuf(locked, back_m=back_m, up_m=up_m)
                print(f"gz follow: model {locked} listed — chasing", flush=True)
        if locked is not None:
            publish_follow(locked, protobuf)
        time.sleep(max(0.2, period_s))
    return 0 if locked is not None else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Gazebo GUI chase-follow the PX4 plane")
    parser.add_argument("--model", default="rc_cessna")
    parser.add_argument(
        "--pose",
        default=os.environ.get("PX4_GZ_MODEL_POSE", "0,0,500,0,0,1.570796"),
    )
    parser.add_argument("--write-gui-config", type=Path, default=None)
    parser.add_argument("--follow", action="store_true")
    parser.add_argument("--timeout-s", type=float, default=0.0)
    parser.add_argument("--back-m", type=float, default=DEFAULT_BACK_M)
    parser.add_argument("--up-m", type=float, default=DEFAULT_UP_M)
    args = parser.parse_args(argv)

    if args.write_gui_config is not None:
        write_follow_gui_config(
            args.write_gui_config,
            args.pose,
            back_m=args.back_m,
            up_m=args.up_m,
            model=args.model,
        )
        print(f"wrote {args.write_gui_config} pose={args.pose}")

    if args.follow:
        return follow_loop(
            args.model,
            timeout_s=args.timeout_s,
            back_m=args.back_m,
            up_m=args.up_m,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
