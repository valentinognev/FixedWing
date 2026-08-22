"""Inject race camera + spawn-velocity plugin into a PX4 Gazebo plane SDF."""
from __future__ import annotations

import math
import re

_LINK_NAME = re.compile(r"""<link\s+name=["']([^"']+)["']""")


def canonical_link_name(sdf: str) -> str:
    names = _LINK_NAME.findall(sdf)
    if not names:
        raise ValueError("no <link name=...> in SDF")
    if "base_link" in names:
        return "base_link"
    return names[0]


def inject_race_cam(
    sdf: str,
    *,
    width: int,
    height: int,
    hfov_deg: float,
    eye_forward_m: float,
    update_rate_hz: float = 10.0,
) -> str:
    parent = canonical_link_name(sdf)
    hfov = math.radians(float(hfov_deg))
    snippet = f"""
    <link name="race_cam_link">
      <pose relative_to="{parent}">{float(eye_forward_m):g} 0 0 0 0 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <sensor name="race_cam" type="camera">
        <always_on>1</always_on>
        <update_rate>{float(update_rate_hz):.1f}</update_rate>
        <visualize>false</visualize>
        <camera>
          <horizontal_fov>{hfov:.6f}</horizontal_fov>
          <image>
            <width>{int(width)}</width>
            <height>{int(height)}</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>2000</far>
          </clip>
          <anti_aliasing>0</anti_aliasing>
        </camera>
      </sensor>
    </link>
    <joint name="race_cam_joint" type="fixed">
      <parent>{parent}</parent>
      <child>race_cam_link</child>
    </joint>
"""
    idx = sdf.rfind("</model>")
    if idx < 0:
        raise ValueError("SDF missing </model>")
    return sdf[:idx] + snippet + sdf[idx:]


def inject_spawn_velocity_plugin(sdf: str) -> str:
    snippet = """
    <plugin filename="gz-sim-python-system-loader-system"
            name="gz::sim::systems::PythonSystemLoader">
      <module_name>race_spawn_velocity</module_name>
    </plugin>
"""
    idx = sdf.rfind("</model>")
    if idx < 0:
        raise ValueError("SDF missing </model>")
    return sdf[:idx] + snippet + sdf[idx:]


def apply_plane_overlay(
    sdf: str,
    *,
    width: int,
    height: int,
    hfov_deg: float,
    eye_forward_m: float,
    update_rate_hz: float = 10.0,
) -> str:
    with_cam = inject_race_cam(
        sdf,
        width=width,
        height=height,
        hfov_deg=hfov_deg,
        eye_forward_m=eye_forward_m,
        update_rate_hz=update_rate_hz,
    )
    return inject_spawn_velocity_plugin(with_cam)
