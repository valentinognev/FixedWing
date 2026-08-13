"""Thin ZMQ PUB/SUB helpers for balloon-race image / color / track channels.

Bind vs connect convention
--------------------------
Producers **bind**, subscribers **connect** (one publisher per endpoint):

- ``image`` — image source binds PUB; camera connects SUB
- ``color`` — control binds PUB; camera connects SUB
- ``track`` — camera binds PUB; control connects SUB

Endpoints come from ``FlightSetup.zmq`` (see ``flightSetup.json``).
Camera/control take the latest message (SUB uses CONFLATE where applicable).
"""
from __future__ import annotations

import json
import struct
import time
from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np
import zmq

TOPIC_IMAGE = b"image"
TOPIC_COLOR = b"color"
TOPIC_TRACK = b"track"

RgbLike = tuple[int, int, int] | Sequence[int]


@dataclass(frozen=True)
class ImageFrame:
    """RGB8 frame on the image→camera channel."""

    stamp: float
    width: int
    height: int
    rgb: bytes

    def as_numpy(self) -> np.ndarray:
        """Return HxWx3 uint8 RGB array (copy)."""
        arr = np.frombuffer(self.rgb, dtype=np.uint8)
        expected = self.width * self.height * 3
        if arr.size != expected:
            raise ValueError(f"rgb size {arr.size} != width*height*3 ({expected})")
        return arr.reshape((self.height, self.width, 3)).copy()

    @classmethod
    def from_numpy(cls, image: np.ndarray, stamp: float | None = None) -> ImageFrame:
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("image must be HxWx3")
        if image.dtype != np.uint8:
            image = np.asarray(image, dtype=np.uint8)
        h, w, _ = image.shape
        return cls(
            stamp=float(time.time() if stamp is None else stamp),
            width=int(w),
            height=int(h),
            rgb=np.ascontiguousarray(image).tobytes(),
        )


@dataclass(frozen=True)
class TargetColor:
    """Commanded balloon RGB (+ assisted flag) on the control→camera channel."""

    r: int
    g: int
    b: int
    stamp: float = 0.0
    assisted: bool = False

    def as_tuple(self) -> tuple[int, int, int]:
        return (int(self.r), int(self.g), int(self.b))


@dataclass(frozen=True)
class TrackMessage:
    """Camera→control track message: in_view + unit LOS in camera frame."""

    in_view: bool
    dir_cam: tuple[float, float, float] | None
    stamp: float = 0.0
    centroid_uv: tuple[float, float] | None = None


def _ctx(context: zmq.Context | None) -> zmq.Context:
    return context if context is not None else zmq.Context.instance()


def bind_pub(endpoint: str, context: zmq.Context | None = None) -> zmq.Socket:
    """Create a PUB socket and bind it (producer side)."""
    sock = _ctx(context).socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 2)
    sock.bind(endpoint)
    return sock


def connect_sub(
    endpoint: str,
    topic: bytes,
    context: zmq.Context | None = None,
    *,
    conflate: bool = True,
) -> zmq.Socket:
    """Create a SUB socket, connect, and subscribe to ``topic``."""
    sock = _ctx(context).socket(zmq.SUB)
    sock.setsockopt(zmq.RCVHWM, 2)
    if conflate:
        sock.setsockopt(zmq.CONFLATE, 1)
    sock.connect(endpoint)
    sock.setsockopt(zmq.SUBSCRIBE, topic)
    return sock


def send_image(sock: zmq.Socket, frame: ImageFrame) -> None:
    """PUB multipart: topic | stamp,f64 + width,u32 + height,u32 | rgb bytes."""
    meta = struct.pack("<dII", float(frame.stamp), int(frame.width), int(frame.height))
    sock.send_multipart([TOPIC_IMAGE, meta, frame.rgb], copy=False)


def recv_image(sock: zmq.Socket, flags: int = 0) -> ImageFrame | None:
    """Receive one image frame, or None on EAGAIN."""
    try:
        parts = sock.recv_multipart(flags=flags, copy=False)
    except zmq.Again:
        return None
    if len(parts) != 3:
        raise ValueError(f"image message expected 3 parts, got {len(parts)}")
    stamp, width, height = struct.unpack_from("<dII", parts[1].buffer)
    return ImageFrame(
        stamp=float(stamp),
        width=int(width),
        height=int(height),
        rgb=parts[2].bytes,
    )


def send_color(sock: zmq.Socket, color: TargetColor | RgbLike) -> None:
    """PUB multipart: topic | JSON {r,g,b,stamp,assisted}."""
    if isinstance(color, TargetColor):
        payload = {
            "r": int(color.r),
            "g": int(color.g),
            "b": int(color.b),
            "stamp": float(color.stamp),
            "assisted": bool(color.assisted),
        }
    else:
        r, g, b = color
        payload = {
            "r": int(r),
            "g": int(g),
            "b": int(b),
            "stamp": float(time.time()),
            "assisted": False,
        }
    sock.send_multipart([TOPIC_COLOR, json.dumps(payload, separators=(",", ":")).encode("utf-8")])


def recv_color(sock: zmq.Socket, flags: int = 0) -> TargetColor | None:
    """Receive one target color, or None on EAGAIN.

    Legacy payloads without ``assisted`` default to ``assisted=False``.
    """
    try:
        parts = sock.recv_multipart(flags=flags)
    except zmq.Again:
        return None
    if len(parts) != 2:
        raise ValueError(f"color message expected 2 parts, got {len(parts)}")
    data = json.loads(parts[1].decode("utf-8"))
    return TargetColor(
        r=int(data["r"]),
        g=int(data["g"]),
        b=int(data["b"]),
        stamp=float(data.get("stamp", 0.0)),
        assisted=bool(data.get("assisted", False)),
    )


def send_track(sock: zmq.Socket, result: TrackMessage) -> None:
    """PUB multipart: topic | JSON {in_view, dir_cam, stamp, centroid_uv?}."""
    payload: dict[str, Any] = {
        "in_view": bool(result.in_view),
        "dir_cam": list(result.dir_cam) if result.dir_cam is not None else None,
        "stamp": float(result.stamp),
    }
    if result.centroid_uv is not None:
        payload["centroid_uv"] = [float(result.centroid_uv[0]), float(result.centroid_uv[1])]
    sock.send_multipart([TOPIC_TRACK, json.dumps(payload, separators=(",", ":")).encode("utf-8")])


def recv_track(sock: zmq.Socket, flags: int = 0) -> TrackMessage | None:
    """Receive one track message, or None on EAGAIN."""
    try:
        parts = sock.recv_multipart(flags=flags)
    except zmq.Again:
        return None
    if len(parts) != 2:
        raise ValueError(f"track message expected 2 parts, got {len(parts)}")
    data = json.loads(parts[1].decode("utf-8"))
    dir_raw = data.get("dir_cam")
    dir_cam: tuple[float, float, float] | None
    if dir_raw is None:
        dir_cam = None
    else:
        if len(dir_raw) != 3:
            raise ValueError("dir_cam must be length 3 or null")
        dir_cam = (float(dir_raw[0]), float(dir_raw[1]), float(dir_raw[2]))
    centroid = data.get("centroid_uv")
    centroid_uv: tuple[float, float] | None
    if centroid is None:
        centroid_uv = None
    else:
        centroid_uv = (float(centroid[0]), float(centroid[1]))
    return TrackMessage(
        in_view=bool(data["in_view"]),
        dir_cam=dir_cam,
        stamp=float(data.get("stamp", 0.0)),
        centroid_uv=centroid_uv,
    )


class ImagePublisher:
    """Bind PUB on the image endpoint (image-source process)."""

    def __init__(self, endpoint: str, context: zmq.Context | None = None) -> None:
        self._sock = bind_pub(endpoint, context)

    def publish(self, frame: ImageFrame | np.ndarray, stamp: float | None = None) -> None:
        if isinstance(frame, ImageFrame):
            send_image(self._sock, frame)
        else:
            send_image(self._sock, ImageFrame.from_numpy(frame, stamp=stamp))

    def close(self) -> None:
        self._sock.close(linger=0)


class ImageSubscriber:
    """Connect SUB on the image endpoint (camera process)."""

    def __init__(self, endpoint: str, context: zmq.Context | None = None) -> None:
        self._sock = connect_sub(endpoint, TOPIC_IMAGE, context)
        self._latest: ImageFrame | None = None

    def poll_and_update(self) -> bool:
        updated = False
        while True:
            frame = recv_image(self._sock, flags=zmq.NOBLOCK)
            if frame is None:
                break
            self._latest = frame
            updated = True
        return updated

    def latest(self) -> ImageFrame | None:
        return self._latest

    def close(self) -> None:
        self._sock.close(linger=0)


class ColorPublisher:
    """Bind PUB on the color endpoint (control process)."""

    def __init__(self, endpoint: str, context: zmq.Context | None = None) -> None:
        self._sock = bind_pub(endpoint, context)

    def publish(self, color: TargetColor | RgbLike) -> None:
        send_color(self._sock, color)

    def close(self) -> None:
        self._sock.close(linger=0)


class ColorSubscriber:
    """Connect SUB on the color endpoint (camera process)."""

    def __init__(self, endpoint: str, context: zmq.Context | None = None) -> None:
        self._sock = connect_sub(endpoint, TOPIC_COLOR, context)
        self._latest: TargetColor | None = None

    def poll_and_update(self) -> bool:
        updated = False
        while True:
            color = recv_color(self._sock, flags=zmq.NOBLOCK)
            if color is None:
                break
            self._latest = color
            updated = True
        return updated

    def latest(self) -> TargetColor | None:
        return self._latest

    def close(self) -> None:
        self._sock.close(linger=0)


class TrackPublisher:
    """Bind PUB on the track endpoint (camera process)."""

    def __init__(self, endpoint: str, context: zmq.Context | None = None) -> None:
        self._sock = bind_pub(endpoint, context)

    def publish(self, result: TrackMessage) -> None:
        send_track(self._sock, result)

    def publish_track(
        self,
        in_view: bool,
        dir_cam: tuple[float, float, float] | None,
        centroid_uv: tuple[float, float] | None = None,
        *,
        stamp: float | None = None,
    ) -> None:
        send_track(
            self._sock,
            TrackMessage(
                in_view=in_view,
                dir_cam=dir_cam,
                stamp=float(time.time() if stamp is None else stamp),
                centroid_uv=centroid_uv,
            ),
        )

    def close(self) -> None:
        self._sock.close(linger=0)


class TrackSubscriber:
    """Connect SUB on the track endpoint (control process)."""

    def __init__(self, endpoint: str, context: zmq.Context | None = None) -> None:
        self._sock = connect_sub(endpoint, TOPIC_TRACK, context)
        self._latest: TrackMessage | None = None

    def poll_and_update(self) -> bool:
        updated = False
        while True:
            msg = recv_track(self._sock, flags=zmq.NOBLOCK)
            if msg is None:
                break
            self._latest = msg
            updated = True
        return updated

    def latest(self) -> TrackMessage | None:
        return self._latest

    def close(self) -> None:
        self._sock.close(linger=0)
