"""Viewport frame capture: a request parked by the service thread, served by the render loop."""

from __future__ import annotations

import ctypes
import threading
from collections.abc import Callable

import attrs
import carb
import numpy as np
import sensor_msgs.msg
from builtin_interfaces.msg import Time
from omni.kit.viewport.utility import capture_viewport_to_buffer, get_active_viewport

from .controller import Pose

# the pose lands in Fabric one frame after the USD write, two more renders let the
# temporal passes converge on the new view, then the capture takes the next one
SETTLE_FRAMES = 3
TIMEOUT_S = 15.0


@attrs.define
class CaptureRequest:
    local: Pose
    world_orientation: bool
    fov: float
    min_sim_time: float  # 0 = no constraint
    done: threading.Event = attrs.field(factory=threading.Event)
    image: sensor_msgs.msg.Image | None = None
    message: str = ""
    posed: bool = False
    settle: int = SETTLE_FRAMES
    scheduled: bool = False

    def finish(self, image: sensor_msgs.msg.Image | None, message: str) -> None:
        self.image = image
        self.message = message
        self.done.set()


def _capsule_pointer(buffer: object, size: int) -> ctypes.Array:
    """The renderer hands the pixels over as a PyCapsule around a raw pointer."""
    get_pointer = ctypes.pythonapi.PyCapsule_GetPointer
    get_pointer.restype = ctypes.c_void_p
    get_pointer.argtypes = [ctypes.py_object, ctypes.c_char_p]
    address = get_pointer(buffer, None)
    return ctypes.cast(address, ctypes.POINTER(ctypes.c_uint8 * size)).contents


def _stamp(sim_time: float | None) -> Time:
    seconds = sim_time or 0.0
    return Time(sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))


def to_image(buffer: object, size: int, width: int, height: int, byte_format: object, stamp: Time, frame_id: str) -> sensor_msgs.msg.Image:
    """Repack the LdrColor AOV (8-bit RGBA or BGRA) as rgb8."""
    channels = size // (width * height)
    if channels not in (3, 4) or channels * width * height != size:
        raise ValueError(f"unexpected buffer layout {size} bytes for {width}x{height} {byte_format}")
    pixels = np.ctypeslib.as_array(_capsule_pointer(buffer, size)).reshape(height, width, channels)[:, :, :3]
    if "B8G8R8" in str(byte_format):
        pixels = pixels[:, :, ::-1]
    image = sensor_msgs.msg.Image()
    image.header.stamp = stamp
    image.header.frame_id = frame_id
    image.width = width
    image.height = height
    image.encoding = "rgb8"
    image.is_bigendian = False
    image.step = width * 3
    image.data = np.ascontiguousarray(pixels).tobytes()
    return image


class CaptureQueue:
    """Single slot handed from the service thread to the render loop. The service spins
    single-threaded and blocks per request, so requests serialize and the slot never holds
    more than one."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._pending: CaptureRequest | None = None

    def submit(self, request: CaptureRequest) -> None:
        with self._lock:
            self._pending = request

    def peek(self) -> CaptureRequest | None:
        with self._lock:
            return self._pending

    def drop(self, request: CaptureRequest) -> None:
        with self._lock:
            if self._pending is request:
                self._pending = None

    def serve(self, request: CaptureRequest, sim_time: float | None, frame_id: str, place: Callable[[Pose, bool, float], None]) -> None:
        """Advance the request by one render-loop frame: gate, pose, settle, grab."""
        # a microsecond of slack: min_sim_time arrives as ns, sim_time is the tick's double
        if request.min_sim_time > 0.0 and (sim_time is None or sim_time + 1e-6 < request.min_sim_time):
            return
        if not request.posed:
            place(request.local, request.world_orientation, request.fov)
            request.posed = True
            return
        if request.settle > 0:
            request.settle -= 1
            return
        if request.scheduled:
            return
        request.scheduled = True
        stamp = _stamp(sim_time)
        viewport = get_active_viewport()
        if viewport is None:
            self.drop(request)
            request.finish(None, "no active viewport")
            return

        def on_buffer(buffer: object, size: int, width: int, height: int, byte_format: object) -> None:
            self.drop(request)
            if request.done.is_set():
                return
            try:
                request.finish(to_image(buffer, size, width, height, byte_format, stamp, frame_id), "ok")
            except Exception as e:
                carb.log_warn(f"arena: viewport capture failed: {e}")
                request.finish(None, f"capture failed: {e}")

        capture_viewport_to_buffer(viewport, on_buffer)
