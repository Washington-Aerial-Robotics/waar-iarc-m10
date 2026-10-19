"""Small PC-to-Pi pose relay; no Vicon SDK is needed on the Pi.

Both hosts must synchronize their UTC clocks. The timestamp is the PC's SDK
receipt time, not a camera exposure timestamp. Use only on the private lab LAN:
source-IP filtering is not authentication. Never forward a cached pose as new.
"""

from dataclasses import dataclass
import math
import socket
import struct
import threading
import time


UDP_PORT = 5100
MAX_POSE_AGE = 0.100
CLOCK_MARGIN = 0.020
PACKET = struct.Struct("<4sId4d")  # magic, frame, Unix seconds, x/y/z metres, yaw radians
MAGIC = b"KAF1"


@dataclass
class Pose:
    frame: int
    x: float
    y: float
    z: float
    yaw: float
    stamp: float  # local perf_counter(), preserving age across the relay


def encode_pose(pose, *, wall_now=None, mono_now=None):
    wall_now = time.time() if wall_now is None else wall_now
    mono_now = time.perf_counter() if mono_now is None else mono_now
    captured = wall_now - (mono_now - pose.stamp)
    return PACKET.pack(MAGIC, pose.frame & 0xFFFFFFFF, captured,
                       pose.x, pose.y, pose.z, pose.yaw)


class UdpSource:
    """Accept fresh, ordered poses from one explicitly selected IPv4 host."""

    def __init__(self, source_ip, bind="0.0.0.0", port=UDP_PORT):
        self.source_ip = socket.gethostbyname(source_ip)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind, port))
        self.sock.settimeout(0.1)
        self.latest = None
        self.error = None
        self.occluded_frames = 0  # same display interface as ViconSource
        self.rejected = 0
        self._last_captured = -math.inf
        self._last_frame = None
        self._last_received = -math.inf
        self._window_start = time.perf_counter()
        self._frames = 0
        self._gap = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=0.5)
        self.sock.close()

    def get(self):
        with self._lock:
            return self.latest

    def accept(self, data, peer_ip, *, wall_now=None, mono_now=None):
        """Decode one datagram. Rejecting it must never refresh the current pose."""
        wall_now = time.time() if wall_now is None else wall_now
        mono_now = time.perf_counter() if mono_now is None else mono_now
        if peer_ip != self.source_ip or len(data) != PACKET.size:
            self.rejected += 1
            return False
        magic, frame, captured, x, y, z, yaw = PACKET.unpack(data)
        age = wall_now - captured
        if (magic != MAGIC or not all(map(math.isfinite, (captured, x, y, z, yaw)))
                or not -CLOCK_MARGIN <= age < MAX_POSE_AGE - CLOCK_MARGIN
                or captured <= self._last_captured
                or (self._last_frame is not None and frame <= self._last_frame
                    and mono_now - self._last_received < 1.0)):
            self.rejected += 1
            return False
        with self._lock:
            if self.latest is not None:
                self._gap = max(self._gap, mono_now - self._last_received)
            self._last_captured = captured
            self._last_frame = frame
            self._last_received = mono_now
            # Charge the full clock uncertainty against the freshness budget.
            self.latest = Pose(frame, x, y, z, yaw,
                               mono_now - max(0.0, age) - CLOCK_MARGIN)
            self._frames += 1
        return True

    def _run(self):
        while not self._stop.is_set():
            try:
                data, peer = self.sock.recvfrom(2048)
                self.accept(data, peer[0])
            except socket.timeout:
                continue
            except OSError as exc:
                if not self._stop.is_set():
                    self.error = f"UDP relay failed: {exc}"
                return

    def stats(self):
        with self._lock:
            now = time.perf_counter()
            rate = self._frames / max(now - self._window_start, 1e-9)
            gap = self._gap * 1000.0
            self._window_start, self._frames, self._gap = now, 0, 0.0
            return rate, gap
