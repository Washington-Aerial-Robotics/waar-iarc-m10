"""
Minimal client for the KAF drone's raw TCP command link (periph_wifi.cpp, port 70).

Wire format, traced from communication.cpp - keep in sync if the firmware changes:
    packet  = header + content, one packet per TCP write, no length prefix
    header  = toID, fromID, messageType, messageID   (4 x uint8, packet_header)
    content = the packed (#pragma pack 1) little-endian struct for that messageType

The drone reads whatever bytes are available as ONE packet, so packets sent
back-to-back can merge and the second is silently lost. Space them out by a
few ms (vicon_link.py does) and keep Nagle off (done here).

The drone only accepts one TCP client at a time - the Flutter ground station
also uses port 70, so it can't be connected while this is. The browser ground
station (HTTP, port 80) is unaffected.
"""

import math
import select
import socket
import struct

DEFAULT_IP = "192.168.1.240"  # drone's own AP address (periph_wifi.cpp)
DEFAULT_PORT = 70
DEFAULT_DRONE_ID = "U"        # kafenv.info.deviceID default (kaf_drone.cpp)
DEFAULT_SENDER_ID = "M"       # anything except the drone's own ID

COM_CMD = 0x40
COM_ID = 0x3F
COM_REQUEST_STEST = COM_CMD | 4
COM_REPLY_STEST = 4
COM_SET_PIDTUNING = COM_CMD | 30   # pid_tuner.cpp
COM_SET_MOCAP = COM_CMD | 40       # mocap.cpp - only handled by firmware_vicon
COM_REQUEST_RELAYTUNE = COM_CMD | 41  # pid_tuner.cpp relay autotune status
COM_REPLY_RELAYTUNE = 41
COM_SUCCESS = 60
COM_FAILURE = 61
COM_ACKNOWLEDGED = 62
COM_INVALID = 63

HEADER = struct.Struct("<BBBB")
# drone_state::stateestimate - x, v, q, w (4 coordinates x 3 floats)
STATE = struct.Struct("<12f")
# mocap.cpp's mocappose - sequence, x, y, z, yaw
MOCAP_PAYLOAD = struct.Struct("<I4f")
# pid_tuner.cpp's relaystatus - phase, axis, cycles, abortReason, then 10 floats
RELAY_STATUS = struct.Struct("<BBBB10f")

# Content length of every reply this client can receive. Custom (com_receiveMessage) commands are
# acknowledged with their own ID minus COM_CMD (communication.cpp replyMessage's default case), e.g.
# COM_SET_MOCAP -> 40 - NOT with COM_SUCCESS. A type missing here can't be framed, so the buffer is dropped.
REPLY_SIZES = {
    COM_REPLY_STEST: STATE.size,
    COM_REPLY_RELAYTUNE: RELAY_STATUS.size,
    COM_SET_MOCAP & COM_ID: 0,
    COM_SET_PIDTUNING & COM_ID: 0,
    COM_SUCCESS: 0, COM_FAILURE: 0, COM_ACKNOWLEDGED: 0, COM_INVALID: 0,
}


class DroneState:
    """Decoded COM_REPLY_STEST (kafenv.state)."""

    def __init__(self, values):
        self.x = values[0:3]    # position, m, world frame
        self.v = values[3:6]    # velocity, m/s, world frame
        self.q = values[6:9]    # q.x, q.y body-frame quaternion terms, q.z = yaw in rad
        self.w = values[9:12]   # angular rate, rad/s, body frame

    @property
    def yaw(self):
        return self.q[2]


class KafLink:
    def __init__(self, ip=DEFAULT_IP, port=DEFAULT_PORT, drone_id=DEFAULT_DRONE_ID,
                 sender_id=DEFAULT_SENDER_ID, timeout=3.0):
        self.drone_id = ord(drone_id)
        self.sender_id = ord(sender_id)
        self.sock = socket.create_connection((ip, port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._rx = bytearray()
        self._message_id = 0
        self.latest_state = None
        self.replies = {}   # latest raw payload per reply type
        self.mocap_failures = 0
        self.mocap_acks = 0

    def close(self):
        self.sock.close()

    def _send(self, message_type, content=b""):
        self._message_id = (self._message_id + 1) & 0xFF
        header = HEADER.pack(self.drone_id, self.sender_id, message_type, self._message_id)
        self.sock.sendall(header + content)

    def send(self, message_type, content=b""):
        """Sends any command; its reply type/payload lands in self.replies after a later poll()."""
        self._send(message_type, content)

    def send_mocap(self, sequence, x, y, z, yaw=math.nan):
        """Position in metres in the firmware world frame; yaw in rad (stored by the drone, not fused)."""
        self._send(COM_SET_MOCAP, MOCAP_PAYLOAD.pack(sequence & 0xFFFFFFFF, x, y, z, yaw))

    def request_state(self):
        """Asks for kafenv.state - the reply shows up in latest_state after a later poll()."""
        self._send(COM_REQUEST_STEST)

    def poll(self):
        """Reads and decodes whatever replies have arrived, without blocking."""
        while select.select([self.sock], [], [], 0)[0]:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("drone closed the TCP connection")
            self._rx += chunk
        while len(self._rx) >= HEADER.size:
            _, _, message_type, _ = HEADER.unpack_from(self._rx)
            size = REPLY_SIZES.get(message_type)
            if size is None:
                # unknown reply - we can't know its length, so drop the buffer to resync
                self._rx.clear()
                break
            end = HEADER.size + size
            if len(self._rx) < end:
                break
            payload = bytes(self._rx[HEADER.size:end])
            del self._rx[:end]
            self.replies[message_type] = payload
            if message_type == COM_REPLY_STEST:
                self.latest_state = DroneState(STATE.unpack(payload))
            elif message_type == COM_SET_MOCAP & COM_ID:
                self.mocap_acks += 1
            elif message_type == COM_FAILURE:
                # the only command vicon_link streams is COM_SET_MOCAP, so failures there are mocap rejections
                self.mocap_failures += 1
