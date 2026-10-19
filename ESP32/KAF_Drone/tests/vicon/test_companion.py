"""Offline protocol/freshness checks. All network traffic is loopback only."""

import math
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import unittest

from kaf_link import (COM_ID, COM_REQUEST_STEST, COM_REPLY_STEST, COM_SET_MOCAP,
                      HEADER, MOCAP_PAYLOAD, STATE, KafLink)
from udp_pose import MAGIC, PACKET, Pose, UdpSource, encode_pose
from vicon_link import quaternion_yaw, to_drone_frame, wrap_angle


class FreshnessTests(unittest.TestCase):
    def setUp(self):
        self.source = UdpSource("127.0.0.1", bind="127.0.0.1", port=0)
        self.addCleanup(self.source.stop)

    def packet(self, frame=10, captured=99.97, x=1.0):
        return PACKET.pack(MAGIC, frame, captured, x, 2.0, 3.0, 0.4)

    def accept(self, packet, peer="127.0.0.1", mono=5.0, wall=100.0):
        return self.source.accept(packet, peer, wall_now=wall, mono_now=mono)

    def test_relay_preserves_age_and_units(self):
        data = encode_pose(Pose(10, 1, 2, 3, 0.4, 9.97), wall_now=100, mono_now=10)
        self.assertTrue(self.source.accept(data, "127.0.0.1", wall_now=100.01, mono_now=20))
        pose = self.source.get()
        self.assertEqual((pose.x, pose.y, pose.z), (1, 2, 3))
        self.assertAlmostEqual(20 - pose.stamp, 0.06)  # 40 ms age + clock margin

    def test_invalid_packets_never_refresh_a_pose(self):
        self.assertTrue(self.accept(self.packet()))
        original = self.source.get()
        cases = [
            (self.packet(), "127.0.0.2"),
            (b"bad", "127.0.0.1"),
            (b"BAD!" + self.packet()[4:], "127.0.0.1"),
            (self.packet(frame=11, captured=99.0), "127.0.0.1"),  # delayed
            (self.packet(frame=11, captured=101.0), "127.0.0.1"),  # clock mismatch
            (self.packet(frame=11, captured=99.98, x=math.nan), "127.0.0.1"),
            (self.packet(frame=11, captured=99.98, x=math.inf), "127.0.0.1"),
            (self.packet(), "127.0.0.1"),  # duplicate
            (self.packet(frame=9, captured=99.98), "127.0.0.1"),  # out of order
        ]
        for packet, peer in cases:
            with self.subTest(peer=peer, packet=packet):
                self.assertFalse(self.accept(packet, peer))
                self.assertIs(self.source.get(), original)

    def test_sender_restart_allowed_only_after_silence(self):
        self.assertTrue(self.accept(self.packet()))
        self.assertFalse(self.accept(self.packet(frame=0, captured=99.98)))
        self.assertTrue(self.accept(self.packet(frame=0, captured=101.97), mono=7, wall=102))

    def test_coordinate_rotation(self):
        result = to_drone_frame(Pose(1, 1, 0, 2, math.pi / 2, 0), math.pi / 2)
        for actual, expected in zip(result, (0, -1, 2, 0)):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(quaternion_yaw(0, 0, math.sin(0.5), math.cos(0.5)), 1)
        self.assertAlmostEqual(wrap_angle(3 * math.pi), -math.pi)


class MockDrone:
    """Models TCP stream framing/replies, not ESP32 flight dynamics or timing."""

    def __init__(self):
        self.server = socket.socket()
        self.server.bind(("127.0.0.1", 0))
        self.server.listen(1)
        self.server.settimeout(5)
        self.port = self.server.getsockname()[1]
        self.connected = threading.Event()
        self.done = threading.Event()
        self.commands = []
        self.poses = []
        self.error = None
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        try:
            with self.server.accept()[0] as client:
                client.settimeout(0.1)
                self.connected.set()
                buffer = bytearray()
                while not self.done.is_set():
                    try:
                        chunk = client.recv(4096)
                    except socket.timeout:
                        continue
                    if not chunk:
                        return
                    buffer.extend(chunk)
                    while len(buffer) >= HEADER.size:
                        to_id, from_id, kind, msg_id = HEADER.unpack_from(buffer)
                        if kind not in (COM_SET_MOCAP, COM_REQUEST_STEST):
                            raise AssertionError(f"Unexpected flight command: {kind}")
                        length = MOCAP_PAYLOAD.size if kind == COM_SET_MOCAP else 0
                        if len(buffer) < HEADER.size + length:
                            break
                        payload = bytes(buffer[HEADER.size:HEADER.size + length])
                        del buffer[:HEADER.size + length]
                        self.commands.append(kind)
                        if kind == COM_SET_MOCAP:
                            self.poses.append((time.perf_counter(), MOCAP_PAYLOAD.unpack(payload)))
                            reply = HEADER.pack(from_id, to_id, kind & COM_ID, msg_id)
                        else:
                            reply = HEADER.pack(from_id, to_id, COM_REPLY_STEST, msg_id) + STATE.pack(*range(12))
                        # Exercise the client's ability to buffer a fragmented TCP reply.
                        client.sendall(reply[:2])
                        client.sendall(reply[2:])
        except Exception as exc:
            self.error = exc
        finally:
            self.server.close()

    def close(self):
        self.done.set()
        self.thread.join(timeout=6)


class LoopbackTests(unittest.TestCase):
    def test_pc_relay_cli_sends_to_pi_without_a_drone_connection(self):
        source = UdpSource("127.0.0.1", bind="127.0.0.1", port=0)
        self.addCleanup(source.stop)
        port = source.sock.getsockname()[1]
        source.start()
        result = subprocess.run([
            sys.executable, str(Path(__file__).with_name("vicon_link.py")), "relay",
            "--fake", "--pi-ip", "127.0.0.1", "--udp-port", str(port), "--duration", "0.3",
        ], capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        pose = source.get()
        self.assertIsNotNone(pose, result.stdout)
        self.assertGreater(pose.frame, 2)
        self.assertAlmostEqual(math.hypot(pose.x, pose.y), 0.5)
        self.assertEqual(pose.z, 1.0)
        self.assertIsNone(source.error)

    def test_fragmented_telemetry_and_mocap_ack(self):
        drone = MockDrone()
        self.addCleanup(drone.close)
        link = KafLink("127.0.0.1", port=drone.port)
        self.addCleanup(link.close)
        link.send_mocap(7, 1, 2, 3, 0.4)
        link.request_state()
        deadline = time.perf_counter() + 2
        while time.perf_counter() < deadline:
            link.poll()
            if link.latest_state and link.mocap_acks:
                break
            time.sleep(0.005)
        self.assertEqual(link.mocap_acks, 1)
        self.assertIsNotNone(link.latest_state)
        self.assertEqual(link.latest_state.x, (0.0, 1.0, 2.0))
        self.assertEqual(link.latest_state.yaw, 8.0)
        self.assertIsNone(drone.error)

    def test_udp_to_tcp_bridge_stops_forwarding_after_source_dropout(self):
        drone = MockDrone()
        self.addCleanup(drone.close)
        # Reserve a port, then release it for the bridge subprocess.
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
            reserve.bind(("127.0.0.1", 0))
            udp_port = reserve.getsockname()[1]
        proc = subprocess.Popen([
            sys.executable, str(Path(__file__).with_name("vicon_link.py")), "bridge",
            "--udp-source", "127.0.0.1", "--udp-bind", "127.0.0.1",
            "--udp-port", str(udp_port), "--drone-ip", "127.0.0.1",
            "--drone-port", str(drone.port), "--duration", "2", "--yaw-offset-deg", "90",
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.addCleanup(lambda: proc.kill() if proc.poll() is None else None)
        self.assertTrue(drone.connected.wait(timeout=5), "bridge never connected")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            for frame in range(40):
                sender.sendto(encode_pose(Pose(frame, 1, 0, 2, math.pi / 2, time.perf_counter())),
                              ("127.0.0.1", udp_port))
                time.sleep(0.01)
        stopped_at = time.perf_counter()
        output, _ = proc.communicate(timeout=6)
        self.assertEqual(proc.returncode, 0, output)
        self.assertIsNone(drone.error)
        self.assertGreater(len(drone.poses), 5, output)
        self.assertLess(drone.poses[-1][0], stopped_at + 0.3, "stale data was forwarded")
        _, (_, x, y, z, yaw) = drone.poses[-1]
        for actual, expected in zip((x, y, z, yaw), (0, -1, 2, 0)):
            self.assertAlmostEqual(actual, expected, places=5)
        self.assertTrue(set(drone.commands) <= {COM_SET_MOCAP, COM_REQUEST_STEST})


if __name__ == "__main__":
    unittest.main()
