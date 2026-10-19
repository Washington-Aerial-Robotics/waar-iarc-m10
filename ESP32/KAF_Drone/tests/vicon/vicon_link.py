"""
Vicon Tracker -> KAF drone link. Runs on any laptop that can reach both the
Vicon PC (Ethernet/lab network, DataStream port 801) and the drone (its WiFi).
See README.md in this folder for the full lab setup.

Modes:
    vicon    Print the Vicon pose only - no drone needed. Use this first.
    monitor  Vicon pose next to the drone's own state estimate. Sends nothing
             but state requests, so it works with ANY firmware (firmware_full
             included) - use it to check axes/yaw before flying.
    bridge   Streams Vicon positions to the drone as COM_SET_MOCAP (needs
             firmware_vicon) and shows the drone's state alongside.
    relay    On the Vicon PC, forward poses to --pi-ip via UDP. On the Pi,
             use monitor/bridge --udp-source <PC-IP>. See deploy/raspberry-pi.

Examples:
    python vicon_link.py vicon   --vicon 192.168.10.2 --object waar1
    python vicon_link.py monitor --vicon 192.168.10.2 --object waar1
    python vicon_link.py bridge  --vicon 192.168.10.2 --object waar1 --log flight1.csv
    python vicon_link.py bridge  --fake            # synthetic circle, no Vicon needed

Frames: Vicon Tracker world is Z-up, millimetres. The drone's body frame is
x FRONT, y LEFT, z UP (what the motor mixing needs - verify with
../relay_autotune.py check), and yaw 0 = the direction the drone faced when it
calibrated at boot, so at yaw 0 the front points along world +x. Boot the
drone facing Vicon +X and no rotation is needed; otherwise pass
--yaw-offset-deg (monitor mode prints the measured difference).
"""

import argparse
import csv
import math
import socket
import sys
import threading
import time

from kaf_link import DEFAULT_IP, DEFAULT_DRONE_ID, KafLink
from udp_pose import MAX_POSE_AGE, UDP_PORT, Pose, UdpSource, encode_pose


def quaternion_yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class ViconSource:
    """Background thread keeping the latest pose of one Tracker object."""

    def __init__(self, host, subject, fake=False):
        self.host = host if ":" in host else f"{host}:801"
        self.subject = subject
        self.fake = fake
        self.latest = None
        self.occluded_frames = 0
        self.error = None
        self._last_frame_time = None
        self._frames_in_window = 0
        self._max_gap = 0.0
        self._window_start = time.perf_counter()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run_fake if fake else self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()

    def get(self):
        with self._lock:
            return self.latest

    def _publish(self, pose):
        with self._lock:
            self.latest = pose

    def _run_fake(self):
        # 0.5 m circle at 1 m height, one lap per 10 s, 100 Hz - for testing without the lab
        frame = 0
        start = time.perf_counter()
        while not self._stop.is_set():
            t = time.perf_counter() - start
            angle = 2.0 * math.pi * t / 10.0
            self._record_frame(time.perf_counter())
            self._publish(Pose(frame, 0.5 * math.cos(angle), 0.5 * math.sin(angle), 1.0,
                               wrap_angle(angle), time.perf_counter()))
            frame += 1
            time.sleep(0.01)

    def _run(self):
        try:
            from vicon_dssdk import ViconDataStream
        except ImportError:
            self.error = ("vicon_dssdk not installed - install the Python wheel that ships with the "
                          "Vicon DataStream SDK (see README.md), or use --fake")
            return
        while not self._stop.is_set():
            client = ViconDataStream.Client()
            self._connect(client, ViconDataStream)
            self._stream(client, ViconDataStream)  # returns on disconnect -> reconnect

    def _connect(self, client, ViconDataStream):
        print(f"Connecting to Vicon DataStream at {self.host} ...")
        while not self._stop.is_set():
            try:
                client.Connect(self.host)
                break
            except ViconDataStream.DataStreamException as exc:
                print(f"  not connected ({exc}), retrying")
                time.sleep(1.0)
        client.EnableSegmentData()
        # ServerPush blocks until each new frame; a 1-frame buffer keeps latency minimal
        client.SetStreamMode(ViconDataStream.Client.StreamMode.EServerPush)
        try:
            client.SetBufferSize(1)
        except (AttributeError, ViconDataStream.DataStreamException):
            pass
        print("Connected to Vicon.")

    def _stream(self, client, ViconDataStream):
        no_frame_since = time.perf_counter()
        segment = None
        while not self._stop.is_set():
            try:
                client.GetFrame()
            except ViconDataStream.DataStreamException as exc:
                if "NotConnected" in str(exc):
                    print("Vicon connection lost - reconnecting (is Tracker still running / PC on the router?)")
                    return
                # NoFrame: no new frame yet (normal right after connecting and between frames) - retry fast,
                # a long sleep here would itself make the pose stale
                if "NoFrame" not in str(exc):
                    print(f"Vicon read error: {exc}")
                    time.sleep(0.05)
                elif time.perf_counter() - no_frame_since > 5.0:
                    print("No frames from Vicon for 5 s - is Tracker running with the cameras live?")
                    no_frame_since = time.perf_counter()
                time.sleep(0.001)
                continue
            now = time.perf_counter()
            no_frame_since = now
            self._record_frame(now)
            try:
                if segment is None:
                    subjects = client.GetSubjectNames()
                    if self.subject not in subjects:
                        self.error = f"object '{self.subject}' not in Tracker - available: {subjects}"
                        return
                    segment = client.GetSubjectRootSegmentName(self.subject)
                (tx, ty, tz), t_occluded = client.GetSegmentGlobalTranslation(self.subject, segment)
                (qx, qy, qz, qw), q_occluded = client.GetSegmentGlobalRotationQuaternion(self.subject, segment)
            except ViconDataStream.DataStreamException as exc:
                print(f"Vicon read error: {exc}")
                continue
            if t_occluded or q_occluded:
                self.occluded_frames += 1
                continue  # send nothing - the drone's MOCAP_STALE_MS timeout handles dropouts
            self._publish(Pose(client.GetFrameNumber(), tx / 1000.0, ty / 1000.0, tz / 1000.0,
                               quaternion_yaw(qx, qy, qz, qw), now))

    def _record_frame(self, now):
        with self._lock:
            if self._last_frame_time is not None:
                self._max_gap = max(self._max_gap, now - self._last_frame_time)
            self._last_frame_time = now
            self._frames_in_window += 1

    def stats(self):
        """Frames/s and largest gap between frames (ms) since the previous call."""
        with self._lock:
            now = time.perf_counter()
            elapsed = now - self._window_start
            rate = self._frames_in_window / elapsed if elapsed > 0 else 0.0
            gap = self._max_gap * 1000.0
            self._window_start, self._frames_in_window, self._max_gap = now, 0, 0.0
            return rate, gap


def to_drone_frame(pose, yaw_offset):
    """Rotates a Vicon-world pose by -yaw_offset about Z into the drone's world frame."""
    c, s = math.cos(-yaw_offset), math.sin(-yaw_offset)
    return (c * pose.x - s * pose.y, s * pose.x + c * pose.y, pose.z, wrap_angle(pose.yaw - yaw_offset))


def fmt(v):
    return "   ---" if v is None or math.isnan(v) else f"{v:+6.2f}"


def positive_number(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a finite number greater than zero")
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", choices=("vicon", "monitor", "bridge", "relay"))
    parser.add_argument("--vicon", default="localhost", help="Vicon PC IP (port 801 assumed)")
    parser.add_argument("--object", default="waar1", help="object name in Vicon Tracker")
    parser.add_argument("--fake", action="store_true", help="synthetic Vicon data, no Vicon PC needed")
    parser.add_argument("--drone-ip", default=DEFAULT_IP)
    parser.add_argument("--drone-port", type=int, default=70, help="TCP port (normally 70)")
    parser.add_argument("--drone-id", default=DEFAULT_DRONE_ID, help="kafenv.info.deviceID of the drone")
    parser.add_argument("--rate", type=positive_number, default=100.0, help="pose send rate, Hz")
    parser.add_argument("--state-rate", type=positive_number, default=10.0, help="drone state request rate, Hz")
    parser.add_argument("--pi-ip", help="relay mode: Pi IPv4 address (UDP destination)")
    parser.add_argument("--udp-source", help="receive poses from this PC IPv4 address instead of the Vicon SDK")
    parser.add_argument("--udp-bind", default="0.0.0.0", help="local UDP receive interface")
    parser.add_argument("--udp-port", type=int, default=UDP_PORT)
    parser.add_argument("--duration", type=positive_number, help="stop after this many seconds (default: until Ctrl+C)")
    parser.add_argument("--yaw-offset-deg", type=float, default=0.0,
                        help="Vicon-world yaw of the drone's frame (rotate Vicon data by minus this)")
    parser.add_argument("--log", help="CSV file to record Vicon and drone state")
    args = parser.parse_args()
    if args.mode == "relay" and not args.pi_ip:
        parser.error("relay requires --pi-ip")
    if args.udp_source and (args.fake or args.mode == "relay"):
        parser.error("--udp-source cannot be combined with --fake or relay mode")
    if not 1 <= args.udp_port <= 65535 or not 1 <= args.drone_port <= 65535:
        parser.error("ports must be between 1 and 65535")
    if len(args.drone_id) != 1 or ord(args.drone_id) > 255:
        parser.error("--drone-id must be one byte")
    if not math.isfinite(args.yaw_offset_deg):
        parser.error("--yaw-offset-deg must be finite")

    vicon = (UdpSource(args.udp_source, args.udp_bind, args.udp_port) if args.udp_source
             else ViconSource(args.vicon, args.object, fake=args.fake))
    vicon.start()
    relay = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if args.mode == "relay" else None
    relay_address = (socket.gethostbyname(args.pi_ip), args.udp_port) if relay else None
    drone = None
    if args.mode in ("monitor", "bridge"):
        print(f"Connecting to drone at {args.drone_ip}:{args.drone_port} ...")
        try:
            drone = KafLink(args.drone_ip, port=args.drone_port, drone_id=args.drone_id)
        except OSError as exc:
            vicon.stop()
            sys.exit(f"Could not reach the drone at {args.drone_ip}:70 ({exc}).\n"
                     "  - Is the drone powered, and is this laptop on the same router?\n"
                     "  - Check the drone's serial output at boot: 'GG>G[ip=...]' shows its real IP; "
                     "'Falling Back To AP' means it couldn't join the router.\n"
                     "  - Only one TCP client at a time: close the phone app or other vicon_link windows.\n"
                     "  - You can run 'vicon' mode meanwhile, it doesn't need the drone.")
        print("Connected to drone.")

    log_file = open(args.log, "w", newline="") if args.log else None
    log = csv.writer(log_file) if log_file else None
    if log:
        log.writerow(["t", "vicon_frame", "vx", "vy", "vz", "vyaw", "sent",
                      "dx", "dy", "dz", "dvx", "dvy", "dvz", "dyaw"])

    yaw_offset = math.radians(args.yaw_offset_deg)
    send_period = 1.0 / args.rate
    state_period = 1.0 / args.state_rate
    sequence = 0
    last_sent_frame = None
    last_logged_frame = None
    last_print = 0.0
    next_send = time.perf_counter()
    next_state = next_send + send_period / 2  # offset from mocap sends so the drone doesn't merge packets
    sent_count = 0
    start = time.perf_counter()
    warned_firmware = False
    last_drone_send = -math.inf

    try:
        while True:
            if vicon.error:
                sys.exit(f"Vicon error: {vicon.error}")
            now = time.perf_counter()
            if args.duration is not None and now - start >= args.duration:
                break
            pose = vicon.get()
            fresh = pose is not None and 0 <= (now - pose.stamp) < MAX_POSE_AGE
            drone_pose = to_drone_frame(pose, yaw_offset) if pose else None

            sent = False
            if now >= next_send:
                # never catch up with a burst - back-to-back packets merge on the drone and get lost
                next_send = max(next_send + send_period, now + send_period / 2)
                if (args.mode in ("bridge", "relay") and fresh and pose.frame != last_sent_frame
                        and (not drone or now - last_drone_send >= 0.003)):
                    last_sent_frame = pose.frame
                    sequence += 1
                    if relay:
                        # Send the original world frame; the Pi applies the yaw offset once.
                        relay.sendto(encode_pose(pose), relay_address)
                    else:
                        drone.send_mocap(sequence, *drone_pose)
                        last_drone_send = now
                    sent = True
                    sent_count += 1
            # Scheduler stalls can erase the initial half-period offset. Keep an
            # actual minimum gap so telemetry isn't written immediately after mocap.
            if drone and now >= next_state and now - last_drone_send >= 0.003:
                next_state = max(next_state + state_period, now + state_period / 2)
                drone.request_state()
                last_drone_send = now
            if drone:
                drone.poll()
                if (args.mode == "bridge" and not warned_firmware and drone.mocap_failures > 20
                        and drone.mocap_acks == 0):
                    print("WARNING: the drone rejects COM_SET_MOCAP - is it running firmware_vicon?")
                    warned_firmware = True

            state = drone.latest_state if drone else None
            new_frame = pose is not None and pose.frame != last_logged_frame
            if new_frame:
                last_logged_frame = pose.frame
            if log and (sent or (args.mode not in ("bridge", "relay") and new_frame)):
                log.writerow([f"{now - start:.4f}", pose.frame if pose else "",
                              *(f"{v:.4f}" for v in (drone_pose or (math.nan,) * 4)), int(sent),
                              *(f"{v:.4f}" for v in ((*state.x, *state.v, state.yaw) if state else (math.nan,) * 7))])

            if now - last_print >= 0.5:
                last_print = now
                if pose is None:
                    suffix = f" (UDP rejected={vicon.rejected}; check clocks/source IP)" if args.udp_source else ""
                    print(f"waiting for Vicon object '{args.object}' ...{suffix}")
                else:
                    vx, vy, vz, vyaw = drone_pose
                    rate, gap = vicon.stats()
                    line = (f"vicon {fmt(vx)} {fmt(vy)} {fmt(vz)} m yaw {fmt(math.degrees(vyaw))}"
                            f"{'' if fresh else '  [STALE]'}  {rate:4.0f} Hz gap {gap:4.0f} ms"
                            f"  occl={vicon.occluded_frames}")
                    if state and not any((*state.x, *state.v, *state.q, *state.w)):
                        # every field exactly 0: the flight loop isn't estimating (NULL_MODE, flight.cpp)
                        line += " | drone state all zero - calibrate it, then set INACTIVE mode"
                    elif state:
                        dx, dy, dz = state.x
                        yaw_diff = math.degrees(wrap_angle(vyaw - state.yaw))
                        line += (f" | drone {fmt(dx)} {fmt(dy)} {fmt(dz)} m yaw {fmt(math.degrees(state.yaw))}"
                                 f"  yaw diff {fmt(yaw_diff)} deg")
                    elif drone:
                        line += " | drone: no state reply yet"
                    if args.mode == "bridge":
                        line += f" | sent {sent_count} ok {drone.mocap_acks} fail {drone.mocap_failures}"
                    elif relay:
                        line += f" | relayed {sent_count}"
                    if args.udp_source:
                        line += f" | udp rejected {vicon.rejected}"
                    print(line)
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("\nStopped. In POS_SETPOINT/TRAJECTORY mode the drone's failsafe lands it once Vicon goes "
              "stale (150 ms); in manual/accel modes nothing happens - use the kill switch.")
    finally:
        vicon.stop()
        if drone:
            drone.close()
        if relay:
            relay.close()
        if log_file:
            log_file.close()


if __name__ == "__main__":
    main()
