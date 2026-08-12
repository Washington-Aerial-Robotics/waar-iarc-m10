"""
Spin each motor individually at a low, fixed throttle so you can visually
confirm PWM channel -> physical corner -> propeller/rotation direction are
all correct, one motor at a time - safer and more precise than watching all
four spin together.

Talks over the raw TCP protocol (port 70) - the same one the Flutter Ground
Station app uses - because the webserver's only motor-control endpoint
(GMAN) always mixes all four motors from virtual joystick input; there's no
webserver path to isolate a single motor. This script uses ACTUATION_MODE
(the same pass-through mode manual throttle testing already uses), so
whatever value is sent for a motor is exactly what the ESC gets, with no PID
mixing involved.

Expected mapping (derived from ESC_PINS in common_data.h and the motor
mixing math in flight.cpp's flight_attitudeControl - see the ARM/DISARM
session notes): diagonal pairs must share the same rotation direction.

    Motor 0 - GPIO25 - Front-Left  - expect CCW (viewed from above)
    Motor 1 - GPIO26 - Rear-Left   - expect CW
    Motor 2 - GPIO32 - Front-Right - expect CW
    Motor 3 - GPIO33 - Rear-Right  - expect CCW

Brushless motors can't be slowed to an eye-trackable continuous RPM (even
your lowest tested bench point was already ~10,700 RPM) - lowering --value
further mostly risks unreliable spin-up, not visible slow-down. Instead,
each motor pulses briefly and repeatedly from a dead stop (press Enter to
re-pulse) so you get several short looks at the initial swing direction
instead of one continuous blur. For a fully certain answer, record a pulse
with your phone's slow-motion camera mode and play it back.

SAFETY: this arms the real ESCs and spins real motors. Clear the area,
confirm restraint/bed security, and keep DISARM/KILL within reach before
confirming the prompt. The script disarms automatically on exit, including
Ctrl+C, but that is not a substitute for watching what you're doing.

Usage:
    python motor_direction_test.py
    python motor_direction_test.py --ip 192.168.1.240 --value 0.12
    python motor_direction_test.py --pulse 0.3
"""

import argparse
import socket
import struct
import sys
import time

DEFAULT_IP = "192.168.1.240"
PORT = 70

APP_ID = ord("G")
DRONE_ID = ord("U")

# message types, matching drone_protocol.dart's DroneComms
COM_SET_FLIGHTMODE = 0x4C
COM_SET_ACTUATION = 0x4A
COM_SET_MOTORS = 0x40 | 16

CMD_NULL_MODE = 16     # commander.h CMD_MODE_MASK bits - keeps the commander state machine idle
ACTUATION_MODE = 3     # flight.h DEFAULT_MODES_MASK bits - pure pass-through, no PID mixing

# Same coalescing workaround used in the Flutter app: the firmware reads all
# currently-available TCP bytes into one buffer and only processes the first
# packet in it, so back-to-back sends with no gap can silently drop the second.
PACKET_GAP_S = 0.06

# periph_esc.cpp's ESC_ARM_FRAMES=200 at a 10ms flight loop - ESCs hold at zero
# throttle for this long after actuation goes true before accepting nonzero values.
ESC_ARM_DELAY_S = 2.5

MOTORS = [
    (0, "GPIO25", "Front-Left", "CCW"),
    (1, "GPIO26", "Rear-Left", "CW"),
    (2, "GPIO32", "Front-Right", "CW"),
    (3, "GPIO33", "Rear-Right", "CCW"),
]

_msg_id = 0


def _next_msg_id() -> int:
    global _msg_id
    _msg_id = (_msg_id + 1) & 0xFF
    return _msg_id


def build_packet(msg_type: int, payload: bytes) -> bytes:
    return bytes([DRONE_ID, APP_ID, msg_type, _next_msg_id()]) + payload


def send(sock: socket.socket, msg_type: int, payload: bytes) -> None:
    sock.sendall(build_packet(msg_type, payload))


def arm(sock: socket.socket) -> None:
    mode_byte = CMD_NULL_MODE | ACTUATION_MODE
    send(sock, COM_SET_FLIGHTMODE, struct.pack("<BB", mode_byte, 0))
    time.sleep(PACKET_GAP_S)
    send(sock, COM_SET_ACTUATION, struct.pack("<B", 0xFF))
    print(f"Armed. Waiting {ESC_ARM_DELAY_S:.1f}s for ESC arm sequence...")
    time.sleep(ESC_ARM_DELAY_S)


def disarm(sock: socket.socket) -> None:
    send(sock, COM_SET_ACTUATION, struct.pack("<B", 0x00))
    time.sleep(PACKET_GAP_S)
    send(sock, COM_SET_FLIGHTMODE, struct.pack("<BB", CMD_NULL_MODE, 0))
    print("Disarmed.")


def set_motors(sock: socket.socket, motors: list[float]) -> None:
    send(sock, COM_SET_MOTORS, struct.pack("<4f", *motors))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ip", default=DEFAULT_IP, help=f"Drone IP (default {DEFAULT_IP})")
    parser.add_argument("--value", type=float, default=0.12, help="Test throttle, 0-1 (default 0.12)")
    parser.add_argument("--pulse", type=float, default=0.2, help="Pulse duration in seconds (default 0.2)")
    args = parser.parse_args()

    print(__doc__)
    print(f"Test value: {args.value}")
    confirm = input("Area clear, restraint secure, DISARM/KILL within reach - type 'yes' to arm: ")
    if confirm.strip().lower() != "yes":
        print("Aborted.")
        sys.exit(0)

    sock = socket.create_connection((args.ip, PORT), timeout=5)
    try:
        arm(sock)
        set_motors(sock, [0.0, 0.0, 0.0, 0.0])

        for index, gpio, corner, direction in MOTORS:
            print(f"\n--- Motor {index}: {gpio}, expected corner={corner}, expected direction={direction} ---")
            print(f"  If a DIFFERENT motor spins instead of {corner}: PWM/wiring mismatch - the motor")
            print(f"    connector plugged into {gpio}'s ESC channel is at the wrong corner. Fix by")
            print(f"    rewiring which motor connector goes to that channel (or which GPIO drives it),")
            print(f"    not by touching phase wires.")
            print(f"  If {corner} spins but NOT {direction}: that's not a PWM/GPIO issue - swapping pins")
            print(f"    won't change which way a motor spins. Fix by swapping any 2 of that motor's 3")
            print(f"    phase wires, which reverses its rotation direction.")
            motors = [0.0, 0.0, 0.0, 0.0]
            motors[index] = args.value
            while True:
                cmd = input(
                    f"[Motor {index}] Enter alone = pulse {args.pulse:.1f}s. "
                    f"Type 'n' + Enter = skip to next motor WITHOUT pulsing: "
                )
                if cmd.strip() != "":
                    break
                print(f"  -> pulsing motor {index} now...")
                set_motors(sock, motors)
                time.sleep(args.pulse)
                set_motors(sock, [0.0, 0.0, 0.0, 0.0])
                time.sleep(0.3)

        print("\nAll four tested.")
    finally:
        disarm(sock)
        sock.close()


if __name__ == "__main__":
    main()
