"""
pc6.py  –  Stable rover controller with soda-can auto alignment
===============================================================

Features
--------
• Stable proportional auto-alignment
• Manual alignment cancel button (G)
• Polls every OTHER frame
• Camera offset compensation
• Slows while turning
• Prevents spinouts
• TCP rover link
• Simple curses UI
• Claw & servo controls (toggle and fine adjust)
"""

import argparse
import binascii
import curses
import queue
import socket
import struct
import threading
import time
import tempfile
import os
import cv2

from dataclasses import dataclass, field

from inference_sdk import InferenceHTTPClient


# ──────────────────────────────────────────────────────────────────────────────
# Config
# ──────────────────────────────────────────────────────────────────────────────

CMD_TYPE = 0x01
TELEM_TYPE = 0x02
TELEM_LEN = 17

PROCESS_EVERY = 2

# camera alignment compensation
CAMERA_OFFSET_NORM = 0.00

# alignment tuning
ALIGN_THROTTLE = 0.10
ALIGN_GAIN = 0.40
ALIGN_DEADBAND = 0.05
ALIGN_MAX_STEERING = 0.30

# claw / servo step sizes
CLAW_STEP = 0.05
SERVO_STEP = 0.05


# ──────────────────────────────────────────────────────────────────────────────
# Soda detector
# ──────────────────────────────────────────────────────────────────────────────

class SodaDetector:

    def __init__(
        self,
        stream_url,
        api_key,
        workspace="label-y3rcq",
        workflow="find-soda-can",
    ):

        self.frame_count = 0

        self.client = InferenceHTTPClient(
            api_url="https://serverless.roboflow.com",
            api_key=api_key,
        )

        self.workspace = workspace
        self.workflow = workflow

        self.cap = cv2.VideoCapture(stream_url)

        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open stream: {stream_url}")

    def _run_inference(self, frame):

        with tempfile.NamedTemporaryFile(
            suffix=".jpg",
            delete=False
        ) as tmp:

            temp_path = tmp.name

        try:

            cv2.imwrite(temp_path, frame)

            result = self.client.run_workflow(
                workspace_name=self.workspace,
                workflow_id=self.workflow,
                images={"image": temp_path},
                use_cache=True,
            )

            try:
                preds = result[0]["predictions"]["predictions"]
            except Exception:
                preds = []

            return preds

        finally:

            if os.path.exists(temp_path):
                os.remove(temp_path)

    def get_offset(self):

        while True:

            ret, frame = self.cap.read()

            if not ret:
                return None

            self.frame_count += 1

            # every other frame
            if self.frame_count % PROCESS_EVERY != 0:
                continue

            preds = self._run_inference(frame)

            if not preds:
                return None

            h, w = frame.shape[:2]

            center_x = w / 2
            center_y = h / 2

            best = max(
                preds,
                key=lambda p: p.get("confidence", 0)
            )

            obj_x = float(best["x"])
            obj_y = float(best["y"])

            dx = obj_x - center_x
            dy = obj_y - center_y

            offset_x = (dx / center_x) - CAMERA_OFFSET_NORM
            offset_y = dy / center_y

            return {
                "offset_x": offset_x,
                "offset_y": offset_y,
                "dx": dx,
                "dy": dy,
                "x": obj_x,
                "y": obj_y,
                "confidence": best.get("confidence", 0),
            }

    def close(self):
        self.cap.release()


# ──────────────────────────────────────────────────────────────────────────────
# Packet helpers
# ──────────────────────────────────────────────────────────────────────────────

def build_cmd(
    throttle,
    steering,
    claw,
    servo2=0.5,
    servo3=0.5
):

    t = max(-100, min(100, int(throttle * 100))) & 0xFF
    s = max(-100, min(100, int(steering * 100))) & 0xFF

    c = max(0, min(100, int(claw * 100)))
    s2 = max(0, min(100, int(servo2 * 100)))
    s3 = max(0, min(100, int(servo3 * 100)))

    payload = bytes([
        CMD_TYPE,
        t,
        s,
        c,
        s2,
        s3
    ])

    crc = binascii.crc32(payload) & 0xFFFFFFFF

    return payload + struct.pack("<I", crc)


def frame_cmd(
    throttle,
    steering,
    claw,
    servo2=0.5,
    servo3=0.5
):

    payload = build_cmd(
        throttle,
        steering,
        claw,
        servo2,
        servo3
    )

    return struct.pack("<H", len(payload)) + payload


# ──────────────────────────────────────────────────────────────────────────────
# Telemetry
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class Telemetry:

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    motors_on: bool = False
    claw_cl: bool = False

    sonar_mm: int = -1

    pan: float = 0.5

    age: float = field(default_factory=time.monotonic)

    @property
    def stale(self):
        return (time.monotonic() - self.age) > 1.0


def parse_telem(data):

    if len(data) < TELEM_LEN:
        return None

    if data[0] != TELEM_TYPE:
        return None

    _, roll, pitch, yaw, flags = struct.unpack_from(
        "<BfffB",
        data
    )

    sonar_v = struct.unpack_from("<H", data, 14)[0]

    pan_byte = data[16]

    return Telemetry(
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        motors_on=bool(flags & 0x01),
        claw_cl=bool(flags & 0x02),
        sonar_mm=-1 if sonar_v == 0xFFFF else sonar_v,
        pan=pan_byte / 100.0,
    )


# ──────────────────────────────────────────────────────────────────────────────
# Rover link
# ──────────────────────────────────────────────────────────────────────────────

class RoverLink:

    def __init__(self, host, port=5005):

        self.host = host
        self.port = port

        self.telem = Telemetry()

        self._sock = None

        self._send_q = queue.Queue(maxsize=20)

        self._stop = threading.Event()
        self._connected = threading.Event()

    @property
    def connected(self):
        return self._connected.is_set()

    def start(self):

        threading.Thread(
            target=self._run,
            daemon=True
        ).start()

    def stop(self):
        self._stop.set()

    def send(
        self,
        throttle,
        steering,
        claw,
        servo2=0.5,
        servo3=0.5
    ):

        try:

            self._send_q.put_nowait(
                frame_cmd(
                    throttle,
                    steering,
                    claw,
                    servo2,
                    servo3
                )
            )

        except queue.Full:
            pass

    def _run(self):

        rx_buf = bytearray()

        while not self._stop.is_set():

            try:

                s = socket.socket(
                    socket.AF_INET,
                    socket.SOCK_STREAM
                )

                s.settimeout(5.0)

                s.connect((self.host, self.port))

                s.settimeout(0.05)

                self._sock = s

                self._connected.set()

                print("Connected")

            except OSError:

                self._connected.clear()

                print("Reconnect...")

                time.sleep(2)

                continue

            while not self._stop.is_set():

                while not self._send_q.empty():

                    try:

                        frame = self._send_q.get_nowait()

                        s.sendall(frame)

                    except Exception:
                        break

                try:

                    chunk = s.recv(256)

                    if not chunk:
                        break

                    rx_buf.extend(chunk)

                    while len(rx_buf) >= 2:

                        plen = struct.unpack_from(
                            "<H",
                            rx_buf
                        )[0]

                        if len(rx_buf) < 2 + plen:
                            break

                        payload = bytes(
                            rx_buf[2:2 + plen]
                        )

                        rx_buf = rx_buf[2 + plen:]

                        t = parse_telem(payload)

                        if t:
                            self.telem = t

                except socket.timeout:
                    pass

                except OSError:
                    break

            try:
                s.close()
            except:
                pass

            self._connected.clear()


# ──────────────────────────────────────────────────────────────────────────────
# Drive state
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class DriveState:

    throttle: float = 0.0
    steering: float = 0.0

    claw: float = 0.0

    servo2: float = 0.5
    servo3: float = 0.5


# ──────────────────────────────────────────────────────────────────────────────
# Auto-align
# ──────────────────────────────────────────────────────────────────────────────

def auto_align(
    detector,
    rover,
    drive,
    stop_event,
    invert=False
):

    settled = 0

    while True:

        # manual cancel
        if stop_event.is_set():

            print("Auto-align cancelled")

            rover.send(
                0,
                0,
                drive.claw,
                drive.servo2,
                drive.servo3
            )

            drive.throttle = 0
            drive.steering = 0

            return

        result = detector.get_offset()

        if result is None:

            print("Lost target")

            rover.send(
                0,
                0,
                drive.claw
            )

            drive.throttle = 0
            drive.steering = 0

            return

        offset_x = result["offset_x"]

        print(
            f"offset={offset_x:.3f} "
            f"conf={result['confidence']:.2f}"
        )

        if abs(offset_x) < ALIGN_DEADBAND:

            settled += 1

            drive.throttle = 0
            drive.steering = 0

            rover.send(
                0,
                0,
                drive.claw
            )

            if settled >= 5:

                print("Centered")

                return

            continue

        settled = 0

        steering = ALIGN_GAIN * offset_x

        if invert:
            steering *= -1

        steering = max(
            -ALIGN_MAX_STEERING,
            min(ALIGN_MAX_STEERING, steering)
        )

        # slow while turning
        move_throttle = ALIGN_THROTTLE * (
            1.0 - abs(steering)
        )

        drive.throttle = move_throttle
        drive.steering = steering

        rover.send(
            drive.throttle,
            drive.steering,
            drive.claw,
            drive.servo2,
            drive.servo3
        )

        time.sleep(0.03)


# ──────────────────────────────────────────────────────────────────────────────
# UI
# ──────────────────────────────────────────────────────────────────────────────

def draw(
    stdscr,
    rover,
    drive,
    align_active
):

    stdscr.erase()

    stdscr.addstr(
        1,
        2,
        f"Connected: {rover.connected}"
    )

    stdscr.addstr(
        2,
        2,
        f"Throttle: {drive.throttle:+.2f}"
    )

    stdscr.addstr(
        3,
        2,
        f"Steering: {drive.steering:+.2f}"
    )

    stdscr.addstr(
        4,
        2,
        f"Claw:     {drive.claw:.2f}"
    )

    stdscr.addstr(
        5,
        2,
        f"Servo2:   {drive.servo2:.2f}"
    )

    stdscr.addstr(
        6,
        2,
        f"Servo3:   {drive.servo3:.2f}"
    )

    stdscr.addstr(
        8,
        2,
        f"Roll : {rover.telem.roll:+.2f}"
    )

    stdscr.addstr(
        9,
        2,
        f"Pitch: {rover.telem.pitch:+.2f}"
    )

    stdscr.addstr(
        10,
        2,
        f"Yaw  : {rover.telem.yaw:+.2f}"
    )

    stdscr.addstr(
        12,
        2,
        f"Auto Align: {'ON' if align_active else 'OFF'}"
    )

    stdscr.addstr(15, 2, "W/S throttle")
    stdscr.addstr(16, 2, "A/D steering")
    stdscr.addstr(17, 2, "C toggle claw (0/1)")
    stdscr.addstr(18, 2, "Z/X claw -/+")
    stdscr.addstr(19, 2, "E/R servo2 -/+")
    stdscr.addstr(20, 2, "T/Y servo3 -/+")
    stdscr.addstr(21, 2, "SPACE stop")
    stdscr.addstr(22, 2, "F start auto-align")
    stdscr.addstr(23, 2, "G stop auto-align")
    stdscr.addstr(24, 2, "Q quit")

    stdscr.refresh()


# ──────────────────────────────────────────────────────────────────────────────
# Main TUI
# ──────────────────────────────────────────────────────────────────────────────

def run_tui(
    stdscr,
    rover,
    detector
):

    curses.curs_set(0)

    stdscr.nodelay(True)

    drive = DriveState()

    align_thread = None
    align_stop = threading.Event()

    while True:

        key = stdscr.getch()

        if key in (ord('q'), ord('Q')):
            break

        # throttle
        if key in (ord('w'), ord('W')):
            drive.throttle = min(
                1.0,
                drive.throttle + 0.1
            )

        if key in (ord('s'), ord('S')):
            drive.throttle = max(
                -1.0,
                drive.throttle - 0.1
            )

        # steering
        if key in (ord('a'), ord('A')):
            drive.steering = max(
                -1.0,
                drive.steering - 0.1
            )

        if key in (ord('d'), ord('D')):
            drive.steering = min(
                1.0,
                drive.steering + 0.1
            )

        # stop rover
        if key == ord(' '):

            drive.throttle = 0
            drive.steering = 0

        # claw toggle (C)
        if key in (ord('c'), ord('C')):
            drive.claw = 0.0 if drive.claw > 0.5 else 1.0

        # claw fine adjust
        if key in (ord('z'), ord('Z')):
            drive.claw = max(0.0, drive.claw - CLAW_STEP)

        if key in (ord('x'), ord('X')):
            drive.claw = min(1.0, drive.claw + CLAW_STEP)

        # servo2 adjust
        if key in (ord('e'), ord('E')):
            drive.servo2 = max(0.0, drive.servo2 - SERVO_STEP)

        if key in (ord('r'), ord('R')):
            drive.servo2 = min(1.0, drive.servo2 + SERVO_STEP)

        # servo3 adjust
        if key in (ord('t'), ord('T')):
            drive.servo3 = max(0.0, drive.servo3 - SERVO_STEP)

        if key in (ord('y'), ord('Y')):
            drive.servo3 = min(1.0, drive.servo3 + SERVO_STEP)

        # start auto-align
        if key in (ord('f'), ord('F')):

            if detector and (
                not align_thread
                or not align_thread.is_alive()
            ):

                align_stop.clear()

                align_thread = threading.Thread(
                    target=auto_align,
                    args=(
                        detector,
                        rover,
                        drive,
                        align_stop
                    ),
                    daemon=True
                )

                align_thread.start()

        # stop auto-align
        if key in (ord('g'), ord('G')):

            align_stop.set()

            drive.throttle = 0
            drive.steering = 0

            rover.send(
                0,
                0,
                drive.claw,
                drive.servo2,
                drive.servo3
            )

            print("Auto-align stopped")

        # manual drive only when not aligning
        if not align_thread or not align_thread.is_alive():

            rover.send(
                drive.throttle,
                drive.steering,
                drive.claw,
                drive.servo2,
                drive.servo3
            )

        draw(
            stdscr,
            rover,
            drive,
            align_thread is not None
            and align_thread.is_alive()
        )

        time.sleep(0.03)


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def main():

    ap = argparse.ArgumentParser()

    ap.add_argument(
        "--rover",
        required=True
    )

    ap.add_argument(
        "--port",
        type=int,
        default=5005
    )

    ap.add_argument(
        "--stream",
        required=True
    )

    ap.add_argument(
        "--apikey",
        required=True
    )

    args = ap.parse_args()

    rover = RoverLink(
        args.rover,
        args.port
    )

    rover.start()

    detector = SodaDetector(
        stream_url=args.stream,
        api_key=args.apikey
    )

    try:

        curses.wrapper(
            run_tui,
            rover,
            detector
        )

    finally:

        rover.send(0, 0, 0)

        time.sleep(0.1)

        rover.stop()

        detector.close()

        print("Disconnected")


if __name__ == "__main__":
    main()