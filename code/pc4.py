"""
rover_pc.py  –  PC rover controller with TUI
=============================================
Connects to the ESP32-S3 over TCP (WiFi) for telemetry and commands.
Optionally reads joystick/button data from the ESP32-C3-mini over USB serial.

NEW: Servo2 (wrist) and Servo3 (pan) keyboard controls.

Usage
─────
  pip install pyserial
  python rover_pc.py --rover 192.168.1.42

  # With C3-mini joystick controller:
  python rover_pc.py --rover 192.168.1.42 --controller /dev/ttyUSB0
  python rover_pc.py --rover 192.168.1.42 --controller auto

  # Keyboard-only (no C3):
  python rover_pc.py --rover 192.168.1.42

TUI controls (keyboard always active, even with joystick connected)
─────────────────────────────────────────────────────────────────────
  W / S        throttle +/-
  A / D        steering left/right
  Space        full stop (throttle + steering → 0)
  C            toggle claw open/close
  E / R        servo2 (wrist) -/+
  T / Y        servo3 (pan)   -/+
  Q / Escape   quit

  Arrow keys   same as WASD
  R            reset yaw to 0 (local display only)

Packet spec (must match firmware)
──────────────────────────────────
CMD  10 bytes  type=0x01 | throttle int8×100 | steering int8×100 |
               claw u8×100 | servo2 u8×100 | servo3 u8×100 | CRC32 LE
TELEM 17 bytes type=0x02 | roll f32 | pitch f32 | yaw f32 | flags u8 |
               sonar u16 | pan u8×100
TCP framing: 2-byte LE length prefix + payload
"""

import argparse
import binascii
import curses
import queue
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial not found – run:  pip install pyserial")

# ─── Packet spec constants ────────────────────────────────────────────────────

CMD_TYPE   = 0x01
TELEM_TYPE = 0x02
TELEM_LEN  = 17   # FIXED: was 19, now matches Pico payload length
CMD_LEN    = 10

# ─── CRC ──────────────────────────────────────────────────────────────────────

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF

# ─── CMD builder ──────────────────────────────────────────────────────────────

def build_cmd(throttle: float, steering: float, claw: float,
              servo2: float = 0.5, servo3: float = 0.5) -> bytes:
    t  = max(-100, min(100, int(throttle * 100))) & 0xFF
    s  = max(-100, min(100, int(steering * 100))) & 0xFF
    c  = max(0, min(100, int(claw   * 100)))
    s2 = max(0, min(100, int(servo2 * 100)))
    s3 = max(0, min(100, int(servo3 * 100)))
    payload = bytes([CMD_TYPE, t, s, c, s2, s3])
    crc = binascii.crc32(payload) & 0xFFFFFFFF
    return payload + struct.pack("<I", crc)

def frame_cmd(throttle, steering, claw, servo2=0.5, servo3=0.5) -> bytes:
    """Length-prefix a CMD payload for TCP."""
    payload = build_cmd(throttle, steering, claw, servo2, servo3)
    return struct.pack("<H", len(payload)) + payload

# ─── TELEM parser ─────────────────────────────────────────────────────────────

@dataclass
class Telemetry:
    roll:      float = 0.0
    pitch:     float = 0.0
    yaw:       float = 0.0
    motors_on: bool  = False
    claw_cl:   bool  = False
    sonar_mm:  int   = -1
    pan:       float = 0.5
    age:       float = field(default_factory=time.monotonic)

    @property
    def sonar_str(self) -> str:
        return "---" if self.sonar_mm < 0 else f"{self.sonar_mm} mm"

    @property
    def stale(self) -> bool:
        return (time.monotonic() - self.age) > 1.0

def parse_telem(data: bytes) -> Optional[Telemetry]:
    if len(data) < TELEM_LEN or data[0] != TELEM_TYPE:
        return None
    # No embedded CRC – integrity guaranteed by framing layer (FrameDecoder on S3)
    _, roll, pitch, yaw, flags = struct.unpack_from("<BfffB", data)
    sonar_v  = struct.unpack_from("<H", data, 14)[0]
    pan_byte = data[16]
    return Telemetry(
        roll      = roll,
        pitch     = pitch,
        yaw       = yaw,
        motors_on = bool(flags & 0x01),
        claw_cl   = bool(flags & 0x02),
        sonar_mm  = -1 if sonar_v == 0xFFFF else sonar_v,
        pan       = pan_byte / 100.0,
    )

# ─── Rover TCP connection ─────────────────────────────────────────────────────

class RoverLink:
    """
    Background thread that maintains the TCP connection to the S3.
    Thread-safe send() and latest telem via .telem property.
    """
    def __init__(self, host: str, port: int = 5005):
        self.host       = host
        self.port       = port
        self.telem      = Telemetry()
        self._sock      = None
        self._lock      = threading.Lock()
        self._send_q    = queue.Queue(maxsize=10)
        self._stop      = threading.Event()
        self._connected = threading.Event()
        self.rx_count   = 0
        self._t0        = time.monotonic()

    @property
    def hz(self) -> float:
        elapsed = time.monotonic() - self._t0
        return self.rx_count / elapsed if elapsed > 0 else 0.0

    @property
    def connected(self) -> bool:
        return self._connected.is_set()

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self._stop.set()

    def send(self, throttle, steering, claw, servo2=0.5, servo3=0.5):
        """Queue a CMD frame.  Non-blocking; drops if queue full."""
        try:
            self._send_q.put_nowait(
                frame_cmd(throttle, steering, claw, servo2, servo3))
        except queue.Full:
            pass

    def _run(self):
        rx_buf = bytearray()
        while not self._stop.is_set():
            # ── Connect ───────────────────────────────────────────────────────
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(5.0)
                s.connect((self.host, self.port))
                s.settimeout(0.05)
                with self._lock:
                    self._sock = s
                self._connected.set()
                rx_buf = bytearray()
            except OSError:
                self._connected.clear()
                time.sleep(2)
                continue

            # ── IO loop ───────────────────────────────────────────────────────
            while not self._stop.is_set():
                # TX
                while not self._send_q.empty():
                    try:
                        frame = self._send_q.get_nowait()
                        with self._lock:
                            if self._sock:
                                self._sock.sendall(frame)
                    except (queue.Empty, OSError):
                        break

                # RX
                try:
                    chunk = s.recv(256)
                    if not chunk:
                        break
                    rx_buf.extend(chunk)
                    while len(rx_buf) >= 2:
                        plen = struct.unpack_from("<H", rx_buf)[0]
                        if len(rx_buf) < 2 + plen:
                            break
                        payload = bytes(rx_buf[2:2 + plen])
                        rx_buf  = rx_buf[2 + plen:]
                        t = parse_telem(payload)
                        if t:
                            self.telem   = t
                            self.rx_count += 1
                except socket.timeout:
                    pass
                except OSError:
                    break

            with self._lock:
                try: s.close()
                except: pass
                self._sock = None
            self._connected.clear()

# ─── C3-mini joystick reader (optional) ──────────────────────────────────────

class ControllerReader:
    """
    Reads JSON lines from the C3-mini over USB serial in a background thread.
    Exposes the latest joystick values and a claw_pending edge flag.
    """
    def __init__(self, port: str, baud: int = 115200):
        self.throttle     = 0.0
        self.steering     = 0.0
        self.claw         = 0
        self.claw_pending = False   # True for one consume() cycle after toggle
        self._lock        = threading.Lock()
        self._stop        = threading.Event()
        self._port        = port
        self._baud        = baud
        self.active       = False

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self._stop.set()

    def consume_claw(self) -> bool:
        """Returns True once per physical button press then resets."""
        with self._lock:
            v = self.claw_pending
            self.claw_pending = False
        return v

    @staticmethod
    def find_port() -> Optional[str]:
        for p in serial.tools.list_ports.comports():
            desc = ((p.description or "") + (p.manufacturer or "")).lower()
            if any(k in desc for k in ("cp210", "ch340", "ch341", "esp32", "silicon")):
                return p.device
        return None

    def _run(self):
        import json
        while not self._stop.is_set():
            try:
                ser = serial.Serial(self._port, self._baud, timeout=0.1)
                self.active = True
                buf = b""
                while not self._stop.is_set():
                    chunk = ser.read(256)
                    if chunk:
                        buf += chunk
                        while b'\n' in buf:
                            line, buf = buf.split(b'\n', 1)
                            line = line.strip()
                            if not line:
                                continue
                            try:
                                obj = json.loads(line.decode())
                            except Exception:
                                continue
                            with self._lock:
                                if "t" in obj:
                                    self.throttle = float(obj["t"])
                                    self.steering = float(obj.get("s", 0.0))
                                    self.claw     = int(obj.get("claw", self.claw))
                                if obj.get("event") == "claw_toggle":
                                    self.claw         = int(obj.get("claw", 1 - self.claw))
                                    self.claw_pending = True
                ser.close()
            except serial.SerialException:
                self.active = False
                time.sleep(2)

# ─── Drive state ──────────────────────────────────────────────────────────────

@dataclass
class DriveState:
    throttle:  float = 0.0
    steering:  float = 0.0
    claw:      float = 0.0   # 0.0 open, 1.0 closed
    servo2:    float = 0.5   # wrist/tilt
    servo3:    float = 0.5   # sonar pan
    claw_locked: bool = False  # True once physical button used

    THROTTLE_STEP = 0.1
    STEERING_STEP = 0.1
    SERVO2_STEP   = 0.05      # 5% per key press
    SERVO3_STEP   = 0.05

# ─── TUI ──────────────────────────────────────────────────────────────────────

def draw(stdscr, rover: RoverLink, drive: DriveState,
         ctrl: Optional[ControllerReader], cmd_hz: float):

    stdscr.erase()
    h, w = stdscr.getmaxyx()
    t = rover.telem

    # colour pairs
    curses.init_pair(1, curses.COLOR_GREEN,  curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_RED,    curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_CYAN,   curses.COLOR_BLACK)
    curses.init_pair(5, curses.COLOR_WHITE,  curses.COLOR_BLACK)

    GREEN  = curses.color_pair(1)
    YELLOW = curses.color_pair(2)
    RED    = curses.color_pair(3)
    CYAN   = curses.color_pair(4)
    BOLD   = curses.color_pair(5) | curses.A_BOLD

    def put(row, col, text, attr=0):
        try:
            stdscr.addstr(row, col, text, attr)
        except curses.error:
            pass

    # ── Header ───────────────────────────────────────────────────────────────
    title = "  ROVER CONTROL  "
    put(0, (w - len(title)) // 2, title, BOLD | curses.A_REVERSE)

    # ── Connection status ─────────────────────────────────────────────────────
    conn_colour = GREEN if rover.connected else RED
    conn_text   = f"S3  {'CONNECTED' if rover.connected else 'CONNECTING…'}  {rover.hz:.1f} Hz"
    put(2, 2, conn_text, conn_colour | curses.A_BOLD)

    ctrl_text = "C3  "
    if ctrl is None:
        ctrl_text += "not connected (keyboard only)"
        ctrl_colour = YELLOW
    elif ctrl.active:
        ctrl_text += "CONNECTED"
        ctrl_colour = GREEN
    else:
        ctrl_text += "searching…"
        ctrl_colour = YELLOW
    put(3, 2, ctrl_text, ctrl_colour | curses.A_BOLD)

    # ── Telemetry panel ───────────────────────────────────────────────────────
    stale = t.stale
    tc    = YELLOW if stale else CYAN

    put(5, 2,  "─── IMU ─────────────────────────────", tc)
    put(6, 4,  f"Roll   {t.roll:+7.2f} °", tc)
    put(7, 4,  f"Pitch  {t.pitch:+7.2f} °", tc)
    put(8, 4,  f"Yaw    {t.yaw:+7.2f} °", tc)

    put(10, 2, "─── Sonar ───────────────────────────", tc)
    sonar_colour = RED if t.sonar_mm > 0 and t.sonar_mm < 300 else tc
    put(11, 4, f"Distance  {t.sonar_str:<10}", sonar_colour | curses.A_BOLD)
    pan_deg = int((t.pan * 180) - 90)
    put(12, 4, f"Pan angle {pan_deg:+d} °", tc)

    put(14, 2, "─── Status ──────────────────────────", tc)
    m_col = GREEN if t.motors_on else RED
    put(15, 4, f"Motors  {'ON ' if t.motors_on else 'OFF'}", m_col | curses.A_BOLD)
    c_col = YELLOW if t.claw_cl else tc
    put(16, 4, f"Claw    {'CLOSED' if t.claw_cl else 'OPEN  '}", c_col | curses.A_BOLD)

    # ── Drive state ───────────────────────────────────────────────────────────
    put(18, 2, "─── Drive ───────────────────────────", curses.color_pair(5))

    # Throttle bar
    bar_w   = 20
    thr_pct = int((drive.throttle + 1.0) / 2.0 * bar_w)
    bar     = "█" * thr_pct + "░" * (bar_w - thr_pct)
    put(19, 4, f"Throttle [{bar}] {drive.throttle:+.2f}", curses.color_pair(5))

    # Steering bar
    str_pct = int((drive.steering + 1.0) / 2.0 * bar_w)
    bar2    = "█" * str_pct + "░" * (bar_w - str_pct)
    put(20, 4, f"Steering [{bar2}] {drive.steering:+.2f}", curses.color_pair(5))

    # Claw
    claw_col = YELLOW if drive.claw > 0.5 else curses.color_pair(5)
    lock_str = " [BTN]" if drive.claw_locked else ""
    put(21, 4, f"Claw     {'CLOSED' if drive.claw > 0.5 else 'OPEN  '}{lock_str}", claw_col)

    # Servo2 (wrist)
    s2_pct = int(drive.servo2 * bar_w)
    bar_s2 = "█" * s2_pct + "░" * (bar_w - s2_pct)
    put(22, 4, f"Wrist    [{bar_s2}] {drive.servo2:.2f}", curses.color_pair(5))

    # Servo3 (pan)
    s3_pct = int(drive.servo3 * bar_w)
    bar_s3 = "█" * s3_pct + "░" * (bar_w - s3_pct)
    put(23, 4, f"Pan      [{bar_s3}] {drive.servo3:.2f}", curses.color_pair(5))

    # CMD rate
    put(24, 4, f"CMD rate {cmd_hz:.1f} Hz", curses.color_pair(5))

    # ── Controls legend ───────────────────────────────────────────────────────
    put(26, 2, "─── Keys ────────────────────────────", curses.color_pair(5))
    put(27, 4, "W/S  Throttle    A/D  Steering", curses.color_pair(5))
    put(28, 4, "Space  Stop      C    Toggle claw", curses.color_pair(5))
    put(29, 4, "E/R   Wrist +/-  T/Y  Pan +/-", curses.color_pair(5))
    put(30, 4, "Q / Esc   Quit", curses.color_pair(5))

    if stale:
        put(h-1, 2, " NO TELEMETRY – check rover connection ", RED | curses.A_REVERSE)

    stdscr.refresh()


def run_tui(stdscr, rover: RoverLink, ctrl: Optional[ControllerReader]):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(0)

    drive    = DriveState()
    cmd_count = 0
    t0        = time.monotonic()
    last_cmd  = time.monotonic()
    last_draw = time.monotonic()
    CMD_HZ    = 20.0
    DRAW_HZ   = 15.0

    while True:
        now = time.monotonic()

        # ── Keyboard input ────────────────────────────────────────────────────
        try:
            key = stdscr.getch()
        except Exception:
            key = -1

        if key in (ord('q'), ord('Q'), 27):   # Q or Escape
            break

        if key in (ord('w'), ord('W'), curses.KEY_UP):
            drive.throttle = min(1.0, drive.throttle + DriveState.THROTTLE_STEP)
        if key in (ord('s'), ord('S'), curses.KEY_DOWN):
            drive.throttle = max(-1.0, drive.throttle - DriveState.THROTTLE_STEP)
        if key in (ord('a'), ord('A'), curses.KEY_LEFT):
            drive.steering = max(-1.0, drive.steering - DriveState.STEERING_STEP)
        if key in (ord('d'), ord('D'), curses.KEY_RIGHT):
            drive.steering = min(1.0, drive.steering + DriveState.STEERING_STEP)
        if key == ord(' '):
            drive.throttle = 0.0
            drive.steering = 0.0
        if key in (ord('c'), ord('C')):
            drive.claw = 0.0 if drive.claw > 0.5 else 1.0

        # Servo2 (wrist) controls
        if key in (ord('e'), ord('E')):
            drive.servo2 = max(0.0, drive.servo2 - DriveState.SERVO2_STEP)
        if key in (ord('r'), ord('R')):
            drive.servo2 = min(1.0, drive.servo2 + DriveState.SERVO2_STEP)

        # Servo3 (pan) controls
        if key in (ord('t'), ord('T')):
            drive.servo3 = max(0.0, drive.servo3 - DriveState.SERVO3_STEP)
        if key in (ord('y'), ord('Y')):
            drive.servo3 = min(1.0, drive.servo3 + DriveState.SERVO3_STEP)

        # ── Joystick override (if C3 connected) ───────────────────────────────
        if ctrl and ctrl.active:
            with ctrl._lock:
                drive.throttle = ctrl.throttle
                drive.steering = ctrl.steering
                if ctrl.consume_claw():
                    drive.claw       = float(ctrl.claw)
                    drive.claw_locked = True
                elif not drive.claw_locked:
                    drive.claw = float(ctrl.claw)

        # ── Send CMD at CMD_HZ ────────────────────────────────────────────────
        if now - last_cmd >= 1.0 / CMD_HZ:
            rover.send(drive.throttle, drive.steering, drive.claw,
                       drive.servo2, drive.servo3)
            cmd_count += 1
            last_cmd = now

        # ── Redraw TUI at DRAW_HZ ─────────────────────────────────────────────
        if now - last_draw >= 1.0 / DRAW_HZ:
            elapsed  = now - t0
            cmd_hz   = cmd_count / elapsed if elapsed > 0 else 0.0
            draw(stdscr, rover, drive, ctrl, cmd_hz)
            last_draw = now

        time.sleep(0.001)

# ─── Entry point ──────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Rover TUI controller")
    ap.add_argument("--rover",      required=True,
                    help="S3 IP address  e.g. 192.168.1.42")
    ap.add_argument("--port",       type=int, default=5005,
                    help="S3 TCP port (default 5005)")
    ap.add_argument("--controller", default=None,
                    help="C3-mini serial port, or 'auto'  (optional)")
    ap.add_argument("--baud",       type=int, default=115200)
    args = ap.parse_args()

    # ── Rover TCP link ────────────────────────────────────────────────────────
    rover = RoverLink(args.rover, args.port)
    rover.start()

    # ── Optional C3-mini controller ───────────────────────────────────────────
    ctrl = None
    if args.controller:
        port = args.controller
        if port.lower() == "auto":
            port = ControllerReader.find_port()
            if port is None:
                print("C3-mini not found – running keyboard only", file=sys.stderr)
            else:
                print(f"C3-mini auto-detected on {port}", file=sys.stderr)
        if port:
            ctrl = ControllerReader(port, args.baud)
            ctrl.start()

    # ── Wait briefly for initial connection ───────────────────────────────────
    rover._connected.wait(timeout=3.0)

    # ── Run TUI ───────────────────────────────────────────────────────────────
    try:
        curses.wrapper(run_tui, rover, ctrl)
    finally:
        # Send stop command before exit
        rover.send(0.0, 0.0, 0.0)
        time.sleep(0.1)
        rover.stop()
        if ctrl:
            ctrl.stop()

    print("Rover disconnected.")

if __name__ == "__main__":
    main()
