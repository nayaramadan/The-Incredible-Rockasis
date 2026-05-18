"""
c3_controller.py  –  ESP32-C3-mini PC-side controller
======================================================
Responsibilities
  • Read throttle joystick (GPIO1 – yPinThrot)
  • Read steering joystick (GPIO4 – yPinDir)
  • Momentary claw toggle button (GPIO6, active-low)
  • Send CMD packets to rover S3 over ESP-NOW at 50 Hz
  • Receive TELEM frames from rover S3 over ESP-NOW
  • Bridge all data over USB serial (115200) to PC as JSON lines

Wiring
──────
Throttle joystick Y  → GPIO1   (yPinThrot)
Steering joystick Y  → GPIO4   (yPinDir)
Claw toggle button   → GPIO6   (active-low, internal pull-up – no resistor needed)

On first boot the C3-mini prints its own MAC so you can paste it
into PEER_MAC in s3_rover.py.

Flash as main.py.
"""

import struct, time, json, sys, network
from machine import ADC, Pin, UART
import espnow

# ─── Configuration ────────────────────────────────────────────────────────────
# 10:51:db:82:60:54
# MAC of the rover-side ESP32-S3.
PEER_MAC = b'\x10\x51\xdb\x82\x60\x54'   # ← replace with real MAC

PIN_THROTTLE    = 1   # yPinThrot
PIN_STEERING    = 4   # yPinDir
PIN_CLAW_TOGGLE = 6   # momentary button

# Joystick calibration (raw ADC 0–4095)
JOY_CENTER = 2048
JOY_DEAD   = 80     # deadzone radius
JOY_MAX    = 1900   # usable range from centre

CMD_INTERVAL_MS = 20   # 50 Hz

# ─── Packet type IDs ──────────────────────────────────────────────────────────

CMD_TYPE   = 0x01
TELEM_TYPE = 0x02

# ─── CRC ──────────────────────────────────────────────────────────────────────

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF

# ─── CMD packet builder ───────────────────────────────────────────────────────
#
#   [0]    type      0x01
#   [1]    throttle  int8  (-100 … +100)
#   [2]    steering  int8  (-100 … +100)
#   [3]    claw      uint8 (0 … 100)
#   [4]    servo2    uint8 (0 … 100)
#   [5]    servo3    uint8 (0 … 100)
#   [6:10] CRC-32 LE

def build_cmd(throttle: float, steering: float, claw: float,
              servo2: float = 0.5, servo3: float = 0.5) -> bytes:
    import binascii
    t  = max(-100, min(100, int(throttle * 100))) & 0xFF
    s  = max(-100, min(100, int(steering * 100))) & 0xFF
    c  = max(0, min(100, int(claw   * 100)))
    s2 = max(0, min(100, int(servo2 * 100)))
    s3 = max(0, min(100, int(servo3 * 100)))
    payload = bytes([CMD_TYPE, t, s, c, s2, s3])
    crc = binascii.crc32(payload) & 0xFFFFFFFF
    return payload + struct.pack("<I", crc)

# ─── TELEM parser ─────────────────────────────────────────────────────────────

def parse_telem(data: bytes) -> dict | None:
    # Layout: B + 3×f + B + H + B + H(crc) = 19 bytes
    if len(data) < 19 or data[0] != TELEM_TYPE:
        return None
    if crc16(data[:17]) != struct.unpack_from("<H", data, 17)[0]:
        return None
    _, roll, pitch, yaw, flags = struct.unpack_from("<BfffB", data)
    sonar_v  = struct.unpack_from("<H", data, 14)[0]
    pan_byte = data[16]
    return {
        "roll":      round(roll,  2),
        "pitch":     round(pitch, 2),
        "yaw":       round(yaw,   2),
        "motors":    bool(flags & 0x01),
        "claw":      bool(flags & 0x02),
        "sonar":     [-1 if sonar_v == 0xFFFF else sonar_v],
        "sonar_pan": round(pan_byte / 100.0, 2),
    }

# ─── Joystick ─────────────────────────────────────────────────────────────────

class Joystick:
    def __init__(self, pin):
        self.adc = ADC(Pin(pin), atten=ADC.ATTN_11DB)

    def read(self) -> float:
        """Returns -1.0 … +1.0 with deadzone applied."""
        raw    = self.adc.read()
        offset = raw - JOY_CENTER
        if abs(offset) < JOY_DEAD:
            return 0.0
        sign = 1 if offset > 0 else -1
        mag  = (abs(offset) - JOY_DEAD) / (JOY_MAX - JOY_DEAD)
        return max(-1.0, min(1.0, sign * mag))

# ─── Button (debounced active-low) ────────────────────────────────────────────

class Button:
    DEBOUNCE_MS = 30

    def __init__(self, pin):
        self.pin   = Pin(pin, Pin.IN, Pin.PULL_UP)
        self._last = True
        self._t    = 0

    def pressed(self) -> bool:
        """Returns True once per physical press."""
        now = time.ticks_ms()
        val = self.pin.value()
        if val == False and self._last == True:
            if time.ticks_diff(now, self._t) > self.DEBOUNCE_MS:
                self._last = val
                self._t    = now
                return True
        self._last = val
        return False

# ─── Serial helpers ───────────────────────────────────────────────────────────

def serial_write(obj: dict):
    sys.stdout.write(json.dumps(obj) + '\n')

# ─── ESP-NOW setup ────────────────────────────────────────────────────────────

def setup_espnow():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.disconnect()
    print("C3-mini MAC:", ':'.join(f'{b:02x}' for b in wlan.config('mac')))
    en = espnow.ESPNow()
    en.active(True)
    en.add_peer(PEER_MAC)
    return en

# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    en = setup_espnow()

    throttle_joy = Joystick(PIN_THROTTLE)
    steering_joy = Joystick(PIN_STEERING)
    btn_claw     = Button(PIN_CLAW_TOGGLE)

    # UART0 for non-blocking PC serial RX
    uart0 = UART(0, baudrate=115200, tx=Pin(21), rx=Pin(20), rxbuf=256)

    claw_pos        = 0.0    # 0.0 = open, 1.0 = closed
    claw_btn_locked = False  # once button pressed, PC serial cannot change claw
    last_cmd        = time.ticks_ms()
    pc_rx_buf       = bytearray()

    print("C3 controller ready")

    while True:
        now = time.ticks_ms()

        # ── Claw button ───────────────────────────────────────────────────────
        if btn_claw.pressed():
            claw_pos        = 0.0 if claw_pos > 0.5 else 1.0
            claw_btn_locked = True
            # Fire immediately – don't wait for the next scheduled tick
            try:
                en.send(PEER_MAC, build_cmd(
                    throttle_joy.read(), steering_joy.read(), claw_pos))
            except Exception as e:
                serial_write({"error": str(e)})
            serial_write({"event": "claw_toggle",
                          "claw": round(claw_pos, 2)})

        # ── Scheduled CMD send ────────────────────────────────────────────────
        if time.ticks_diff(now, last_cmd) >= CMD_INTERVAL_MS:
            t = throttle_joy.read()
            s = steering_joy.read()
            try:
                en.send(PEER_MAC, build_cmd(t, s, claw_pos))
            except Exception as e:
                serial_write({"error": str(e)})
            serial_write({"joy": {"t": round(t, 3),
                                  "s": round(s, 3),
                                  "claw": round(claw_pos, 2),
                                  "claw_locked": claw_btn_locked}})
            last_cmd = now

        # ── Receive telem from rover S3 ───────────────────────────────────────
        try:
            host, msg = en.irecv(0)
            if msg:
                parsed = parse_telem(bytes(msg))
                if parsed:
                    serial_write({"telem": parsed})
        except Exception:
            pass

        # ── Receive commands from PC over serial ──────────────────────────────
        while uart0.any():
            b = uart0.read(1)
            if b in (b'\n', b'\r'):
                if pc_rx_buf:
                    try:
                        obj = json.loads(pc_rx_buf.decode())
                        if "throttle" in obj or "steering" in obj:
                            t  = float(obj.get("throttle", 0.0))
                            s  = float(obj.get("steering", 0.0))
                            # PC claw value ignored if button has taken ownership
                            c  = claw_pos if claw_btn_locked \
                                 else float(obj.get("claw", claw_pos))
                            en.send(PEER_MAC, build_cmd(t, s, c))
                    except Exception as e:
                        serial_write({"parse_error": str(e)})
                pc_rx_buf = bytearray()
            else:
                pc_rx_buf.extend(b)

        time.sleep_ms(1)


if __name__ == "__main__":
    main()