"""
pico_main.py  –  Orpheus Pico (RP2040) rover firmware
======================================================
Responsibilities
  • Read MPU-6050 over I²C → complementary-filter roll/pitch/yaw
  • Drive 4× DC motors via TB6612FNG (2 ICs, 4 channels)
  • Control servo claw via hardware PWM
  • Receive COBS-framed command packets from ESP32-S3 over UART1
  • Send COBS-framed telemetry packets back to ESP32-S3

Wiring (edit PIN constants below to match your board)
─────────────────────────────────────────────────────
I²C (MPU-6050)
  SDA  → GP4      SCL → GP5      (I2C0)

UART (ESP32-S3)
  TX   → GP8      RX  → GP9      (UART1)  921600 baud

Single TB6612FNG motor driver
  STBY → GP16
  PWMA → GP10     AIN1 → GP11    AIN2 → GP12   (left motors – both wired to channel A)
  PWMB → GP13     BIN1 → GP14    BIN2 → GP15   (right motors – both wired to channel B)

Servo 1 (claw open/close)
  PWM  → GP28

Servo 2 (claw tilt / wrist)
  PWM  → GP27

Servo 3 (sonar yaw pan)
  PWM  → GP26

HC-SR04 (single sonar, direct to Pico)
  TRIG → GP19    ECHO → GP20
  ECHO needs a 1 kΩ/2 kΩ divider to 3.3 V if using a 5 V sensor board
"""

import struct
import time
import math
from machine import I2C, UART, Pin, PWM

# ─── Pin constants ────────────────────────────────────────────────────────────

I2C_SDA   = 4
I2C_SCL   = 5

UART_TX   = 8
UART_RX   = 9
UART_BAUD = 921600

# Single TB6612FNG
# Channel A → both left motors  (front-left + rear-left wired in parallel)
# Channel B → both right motors (front-right + rear-right wired in parallel)
M_STBY = 16
M_PWMA = 10
M_AIN1 = 11
M_AIN2 = 12
M_PWMB = 13
M_BIN1 = 14
M_BIN2 = 15

SERVO_PIN  = 28   # claw open/close
SERVO2_PIN = 27   # claw tilt / wrist
SERVO3_PIN = 26   # sonar yaw pan

# HC-SR04 (single, connected directly to Pico)
SONAR_TRIG = 19
SONAR_ECHO = 20
SONAR_TIMEOUT_US = 30_000   # 30 ms → ~5 m max range

# ─── MPU-6050 ─────────────────────────────────────────────────────────────────

MPU_ADDR  = 0x68
PWR_MGMT1 = 0x6B
ACCEL_OUT = 0x3B   # 6 bytes: AX_H AX_L AY_H AY_L AZ_H AZ_L
GYRO_OUT  = 0x43   # 6 bytes: GX_H GX_L GY_H GY_L GZ_H GZ_L
GYRO_CFG  = 0x1B   # ±250 °/s  → sensitivity 131 LSB/(°/s)
ACCEL_CFG = 0x1C   # ±2 g      → sensitivity 16384 LSB/g

GYRO_SENS  = 131.0
ACCEL_SENS = 16384.0

class MPU6050:
    def __init__(self, i2c):
        self.i2c = i2c
        # Wake the chip
        self.i2c.writeto_mem(MPU_ADDR, PWR_MGMT1, bytes([0x00]))
        time.sleep_ms(100)
        # Set gyro ±250 °/s, accel ±2 g
        self.i2c.writeto_mem(MPU_ADDR, GYRO_CFG,  bytes([0x00]))
        self.i2c.writeto_mem(MPU_ADDR, ACCEL_CFG, bytes([0x00]))

    def _read_raw(self, reg):
        raw = self.i2c.readfrom_mem(MPU_ADDR, reg, 6)
        vals = struct.unpack(">3h", raw)   # three signed 16-bit big-endian
        return vals

    def read_accel_g(self):
        ax, ay, az = self._read_raw(ACCEL_OUT)
        return ax / ACCEL_SENS, ay / ACCEL_SENS, az / ACCEL_SENS

    def read_gyro_dps(self):
        gx, gy, gz = self._read_raw(GYRO_OUT)
        return gx / GYRO_SENS, gy / GYRO_SENS, gz / GYRO_SENS


class ComplementaryFilter:
    """Simple roll/pitch/yaw complementary filter.
    Yaw integrates gyro only (no magnetometer), so it drifts slowly.
    """
    ALPHA = 0.96   # weight for gyro vs accel

    def __init__(self):
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0
        self._last_us = time.ticks_us()

    def update(self, ax, ay, az, gx, gy, gz):
        now = time.ticks_us()
        dt  = time.ticks_diff(now, self._last_us) * 1e-6
        self._last_us = now

        # Accel-derived angles
        roll_acc  = math.atan2(ay, az) * 57.2958
        pitch_acc = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * 57.2958

        # Complementary blend
        self.roll  = self.ALPHA * (self.roll  + gx * dt) + (1 - self.ALPHA) * roll_acc
        self.pitch = self.ALPHA * (self.pitch + gy * dt) + (1 - self.ALPHA) * pitch_acc
        self.yaw  += gz * dt   # gyro-only; reset to 0 on boot

        return self.roll, self.pitch, self.yaw


# ─── Motor driver ─────────────────────────────────────────────────────────────

PWM_FREQ = 20_000   # 20 kHz – above audible range

class MotorChannel:
    """One half-bridge channel on a TB6612FNG."""
    def __init__(self, pwm_pin, in1_pin, in2_pin):
        self.pwm = PWM(Pin(pwm_pin), freq=PWM_FREQ, duty_u16=0)
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)

    def set(self, speed: float):
        """speed: -1.0 … +1.0"""
        speed = max(-1.0, min(1.0, speed))
        duty  = int(abs(speed) * 65535)
        self.pwm.duty_u16(duty)
        if speed > 0:
            self.in1.value(1); self.in2.value(0)
        elif speed < 0:
            self.in1.value(0); self.in2.value(1)
        else:
            self.in1.value(0); self.in2.value(0)   # coast

    def brake(self):
        self.pwm.duty_u16(65535)
        self.in1.value(1); self.in2.value(1)


class RoverMotors:
    """
    Single TB6612FNG, two channels:
      Channel A → left side  (front-left + rear-left motors in parallel)
      Channel B → right side (front-right + rear-right motors in parallel)
    """
    def __init__(self):
        self.stby  = Pin(M_STBY, Pin.OUT, value=0)
        self.left  = MotorChannel(M_PWMA, M_AIN1, M_AIN2)
        self.right = MotorChannel(M_PWMB, M_BIN1, M_BIN2)
        self.enable()

    def enable(self):
        self.stby.value(1)

    def disable(self):
        self.stby.value(0)

    def drive(self, throttle: float, steering: float):
        """
        throttle: -1.0 (full reverse) … +1.0 (full forward)
        steering: -1.0 (full left)    … +1.0 (full right)
        Uses differential/tank mixing.
        """
        left  = max(-1.0, min(1.0, throttle - steering))
        right = max(-1.0, min(1.0, throttle + steering))
        self.left.set(left)
        self.right.set(right)

    def stop(self):
        self.left.set(0.0)
        self.right.set(0.0)


# ─── Servo ────────────────────────────────────────────────────────────────────

SERVO_FREQ      = 50          # 50 Hz = 20 ms period
SERVO_MIN_US    = 1000        # 1 ms  → fully open
SERVO_MAX_US    = 2000        # 2 ms  → fully closed
SERVO_CENTER_US = 1500

class ServoClaw:
    def __init__(self, pin):
        self.pwm = PWM(Pin(pin), freq=SERVO_FREQ)
        self.set(0.0)          # start centred / open

    def _us_to_duty(self, us):
        # RP2040 PWM period = 1/50 = 20 000 µs → 65535 counts
        return int(us / 20_000 * 65535)

    def set(self, pos: float):
        """pos: 0.0 = open, 1.0 = closed"""
        pos = max(0.0, min(1.0, pos))
        us  = int(SERVO_MIN_US + pos * (SERVO_MAX_US - SERVO_MIN_US))
        self.pwm.duty_u16(self._us_to_duty(us))

    def open(self):  self.set(0.0)
    def close(self): self.set(1.0)


# ─── HC-SR04 sonar ────────────────────────────────────────────────────────────

from machine import time_pulse_us

class HCSR04:
    """Single HC-SR04 ultrasonic sensor."""
    def __init__(self, trig_pin, echo_pin):
        self.trig = Pin(trig_pin, Pin.OUT, value=0)
        self.echo = Pin(echo_pin, Pin.IN)

    def distance_mm(self) -> int:
        """Returns distance in mm, or -1 on timeout / no echo."""
        self.trig.value(0)
        time.sleep_us(2)
        self.trig.value(1)
        time.sleep_us(10)
        self.trig.value(0)
        duration = time_pulse_us(self.echo, 1, SONAR_TIMEOUT_US)
        if duration < 0:
            return -1
        return int(duration * 0.1715)   # mm = µs × (343 000 mm/s / 2 / 1 000 000)


class SonarPan:
    """
    Combines the HC-SR04 with servo3 to sweep the sonar.
    Call update() every loop tick; it steps the servo and fires
    a ping at each position, building a simple scan buffer.
    """
    POSITIONS  = [0.0, 0.25, 0.5, 0.75, 1.0]   # 5 pan angles
    DWELL_MS   = 60    # ms to wait after moving before pinging

    def __init__(self, sonar: HCSR04, pan_servo: 'ServoClaw'):
        self.sonar     = sonar
        self.servo     = pan_servo
        self._idx      = 0
        self._moved_at = 0
        self._readings = [-1] * len(self.POSITIONS)
        # Move to first position immediately
        self.servo.set(self.POSITIONS[0])
        self._moved_at = time.ticks_ms()

    @property
    def readings(self) -> list:
        """Latest distance (mm) for each pan position. -1 = no echo."""
        return list(self._readings)

    @property
    def current_pos(self) -> float:
        """Current servo pan position 0.0–1.0."""
        return self.POSITIONS[self._idx]

    def update(self):
        """Call every loop iteration. Non-blocking step machine."""
        now = time.ticks_ms()
        if time.ticks_diff(now, self._moved_at) < self.DWELL_MS:
            return   # still settling
        # Ping at current position
        self._readings[self._idx] = self.sonar.distance_mm()
        # Advance to next position
        self._idx = (self._idx + 1) % len(self.POSITIONS)
        self.servo.set(self.POSITIONS[self._idx])
        self._moved_at = now


# ─── COBS framing ─────────────────────────────────────────────────────────────
# Consistent Overhead Byte Stuffing – zero-free framing over serial.
# Packet boundary = 0x00 byte.

def cobs_encode(data: bytes) -> bytes:
    out    = bytearray()
    code   = 1
    block  = bytearray()
    for byte in data:
        if byte == 0:
            out.append(code)
            out.extend(block)
            code  = 1
            block = bytearray()
        else:
            block.append(byte)
            code += 1
            if code == 0xFF:
                out.append(code)
                out.extend(block)
                code  = 1
                block = bytearray()
    out.append(code)
    out.extend(block)
    out.append(0x00)   # frame delimiter
    return bytes(out)


def cobs_decode(frame: bytes) -> bytes:
    """frame must NOT include the trailing 0x00 delimiter."""
    out = bytearray()
    i   = 0
    while i < len(frame):
        code = frame[i]
        i   += 1
        for _ in range(code - 1):
            if i >= len(frame):
                raise ValueError("COBS: truncated")
            out.append(frame[i])
            i += 1
        if code < 0xFF and i < len(frame):
            out.append(0x00)
    return bytes(out)


# ─── Packet formats  (little-endian structs) ──────────────────────────────────
#
# CMD packet  (ESP32-S3 → Pico)   10 bytes
#   [0]    packet type  0x01
#   [1]    throttle     int8  (-100 … +100, scale /100 → float)
#   [2]    steering     int8  (-100 … +100)
#   [3]    claw         uint8 (0 … 100)
#   [4]    servo2       uint8 (0 … 100)
#   [5]    servo3       uint8 (0 … 100)  sonar yaw pan
#   [6:10] CRC-32       uint32 LE (over bytes 0-5)
#
# TELEM packet (Pico → ESP32-S3)  20 bytes
#   [0]     packet type  0x02
#   [1:5]   roll         float32 LE
#   [5:9]   pitch        float32 LE
#   [9:13]  yaw          float32 LE
#   [13]    flags        uint8  (bit0=motors_on, bit1=claw_closed)
#   [14:16] sonar_mm     uint16 LE  (0xFFFF = no echo)
#   [16]    sonar_pan    uint8  (0-100, current servo3 position ×100)
#   [17:19] CRC-16       uint16 LE (over bytes 0-16)

CMD_TYPE   = 0x01
TELEM_TYPE = 0x02

def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

def build_telem(roll, pitch, yaw, motors_on, claw_closed,
                sonar_mm: int, sonar_pan: float) -> bytes:
    flags    = (1 if motors_on else 0) | (2 if claw_closed else 0)
    sonar_v  = 0xFFFF if sonar_mm < 0 else min(0xFFFE, sonar_mm)
    pan_byte = max(0, min(100, int(sonar_pan * 100)))
    payload  = (struct.pack("<BfffB", TELEM_TYPE, roll, pitch, yaw, flags)
                + struct.pack("<HB", sonar_v, pan_byte))
    crc = _crc16(payload)
    return cobs_encode(payload + struct.pack("<H", crc))

def parse_cmd(raw: bytes):
    """Returns (throttle, steering, claw, servo2, servo3) or None."""
    if len(raw) < 10 or raw[0] != CMD_TYPE:
        return None
    import binascii
    crc_recv = struct.unpack("<I", raw[6:10])[0]
    crc_calc = binascii.crc32(raw[:6]) & 0xFFFFFFFF
    if crc_recv != crc_calc:
        return None
    throttle   = raw[1] / 100.0 if raw[1] < 128 else (raw[1] - 256) / 100.0
    steering   = raw[2] / 100.0 if raw[2] < 128 else (raw[2] - 256) / 100.0
    claw_pos   = raw[3] / 100.0
    servo2_pos = raw[4] / 100.0
    servo3_pos = raw[5] / 100.0
    return throttle, steering, claw_pos, servo2_pos, servo3_pos


# ─── Main loop ────────────────────────────────────────────────────────────────

def main():
    # Init hardware
    i2c   = I2C(0, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=400_000)
    uart  = UART(1, baudrate=UART_BAUD, tx=Pin(UART_TX), rx=Pin(UART_RX),
                 rxbuf=256)

    imu    = MPU6050(i2c)
    filt   = ComplementaryFilter()
    motors = RoverMotors()
    claw   = ServoClaw(SERVO_PIN)
    servo2 = ServoClaw(SERVO2_PIN)
    servo3 = ServoClaw(SERVO3_PIN)   # sonar yaw pan

    sonar    = HCSR04(SONAR_TRIG, SONAR_ECHO)
    sonar_pan = SonarPan(sonar, servo3)

    rx_buf       = bytearray()
    telem_period = 20          # ms between telemetry packets (~50 Hz)
    last_telem   = time.ticks_ms()
    last_cmd_ms  = time.ticks_ms()
    CMD_TIMEOUT  = 500         # ms – stop motors if no command received

    claw_closed = False
    motors_on   = True

    print("Rover Pico firmware ready")

    while True:
        # ── Read UART (non-blocking) ──────────────────────────────────────────
        while uart.any():
            byte = uart.read(1)
            if byte == b'\x00':
                if rx_buf:
                    try:
                        raw    = cobs_decode(bytes(rx_buf))
                        result = parse_cmd(raw)
                        if result:
                            throttle, steering, claw_pos, servo2_pos, servo3_pos = result
                            motors.drive(throttle, steering)
                            claw.set(claw_pos)
                            servo2.set(servo2_pos)
                            # servo3 is driven by SonarPan automatically;
                            # only override if the PC explicitly commands a
                            # fixed pan position (servo3_pos >= 0 means override)
                            # Leave pan sweeping if servo3_pos == 0.5 (centre = no-op)
                            claw_closed = claw_pos > 0.5
                            last_cmd_ms = time.ticks_ms()
                    except Exception as e:
                        print("RX error:", e)
                    rx_buf = bytearray()
            else:
                rx_buf.extend(byte)

        # ── Command watchdog ──────────────────────────────────────────────────
        if time.ticks_diff(time.ticks_ms(), last_cmd_ms) > CMD_TIMEOUT:
            motors.stop()
            motors_on = False
        else:
            motors_on = True

        # ── Sonar pan sweep ───────────────────────────────────────────────────
        sonar_pan.update()

        # ── Read IMU + filter ─────────────────────────────────────────────────
        ax, ay, az = imu.read_accel_g()
        gx, gy, gz = imu.read_gyro_dps()
        roll, pitch, yaw = filt.update(ax, ay, az, gx, gy, gz)

        # ── Send telemetry ────────────────────────────────────────────────────
        now = time.ticks_ms()
        if time.ticks_diff(now, last_telem) >= telem_period:
            # Report the reading at the current pan position
            readings = sonar_pan.readings
            idx      = sonar_pan._idx
            dist_mm  = readings[idx] if readings else -1
            pkt = build_telem(roll, pitch, yaw, motors_on, claw_closed,
                              dist_mm, sonar_pan.current_pos)
            uart.write(pkt)
            last_telem = now

        time.sleep_us(500)


if __name__ == "__main__":
    main()