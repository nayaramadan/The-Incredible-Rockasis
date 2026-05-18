"""
s3_rover.py  –  ESP32-S3 rover-side co-processor  (v2)
=======================================================
Responsibilities
  • Talk to Orpheus Pico over UART (921600 baud)
    – Forward CMD packets from ESP-NOW to Pico
    – Receive TELEM packets from Pico and relay over ESP-NOW
  • No sonar – the single HC-SR04 is now wired directly to the Pico

Wiring
──────
UART to Pico
  TX → GPIO17    RX → GPIO18    (UART1, 921600 baud)
"""

import struct, time, network
from machine import UART, Pin
import espnow

# ─── Configuration ─────────────────────────────────────────────────────────────
# e0:72:a1:72:a3:54
PEER_MAC  = b'\xe0\x72\xa1\x72\xa3\x54'   # ← replace with C3-mini MAC

UART_TX   = 17
UART_RX   = 18
UART_BAUD = 921600

# ─── Packet type IDs ──────────────────────────────────────────────────────────

CMD_TYPE   = 0x01
TELEM_TYPE = 0x02

# ─── COBS ─────────────────────────────────────────────────────────────────────

def cobs_encode(data: bytes) -> bytes:
    out = bytearray(); code = 1; block = bytearray()
    for byte in data:
        if byte == 0:
            out.append(code); out.extend(block); code = 1; block = bytearray()
        else:
            block.append(byte); code += 1
            if code == 0xFF:
                out.append(code); out.extend(block); code = 1; block = bytearray()
    out.append(code); out.extend(block); out.append(0x00)
    return bytes(out)

def cobs_decode(frame: bytes) -> bytes:
    out = bytearray(); i = 0
    while i < len(frame):
        code = frame[i]; i += 1
        for _ in range(code - 1):
            if i >= len(frame): raise ValueError("COBS truncated")
            out.append(frame[i]); i += 1
        if code < 0xFF and i < len(frame):
            out.append(0x00)
    return bytes(out)

# ─── ESP-NOW setup ────────────────────────────────────────────────────────────

def setup_espnow():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.disconnect()
    print("S3 MAC:", ':'.join(f'{b:02x}' for b in wlan.config('mac')))
    en = espnow.ESPNow()
    en.active(True)
    en.add_peer(PEER_MAC)
    return en

# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    print("S3 rover bridge starting")
    uart = UART(1, baudrate=UART_BAUD,
                tx=Pin(UART_TX), rx=Pin(UART_RX), rxbuf=512)
    en   = setup_espnow()

    pico_rx_buf = bytearray()

    print("S3 rover bridge ready")

    while True:
        # ── UART (Pico → S3): collect TELEM frames, relay over ESP-NOW ───────
        while uart.any():
            byte = uart.read(1)
            if byte == b'\x00':
                if pico_rx_buf:
                    try:
                        raw = cobs_decode(bytes(pico_rx_buf))
                        if len(raw) >= 2 and raw[0] == TELEM_TYPE:
                            # Forward the raw decoded payload to C3-mini
                            en.send(PEER_MAC, bytes(raw))
                    except Exception as e:
                        print("UART RX err:", e)
                    pico_rx_buf = bytearray()
            else:
                pico_rx_buf.extend(byte)

        # ── ESP-NOW (C3-mini → S3): receive CMD, forward to Pico over UART ──
        try:
            host, msg = en.irecv(0)
            if msg and len(msg) >= 10 and msg[0] == CMD_TYPE:
                import binascii
                crc_recv = struct.unpack_from("<I", msg, 6)[0]
                crc_calc = binascii.crc32(bytes(msg[:6])) & 0xFFFFFFFF
                if crc_recv == crc_calc:
                    uart.write(cobs_encode(bytes(msg)))
        except Exception:
            pass

        time.sleep_us(200)


if __name__ == "__main__":
    main()