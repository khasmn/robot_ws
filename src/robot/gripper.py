"""
gripper.py
----------
Python controller for the MS42DC Stepper Motor Gripper.

Protocol (reverse-engineered from motocontrol.exe):
  12 bytes: 7A | CMD | DATA[8] | CHECKSUM | 7B
  CHECKSUM = XOR of all bytes from 7A through DATA[7]

Commands:
  0x08  enable/connect
  0x0A  forward (CW)
  0x0B  reverse (CCW)
  0x0C  move to position  (direction + steps encoded in data)
  0x0D  set speed (lower value = faster, e.g. 800)
  0x0E  set speed (higher value = slower, e.g. 1000)
  0x0F  config / multi-param

Usage:
    python gripper.py              # runs open/close demo

    from gripper import Gripper
    g = Gripper('COM5')
    g.open()
    g.close()
    g.stop()
    g.disconnect()
"""

import serial
import time
import struct


def _build_packet(cmd: int, data: bytes) -> bytes:
    """
    Build a 12-byte command packet:
      7A | CMD | data[8] | CHECKSUM | 7B
    data must be exactly 8 bytes. Checksum = XOR of bytes 0-9.
    """
    assert len(data) == 8, "data must be 8 bytes"
    body = bytes([0x7A, cmd]) + data  # bytes 0-9
    chk = 0
    for b in body:
        chk ^= b
    return body + bytes([chk, 0x7B])  # + checksum + end marker


class Gripper:
    """Controls the MS42DC stepper motor gripper over serial."""

    # ── Pre-built command packets (sniffed from motocontrol.exe) ──────────────
    CMD_ENABLE = _build_packet(
        0x08, bytes([0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    )
    CMD_OPEN = _build_packet(
        0x0C, bytes([0x01, 0x1B, 0x58, 0x00, 0x00, 0x00, 0x00, 0x00])
    )
    CMD_CLOSE = _build_packet(
        0x0C, bytes([0x02, 0x1B, 0x58, 0x00, 0x00, 0x00, 0x00, 0x00])
    )
    CMD_STOP = _build_packet(
        0x0C, bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    )
    CMD_FWD = _build_packet(
        0x0A, bytes([0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    )
    CMD_REV = _build_packet(
        0x0B, bytes([0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    )

    def __init__(self, port: str = "COM5", baudrate: int = 115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(0.1)
        self._send(self.CMD_ENABLE)
        print(f"[Gripper] Connected on {port} @ {baudrate} baud")

    def _send(self, packet: bytes):
        self.ser.write(packet)

    def set_speed(self, speed: int):
        """
        Set gripper speed. speed is the step interval in microseconds.
        Lower = faster. Typical values: 500 (fast) – 1000 (slow).
        Sends both 0x0D and 0x0E speed registers.
        """
        speed = max(100, min(9999, speed))
        data = struct.pack(">H", speed) + bytes(6)
        self._send(_build_packet(0x0D, data))
        self._send(_build_packet(0x0E, data))
        print(f"[Gripper] Speed set to {speed}")

    def open(self):
        """Open the gripper."""
        self._send(self.CMD_OPEN)
        print("[Gripper] Open")

    def close(self):
        """Close the gripper."""
        self._send(self.CMD_CLOSE)
        print("[Gripper] Close")

    def stop(self):
        """Stop movement immediately."""
        self._send(self.CMD_STOP)
        print("[Gripper] Stop")

    def forward(self):
        """Run motor forward (CW) continuously."""
        self._send(self.CMD_FWD)
        print("[Gripper] Forward")

    def reverse(self):
        """Run motor reverse (CCW) continuously."""
        self._send(self.CMD_REV)
        print("[Gripper] Reverse")

    def disconnect(self):
        """Close the serial port."""
        self.ser.close()
        print("[Gripper] Disconnected")


# ── Demo ───────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    g = Gripper("/dev/ttyACM0")
    g.set_speed(400)

    for i in range(5):
        print(f"\n--- Cycle {i + 1}/5 ---")
        g.open()
        time.sleep(2)
        g.close()
        time.sleep(2)
    time.sleep(5)
    g.stop()
    g.disconnect()
