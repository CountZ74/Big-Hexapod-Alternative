"""Maestro-Status: aktueller Soll-Wert pro Kanal."""

from __future__ import annotations

import sys

import serial

MAESTRO_PORT = "/dev/maestro_cmd"
BAUDRATE = 9600
NUM_CHANNELS = 24

CMD_GET_POSITION = 0x90  # Compact-Protocol: 0x90 <channel>


def get_position(ser: serial.Serial, channel: int) -> int:
    """Liest die aktuelle Soll-Position eines Kanals (in 0.25-µs-Einheiten)."""
    ser.write(bytes([CMD_GET_POSITION, channel]))
    ser.flush()
    raw = ser.read(2)
    if len(raw) != 2:
        raise IOError(f"Kanal {channel}: nur {len(raw)} Bytes empfangen")
    return raw[0] | (raw[1] << 8)


def main() -> int:
    with serial.Serial(MAESTRO_PORT, BAUDRATE, timeout=1.0) as ser:
        print(f"{'Kanal':>6} {'Quarter-µs':>12} {'µs':>10}")
        print("-" * 32)
        for ch in range(NUM_CHANNELS):
            quarter_us = get_position(ser, ch)
            us = quarter_us / 4.0
            print(f"{ch:>6} {quarter_us:>12} {us:>10.1f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
