"""Schreib-Test: Position setzen und zurücklesen.

ACHTUNG: Das schickt einen echten Puls-Befehl an Kanal 0. Solange die
Servos stromlos sind, bewegt sich nichts. Vor Einschalten des Servo-
Stroms muss klar sein, dass 1500 µs eine sinnvolle Position für den
angeschlossenen Servo ist — bei den meisten ist das die Mitte.
"""

from __future__ import annotations

import sys
import time

import serial

MAESTRO_PORT = "/dev/maestro_cmd"
BAUDRATE = 9600

CMD_SET_TARGET = 0x84  # Compact: 0x84 <channel> <target_low> <target_high>
CMD_GET_POSITION = 0x90

CHANNEL = 0
TARGET_US = 1500  # Mittelstellung
TARGET_QUARTER_US = TARGET_US * 4  # = 6000


def set_target(ser: serial.Serial, channel: int, target_qus: int) -> None:
    """Setzt die Ziel-Position eines Kanals in Quarter-µs."""
    low = target_qus & 0x7F          # untere 7 Bit
    high = (target_qus >> 7) & 0x7F  # nächste 7 Bit
    ser.write(bytes([CMD_SET_TARGET, channel, low, high]))
    ser.flush()


def get_position(ser: serial.Serial, channel: int) -> int:
    ser.write(bytes([CMD_GET_POSITION, channel]))
    ser.flush()
    raw = ser.read(2)
    if len(raw) != 2:
        raise IOError(f"Kanal {channel}: nur {len(raw)} Bytes")
    return raw[0] | (raw[1] << 8)


def main() -> int:
    with serial.Serial(MAESTRO_PORT, BAUDRATE, timeout=1.0) as ser:
        print(f"Kanal {CHANNEL} vor dem Setzen:")
        before = get_position(ser, CHANNEL)
        print(f"  {before} ({before/4:.1f} µs)")

        print(f"Setze Kanal {CHANNEL} auf {TARGET_US} µs ({TARGET_QUARTER_US} qus) ...")
        set_target(ser, CHANNEL, TARGET_QUARTER_US)
        time.sleep(0.05)

        after = get_position(ser, CHANNEL)
        print(f"Kanal {CHANNEL} danach:")
        print(f"  {after} ({after/4:.1f} µs)")

        if after == TARGET_QUARTER_US:
            print("✓ Maestro hat den Befehl angenommen.")
        else:
            print("⚠ Wert weicht ab — eventuell Min/Max-Limits im Maestro?")

        # Auf 0 zurück (keine Pulse mehr senden)
        print("Setze Kanal zurück auf 0 (kein Puls)")
        set_target(ser, CHANNEL, 0)

    return 0


if __name__ == "__main__":
    sys.exit(main())
