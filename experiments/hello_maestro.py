"""Erstes Lebenszeichen vom Pololu Maestro.

Liest die Fehler-Flags aus dem Maestro und gibt sie aus.
Bewegt noch keine Servos.
"""

from __future__ import annotations

import sys
import time

import serial

MAESTRO_PORT = "/dev/maestro_cmd"
BAUDRATE = 9600  # für USB egal, der Maestro ignoriert es im USB-Modus

# Compact-Protocol-Kommandos
CMD_GET_ERRORS = 0xA1


def main() -> int:
    print(f"Öffne {MAESTRO_PORT} ...")
    try:
        ser = serial.Serial(MAESTRO_PORT, BAUDRATE, timeout=1.0)
    except serial.SerialException as e:
        print(f"FEHLER: {e}", file=sys.stderr)
        return 1

    with ser:
        # Kurz warten, manche USB-Stacks brauchen einen Moment
        time.sleep(0.05)

        # "Get Errors" senden
        ser.write(bytes([CMD_GET_ERRORS]))
        ser.flush()

        # Maestro antwortet mit 2 Bytes (Low-Byte, High-Byte)
        raw = ser.read(2)

        if len(raw) != 2:
            print(f"FEHLER: nur {len(raw)} Bytes empfangen, erwartet 2")
            print("Hinweis: Maestro vielleicht im 'UART, fixed baud' Modus?")
            return 1

        error_code = raw[0] | (raw[1] << 8)
        print(f"Maestro hat geantwortet: errors = 0x{error_code:04X} ({error_code})")

        if error_code == 0:
            print("✓ Keine Fehler. Maestro ist gesund.")
        else:
            print("⚠ Fehler-Flags gesetzt. Details siehe Pololu User Guide §6.4")

    return 0


if __name__ == "__main__":
    sys.exit(main())
