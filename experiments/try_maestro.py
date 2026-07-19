"""Smoke-Test des MaestroDrivers mit echter Hardware.

Servos sollten STROMLOS sein — wir senden zwar Befehle, aber nichts
soll sich bewegen.
"""

from __future__ import annotations

import time

from hexapod.drivers.maestro import MaestroDriver


def main() -> None:
    with MaestroDriver("/dev/maestro_cmd", num_channels=24) as driver:
        # 1. Fehler-Status lesen (löscht Fehler dabei)
        errors = driver.get_errors()
        print(f"Errors: 0x{errors:04X}")

        # 2. Eine Position setzen und zurücklesen
        driver.set_position(0, 1500.0)
        time.sleep(0.05)
        pos = driver.get_position(0)
        print(f"ch=0 set 1500.0 -> read {pos}")

        # 3. Mehrere Positionen auf einmal
        driver.set_positions({0: 1400.0, 1: 1500.0, 2: 1600.0})
        time.sleep(0.05)
        for ch in [0, 1, 2]:
            print(f"ch={ch}: {driver.get_position(ch)}")

        # 4. Wieder deaktivieren
        for ch in [0, 1, 2]:
            driver.disable(ch)
        time.sleep(0.05)
        for ch in [0, 1, 2]:
            print(f"ch={ch} nach disable: {driver.get_position(ch)}")


if __name__ == "__main__":
    main()
