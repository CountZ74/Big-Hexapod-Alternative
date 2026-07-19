"""ADS7830 8-Kanal-I2C-ADC -- Batteriespannungs-Ueberwachung.

Liest die Akkuspannungen ueber den ADS7830 (Freenove-Platine, I2C 0x48).
Umrechnung wie im Freenove-Code: U = wert/255 * 5 * Koeffizient (Teiler=3).

Robust: fehlender I2C-Bus oder Lesefehler (OSError/Remote-I/O) liefern None
statt zu werfen -- die Telemetrie laeuft dann einfach ohne Batteriewerte weiter.
"""
from __future__ import annotations

import logging
import os
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from smbus2 import SMBus

logger = logging.getLogger(__name__)

ADS7830_ADDR = 0x48
ADS7830_CMD = 0x84
DEFAULT_COEFFICIENT = 3.0   # Spannungsteiler-Faktor der Platine

# Kanalzuordnung -- bitte am Roboter verifizieren (z.B. einen Pack abklemmen
# und schauen, welcher Wert faellt). Per ENV ueberschreibbar.
CH_PI = int(os.environ.get("HEXAPOD_ADC_CH_PI", "4"))
CH_SERVO = int(os.environ.get("HEXAPOD_ADC_CH_SERVO", "0"))

# Schwellen pro 2S-Li-Ion-Pack (Volt).
WARN_V = float(os.environ.get("HEXAPOD_BATT_WARN", "6.6"))
CRIT_V = float(os.environ.get("HEXAPOD_BATT_CRIT", "6.2"))
PRESENT_V = float(os.environ.get("HEXAPOD_BATT_PRESENT", "3.0"))  # darunter = nicht angeschlossen


def classify(voltage: float | None) -> str:
    """Bewertet eine Pack-Spannung: ok | warn | critical | absent."""
    if voltage is None or voltage < PRESENT_V:
        return "absent"
    if voltage < CRIT_V:
        return "critical"
    if voltage < WARN_V:
        return "warn"
    return "ok"


class ADS7830:
    """Minimaler ADS7830-Treiber (single-ended Kanaele)."""

    def __init__(self, bus: int = 1, address: int = ADS7830_ADDR,
                 coefficient: float = DEFAULT_COEFFICIENT) -> None:
        self._busno = bus
        self._addr = address
        self._coeff = coefficient
        self._bus: SMBus | None = None
        self._failed = False

    def _ensure_bus(self) -> SMBus | None:
        if self._bus is None and not self._failed:
            try:
                import smbus2
                self._bus = smbus2.SMBus(self._busno)
            except Exception as e:  # smbus2 fehlt oder Bus nicht da
                logger.warning("ADC: I2C nicht verfuegbar (%s) -- keine Batteriewerte", e)
                self._failed = True
        return self._bus

    def read_channel_voltage(self, channel: int) -> float | None:
        bus = self._ensure_bus()
        if bus is None:
            return None
        try:
            cmd = ADS7830_CMD | ((((channel << 2) | (channel >> 1)) & 0x07) << 4)
            bus.write_byte(self._addr, cmd)
            v1 = bus.read_byte(self._addr)
            v2 = bus.read_byte(self._addr)
            val = v1 if v1 == v2 else bus.read_byte(self._addr)
            return round(val / 255.0 * 5.0 * self._coeff, 2)
        except OSError as e:
            logger.debug("ADC-Lesefehler ch%d: %s", channel, e)
            return None

    def read_batteries(self) -> dict[str, float | None]:
        """Liefert {"pi": U, "servo": U} (None bei Lesefehler)."""
        return {
            "pi": self.read_channel_voltage(CH_PI),
            "servo": self.read_channel_voltage(CH_SERVO),
        }

    def close(self) -> None:
        if self._bus is not None:
            import contextlib
            with contextlib.suppress(Exception):
                self._bus.close()
            self._bus = None


# Schutzaktionen (sicher per Default = nur warnen; per ENV aktivierbar):
#   HEXAPOD_SERVO_CRIT_ACTION = warn | lie_down | disable
#   HEXAPOD_PI_CRIT_ACTION    = warn | shutdown
SERVO_CRIT_ACTION = os.environ.get("HEXAPOD_SERVO_CRIT_ACTION", "warn")
PI_CRIT_ACTION = os.environ.get("HEXAPOD_PI_CRIT_ACTION", "warn")


class BatteryMonitor:
    """Bewertet Pack-Spannungen ueber die Zeit (Entprellung kritischer Lage).

    update() liefert pro Pack: Spannung, Zustand (ok/warn/critical/absent),
    ob sich der Zustand geaendert hat (changed) und ob ``critical`` lange genug
    anliegt (confirmed_critical) -- damit ein einzelner Aussetzer keine
    Schutzaktion ausloest. ``acted`` verhindert wiederholtes Ausloesen.
    """

    def __init__(self, crit_confirm: int = 3) -> None:
        self._crit_confirm = max(1, crit_confirm)
        self._count: dict[str, int] = {}
        self._last: dict[str, str | None] = {}
        self._acted: dict[str, bool] = {}

    def update(self, voltages: dict[str, float | None]) -> dict[str, dict[str, Any]]:
        out: dict[str, dict[str, Any]] = {}
        for pack, v in voltages.items():
            st = classify(v)
            self._count[pack] = self._count.get(pack, 0) + 1 if st == "critical" else 0
            confirmed = self._count[pack] >= self._crit_confirm
            changed = st != self._last.get(pack)
            self._last[pack] = st
            # acted nur zuruecksetzen, wenn der Pack die kritische Lage verlaesst
            if st != "critical":
                self._acted[pack] = False
            fire = confirmed and not self._acted.get(pack, False)
            if fire:
                self._acted[pack] = True
            out[pack] = {
                "voltage": v, "state": st,
                "changed": changed, "confirmed_critical": confirmed,
                "fire_action": fire,
            }
        return out
