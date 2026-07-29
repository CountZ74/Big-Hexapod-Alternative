"""Fußsensoren: Hall-Sensor an einer federbelasteten Schubstange.

Mechanik: Unten an der Schubstange sitzt der Fuß, oben ein Magnet. Setzt das
Bein auf, schiebt sich die Stange gegen die Feder nach oben und der Magnet
wandert am Hall-Sensor vorbei. Der Sensor gibt eine analoge Spannung aus, die
der Maestro auf einem als *Input* konfigurierten Kanal als 10-Bit-Wert
(0..1023 für 0..5 V) liefert.

Das ist ein **Wegaufnehmer, kein Taster**. Die interessante Größe ist der
normierte Federweg zwischen den beiden mechanischen Endpunkten: 0.0 = Stange
ausgefahren, keine Last; 1.0 = Stange am Anschlag. Weil die Feder linear
arbeitet, ist dieser Weg ein Maß für die Auflagekraft. Ein Fuß, der im
Schwung gerade den Boden touchiert, liegt weit unten im Bereich; ein Bein,
das im ruhigen Stand ein Sechstel des Roboters trägt, deutlich höher; ein
Bein, das im Gait die Last abfängt, noch höher.

Deshalb liefert dieses Modul absichtlich **kein** "hat Boden"-Flag. Wo eine
Schwelle nötig ist, kennt der Aufrufer den Kontext (Schwungphase, Stand,
Klettern) und setzt sie dort — hier gäbe es nur eine falsche.

Dieses Modul liest ausschließlich — es sendet nie ein Servo-Target. Damit ist
es auf echter Hardware jederzeit gefahrlos benutzbar.

Wichtig für die Inbetriebnahme: Der betreffende Maestro-Kanal muss im Maestro
Control Center einmalig von "Servo" auf "Input" umgestellt und die Einstellung
im Gerät gespeichert werden. Ein Kanal, der noch als Servo konfiguriert ist,
liefert statt des Sensorwerts seine eigene Soll-Pulsweite zurück.
"""

from __future__ import annotations

import logging
import statistics
from collections.abc import Iterable, Mapping
from dataclasses import dataclass

from hexapod.config.model import (
    ANALOG_MAX,
    FootSensorConfig,
    FootSensorsConfig,
)

from .base import AnalogInput

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class FootSensorReading:
    """Ein Messwert eines Fußsensors."""

    leg: str
    channel: int
    raw: float
    """Rohwert 0..1023 (Maestro-Analogeingang)."""
    level: float | None
    """Normierter Federweg: 0.0 = unbelastet, 1.0 = am Anschlag.

    None, solange der Sensor nicht kalibriert ist.
    """

    @property
    def volts(self) -> float:
        """Rohwert als Spannung (nur zur Anzeige/Diagnose)."""
        return self.raw / ANALOG_MAX * 5.0

    @property
    def calibrated(self) -> bool:
        return self.level is not None

    @property
    def percent(self) -> float | None:
        """Federweg in Prozent — dieselbe Zahl, nur lesbarer."""
        return None if self.level is None else self.level * 100.0


class FootSensorArray:
    """Liest alle konfigurierten Fußsensoren über einen Analogeingang-Treiber.

    Zustandslos: Jeder Aufruf misst frisch und wertet nur diese Messung aus.
    Es gibt nichts zu erinnern, weil es keinen geschalteten Zustand gibt —
    nur einen fortlaufenden Messwert.

    Args:
        driver: Alles, was `read_analog(channel)` kann (Maestro, Simulator).
        config: Der `foot_sensors`-Block aus der robot.yaml.
    """

    def __init__(self, driver: AnalogInput, config: FootSensorsConfig) -> None:
        self._driver = driver
        self._config = config
        self._sensors: dict[str, FootSensorConfig] = {
            s.leg: s for s in config.active
        }
        logger.info(
            "FootSensorArray: %d aktive Sensoren (%s)",
            len(self._sensors),
            ", ".join(f"{leg}=ch{s.channel}" for leg, s in self._sensors.items()) or "keine",
        )

    # ---- Bestandsaufnahme ----

    @property
    def legs(self) -> list[str]:
        """Beine mit aktivem Sensor, in Konfigurationsreihenfolge."""
        return list(self._sensors)

    @property
    def any_configured(self) -> bool:
        return bool(self._sensors)

    def has_sensor(self, leg: str) -> bool:
        return leg in self._sensors

    def sensor_config(self, leg: str) -> FootSensorConfig:
        try:
            return self._sensors[leg]
        except KeyError:
            raise KeyError(f"Bein {leg} hat keinen aktiven Fußsensor.") from None

    # ---- Rohwerte ----

    def read_raw(self, leg: str, *, samples: int | None = None) -> float:
        """Rohwert eines Beins, Median über mehrere Einzelmessungen.

        Der Median (nicht der Mittelwert) filtert die gelegentlichen
        Ausreißer des Maestro-ADC weg, ohne die Flanke beim Aufsetzen
        weichzuzeichnen.
        """
        sensor = self.sensor_config(leg)
        n = self._config.samples if samples is None else max(1, samples)
        values = [float(self._driver.read_analog(sensor.channel)) for _ in range(n)]
        return statistics.median(values)

    def read_all_raw(self, *, samples: int | None = None) -> dict[str, float]:
        return {leg: self.read_raw(leg, samples=samples) for leg in self._sensors}

    # ---- Ausgewertete Messwerte ----

    def read(self, leg: str, *, samples: int | None = None) -> FootSensorReading:
        """Ein Bein messen und auf den normierten Federweg umrechnen."""
        sensor = self.sensor_config(leg)
        raw = self.read_raw(leg, samples=samples)
        return self._evaluate(sensor, raw)

    def read_all(self, *, samples: int | None = None) -> dict[str, FootSensorReading]:
        """Alle aktiven Sensoren messen."""
        return {leg: self.read(leg, samples=samples) for leg in self._sensors}

    def levels(self, *, samples: int | None = None) -> dict[str, float]:
        """Nur die Federwege — unkalibrierte Beine fallen raus."""
        return {
            leg: reading.level
            for leg, reading in self.read_all(samples=samples).items()
            if reading.level is not None
        }

    def load_share(self, *, samples: int | None = None) -> dict[str, float]:
        """Lastverteilung: Anteil jedes Beins an der Summe aller Federwege.

        Nützlich im Stand: Sechs gleichmäßig belastete Beine liefern je ~1/6.
        Ein Bein, das deutlich darunter liegt, trägt nicht mit (Boden uneben,
        z_trim daneben). Bewusst relativ — dafür braucht es keine Umrechnung
        in Newton und keine Federkonstante.

        Leerer Dict, wenn kein Bein Last hat (alle in der Luft).
        """
        levels = self.levels(samples=samples)
        total = sum(levels.values())
        if total <= 0.0:
            return {}
        return {leg: value / total for leg, value in levels.items()}

    # ---- Interna ----

    def _evaluate(self, sensor: FootSensorConfig, raw: float) -> FootSensorReading:
        calib = sensor.calibration
        level = None if calib is None else calib.level(raw)
        return FootSensorReading(
            leg=sensor.leg, channel=sensor.channel, raw=raw, level=level
        )


def endpoints_from_samples(
    unloaded: Iterable[float],
    full: Iterable[float],
) -> tuple[float, float]:
    """Rohwert-Messreihen → (raw_unloaded, raw_full) als Median.

    Bewusst getrennt von der Pydantic-Klasse: Das CLI-Tool bildet damit
    die Endpunkte, zeigt sie dem Nutzer und lässt sie erst danach vom
    Modell validieren (Messbereich groß genug?).
    """
    unloaded_values = list(unloaded)
    full_values = list(full)
    if not unloaded_values or not full_values:
        raise ValueError("Beide Messreihen brauchen mindestens einen Wert.")
    return statistics.median(unloaded_values), statistics.median(full_values)


def describe(readings: Mapping[str, FootSensorReading]) -> str:
    """Einzeiler für Logs: 'front_left 62% | mid_left roh=511 (unkalibriert)'."""
    parts: list[str] = []
    for leg, r in readings.items():
        if r.percent is None:
            parts.append(f"{leg} roh={r.raw:.0f} (unkalibriert)")
        else:
            parts.append(f"{leg} {r.percent:.0f}%")
    return " | ".join(parts)


__all__ = [
    "FootSensorArray",
    "FootSensorReading",
    "describe",
    "endpoints_from_samples",
]
