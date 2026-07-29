"""Fußsensoren: Hall-Sensor an einer federbelasteten Schubstange.

Mechanik: Unten an der Schubstange sitzt der Fuß, oben ein Magnet. Setzt das
Bein auf, schiebt sich die Stange gegen die Feder nach oben und der Magnet
wandert am Hall-Sensor vorbei. Der Sensor gibt eine analoge Spannung aus, die
der Maestro auf einem als *Input* konfigurierten Kanal als 10-Bit-Wert
(0..1023 für 0..5 V) liefert.

Warum das mehr ist als ein Taster: Der Hall-Sensor misst nicht nur "Kontakt
ja/nein", sondern *wie weit* die Feder eingedrückt ist — also näherungsweise
die Auflagekraft. Der normierte Pegel (0 = frei, 1 = voll eingefedert) ist
deshalb das eigentlich interessante Signal; der Kontakt-Bool ist nur die
geschwellte Variante davon.

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
    """Normiert: 0.0 = frei, 1.0 = voll eingefedert. None = unkalibriert."""
    contact: bool | None
    """Bodenkontakt mit Hysterese. None = unkalibriert."""

    @property
    def volts(self) -> float:
        """Rohwert als Spannung (nur zur Anzeige/Diagnose)."""
        return self.raw / ANALOG_MAX * 5.0

    @property
    def calibrated(self) -> bool:
        return self.level is not None


class FootSensorArray:
    """Liest alle konfigurierten Fußsensoren über einen Analogeingang-Treiber.

    Der Kontaktzustand wird pro Bein gehalten, weil die Hysterese vom
    vorherigen Zustand abhängt: Einschalten bei `threshold`, Ausschalten
    erst unter `threshold - hysteresis`. Ohne diese Erinnerung würde ein
    Fuß, der genau auf der Schwelle steht, im Takt der Abtastung flattern.

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
        # Startannahme: kein Bein hat Kontakt. Der erste Messwert korrigiert
        # das sofort, weil die Einschaltschwelle zustandsunabhängig ist.
        self._contact: dict[str, bool] = dict.fromkeys(self._sensors, False)
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
        """Ein Bein messen und auswerten (aktualisiert den Hysterese-Zustand)."""
        sensor = self.sensor_config(leg)
        raw = self.read_raw(leg, samples=samples)
        return self._evaluate(sensor, raw)

    def read_all(self, *, samples: int | None = None) -> dict[str, FootSensorReading]:
        """Alle aktiven Sensoren messen."""
        return {
            leg: self.read(leg, samples=samples)
            for leg in self._sensors
        }

    def contacts(self, *, samples: int | None = None) -> dict[str, bool]:
        """Nur die Kontaktzustände — unkalibrierte Beine fallen raus."""
        return {
            leg: reading.contact
            for leg, reading in self.read_all(samples=samples).items()
            if reading.contact is not None
        }

    def reset_state(self, legs: Iterable[str] | None = None) -> None:
        """Hysterese-Zustand zurücksetzen (z.B. vor einer neuen Messreihe)."""
        for leg in self._sensors if legs is None else legs:
            if leg in self._contact:
                self._contact[leg] = False

    # ---- Interna ----

    def _evaluate(self, sensor: FootSensorConfig, raw: float) -> FootSensorReading:
        calib = sensor.calibration
        if calib is None:
            return FootSensorReading(
                leg=sensor.leg, channel=sensor.channel, raw=raw, level=None, contact=None
            )

        level = calib.level(raw)
        was_in_contact = self._contact.get(sensor.leg, False)
        if was_in_contact:
            # Halten, bis der Pegel klar unter die Schwelle fällt.
            now = level >= (calib.threshold - calib.hysteresis)
        else:
            now = level >= calib.threshold
        self._contact[sensor.leg] = now

        return FootSensorReading(
            leg=sensor.leg, channel=sensor.channel, raw=raw, level=level, contact=now
        )


def endpoints_from_samples(
    released: Iterable[float],
    contact: Iterable[float],
) -> tuple[float, float]:
    """Rohwert-Messreihen → (raw_released, raw_contact) als Median.

    Bewusst getrennt von der Pydantic-Klasse: Das CLI-Tool bildet damit
    die Endpunkte, zeigt sie dem Nutzer und lässt sie erst danach vom
    Modell validieren (Spanne groß genug?).
    """
    released_values = list(released)
    contact_values = list(contact)
    if not released_values or not contact_values:
        raise ValueError("Beide Messreihen brauchen mindestens einen Wert.")
    return statistics.median(released_values), statistics.median(contact_values)


def describe(readings: Mapping[str, FootSensorReading]) -> str:
    """Einzeiler für Logs: 'front_left 0.62 KONTAKT | mid_left 0.03 frei'."""
    parts: list[str] = []
    for leg, r in readings.items():
        if r.level is None:
            parts.append(f"{leg} roh={r.raw:.0f} (unkalibriert)")
        else:
            parts.append(f"{leg} {r.level:.2f} {'KONTAKT' if r.contact else 'frei'}")
    return " | ".join(parts)


__all__ = [
    "FootSensorArray",
    "FootSensorReading",
    "describe",
    "endpoints_from_samples",
]
