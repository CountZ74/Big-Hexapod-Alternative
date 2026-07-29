"""Simulator-Servo-Treiber für Tests und Entwicklung ohne Hardware.

Hält Positionen im RAM, druckt jeden Befehl auf die Konsole (optional)
und kann den letzten Zustand für Assertions in Tests abfragen.
"""

from __future__ import annotations

import logging
from collections.abc import Mapping

from .base import ServoDriver

logger = logging.getLogger(__name__)

DISABLED = 0.0
# Groesster Rohwert eines 10-Bit-Analogeingangs (wie beim Maestro).
ANALOG_MAX = 1023
# Fallback-Grenzen — werden durch Konfig überschrieben
_DEFAULT_MIN_US = 400.0
_DEFAULT_MAX_US = 2600.0


class SimulatorDriver(ServoDriver):
    """Servo-Treiber, der nichts wirklich tut.

    Speichert kommandierte Positionen im RAM und loggt jeden Befehl.
    Perfekt zum Testen von Kinematik, Gangart und Verhalten ohne
    angeschlossene Hardware.
    """

    def __init__(self, num_channels: int = 24, *, verbose: bool = False, min_pulse_us: float = _DEFAULT_MIN_US, max_pulse_us: float = _DEFAULT_MAX_US) -> None:
        if num_channels <= 0:
            raise ValueError(f"num_channels muss positiv sein, war {num_channels}")
        self._num_channels = num_channels
        self._positions: dict[int, float] = dict.fromkeys(range(num_channels), DISABLED)
        self._verbose = verbose
        self._closed = False
        self._min_pulse_us = min_pulse_us
        self._max_pulse_us = max_pulse_us
        # Vorgebbare Analogwerte, damit Sensor-Code ohne Hardware testbar ist.
        self._analog: dict[int, int] = {}

    def _check_channel(self, channel: int) -> None:
        if not 0 <= channel < self._num_channels:
            raise ValueError(f"Kanal {channel} außerhalb [0, {self._num_channels})")

    def _check_open(self) -> None:
        if self._closed:
            raise RuntimeError("SimulatorDriver wurde bereits geschlossen")

    def _validate_pulse(self, channel: int, microseconds: float) -> None:
        if microseconds < 0:
            raise ValueError(f"Kanal {channel}: Pulsweite {microseconds} < 0")
        if microseconds != DISABLED and not (self._min_pulse_us <= microseconds <= self._max_pulse_us):
            raise ValueError(
                f"Kanal {channel}: Pulsweite {microseconds} us außerhalb "
                f"plausibler Grenzen [{self._min_pulse_us}, {self._max_pulse_us}]"
            )

    def _log(self, channel: int, microseconds: float, *, disabled: bool = False) -> None:
        if disabled:
            msg = f"[SIM] ch={channel:2d} -> DISABLED"
        else:
            msg = f"[SIM] ch={channel:2d} -> {microseconds:7.1f} us"
        logger.debug(msg)
        if self._verbose:
            print(msg)

    def set_position(self, channel: int, microseconds: float) -> None:
        self._check_open()
        self._check_channel(channel)
        self._validate_pulse(channel, microseconds)
        self._positions[channel] = microseconds
        self._log(channel, microseconds)

    def set_positions(self, positions: Mapping[int, float]) -> None:
        self._check_open()
        # Erst alle validieren, dann committen (atomar)
        for channel, us in positions.items():
            self._check_channel(channel)
            self._validate_pulse(channel, us)
        for channel, us in positions.items():
            self._positions[channel] = us
            self._log(channel, us)

    def get_position(self, channel: int) -> float:
        self._check_open()
        self._check_channel(channel)
        return self._positions[channel]

    def read_analog(self, channel: int) -> int:
        """Rohwert eines simulierten Analogeingangs (Default 0)."""
        self._check_open()
        self._check_channel(channel)
        return self._analog.get(channel, 0)

    def set_analog(self, channel: int, value: int) -> None:
        """Analogwert vorgeben — das Gegenstueck zu `read_analog` fuer Tests."""
        self._check_channel(channel)
        if not 0 <= value <= ANALOG_MAX:
            raise ValueError(f"Analogwert {value} ausserhalb [0, {ANALOG_MAX}]")
        self._analog[channel] = value

    def disable(self, channel: int) -> None:
        self._check_open()
        self._check_channel(channel)
        self._positions[channel] = DISABLED
        self._log(channel, 0.0, disabled=True)

    def close(self, *, disable: bool = True) -> None:
        if self._closed:
            return
        for ch in range(self._num_channels):
            self._positions[ch] = DISABLED
        self._closed = True
        logger.debug("[SIM] closed")

    @property
    def num_channels(self) -> int:
        return self._num_channels

    @property
    def is_closed(self) -> bool:
        return self._closed

    def snapshot(self) -> dict[int, float]:
        return dict(self._positions)
