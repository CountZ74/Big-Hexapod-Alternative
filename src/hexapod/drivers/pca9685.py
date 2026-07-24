"""PCA9685 16-Kanal-PWM-Treiber über I2C — für die Kamera-Servos.

Die Freenove-Platine trägt ZWEI PCA9685 und führt sie als durchgehend
nummerierte Servo-Anschlüsse 0..31 auf die Stiftleisten:

    Anschluss  0..15  ->  Chip 0x41, Chip-Kanal 0..15
    Anschluss 16..31  ->  Chip 0x40, Chip-Kanal 0..15

Dieses Mapping stammt aus der Original-Freenove-Software (``servo.py``) und
ist hier bewusst identisch nachgebildet: die Kanalnummer in ``robot.yaml``
entspricht damit exakt der Beschriftung auf der Platine. Die Kamera-Servos
hängen an Anschluss 29 (Pan) und 30 (Tilt), also auf Chip 0x40, Kanal 13/14.

Einheiten-Konvention wie im ganzen Projekt: Positionen IMMER in Mikrosekunden
Pulsweite. Die 12-Bit-Ticks des PCA9685 sind ein Implementierungsdetail und
treten ausserhalb dieses Moduls nicht auf.

Anders als der Maestro hat der PCA9685 KEINE interne Speed-/Acceleration-
Rampe: er gibt die kommandierte Pulsweite sofort aus. Die Glaettung der
Kamerabewegung macht deshalb weiterhin der ``CameraThread`` in Software
(slew-ratenbegrenzt). ``set_speed``/``set_acceleration`` sind hier bewusst
No-ops (geerbt aus ``ServoDriver``).

Robustheit: Der I2C-Bus wird lazy geoeffnet. Faellt er aus (fehlendes smbus2,
kein /dev/i2c-1), wird das einmal geloggt und der Treiber laeuft als
No-op weiter — ein defekter Kamera-Bus soll den Roboter nicht lahmlegen.
"""

from __future__ import annotations

import contextlib
import logging
import math
import threading
import time
from typing import Protocol

from .base import ServoDriver

logger = logging.getLogger(__name__)

# --- PCA9685-Register ---
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

# MODE1-Bits
MODE1_RESTART = 0x80
MODE1_SLEEP = 0x10
MODE1_AI = 0x20  # Auto-Increment: erlaubt 4 Bytes am Stueck zu schreiben

# Der interne Oszillator laeuft mit 25 MHz, der Zaehler ist 12 Bit.
OSC_HZ = 25_000_000.0
TICKS = 4096

# "Full off"-Bit im OFF_H-Register: Kanal gibt gar keinen Puls mehr aus.
FULL_OFF = 0x10

# Adressen der beiden Chips auf der Freenove-Platine.
ADDR_LOW = 0x41   # traegt die Anschluesse 0..15
ADDR_HIGH = 0x40  # traegt die Anschluesse 16..31

CHANNELS_PER_CHIP = 16
DEFAULT_NUM_CHANNELS = 32
DEFAULT_FREQ_HZ = 50.0

DISABLED = 0.0
_DEFAULT_MIN_US = 400.0
_DEFAULT_MAX_US = 2600.0


class I2CBus(Protocol):
    """Minimales I2C-Interface — genau das, was dieser Treiber braucht.

    Erlaubt es, in Tests einen Fake-Bus zu injizieren, ohne smbus2 oder
    echte Hardware zu benoetigen.
    """

    def write_byte_data(self, addr: int, register: int, value: int) -> None: ...

    def read_byte_data(self, addr: int, register: int) -> int: ...

    def write_i2c_block_data(self, addr: int, register: int, data: list[int]) -> None: ...

    def close(self) -> None: ...


class Pca9685Error(RuntimeError):
    """Allgemeiner PCA9685-Kommunikationsfehler."""


class Pca9685Driver(ServoDriver):
    """Servo-Treiber für die PCA9685-Paare der Freenove-Platine.

    Args:
        bus: I2C-Busnummer (auf dem Pi immer 1).
        num_channels: Anzahl logischer Kanaele (32 = beide Chips).
        freq_hz: PWM-Frequenz. 50 Hz ist Standard fuer RC-Servos.
        min_pulse_us / max_pulse_us: harte Sicherheitsgrenzen.
        i2c: Optional ein bereits geoeffneter Bus (nur fuer Tests).
        addr_low / addr_high: I2C-Adressen der beiden Chips.
    """

    def __init__(
        self,
        bus: int = 1,
        num_channels: int = DEFAULT_NUM_CHANNELS,
        *,
        freq_hz: float = DEFAULT_FREQ_HZ,
        min_pulse_us: float = _DEFAULT_MIN_US,
        max_pulse_us: float = _DEFAULT_MAX_US,
        i2c: I2CBus | None = None,
        addr_low: int = ADDR_LOW,
        addr_high: int = ADDR_HIGH,
    ) -> None:
        if num_channels <= 0 or num_channels > 2 * CHANNELS_PER_CHIP:
            raise ValueError(
                f"num_channels muss zwischen 1 und {2 * CHANNELS_PER_CHIP} liegen, "
                f"war {num_channels}"
            )
        if freq_hz <= 0.0:
            raise ValueError(f"freq_hz muss positiv sein, war {freq_hz}")
        if min_pulse_us >= max_pulse_us:
            raise ValueError(
                f"min_pulse_us ({min_pulse_us}) muss < max_pulse_us ({max_pulse_us}) sein"
            )

        self._busno = bus
        self._num_channels = num_channels
        self._freq_hz = freq_hz
        self._min_pulse_us = min_pulse_us
        self._max_pulse_us = max_pulse_us
        self._addr_low = addr_low
        self._addr_high = addr_high
        self._closed = False
        self._failed = False
        self._initialised: set[int] = set()
        self._positions: dict[int, float] = {}

        # Schuetzt jede I2C-Transaktion. Der Bus wird mit MPU6050 und ADS7830
        # geteilt; innerhalb dieses Treibers serialisieren wir zumindest
        # unsere eigenen Zugriffe (Kamera-Thread vs. Worker).
        self._io_lock = threading.RLock()

        self._i2c: I2CBus | None = i2c
        if i2c is not None:
            # Dependency Injection (Tests): Chips sofort konfigurieren.
            self._init_chips()

    # ---- Kanal-Mapping ----

    def _resolve(self, channel: int) -> tuple[int, int]:
        """Logischer Kanal 0..31 -> (I2C-Adresse, Chip-Kanal 0..15)."""
        if not 0 <= channel < self._num_channels:
            raise ValueError(
                f"Kanal {channel} ausserhalb [0..{self._num_channels - 1}]"
            )
        if channel < CHANNELS_PER_CHIP:
            return self._addr_low, channel
        return self._addr_high, channel - CHANNELS_PER_CHIP

    # ---- I2C-Zugriff ----

    def _ensure_bus(self) -> I2CBus | None:
        if self._closed:
            raise Pca9685Error("Treiber ist bereits geschlossen")
        if self._i2c is None and not self._failed:
            try:
                import smbus2

                self._i2c = smbus2.SMBus(self._busno)
            except Exception as e:  # smbus2 fehlt oder Bus nicht vorhanden
                logger.warning(
                    "PCA9685: I2C nicht verfuegbar (%s) — Kamera-Servos inaktiv", e
                )
                self._failed = True
                return None
            self._init_chips()
        return self._i2c

    def _write_reg(self, addr: int, register: int, value: int) -> None:
        bus = self._i2c
        if bus is None:
            return
        with self._io_lock:
            bus.write_byte_data(addr, register, value)

    def _init_chips(self) -> None:
        """Setzt MODE1 und die PWM-Frequenz auf allen benutzten Chips."""
        addrs = {self._addr_low}
        if self._num_channels > CHANNELS_PER_CHIP:
            addrs.add(self._addr_high)
        for addr in sorted(addrs):
            with contextlib.suppress(OSError):
                self._set_freq(addr, self._freq_hz)
                self._initialised.add(addr)

    def _set_freq(self, addr: int, freq_hz: float) -> None:
        """Schreibt den Prescaler — nur im Sleep-Modus erlaubt."""
        prescale = math.floor(OSC_HZ / (TICKS * freq_hz) - 1.0 + 0.5)
        prescale = max(3, min(255, prescale))  # Datenblatt: 3..255
        bus = self._i2c
        if bus is None:
            return
        with self._io_lock:
            oldmode = bus.read_byte_data(addr, MODE1)
            # Sleep setzen, Restart-Bit dabei loeschen.
            bus.write_byte_data(addr, MODE1, (oldmode & ~MODE1_RESTART) | MODE1_SLEEP)
            bus.write_byte_data(addr, PRESCALE, prescale)
            bus.write_byte_data(addr, MODE1, oldmode)
            time.sleep(0.005)  # Oszillator braucht max. 500 us zum Anlaufen
            # Auto-Increment aktivieren, damit wir 4 Bytes am Stueck schreiben.
            bus.write_byte_data(addr, MODE1, oldmode | MODE1_RESTART | MODE1_AI)

    def _set_pwm(self, addr: int, chip_channel: int, on: int, off: int) -> None:
        bus = self._i2c
        if bus is None:
            return
        register = LED0_ON_L + 4 * chip_channel
        data = [on & 0xFF, on >> 8, off & 0xFF, off >> 8]
        with self._io_lock:
            bus.write_i2c_block_data(addr, register, data)

    # ---- Umrechnung ----

    def _us_to_ticks(self, microseconds: float) -> int:
        """Pulsweite [us] -> 12-Bit-Tick des PCA9685."""
        period_us = 1_000_000.0 / self._freq_hz
        ticks = round(microseconds / period_us * TICKS)
        return max(0, min(TICKS - 1, ticks))

    def _clamp(self, microseconds: float) -> float:
        return max(self._min_pulse_us, min(self._max_pulse_us, microseconds))

    # ---- ServoDriver-Interface ----

    def set_position(self, channel: int, microseconds: float) -> None:
        """Setze die Pulsweite eines Kanals. 0.0 us deaktiviert den Kanal."""
        if microseconds < 0.0:
            raise ValueError(
                f"Position muss >= 0 sein, war {microseconds} (Kanal {channel})"
            )
        addr, chip_channel = self._resolve(channel)
        if microseconds == DISABLED:
            self.disable(channel)
            return
        us = self._clamp(microseconds)
        if us != microseconds:
            logger.debug(
                "Kanal %d: %.1f us auf %.1f us begrenzt", channel, microseconds, us
            )
        if self._ensure_bus() is None:
            self._positions[channel] = us
            return
        try:
            self._set_pwm(addr, chip_channel, 0, self._us_to_ticks(us))
        except OSError as e:
            logger.warning("PCA9685: Schreibfehler auf Kanal %d: %s", channel, e)
        self._positions[channel] = us

    def get_position(self, channel: int) -> float:
        """Zuletzt kommandierte Pulsweite [us]. 0.0 = deaktiviert/unbekannt.

        Der PCA9685 kann seine Register zwar zurueckgelesen werden, aber wie
        bei allen RC-Servos gibt es kein Positions-Feedback — deshalb liefern
        wir denselben Wert wie der Maestro-Treiber: den Sollwert.
        """
        self._resolve(channel)  # validiert den Kanal
        return self._positions.get(channel, DISABLED)

    def disable(self, channel: int) -> None:
        """Kanal stromlos schalten (Full-Off-Bit, kein Puls mehr)."""
        addr, chip_channel = self._resolve(channel)
        if self._ensure_bus() is not None:
            try:
                self._set_pwm(addr, chip_channel, 0, FULL_OFF << 8)
            except OSError as e:
                logger.warning("PCA9685: Disable auf Kanal %d fehlgeschlagen: %s", channel, e)
        self._positions[channel] = DISABLED

    def close(self, *, disable: bool = True) -> None:
        """Gibt den I2C-Bus frei. disable=True schaltet vorher alle Kanaele ab."""
        if self._closed:
            return
        if disable and self._i2c is not None:
            with contextlib.suppress(Exception):
                self.disable_all(self._num_channels)
        self._closed = True
        if self._i2c is not None:
            with contextlib.suppress(Exception):
                self._i2c.close()
            self._i2c = None
        logger.info("Pca9685Driver geschlossen (I2C-Bus %d)", self._busno)

    # ---- Properties ----

    @property
    def num_channels(self) -> int:
        return self._num_channels

    @property
    def freq_hz(self) -> float:
        return self._freq_hz

    @property
    def is_closed(self) -> bool:
        return self._closed
