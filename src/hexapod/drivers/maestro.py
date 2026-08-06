"""Pololu Mini Maestro Servo-Treiber über USB.

Spricht das "Compact Protocol" über die USB-CDC-ACM-Schnittstelle.
Eine separate udev-Rule sorgt für stabile Symlinks (`/dev/maestro_cmd`).

Pololu User's Guide: https://www.pololu.com/docs/0J40
"""

from __future__ import annotations

import contextlib
import logging
import threading
from collections.abc import Collection, Mapping

import serial

from .base import ServoDriver

logger = logging.getLogger(__name__)

# --- Compact-Protocol-Kommandos ---
CMD_SET_TARGET = 0x84       # 0x84 <ch> <low> <high>      -> kein Response
CMD_SET_MULTIPLE = 0x9F     # 0x9F <n> <first_ch> <low><high>...  -> kein Response
CMD_GET_POSITION = 0x90     # 0x90 <ch>                   -> 2 Bytes (low, high)
CMD_GET_ERRORS = 0xA1       # 0xA1                        -> 2 Bytes (low, high)
CMD_GO_HOME = 0xA2          # 0xA2                        -> kein Response
CMD_SET_SPEED = 0x87        # 0x87 <ch> <low> <high>      -> kein Response
CMD_SET_ACCELERATION = 0x89 # 0x89 <ch> <low> <high>      -> kein Response

# Nur die Kanaele 0..11 des Mini Maestro 24 haben einen ADC; 12..23 sind reine
# Digital-Ein-/Ausgaenge und liefern als Eingang nur 0 oder 1023.
MAX_ANALOG_CHANNEL = 11

# Mikrosekunden <-> Quarter-Mikrosekunden
# Maestro arbeitet intern in 0.25-us-Schritten.
QUARTER_US_PER_US = 4

# Plausibilitätsgrenzen für RC-Servo-Pulsweiten in Mikrosekunden.
DISABLED = 0.0
_DEFAULT_MIN_US = 400.0
_DEFAULT_MAX_US = 2600.0


class MaestroError(RuntimeError):
    """Allgemeiner Maestro-Kommunikationsfehler."""


class MaestroDriver(ServoDriver):
    """Servo-Treiber für Pololu Mini Maestro (USB Compact-Protocol).

    Args:
        port: Serielles Device, üblicherweise `/dev/maestro_cmd`.
        num_channels: Anzahl Kanäle. 24 für Mini Maestro 24.
        timeout: Lese-Timeout in Sekunden für Antworten.
        ser: Optional ein bereits geöffnetes Serial-Objekt (nur für Tests).
    """

    def __init__(
        self,
        port: str = "/dev/maestro_cmd",
        num_channels: int = 24,
        *,
        timeout: float = 1.0,
        ser: serial.Serial | None = None,
        min_pulse_us: float = _DEFAULT_MIN_US,
        max_pulse_us: float = _DEFAULT_MAX_US,
        initial_speed: int | None = 0,
        initial_acceleration: int | None = 0,
    ) -> None:
        if num_channels <= 0:
            raise ValueError(f"num_channels muss positiv sein, war {num_channels}")
        self._num_channels = num_channels
        self._port = port
        self._closed = False
        self._min_pulse_us = min_pulse_us
        self._max_pulse_us = max_pulse_us
        # Schuetzt jede serielle Transaktion (write/flush[/read]) als Einheit,
        # damit Gang-Worker und Kamera-Thread sich die Bytes nicht verschachteln.
        self._io_lock = threading.Lock()

        if ser is not None:
            # Dependency Injection: in Tests übergeben wir ein Fake-Serial.
            self._ser = ser
        else:
            # Baudrate ist bei USB-CDC egal (der Maestro ignoriert sie),
            # aber pyserial braucht einen Wert.
            self._ser = serial.Serial(port, baudrate=9600, timeout=timeout)

        logger.info("MaestroDriver geöffnet auf %s (%d Kanäle)", port, num_channels)

        # Speed/Acceleration explizit setzen, bevor irgendein Servo bewegt wird.
        # Default 0 = UNBEGRENZT: Bewegungen werden allein über den Executor
        # (max_step_deg) begrenzt, nicht servo-intern — sonst laufen Femur und
        # Tibia nicht synchron und der Fuss macht beim Heben/Senken einen Bogen.
        # Die 0 wird bewusst GESENDET (nicht uebersprungen), um eine evtl. aus
        # einer frueheren Sitzung im Maestro gespeicherte Begrenzung zu loeschen.
        # Der Einschalt-Schutz uebernimmt stattdessen die definierte
        # Ausgangslage vor dem Power-up (Beine flach in Kalibrierposition).
        # None = gar nicht senden (fuer Tests mit sauberem Byte-Protokoll).
        if initial_speed is not None:
            for ch in range(num_channels):
                self.set_speed(ch, initial_speed)
        if initial_acceleration is not None:
            for ch in range(num_channels):
                self.set_acceleration(ch, initial_acceleration)
        # Warten bis alle Speed/Acceleration-Bytes vom Maestro verarbeitet sind.
        if initial_speed is not None or initial_acceleration is not None:
            import time
            time.sleep(0.5)

    # ---- Thread-sichere serielle IO ----

    def _write(self, data: bytes) -> None:
        with self._io_lock:
            self._ser.write(data)
            self._ser.flush()

    def _write_read(self, data: bytes, n: int) -> bytes:
        with self._io_lock:
            self._ser.write(data)
            self._ser.flush()
            return self._ser.read(n)

    # ---- Validation ----

    def _check_channel(self, channel: int) -> None:
        if not 0 <= channel < self._num_channels:
            raise ValueError(f"Kanal {channel} außerhalb [0, {self._num_channels})")

    def _check_open(self) -> None:
        if self._closed:
            raise RuntimeError("MaestroDriver wurde bereits geschlossen")

    def _validate_pulse(self, channel: int, microseconds: float) -> None:
        if microseconds < 0:
            raise ValueError(f"Kanal {channel}: Pulsweite {microseconds} < 0")
        if microseconds != DISABLED and not (self._min_pulse_us <= microseconds <= self._max_pulse_us):
            raise ValueError(
                f"Kanal {channel}: Pulsweite {microseconds} us außerhalb "
                f"plausibler Grenzen [{self._min_pulse_us}, {self._max_pulse_us}]"
            )

    # ---- 7-Bit-Encoding ----

    @staticmethod
    def _encode_14bit(value: int) -> tuple[int, int]:
        """Teile einen 14-Bit-Wert in zwei 7-Bit-Bytes (low, high).

        Pololu-Protokoll: Datenbytes haben Bit 7 immer = 0.
        """
        if not 0 <= value < (1 << 14):
            raise ValueError(f"Wert {value} außerhalb 14-Bit-Bereich")
        return value & 0x7F, (value >> 7) & 0x7F

    @staticmethod
    def _decode_response_16bit(low: int, high: int) -> int:
        return low | (high << 8)

    # ---- ServoDriver-Interface ----

    def set_position(self, channel: int, microseconds: float) -> None:
        self._check_open()
        self._check_channel(channel)
        self._validate_pulse(channel, microseconds)

        quarter_us = round(microseconds * QUARTER_US_PER_US)
        low, high = self._encode_14bit(quarter_us)
        self._write(bytes([CMD_SET_TARGET, channel, low, high]))
        logger.debug("set ch=%d -> %.1f us (%d qus)", channel, microseconds, quarter_us)

    def set_positions(self, positions: Mapping[int, float]) -> None:
        """Setzt mehrere Positionen mit einem einzigen "Set Multiple Targets".

        Voraussetzung des Maestro-Kommandos: die Kanäle müssen
        *aufeinanderfolgend* (consecutive) sein. Sonst splitten wir
        automatisch in mehrere Aufrufe.
        """
        self._check_open()

        # Erst alles validieren (atomar)
        for channel, us in positions.items():
            self._check_channel(channel)
            self._validate_pulse(channel, us)

        if not positions:
            return

        # In aufeinanderfolgende Blöcke gruppieren
        sorted_channels = sorted(positions.keys())
        block: list[int] = []
        for ch in sorted_channels:
            if not block or ch == block[-1] + 1:
                block.append(ch)
            else:
                self._send_block(block, positions)
                block = [ch]
        if block:
            self._send_block(block, positions)

    def _send_block(self, channels: list[int], positions: Mapping[int, float]) -> None:
        """Sendet einen Block aufeinanderfolgender Kanäle als 'Set Multiple Targets'."""
        first = channels[0]
        n = len(channels)
        payload = bytearray([CMD_SET_MULTIPLE, n, first])
        for ch in channels:
            qus = round(positions[ch] * QUARTER_US_PER_US)
            low, high = self._encode_14bit(qus)
            payload.append(low)
            payload.append(high)
        self._write(bytes(payload))
        logger.debug("set_multiple first=%d n=%d", first, n)

    def get_position(self, channel: int) -> float:
        self._check_open()
        self._check_channel(channel)
        raw = self._write_read(bytes([CMD_GET_POSITION, channel]), 2)
        if len(raw) != 2:
            raise MaestroError(
                f"get_position(ch={channel}): erwartete 2 Bytes, "
                f"erhalten {len(raw)}"
            )
        quarter_us = self._decode_response_16bit(raw[0], raw[1])
        return quarter_us / QUARTER_US_PER_US

    def read_analog(self, channel: int) -> int:
        """Rohwert eines als *Input* konfigurierten Kanals (0..1023).

        Der Maestro beantwortet "Get Position" je nach Kanal-Konfiguration
        unterschiedlich: Bei einem Ausgang ist die Antwort die Soll-Pulsweite
        in Viertel-Mikrosekunden, bei einem Eingang dagegen direkt der
        ADC-Wert 0..1023 (entspricht 0..5 V). Deshalb liefert diese Methode
        bewusst den ungeteilten 16-Bit-Rohwert, waehrend `get_position` durch
        4 teilt.

        Voraussetzung: Der Kanal wurde im Maestro Control Center einmalig auf
        "Input" gestellt und die Einstellung im Geraet gespeichert. Ein noch
        als Servo konfigurierter Kanal gibt stattdessen seine eigene
        Soll-Pulsweite zurueck — der Wert sieht dann plausibel aus, ist aber
        kein Sensorwert.
        """
        self._check_open()
        self._check_channel(channel)
        if channel > MAX_ANALOG_CHANNEL:
            logger.warning(
                "Kanal %d hat keinen ADC (nur 0..%d) — Wert ist rein digital (0/1023)",
                channel,
                MAX_ANALOG_CHANNEL,
            )
        raw = self._write_read(bytes([CMD_GET_POSITION, channel]), 2)
        if len(raw) != 2:
            raise MaestroError(
                f"read_analog(ch={channel}): erwartete 2 Bytes, "
                f"erhalten {len(raw)}"
            )
        return self._decode_response_16bit(raw[0], raw[1])

    def prime(self, skip: Collection[int] = ()) -> int:
        """Bereite sanftes Anfahren vor (gegen ungebremsten ersten Zug).

        Der Maestro fährt den allerersten Set-Target-Befehl nach einem
        (USB-)Reset IMMER ungebremst an, weil er die Servoposition nicht
        kennt (RC-Servos geben keine Rückmeldung). Diese Methode liest die
        zuletzt kommandierte Position jedes aktiven Kanals aus und schreibt
        sie unverändert zurück. Dadurch ist der ungebremste erste Zug
        "verbraucht", ohne dass sich etwas bewegt — alle folgenden Züge
        gehorchen den Speed/Acceleration-Limits.

        Voraussetzung: Der Maestro war durchgehend mit Strom versorgt, sodass
        seine intern gespeicherte Position noch der echten entspricht. Kanäle
        mit Position 0 (deaktiviert/aus) werden übersprungen.

        WICHTIG bei gemischt belegten Boards: Auf einem als *Input*
        konfigurierten Kanal liefert "Get Position" keinen Puls, sondern den
        ADC-Wert 0..1023 — nach der Umrechnung also 0..256 µs. Das ist keine
        gültige Pulsweite und darf auf gar keinen Fall zurückgeschrieben
        werden. Solche Kanäle gehören in `skip`; zusätzlich werden Werte
        außerhalb der plausiblen Pulsgrenzen sicherheitshalber übersprungen
        statt zu werfen, damit ein vergessener Eintrag den Kaltstart nicht
        abbricht.

        Args:
            skip: Kanäle, die nicht angefasst werden (z.B. Sensor-Eingänge).

        Returns:
            Anzahl der geprimeten (aktiven) Kanäle.
        """
        self._check_open()
        skip_set = set(skip)
        primed = 0
        for ch in range(self._num_channels):
            if ch in skip_set:
                continue
            pos = self.get_position(ch)
            if pos <= 0.0:
                continue  # deaktivierter Kanal, kein Puls
            if not (self._min_pulse_us <= pos <= self._max_pulse_us):
                logger.warning(
                    "prime: Kanal %d liefert %.1f us — keine plausible Pulsweite, "
                    "vermutlich ein Eingang. Wird uebersprungen.",
                    ch, pos,
                )
                continue
            # Exakt dieselbe Position als ersten Set-Target zurückschreiben.
            self.set_position(ch, pos)
            primed += 1
        return primed

    def disable(self, channel: int) -> None:
        self._check_open()
        self._check_channel(channel)
        # "Position 0" deaktiviert den Servo (kein Puls)
        low, high = self._encode_14bit(0)
        self._write(bytes([CMD_SET_TARGET, channel, low, high]))
        logger.debug("disable ch=%d", channel)

    def close(self, *, disable: bool = True) -> None:
        if self._closed:
            return
        # Kompletter Shutdown unter dem IO-Lock, damit sich die Bytes nicht
        # mit parallel sendenden Threads (Gait/Kamera) verschachteln.
        with self._io_lock:
            try:
                # Sicherheits-Default: alle Kanäle deaktivieren beim Schließen.
                # disable=False lässt die Servos unter Signal (Pose halten).
                for ch in (range(self._num_channels) if disable else range(0)):
                    try:
                        low, high = self._encode_14bit(0)
                        self._ser.write(bytes([CMD_SET_TARGET, ch, low, high]))
                    except (OSError, serial.SerialException) as e:
                        logger.warning("Konnte ch=%d beim Schließen nicht deaktivieren: %s", ch, e)
                with contextlib.suppress(OSError, serial.SerialException):
                    self._ser.flush()
            finally:
                try:
                    self._ser.close()
                except (OSError, serial.SerialException) as e:
                    logger.warning("Fehler beim Schließen der seriellen Schnittstelle: %s", e)
                self._closed = True
                logger.info("MaestroDriver geschlossen")

    # ---- Maestro-spezifische Zusatzmethoden ----

    def set_speed(self, channel: int, speed: int) -> None:
        """Setze die Geschwindigkeit für einen Kanal.

        Args:
            channel: Kanal-Nummer.
            speed: Geschwindigkeit in 0.25 µs / 10 ms. 0 = unbegrenzt.
                   Typische Werte: 10 (sehr langsam) ... 100 (zügig).
        """
        self._check_open()
        self._check_channel(channel)
        if not 0 <= speed <= 16383:
            raise ValueError(f"speed {speed} außerhalb [0, 16383]")
        low, high = self._encode_14bit(speed)
        self._write(bytes([CMD_SET_SPEED, channel, low, high]))
        logger.debug("set_speed ch=%d speed=%d", channel, speed)

    def set_acceleration(self, channel: int, acceleration: int) -> None:
        """Setze die Beschleunigung für einen Kanal.

        Args:
            channel: Kanal-Nummer.
            acceleration: Beschleunigung in 0.25 µs / 10 ms / 80 ms.
                          0 = unbegrenzt. Typische Werte: 1 ... 50.
        """
        self._check_open()
        self._check_channel(channel)
        if not 0 <= acceleration <= 255:
            raise ValueError(f"acceleration {acceleration} außerhalb [0, 255]")
        low, high = self._encode_14bit(acceleration)
        self._write(bytes([CMD_SET_ACCELERATION, channel, low, high]))
        logger.debug("set_acceleration ch=%d accel=%d", channel, acceleration)

    def get_errors(self) -> int:
        """Liest das Error-Flags-Register des Maestro (löscht es dabei)."""
        self._check_open()
        raw = self._write_read(bytes([CMD_GET_ERRORS]), 2)
        if len(raw) != 2:
            raise MaestroError(
                f"get_errors: erwartete 2 Bytes, erhalten {len(raw)}"
            )
        return self._decode_response_16bit(raw[0], raw[1])

    @property
    def num_channels(self) -> int:
        return self._num_channels

    @property
    def is_closed(self) -> bool:
        return self._closed

    @property
    def port(self) -> str:
        return self._port
