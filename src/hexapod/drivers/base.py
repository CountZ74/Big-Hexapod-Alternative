"""Abstrakte Basis für alle Servo-Treiber.

Die Hexapod-Logik kennt ausschließlich dieses Interface; konkrete Treiber
(Maestro, PCA9685, Simulator) implementieren es. Dadurch können wir die
Hardware austauschen, ohne die Geschäftslogik anzufassen.

Einheiten-Konvention für das ganze Projekt:
* Servo-Position wird IMMER in Mikrosekunden Pulsweite (us) ausgedrückt.
* Typischer Bereich: 500 ... 2500 us, mit 1500 us als Mitte.
* Maestro-interne Quarter-us-Werte sind ein Implementierungsdetail des
  MaestroDrivers und treten in keinem anderen Modul auf.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Mapping
from types import TracebackType


class ServoDriver(ABC):
    """Abstrakter Servo-Treiber.

    Konkrete Treiber müssen `set_position`, `get_position`, `disable` und
    `close` implementieren. Der Konstruktor und alle anderen Methoden
    können (müssen aber nicht) überschrieben werden.

    Convention:
    * Position 0.0 us = "Servo deaktivieren" (kein Puls). Manche Hardware
      kann das, andere nicht — siehe konkrete Implementierungen.
    * Positionen müssen >= 0.0 sein. Negative Werte werfen ValueError.
    """

    @abstractmethod
    def set_position(self, channel: int, microseconds: float) -> None:
        """Setze die Soll-Position eines Servos in Mikrosekunden."""

    @abstractmethod
    def get_position(self, channel: int) -> float:
        """Lies die aktuelle Soll-Position eines Servos in Mikrosekunden.

        Achtung: Das ist die zuletzt *kommandierte* Position, nicht die
        tatsächlich mechanisch erreichte. RC-Servos haben kein Feedback.
        """

    @abstractmethod
    def disable(self, channel: int) -> None:
        """Deaktiviere einen Kanal (kein Puls mehr senden).

        Der Servo wird stromlos und kann von Hand bewegt werden.
        """

    @abstractmethod
    def close(self, *, disable: bool = True) -> None:
        """Gib alle Ressourcen frei (z.B. serielle Schnittstelle)."""

    def set_speed(self, channel: int, speed: int) -> None:
        """Setze die Geschwindigkeit eines Kanals.

        Speed in Einheiten von 0.25 µs / 10 ms. 0 = unbegrenzt (sofort).
        Typische Werte: 10 (sehr langsam) ... 100 (zügig).
        Wird von Hardware ignoriert, die das nicht unterstützt.
        """

    def set_acceleration(self, channel: int, acceleration: int) -> None:
        """Setze die Beschleunigung eines Kanals.

        Acceleration in Einheiten von 0.25 µs / 10 ms / 80 ms. 0 = unbegrenzt.
        Typische Werte: 1 (sehr sanft) ... 50 (direkt).
        Wird von Hardware ignoriert, die das nicht unterstützt.
        """

    def set_speed_all(self, channels: int, speed: int) -> None:
        """Setzt die Geschwindigkeit für alle Kanäle 0 bis channels-1."""
        for ch in range(channels):
            self.set_speed(ch, speed)

    def set_acceleration_all(self, channels: int, acceleration: int) -> None:
        """Setzt die Beschleunigung für alle Kanäle 0 bis channels-1."""
        for ch in range(channels):
            self.set_acceleration(ch, acceleration)

    def set_positions(self, positions: Mapping[int, float]) -> None:
        """Setze mehrere Positionen auf einmal.

        Default-Implementierung iteriert über `set_position`. Konkrete
        Treiber dürfen das überschreiben, wenn die Hardware Batch-Befehle
        unterstützt (Maestro kann das z.B. mit "Set Multiple Targets").
        """
        for channel, microseconds in positions.items():
            self.set_position(channel, microseconds)

    def disable_all(self, channels: int) -> None:
        """Deaktiviere alle Kanäle von 0 bis `channels - 1`."""
        for ch in range(channels):
            self.disable(ch)

    def __enter__(self) -> ServoDriver:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        self.close()
