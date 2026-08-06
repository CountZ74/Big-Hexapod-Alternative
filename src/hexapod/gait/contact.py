"""Bodenkontakt als Abbruchkriterium für Trajektorien.

Liefert eine ``freeze``-Funktion für den Executor: ein Bein, das beim
Absenken früher Boden findet als erwartet, hält seine Position, während die
übrigen ihre Bahn zu Ende fahren.

Der entscheidende Punkt ist das Wort *früher*. Eine Schwelle allein genügt
nicht — sie wird schon beim Antippen erreicht, lange bevor sich das Bein in
die Standpose gedrückt hat. Ohne Mindesthöhe stoppt deshalb jedes Bein zu
früh, der Körper sinkt nie ganz ab und der Roboter steht auf sechs kaum
eingefederten Beinen. Das war am echten Roboter zu sehen: auf ebenem Boden
hielten alle sechs 1 bis 4 mm über der Standpose an.

Kontakt zählt deshalb erst oberhalb von ``margin_mm``. Darunter ist er
normal und erwünscht.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from hexapod.robot.hexapod import Hexapod

# Aus der Messung: auf ebenem Boden lag der erste Kontakt bis zu 4 mm über
# der Standpose. 5 mm lässt normales Absetzen durch und fängt Hindernisse.
DEFAULT_MARGIN_MM = 5.0
# Im Sechsbeinstand liegen die Beine bei 12 bis 27 % Federweg, das Rauschen
# bei rund 2 %. 5 % trennt sauber zwischen "berührt" und "trägt".
DEFAULT_TOUCH_LEVEL = 0.05


def make_contact_freeze(
    robot: Hexapod,
    *,
    touch_level: float = DEFAULT_TOUCH_LEVEL,
    margin_mm: float = DEFAULT_MARGIN_MM,
    legs: Iterable[str] | None = None,
    treffer: dict[str, float] | None = None,
) -> Callable[[str], bool] | None:
    """Baut die ``freeze``-Funktion für den Executor.

    Args:
        robot: Hexapod-Instanz.
        touch_level: Ab diesem Federweg gilt der Fuß als aufgesetzt.
        margin_mm: Erst oberhalb dieser Höhe über der Standpose zählt der
            Kontakt als "zu früh". Siehe Modul-Docstring.
        legs: Nur diese Beine prüfen (z.B. die schwingende Tripod-Gruppe).
            None = alle mit Sensor.
        treffer: Wird, wenn angegeben, mit Bein -> Höhe über der Standpose
            gefüllt, sobald ein Kontakt greift. So erfährt der Aufrufer, wo
            der Boden tatsächlich lag — die Geländehöhe an dieser Stelle.

    Returns:
        Die Funktion, oder None wenn keine Sensoren nutzbar sind. Damit kann
        der Aufrufer sie bedenkenlos durchreichen.
    """
    sensors = robot.foot_sensors
    if sensors is None:
        return None
    erlaubt = set(legs) if legs is not None else set(sensors.legs)
    erlaubt &= set(sensors.legs)
    if not erlaubt:
        return None

    def freeze(leg: str) -> bool:
        if leg not in erlaubt:
            return False
        hoehe = robot.current_offset(leg)[2]
        # Nahe an der Standpose ist Kontakt normal -- dort soll sich das Bein
        # sauber einfedern statt anzuhalten.
        if hoehe <= margin_mm:
            return False
        messwert = sensors.read(leg, samples=1)
        if messwert.level is None or messwert.level < touch_level:
            return False
        if treffer is not None:
            treffer[leg] = hoehe
        return True

    return freeze
