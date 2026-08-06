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

# Fuer settle_to_stance: dort soll sich das Bein in eine DEFINIERTE Standpose
# druecken, deshalb zaehlt Kontakt erst oberhalb dieser Hoehe als zu frueh.
# Auf ebenem Boden lag der erste Kontakt dort bis zu 4 mm ueber der Standpose.
#
# Im GANG ist das anders: da haengt eine ganze Tripod-Gruppe in der Luft, der
# Koerper sackt weiter nach, und der erste Kontakt liegt entsprechend hoeher.
# Eine feste Grenze taugt dort nicht -- siehe make_contact_freeze.
DEFAULT_MARGIN_MM = 5.0

# Im Gang liegt der erste Kontakt deutlich hoeher. Waehrend eine
# Tripod-Gruppe schwingt, tragen nur drei Beine -- der Koerper sackt ab, und
# der Fuss findet den Boden entsprechend frueher.
#
# Gemessen am Roboter auf ebenem Boden (walk --touch 5): die Beine hielten
# bei rund 6,2 mm ueber der Standpose an. Die 5 mm lagen mitten in diesem
# Bereich, deshalb hielt mal das eine, mal das andere Bein an. 12 mm laesst
# normales Absetzen sicher durch; Stufen darunter fangen ohnehin die Federn
# ab, die knapp 6 mm Weg haben.
DEFAULT_WALK_MARGIN_MM = 12.0
# Im Sechsbeinstand liegen die Beine bei 12 bis 27 % Federweg, das Rauschen
# bei rund 2 %. 5 % trennt sauber zwischen "berührt" und "trägt".
DEFAULT_TOUCH_LEVEL = 0.05


def make_contact_freeze(
    robot: Hexapod,
    *,
    touch_level: float = DEFAULT_TOUCH_LEVEL,
    margin_mm: float = 0.0,
    legs: Iterable[str] | None = None,
    treffer: dict[str, float] | None = None,
) -> Callable[[str], bool] | None:
    """Baut die ``freeze``-Funktion für den Executor.

    Args:
        robot: Hexapod-Instanz.
        touch_level: Ab diesem Federweg gilt der Fuß als aufgesetzt.
        margin_mm: Erst oberhalb dieser Höhe über der Standpose wird
            angehalten. 0 heißt: bei Bodenkontakt sofort halten — im Gang das
            Richtige, denn dort SOLL der Fuß dort landen, wo der Boden ist,
            und die Last baut sich anschließend von selbst auf, wenn die
            vorherige Standgruppe abhebt. Nur `settle_to_stance` braucht
            einen Wert, weil es eine definierte Standpose herstellen will.
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

    # Ein Bein zaehlt erst als "kann aufsetzen", wenn es zwischendurch frei
    # war. Ohne das friert es schon beim ABHEBEN ein: der Fuss verlaesst den
    # Boden nicht in dem Moment, in dem die Bahn steigt -- erst entspannt sich
    # die Feder ueber ihre paar Millimeter, und der Koerper sinkt dabei nach.
    # Das Bein ist also noch belastet, waehrend sein Offset die Mindesthoehe
    # schon ueberschritten hat.
    war_frei: dict[str, bool] = dict.fromkeys(erlaubt, False)

    def freeze(leg: str) -> bool:
        if leg not in erlaubt:
            return False
        messwert = sensors.read(leg, samples=1)
        if messwert.level is None:
            return False

        if messwert.level < touch_level:
            war_frei[leg] = True
            return False
        if not war_frei[leg]:
            return False   # haengt noch am Boden vom Losfahren

        hoehe = robot.current_offset(leg)[2]
        # Nahe an der Standpose ist Kontakt normal -- dort soll sich das Bein
        # sauber einfedern statt anzuhalten.
        if hoehe <= margin_mm:
            return False
        if treffer is not None:
            treffer[leg] = hoehe
        return True

    return freeze
