"""Auswertung der Fußsensor-Lastverteilung für den Bein-Höhenabgleich.

Der Roboter ist ein starrer Körper auf sechs Federn. Für kleine
Auslenkungen ist die Einfederung an jedem Fuß deshalb exakt

    federweg_i  =  z0  +  a * x_i  +  b * y_i  +  e_i

Der affine Anteil ``z0 + a*x + b*y`` fasst alles zusammen, was den Körper
als Ganzes absenkt oder kippt: die Schwerpunktlage, eine Neigung des
Untergrunds, eine ungleiche Zuladung. Der Rest ``e_i`` ist der Fehler des
einzelnen Beins — und nur der gehört in ``z_trim``.

Daraus folgt das ganze Verfahren: eine Ebene durch die sechs Messwerte über
den Fußpositionen legen, und die **Residuen** sind die Bein-Einzelfehler.
Schwerpunkt und Bodenneigung fallen dabei mathematisch heraus, weil beide
nur eine Ebene erzeugen können.

Das ist auch die Antwort auf die naheliegende, aber falsche Idee, einfach
auf gleiche Last zu trimmen: mit einem Schwerpunkt vorne im Roboter *soll*
vorne mehr Last liegen. Dieser Anteil steckt im ``a*x``-Term und bleibt dort.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np

# Kleinste Anzahl Beine, aus der sich eine Ebene ueberhaupt bestimmen laesst.
# Bei genau 3 waere der Fit exakt und alle Residuen null -- dann ist die
# Methode blind. Erst ab 4 traegt sie Information.
MIN_LEGS_FOR_PLANE = 4


@dataclass(frozen=True)
class BalanceResult:
    """Ergebnis einer Lastauswertung."""

    residuals: dict[str, float]
    """Bein-Einzelfehler im Federweg (positiv = Bein traegt zu viel)."""
    plane: tuple[float, float, float]
    """(z0, a, b) der gefitteten Ebene — Schwerpunkt- und Neigungsanteil."""
    levels: dict[str, float]
    """Die Eingangs-Federwege, unveraendert."""

    @property
    def worst_leg(self) -> str:
        """Bein mit dem groessten Betrag im Residuum."""
        return max(self.residuals, key=lambda leg: abs(self.residuals[leg]))

    @property
    def max_abs_residual(self) -> float:
        """Groesster Betrag unter den Residuen — das Konvergenzmass."""
        return max(abs(v) for v in self.residuals.values())

    @property
    def spread(self) -> float:
        """Abstand zwischen groesstem und kleinstem Residuum."""
        values = self.residuals.values()
        return max(values) - min(values)


def fit_load_plane(
    levels: Mapping[str, float],
    positions: Mapping[str, tuple[float, float]],
) -> BalanceResult:
    """Ebene durch die Lastwerte legen und die Residuen bestimmen.

    Args:
        levels: Bein -> gemessener Federweg (0..1).
        positions: Bein -> (x, y) des Fußes im Körperframe, in mm.

    Raises:
        ValueError: bei zu wenigen Beinen oder fehlender Position.
    """
    legs = [leg for leg in levels if leg in positions]
    if len(legs) < MIN_LEGS_FOR_PLANE:
        raise ValueError(
            f"Mindestens {MIN_LEGS_FOR_PLANE} Beine noetig, um Schwerpunkt und "
            f"Bein-Einzelfehler zu trennen — bekommen: {len(legs)}."
        )

    a_matrix = np.array([[1.0, positions[leg][0], positions[leg][1]] for leg in legs])
    b_vector = np.array([levels[leg] for leg in legs])
    coeffs, *_ = np.linalg.lstsq(a_matrix, b_vector, rcond=None)
    fitted = a_matrix @ coeffs

    return BalanceResult(
        residuals={leg: float(b_vector[i] - fitted[i]) for i, leg in enumerate(legs)},
        plane=(float(coeffs[0]), float(coeffs[1]), float(coeffs[2])),
        levels={leg: float(levels[leg]) for leg in legs},
    )


def z_trim_corrections(
    residuals: Mapping[str, float],
    *,
    travel_mm: float,
    damping: float = 0.8,
    deadband: float = 0.0,
) -> dict[str, float]:
    """Residuen in z_trim-Korrekturen umrechnen (in mm, mittelwertfrei).

    Vorzeichen: ``z_trim`` positiv bedeutet Fuß tiefer, also mehr Last. Ein
    Bein mit positivem Residuum traegt zu viel und braucht folglich eine
    negative Korrektur.

    Mittelwertfrei, weil eine gemeinsame Verschiebung aller sechs Beine den
    Koerper nur anhebt oder absenkt — dafuer ist ``stance_z`` zustaendig,
    nicht ``z_trim``. Ohne diese Normierung wuerde die Schleife den Roboter
    langsam nach oben oder unten wandern lassen.

    Args:
        residuals: Bein -> Residuum im Federweg.
        travel_mm: Mechanischer Vollweg der Schubstange in mm. Rechnet
            Federweg-Anteile in Millimeter um.
        damping: Anteil der berechneten Korrektur, der angewendet wird.
            Unter 1.0, damit die Schleife nicht ueberschwingt. Der Ebenen-Fit
            daempft ohnehin schon: er kippt dem Ausreisser hinterher, sodass
            ein Einzelfehler nur etwa zur Haelfte im Residuum landet. Deshalb
            genuegt hier ein milder Wert -- die Schleife naehert sich in
            wenigen Runden an, statt zu schwingen.
        deadband: Residuen mit kleinerem Betrag bleiben unkorrigiert.
            Unterhalb der Messwiederholbarkeit korrigiert man nur noch
            Rauschen -- und weil jede Korrektur die Lastverteilung erneut
            leicht verwuerfelt, wandert z_trim dann zufaellig weiter, statt
            sich zu beruhigen. Beobachtet wurde eine Wiederholbarkeit von
            rund 2 % Federweg.

    Hinweis: Beine im Totband bekommen trotzdem den Ausgleichsanteil aus der
    Mittelwertfreiheit ab. Das ist gewollt -- die Summe muss null bleiben,
    sonst wandert die Koerperhoehe.
    """
    if travel_mm <= 0.0:
        raise ValueError(f"travel_mm muss positiv sein, war {travel_mm}")
    if not 0.0 < damping <= 1.0:
        raise ValueError(f"damping muss in (0, 1] liegen, war {damping}")
    if deadband < 0.0:
        raise ValueError(f"deadband darf nicht negativ sein, war {deadband}")

    raw = {
        leg: (0.0 if abs(r) < deadband else -r * travel_mm * damping)
        for leg, r in residuals.items()
    }
    offset = sum(raw.values()) / len(raw)
    return {leg: value - offset for leg, value in raw.items()}


def estimate_travel_mm(
    applied_mm: Mapping[str, float],
    level_before: Mapping[str, float],
    level_after: Mapping[str, float],
    *,
    min_legs: int = 3,
    min_span_mm: float = 0.3,
) -> float | None:
    """Aus einer angewendeten Korrektur den Federweg in mm zurueckrechnen.

    Besser ist, den Vollweg einmal zu messen und in die robot.yaml zu
    schreiben (``foot_sensors.travel_mm``). Diese Schaetzung ist nur der
    Notnagel, wenn er dort fehlt.

    Gerechnet wird als **Regression der Reaktion ueber die Korrektur**, nicht
    als Median von Quotienten. Die angewendete Korrektur ist exakt bekannt,
    die gemessene Reaktion verrauscht — ein Quotient mit fast leerem Nenner
    sprengt die Schaetzung sonst um Groessenordnungen. Genau das ist einmal
    passiert: aus einem echten Vollweg von 5,5 mm wurden 15 mm, und die
    Schleife hat daraufhin dreifach ueberkorrigiert.

    Returns:
        Geschaetzter Vollweg in mm, oder None wenn die Anregung zu klein war
        oder die Reaktion nicht zur Korrektur passt.
    """
    dz: list[float] = []
    dl: list[float] = []
    for leg, z in applied_mm.items():
        if leg in level_before and leg in level_after:
            dz.append(z)
            dl.append(level_after[leg] - level_before[leg])

    if len(dz) < min_legs:
        return None
    if max(dz) - min(dz) < min_span_mm:
        return None  # zu wenig angeregt, um etwas zu unterscheiden

    nenner = sum(z * lvl for z, lvl in zip(dz, dl, strict=True))
    if nenner <= 0.0:
        return None  # keine oder gegenlaeufige Reaktion -> nichts ableitbar
    return sum(z * z for z in dz) / nenner

def tilt_corrections(
    roll_rad: float,
    pitch_rad: float,
    positions: Mapping[str, tuple[float, float]],
) -> dict[str, float]:
    """z_trim-Korrekturen, die eine gemessene Koerperneigung ausgleichen (mm).

    Das ist der Anteil, den die Lastsensoren **prinzipiell nicht sehen
    koennen**: Das Residuum ist ``(I - H)·fehler`` mit der Hut-Matrix des
    Ebenen-Fits, und diese Projektion loescht genau die Fehlerkomponente,
    die selbst wie eine Ebene aussieht. Sind zum Beispiel alle drei linken
    Beine gleichmaessig zu lang, ist das von einem seitlich abfallenden
    Fussboden nicht zu unterscheiden — die Last verteilt sich in beiden
    Faellen identisch.

    Auf einem Boden, von dem man weiss, dass er waagerecht ist, liefert die
    IMU genau diese fehlende Information. Beide Verfahren zusammen bestimmen
    damit alle sechs Beinfehler; einzeln kann es keines von beiden.

    Vorzeichen (Konvention wie in ``rotation_matrix`` und der
    Selbstnivellierung): Fuer kleine Winkel liegt ein Koerperpunkt (x, y) um
    ``roll*y - pitch*x`` hoeher. Wo der Koerper zu hoch steht, muss der Fuss
    kuerzer werden, also ``z_trim`` kleiner.

    Args:
        roll_rad: Gemessene Seitneigung (Drehung um X) in Radiant.
        pitch_rad: Gemessenes Nicken (Drehung um Y) in Radiant.
        positions: Bein -> (x, y) des Fusses im Koerperframe, in mm.

    Returns:
        Bein -> Korrektur in mm, mittelwertfrei.
    """
    raw = {
        leg: pitch_rad * x - roll_rad * y
        for leg, (x, y) in positions.items()
    }
    offset = sum(raw.values()) / len(raw)
    return {leg: value - offset for leg, value in raw.items()}
