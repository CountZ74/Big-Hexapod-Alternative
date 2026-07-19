"""Vorwärts- und Rückwärtskinematik für ein 3-DOF-Hexapod-Bein.

Konventionen (siehe Architektur-Doku):
    * Leg Frame:
        - Ursprung = Coxa-Drehachse
        - +X zeigt nach außen, wenn θ1 = 0 (Bein "gerade nach vorne")
        - +Y zeigt seitlich, mit Coxa-Drehrichtung
        - +Z zeigt nach oben
    * Winkel sind in Radiant:
        - θ1 (coxa):  Drehung um Z-Achse. θ1 = 0  ⇒ Bein zeigt entlang +X.
        - θ2 (femur): Hebung des Femurs gegenüber horizontal.
                      θ2 > 0  ⇒ Femur zeigt nach oben.
        - θ3 (tibia): Beugung des Tibia *relativ zum Femur*.
                      θ3 = 0  ⇒ Bein gestreckt (Tibia in Femur-Verlängerung).
                      θ3 > 0  ⇒ Knie beugt sich, Fußspitze nach unten/zum Körper.

Alle Längen sind in Millimetern, Positionen ebenfalls.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# ---------------------------------------------------------------------
# Datenklassen und Fehler
# ---------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class LegLengths:
    """Die drei Segmentlängen eines Beins in mm.

    Mit frozen=True ist die Klasse unveränderlich (immutable), was sie
    sicher als Funktions-Argument und in Caches macht.
    """

    coxa: float
    femur: float
    tibia: float

    def __post_init__(self) -> None:
        for name, value in (("coxa", self.coxa), ("femur", self.femur), ("tibia", self.tibia)):
            if value <= 0:
                raise ValueError(f"Bein-Länge '{name}' muss positiv sein, war {value}")


class UnreachableError(ValueError):
    """Der angeforderte Fußpunkt liegt außerhalb des erreichbaren Bereichs."""


# ---------------------------------------------------------------------
# Forward Kinematics
# ---------------------------------------------------------------------


def forward_kinematics(
    theta1: float,
    theta2: float,
    theta3: float,
    lengths: LegLengths,
) -> tuple[float, float, float]:
    """Berechnet die Fußspitze aus den drei Gelenkwinkeln.

    Args:
        theta1: Coxa-Winkel [rad].
        theta2: Femur-Winkel [rad], gemessen ab horizontal.
        theta3: Tibia-Winkel [rad], relativ zum Femur.
        lengths: Bein-Segmentlängen in mm.

    Returns:
        (x, y, z) der Fußspitze im Leg Frame in mm.
    """
    l1, l2, l3 = lengths.coxa, lengths.femur, lengths.tibia

    # In der Bein-Ebene: horizontale Distanz r und Höhe z.
    # Der Tibia-Winkel ist *relativ zum Femur*, daher ist der absolute Tibia-
    # Winkel in der Ebene (theta2 + theta3) gegenüber horizontal — aber mit
    # einem Vorzeichen-Trick: wenn theta3 > 0 (Knie gebeugt nach unten), zeigt
    # die Tibia steiler nach unten, also subtrahieren wir theta3:
    #     absoluter Tibia-Winkel = theta2 - theta3
    # Diese Konvention macht θ3 in der gestreckten Position null und positiv,
    # wenn das Bein nach unten greift.
    r = l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 - theta3)
    z = l2 * math.sin(theta2) + l3 * math.sin(theta2 - theta3)

    # Coxa-Rotation um Z-Achse: Ebene (r) auf 3D (x, y) abbilden.
    x = r * math.cos(theta1)
    y = r * math.sin(theta1)

    return (x, y, z)


# ---------------------------------------------------------------------
# Inverse Kinematics
# ---------------------------------------------------------------------


# Numerische Toleranz für "Punkt liegt exakt auf dem Reichweiten-Rand".
# Floating-Point ist nicht exakt: ein theoretisch gerade noch erreichbarer
# Punkt wird in der Praxis durch winzige Rundungsfehler "unerreichbar".
# Mit diesem Slack akzeptieren wir Punkte, die maximal _REACH_EPS mm außerhalb
# liegen, und clippen sie auf den Rand.
_REACH_EPS = 1e-6


def inverse_kinematics(
    x: float,
    y: float,
    z: float,
    lengths: LegLengths,
) -> tuple[float, float, float]:
    """Berechnet die Gelenkwinkel für einen Fußpunkt im Leg Frame.

    Es gibt prinzipiell ZWEI Lösungen für das Femur-Tibia-Dreieck:
    "Knie oben" und "Knie unten". Für einen Hexapod ist die natürliche
    Wahl "Knie über dem Fußpunkt" (das Bein ähnelt einem umgedrehten
    Spinnen-Bein). Diese Funktion liefert genau diese Lösung.

    Args:
        x, y, z: Zielposition der Fußspitze im Leg Frame in mm.
        lengths: Bein-Segmentlängen in mm.

    Returns:
        (theta1, theta2, theta3) in Radiant.

    Raises:
        UnreachableError: Wenn (x, y, z) außerhalb der Reichweite liegt.
    """
    l1, l2, l3 = lengths.coxa, lengths.femur, lengths.tibia

    # --- Schritt A: Coxa-Winkel ---
    # atan2 liefert den korrekten Winkel in allen vier Quadranten.
    theta1 = math.atan2(y, x)

    # --- Schritt B: Reduktion auf die Bein-Ebene ---
    # Horizontale Distanz vom Coxa-Drehpunkt zum Fußpunkt:
    r = math.hypot(x, y)
    # Effektive Distanz, die Femur+Tibia überbrücken müssen:
    r_eff = r - l1

    # Direkte Distanz vom Femur-Gelenk zum Fußpunkt:
    d = math.hypot(r_eff, z)

    # --- Reichweiten-Check ---
    reach_max = l2 + l3
    reach_min = abs(l2 - l3)
    if d > reach_max + _REACH_EPS:
        raise UnreachableError(
            f"Fußpunkt ({x:.1f}, {y:.1f}, {z:.1f}) zu weit weg: "
            f"d={d:.2f}, max={reach_max:.2f}"
        )
    if d < reach_min - _REACH_EPS:
        raise UnreachableError(
            f"Fußpunkt ({x:.1f}, {y:.1f}, {z:.1f}) zu nah am Coxa: "
            f"d={d:.2f}, min={reach_min:.2f}"
        )

    # Float-Slack: Werte minimal außerhalb auf den Rand klemmen.
    d = max(reach_min, min(reach_max, d))

    # --- Schritt C: Femur- und Tibia-Winkel mit Kosinus-Satz ---
    # Innenwinkel am Knie (zwischen Femur und Tibia):
    #     cos(γ) = (L2² + L3² − D²) / (2·L2·L3)
    cos_gamma = (l2 * l2 + l3 * l3 - d * d) / (2.0 * l2 * l3)
    cos_gamma = max(-1.0, min(1.0, cos_gamma))  # Float-Slack
    gamma = math.acos(cos_gamma)
    theta3 = math.pi - gamma

    # Innenwinkel am Femur-Gelenk im Dreieck (zwischen Femur und der
    # gedachten Verbindungslinie D zum Fußpunkt):
    #     cos(β) = (L2² + D² − L3²) / (2·L2·D)
    cos_beta = (l2 * l2 + d * d - l3 * l3) / (2.0 * l2 * d)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)

    # Höhenwinkel zum Fußpunkt von der Femur-Achse aus:
    alpha = math.atan2(z, r_eff)

    # Der Femur-Winkel ist die Summe: erst zum Fußpunkt zielen (α),
    # dann den Femur über die Verbindungslinie hochheben (β), weil das
    # Knie gebeugt ist und die Tibia "ergänzt".
    theta2 = alpha + beta

    return (theta1, theta2, theta3)
