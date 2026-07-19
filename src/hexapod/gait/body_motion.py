"""Verallgemeinerte Körperbewegung → Fuß-Bewegungsvektoren.

Jede Fortbewegung (vorwärts, seitwärts, drehen und beliebige Mischungen)
wird durch einen Bewegungsbefehl beschrieben:

    vx: Translation vorwärts (+) / rückwärts (−)   [mm pro vollem Schritt]
    vy: Translation links (+) / rechts (−)          [mm pro vollem Schritt]
    omega: Drehung gegen den Uhrzeigersinn (+)      [rad pro vollem Schritt]

Während der Standphase ist der Fuß am Boden fixiert und der Körper bewegt
sich. Der Fuß muss sich relativ zum Körper also GENAU ENTGEGENGESETZT zur
gewünschten Körperbewegung bewegen. Aus der Starrkörpergleichung ergibt
sich der Fuß-Bewegungsvektor an Position r = (rx, ry) relativ zum Zentrum:

    v_fuß = -( v_translation + omega × r )
          = -( vx - omega*ry,  vy + omega*rx )

Das Minuszeichen, weil der Fuß den Körper in die Gegenrichtung schiebt.
Reine Translation → alle Füße parallel. Reine Rotation → Füße auf
Kreisbögen ums Zentrum. Mischung → Kurvenfahrt. Alles aus einer Formel.
"""

from __future__ import annotations

import math

Vec2 = tuple[float, float]


def foot_ground_vector(
    foot_x: float,
    foot_y: float,
    vx: float,
    vy: float,
    omega: float,
) -> Vec2:
    """Bewegungsvektor eines Fußes während der Standphase (Boden).

    Args:
        foot_x, foot_y: Fußposition relativ zum Körperzentrum (mm).
        vx, vy: gewünschte Körper-Translation pro Schritt (mm).
        omega: gewünschte Körper-Drehung pro Schritt (rad, CCW positiv).

    Returns:
        (dx, dy): Bewegung des Fußes am Boden pro vollem Schritt (mm).
        Der Fuß bewegt sich entgegengesetzt zur Körperbewegung.
    """
    # Körperbewegung am Ort des Fußes: Translation + Rotation (omega × r)
    body_dx = vx - omega * foot_y
    body_dy = vy + omega * foot_x
    # Fuß bewegt sich entgegengesetzt (schiebt den Körper vorwärts)
    return (-body_dx, -body_dy)


def stride_vectors(
    foot_positions: dict[str, Vec2],
    vx: float,
    vy: float,
    omega: float,
) -> dict[str, Vec2]:
    """Stand-Bewegungsvektor für jeden Fuß aus einem Bewegungsbefehl.

    Args:
        foot_positions: Bein-Name → Fußposition relativ zum Zentrum (mm).
        vx, vy, omega: Bewegungsbefehl (siehe foot_ground_vector).

    Returns:
        Bein-Name → (dx, dy) Stand-Bewegungsvektor.
    """
    return {
        name: foot_ground_vector(fx, fy, vx, vy, omega)
        for name, (fx, fy) in foot_positions.items()
    }


def clamp_command(
    vx: float,
    vy: float,
    omega: float,
    *,
    max_translation: float,
    max_rotation: float,
    foot_radius: float,
) -> tuple[float, float, float]:
    """Begrenzt einen Bewegungsbefehl auf sichere Schrittweiten.

    Translation und Rotation erzeugen gemeinsam den Fußweg. Der weiteste
    Fuß (foot_radius) bestimmt, wie viel Rotation noch in die Schrittweite
    passt. Wird der kombinierte Fußweg zu groß, wird der ganze Befehl
    proportional herunterskaliert, sodass Richtung und Verhältnis erhalten
    bleiben.

    Args:
        vx, vy, omega: roher Bewegungsbefehl.
        max_translation: maximale Translation pro Schritt (mm).
        max_rotation: maximale Rotation pro Schritt (rad).
        foot_radius: Abstand des äußersten Fußes vom Zentrum (mm).

    Returns:
        Begrenzter (vx, vy, omega).
    """
    # Translationsbetrag
    trans = math.hypot(vx, vy)

    scale = 1.0
    if trans > max_translation and trans > 0:
        scale = min(scale, max_translation / trans)
    if abs(omega) > max_rotation and omega != 0:
        scale = min(scale, max_rotation / abs(omega))

    return (vx * scale, vy * scale, omega * scale)
