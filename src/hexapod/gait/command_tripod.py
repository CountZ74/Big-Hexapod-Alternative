"""Verallgemeinerter Tripod-Schritt aus einem Bewegungsbefehl.

W\u00e4hrend der klassische :func:`hexapod.gait.tripod.half_cycle_paths` fest
in K\u00f6rper-X l\u00e4uft (ein globaler ``stride`` mal ``direction``), nimmt der
verallgemeinerte Schritt pro Bein einen eigenen Boden-Bewegungsvektor
``(dx, dy)`` aus :func:`hexapod.gait.body_motion.stride_vectors`. Damit
f\u00e4llt jede Fortbewegung \u2014 vorw\u00e4rts, seitw\u00e4rts, drehen und beliebige
Mischungen (Kurvenfahrt) \u2014 aus derselben Mechanik heraus.

Geometrie pro Halbzyklus, zentriert um die Standpose:

    Stand-Gruppe:  F\u00fc\u00dfe am Boden, laufen entlang ihres Stride-Vektors
                   von -v/2 nach +v/2 (schieben den K\u00f6rper).
    Schwung-Gruppe: F\u00fc\u00dfe in der Luft, laufen entgegengesetzt von
                   +v/2 nach -v/2 zur\u00fcck (Sinus-Hub in Z).

\u00dcber einen vollen Zyklus (zwei Halbzyklen, Rollentausch) hebt sich der
Weg jedes Fu\u00dfes exakt auf \u2014 er kehrt zur Standpose zur\u00fcck.

Die Bahn-Erzeugung ist eine reine Funktion (keine Zeit, keine Servos):
so isoliert testbar. Die zeitliche Ausf\u00fchrung macht der Executor.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

from hexapod.gait.body_motion import Vec2, clamp_command, stride_vectors
from hexapod.gait.executor import run_multi_leg_trajectory
from hexapod.gait.trajectory import Vec3, smoothstep, stance_path, swing_path
from hexapod.gait.tripod import GROUP_A, GROUP_B

if TYPE_CHECKING:
    from hexapod.robot.hexapod import Hexapod


def command_half_cycle_paths(
    swing_group: Sequence[str],
    stance_group: Sequence[str],
    strides: dict[str, Vec2],
    *,
    height: float,
    steps: int,
    include_start: bool = False,
) -> dict[str, list[Vec3]]:
    """Fu\u00df-Offsets f\u00fcr einen Halbzyklus aus Stride-Vektoren.

    Args:
        swing_group: Beine in der Schwungphase (Luft).
        stance_group: Beine in der Standphase (Boden).
        strides: Bein-Name \u2192 (dx, dy) voller Boden-Bewegungsvektor pro Schritt.
        height: Schwung-Hubh\u00f6he in mm.
        steps: Punkte pro Halbzyklus.
        include_start: Startpunkt mit ausgeben (siehe trajectory-Funktionen).

    Returns:
        Bein-Name \u2192 Punktliste (alle gleich lang).
    """
    paths: dict[str, list[Vec3]] = {}

    for leg in swing_group:
        dx, dy = strides[leg]
        hx, hy = dx / 2.0, dy / 2.0
        # Schwung: von +v/2 zur\u00fcck nach -v/2, angehoben
        paths[leg] = swing_path(
            (hx, hy, 0.0), (-hx, -hy, 0.0), height, steps,
            include_start=include_start, ease=smoothstep,
        )

    for leg in stance_group:
        dx, dy = strides[leg]
        hx, hy = dx / 2.0, dy / 2.0
        # Stand: von -v/2 nach +v/2 entlang des Stride-Vektors, am Boden
        paths[leg] = stance_path(
            (-hx, -hy, 0.0), (hx, hy, 0.0), steps,
            include_start=include_start, ease=smoothstep,
        )

    return paths


def walk_command(
    robot: Hexapod,
    vx: float,
    vy: float,
    omega: float,
    *,
    cycles: int = 3,
    height: float = 30.0,
    steps: int = 30,
    rate_hz: float = 40.0,
    max_step_deg: float = 3.0,
    max_translation: float = 50.0,
    max_rotation: float = 0.30,
    clip: bool = True,
) -> tuple[float, float, float]:
    """Laufe ``cycles`` volle Tripod-Zyklen mit einem Bewegungsbefehl.

    Der Befehl (vx, vy, omega) wird zun\u00e4chst auf sichere Schrittweiten
    begrenzt, dann in Stride-Vektoren je Bein umgerechnet. Ein voller Zyklus
    sind zwei Halbzyklen (A schwingt, dann B schwingt). Vor dem ersten
    Schritt sollten die Beine in der Standpose stehen.

    Args:
        robot: Hexapod-Instanz.
        vx: Translation vorw\u00e4rts(+)/r\u00fcckw\u00e4rts(\u2212) pro Schritt (mm).
        vy: Translation links(+)/rechts(\u2212) pro Schritt (mm).
        omega: Drehung CCW(+) pro Schritt (rad).
        cycles: Anzahl voller Zyklen.
        height: Schwung-Hubh\u00f6he (mm).
        steps: Punkte pro Halbzyklus.
        rate_hz: Sende-Frequenz.
        max_step_deg: Max. Gelenksprung pro Takt (adaptive Unterteilung).
        max_translation: Begrenzung Translation pro Schritt (mm).
        max_rotation: Begrenzung Rotation pro Schritt (rad).
        clip: Winkel-Clipping.

    Returns:
        Der tats\u00e4chlich gelaufene, begrenzte (vx, vy, omega).
    """
    feet = robot.neutral_foot_xy
    foot_radius = max((fx * fx + fy * fy) ** 0.5 for fx, fy in feet.values())

    cvx, cvy, comega = clamp_command(
        vx, vy, omega,
        max_translation=max_translation,
        max_rotation=max_rotation,
        foot_radius=foot_radius,
    )
    strides = stride_vectors(feet, cvx, cvy, comega)

    for _ in range(cycles):
        # Halbzyklus 1: A schwingt, B steht.
        paths = command_half_cycle_paths(
            GROUP_A, GROUP_B, strides,
            height=height, steps=steps, include_start=True,
        )
        run_multi_leg_trajectory(
            robot, paths, rate_hz=rate_hz, max_step_deg=max_step_deg, clip=clip
        )

        # Halbzyklus 2: B schwingt, A steht. Ersten Punkt weglassen (identisch
        # mit dem letzten Punkt von H1 \u2192 kontinuierlicher Bodenkontakt).
        paths = command_half_cycle_paths(
            GROUP_B, GROUP_A, strides,
            height=height, steps=steps, include_start=True,
        )
        paths = {leg: pts[1:] for leg, pts in paths.items()}
        run_multi_leg_trajectory(
            robot, paths, rate_hz=rate_hz, max_step_deg=max_step_deg, clip=clip
        )

    return (cvx, cvy, comega)
