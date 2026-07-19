"""Tripod-Gangart für den Hexapod.

Zwei Beingruppen laufen gegenphasig:
    Gruppe A: front_right, mid_left,  back_right
    Gruppe B: front_left,  mid_right, back_left

In jedem Halbzyklus schwingt eine Gruppe durch die Luft nach vorne
(swing_path), während die andere am Boden nach hinten schiebt (stance_path).
Danach tauschen die Rollen. Die Beingruppen sind so gewählt, dass der
Roboter immer auf einem stabilen Dreieck steht.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

from hexapod.gait.executor import run_multi_leg_trajectory
from hexapod.gait.trajectory import Vec3, stance_path, swing_path

if TYPE_CHECKING:
    from hexapod.robot.hexapod import Hexapod

GROUP_A = ("front_right", "mid_left", "back_right")
GROUP_B = ("front_left", "mid_right", "back_left")


def half_cycle_paths(
    swing_group: Sequence[str],
    stance_group: Sequence[str],
    *,
    stride: float,
    height: float,
    steps: int,
    direction: float = 1.0,
    include_start: bool = False,
) -> dict[str, list[Vec3]]:
    """Erzeuge die Fuß-Offsets für einen Halbzyklus.

    Die Schwung-Gruppe geht von -stride/2*dir nach +stride/2*dir (durch die
    Luft), die Stand-Gruppe von +stride/2*dir nach -stride/2*dir (am Boden).

    Args:
        swing_group: Beine in der Schwungphase.
        stance_group: Beine in der Standphase.
        stride: Schrittlänge in mm (gesamter Weg vorne↔hinten).
        height: Schwung-Hubhöhe in mm.
        steps: Punkte pro Halbzyklus.
        direction: +1 = vorwärts (Körper-X), -1 = rückwärts.

    Returns:
        Abbildung Bein-Name → Punktliste (alle gleich lang).
    """
    a = stride / 2.0 * direction
    paths: dict[str, list[Vec3]] = {}

    for leg in swing_group:
        # Schwung: von hinten (-a) nach vorne (+a), angehoben
        paths[leg] = swing_path(
            (-a, 0.0, 0.0), (a, 0.0, 0.0), height, steps, include_start=include_start
        )

    for leg in stance_group:
        # Stand: von vorne (+a) nach hinten (-a), am Boden
        paths[leg] = stance_path(
            (a, 0.0, 0.0), (-a, 0.0, 0.0), steps, include_start=include_start
        )

    return paths


def walk(
    robot: Hexapod,
    *,
    cycles: int = 3,
    stride: float = 50.0,
    height: float = 30.0,
    steps: int = 30,
    rate_hz: float = 40.0,
    max_step_deg: float = 3.0,
    direction: float = 1.0,
    clip: bool = True,
) -> None:
    """Laufe `cycles` volle Tripod-Zyklen.

    Ein voller Zyklus = zwei Halbzyklen (A schwingt, dann B schwingt).
    Vor dem ersten Schritt sollten die Beine in der Standpose stehen.

    Args:
        robot: Hexapod-Instanz.
        cycles: Anzahl voller Zyklen.
        stride: Schrittlänge in mm.
        height: Schwung-Hubhöhe in mm.
        steps: Punkte pro Halbzyklus.
        rate_hz: Sende-Frequenz.
        max_step_deg: Max. Gelenksprung pro Takt (adaptive Unterteilung).
        direction: +1 vorwärts, -1 rückwärts.
        clip: Winkel-Clipping.
    """
    for _ in range(cycles):
        # Halbzyklus 1: A schwingt, B steht. include_start fuer stetigen
        # Boden-Kontaktpunkt; danach Duplikat am Uebergang ueberspringen.
        paths = half_cycle_paths(
            GROUP_A, GROUP_B,
            stride=stride, height=height, steps=steps, direction=direction,
            include_start=True,
        )
        run_multi_leg_trajectory(
            robot, paths, rate_hz=rate_hz, max_step_deg=max_step_deg, clip=clip
        )

        # Halbzyklus 2: B schwingt, A steht. Ersten Punkt weglassen, da er
        # mit dem letzten Punkt von H1 identisch ist (kontinuierlicher Boden).
        paths = half_cycle_paths(
            GROUP_B, GROUP_A,
            stride=stride, height=height, steps=steps, direction=direction,
            include_start=True,
        )
        paths = {leg: pts[1:] for leg, pts in paths.items()}
        run_multi_leg_trajectory(
            robot, paths, rate_hz=rate_hz, max_step_deg=max_step_deg, clip=clip
        )
