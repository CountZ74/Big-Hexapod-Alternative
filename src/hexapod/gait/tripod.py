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

import statistics
from collections.abc import Callable, Sequence
from typing import TYPE_CHECKING

from hexapod.gait.contact import DEFAULT_WALK_MARGIN_MM, make_contact_freeze
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
    touch_level: float | None = None,
    touch_margin_mm: float = DEFAULT_WALK_MARGIN_MM,
) -> dict[str, float]:
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
        touch_level: Aufsetz-Erkennung. Ist ein Wert gesetzt, hält ein
            Schwungbein an, sobald es früher Boden findet als geplant —
            die beiden anderen der Gruppe laufen weiter. Ohne Wert bleibt
            das Verhalten unverändert.
        touch_margin_mm: Erst oberhalb dieser Höhe über der Standpose gilt
            Kontakt als zu früh.

    Returns:
        Bein -> Geländehöhe relativ zu seiner Tripod-Gruppe, in mm. Positiv
        heißt: der Boden lag dort höher als bei den beiden anderen Beinen
        derselben Gruppe. Nur Abweichungen über 1 mm werden gemeldet. Leer,
        wenn der Untergrund gleichmäßig war oder die Erkennung aus ist.
    """
    gelaende: dict[str, float] = {}
    roh: dict[str, float] = {}

    def halt(gruppe: tuple[str, ...]) -> Callable[[str], bool] | None:
        if touch_level is None:
            return None
        roh.clear()
        return make_contact_freeze(
            robot, touch_level=touch_level, margin_mm=touch_margin_mm,
            legs=gruppe, treffer=roh,
        )

    def auswerten(gruppe: tuple[str, ...]) -> None:
        """Kontakthoehen der Gruppe in Gelaendehoehen umrechnen.

        Die absolute Hoehe taugt dafuer nicht: waehrend eine Tripod-Gruppe
        schwingt, sackt der Koerper ab, und zwar fuer alle drei Beine
        gleichermassen. Erst der Vergleich INNERHALB der Gruppe trennt "der
        Boden ist hier hoeher" von "der ganze Roboter steht tiefer".

        Bezug ist der Median ueber ALLE drei Beine der Gruppe -- ein Bein,
        das nicht angehalten hat, ist regulaer bis in die Standpose
        durchgefedert und zaehlt deshalb mit 0.0. Nur die Angehaltenen zu
        mitteln waere falsch: bei einem einzelnen Hindernis waere der Median
        dann dessen eigene Hoehe, und die Abweichung immer null.
        """
        werte = {leg: roh.get(leg, 0.0) for leg in gruppe}
        mitte = statistics.median(werte.values())
        for leg, hoehe in werte.items():
            if leg not in roh:
                # Nicht angehalten heisst: regulaer bis in die Standpose
                # durchgefedert. Das ist kein Loch. Ein Loch waere "gar kein
                # Kontakt bis zum Bahnende" -- das erkennt walk() nicht, und
                # ein Bein hier als Loch zu melden, nur weil die anderen auf
                # einer Stufe standen, waere schlicht falsch.
                continue
            abweichung = hoehe - mitte
            if abweichung > 1.0:
                gelaende[leg] = abweichung

    for _ in range(cycles):
        # Halbzyklus 1: A schwingt, B steht. include_start fuer stetigen
        # Boden-Kontaktpunkt; danach Duplikat am Uebergang ueberspringen.
        paths = half_cycle_paths(
            GROUP_A, GROUP_B,
            stride=stride, height=height, steps=steps, direction=direction,
            include_start=True,
        )
        run_multi_leg_trajectory(
            robot, paths, rate_hz=rate_hz, max_step_deg=max_step_deg, clip=clip,
            freeze=halt(GROUP_A),
        )
        auswerten(GROUP_A)

        # Halbzyklus 2: B schwingt, A steht. Ersten Punkt weglassen, da er
        # mit dem letzten Punkt von H1 identisch ist (kontinuierlicher Boden).
        paths = half_cycle_paths(
            GROUP_B, GROUP_A,
            stride=stride, height=height, steps=steps, direction=direction,
            include_start=True,
        )
        paths = {leg: pts[1:] for leg, pts in paths.items()}
        run_multi_leg_trajectory(
            robot, paths, rate_hz=rate_hz, max_step_deg=max_step_deg, clip=clip,
            freeze=halt(GROUP_B),
        )
        auswerten(GROUP_B)

    return gelaende
