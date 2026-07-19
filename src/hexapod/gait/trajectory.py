"""Fuß-Trajektorien für Gangarten.

Eine Trajektorie ist eine Funktion s ∈ [0, 1] → (dx, dy, dz), die einen
Fuß-Offset relativ zur Standpose im körperparallelen Frame beschreibt.

Die Bahnen werden bewusst als reine Geometrie gehalten (keine Zeit, keine
Servos): so sind sie isoliert testbar. Die zeitliche Ausführung übernimmt
der Gait-Executor.
"""

from __future__ import annotations

import math
from collections.abc import Callable

Vec3 = tuple[float, float, float]


def lerp(p0: Vec3, p1: Vec3, s: float) -> Vec3:
    """Lineare Interpolation zwischen zwei Punkten."""
    return (
        p0[0] + (p1[0] - p0[0]) * s,
        p0[1] + (p1[1] - p0[1]) * s,
        p0[2] + (p1[2] - p0[2]) * s,
    )


def smoothstep(s: float) -> float:
    """Glatte S-Kurve 3s²−2s³ mit Geschwindigkeit 0 an beiden Enden.

    Bei s=0 und s=1 ist die Ableitung 6s(1−s) = 0. Wird die horizontale
    Bewegung über smoothstep statt linear parametrisiert, hebt und setzt
    der Fuß senkrecht ab/auf (keine Horizontalgeschwindigkeit am Boden →
    kein Rutschen), und ein gerade gelandetes Standbein rampt seinen Schub
    sanft aus null hoch, statt sofort mit voller Kraft zu schieben.
    """
    return s * s * (3.0 - 2.0 * s)


def linear_path(
    p0: Vec3, p1: Vec3, steps: int, *, include_start: bool = False
) -> list[Vec3]:
    """Gerade Linie von p0 nach p1.

    Args:
        p0: Startpunkt.
        p1: Endpunkt.
        steps: Anzahl Teilschritte.
        include_start: Wenn False (Default), werden `steps` Punkte
            zurückgegeben (p0 exklusiv, p1 inklusiv) — gut zum Aneinanderreihen.
            Wenn True, werden `steps + 1` Punkte zurückgegeben (p0 und p1
            beide inklusiv) — gut für in sich geschlossene Bahnen.
    """
    if steps < 1:
        raise ValueError(f"steps muss >= 1 sein, war {steps}")
    if include_start:
        return [lerp(p0, p1, i / steps) for i in range(steps + 1)]
    return [lerp(p0, p1, (i + 1) / steps) for i in range(steps)]


def swing_path(
    p0: Vec3,
    p1: Vec3,
    height: float,
    steps: int,
    *,
    include_start: bool = False,
    ease: Callable[[float], float] | None = None,
) -> list[Vec3]:
    """Schwungphase: von p0 nach p1, dabei eine Bogen-Anhebung in Z.

    Die Z-Anhebung folgt immer einer Sinus-Halbwelle über dem rohen
    Parameter s, sodass der Fuß bei s=0 und s=1 auf Bodenhöhe ist und bei
    s=0.5 die maximale Höhe `height` erreicht.

    Die horizontale (x, y) Bewegung folgt standardmäßig linear demselben s.
    Wird ``ease`` übergeben (z.B. :func:`smoothstep`), wird NUR die
    Horizontale über ``ease(s)`` parametrisiert, während Z auf dem rohen s
    bleibt. Effekt: am Bahnanfang/-ende ist die Horizontalgeschwindigkeit
    null, während Z noch auf-/absteigt → der Fuß hebt und setzt senkrecht,
    statt beim Landen horizontal weiterzuschmieren.

    Args:
        p0: Startpunkt (Offset relativ zur Standpose).
        p1: Endpunkt.
        height: Maximale Anhebung über der Z-Grundlinie in mm.
        steps: Anzahl der erzeugten Punkte.
        include_start: Startpunkt mit ausgeben.
        ease: Optionale Umparametrisierung der Horizontalen, s → s'.

    Returns:
        Liste von Punkten (inklusive p1).
    """
    if steps < 1:
        raise ValueError(f"steps muss >= 1 sein, war {steps}")
    pts: list[Vec3] = []
    rng = range(steps + 1) if include_start else range(1, steps + 1)
    for i in rng:
        s = i / steps
        hs = ease(s) if ease is not None else s
        x = p0[0] + (p1[0] - p0[0]) * hs
        y = p0[1] + (p1[1] - p0[1]) * hs
        z = p0[2] + (p1[2] - p0[2]) * s + height * math.sin(math.pi * s)
        pts.append((x, y, z))
    return pts


def stance_path(
    p0: Vec3,
    p1: Vec3,
    steps: int,
    *,
    include_start: bool = False,
    ease: Callable[[float], float] | None = None,
) -> list[Vec3]:
    """Standphase: Fuß bleibt am Boden und schiebt von p0 nach p1.

    Ohne ``ease`` identisch zu :func:`linear_path` (konstante Geschwindigkeit).
    Mit ``ease`` (z.B. :func:`smoothstep`) wird der Schub zeitlich sanft aus
    null hoch- und wieder heruntergefahren — die Endpunkte bleiben exakt p0
    und p1, nur das Geschwindigkeitsprofil wird weich. So beginnt ein gerade
    belastetes Bein nicht schlagartig mit voller Schubkraft.
    """
    if ease is None:
        return linear_path(p0, p1, steps, include_start=include_start)
    if steps < 1:
        raise ValueError(f"steps muss >= 1 sein, war {steps}")
    rng = range(steps + 1) if include_start else range(1, steps + 1)
    return [lerp(p0, p1, ease(i / steps)) for i in rng]
