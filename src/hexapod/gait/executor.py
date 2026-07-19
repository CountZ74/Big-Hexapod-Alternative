"""Zeitgesteuerte Ausführung von Fuß-Trajektorien.

Der Executor nimmt vorberechnete Trajektorien-Punkte (Offsets relativ zur
Standpose) und sendet sie in festem Zeittakt an den Roboter.

Adaptive Schrittbegrenzung
--------------------------
Eine konstante Punktdichte im kartesischen Raum erzeugt ungleiche
Gelenksprünge: nahe der Coxa-Achse muss sich θ1 sehr schnell drehen
(Polarkoordinaten-Singularität). Übersteigt der Gelenksprung pro Takt das,
was die Servos in der Taktzeit schaffen, franst die Bahn aus ("Rauten").

Der Executor prüft daher den größten Gelenksprung zwischen zwei Zielpunkten
und fügt bei Bedarf kartesische Zwischenpunkte ein, sodass kein Teilschritt
den erlaubten Maximalwinkel überschreitet.
"""

from __future__ import annotations

import math
import time
from collections.abc import Mapping, Sequence
from typing import TYPE_CHECKING

from hexapod.gait.trajectory import Vec3, lerp

if TYPE_CHECKING:
    from hexapod.robot.hexapod import Hexapod


def _v3(p: Sequence[float]) -> Vec3:
    """Normalisiert einen Bahnpunkt auf ein 3er-Tupel."""
    return (p[0], p[1], p[2])


def _max_joint_step_deg(
    robot: Hexapod, leg_name: str, p0: Vec3, p1: Vec3
) -> float:
    """Größter Gelenkwinkel-Sprung (Grad) zwischen zwei Offsets für ein Bein."""
    a0 = robot.offset_to_angles(leg_name, *p0)
    a1 = robot.offset_to_angles(leg_name, *p1)
    return max(abs(math.degrees(a1[k] - a0[k])) for k in range(3))


def _subdivide(
    robot: Hexapod,
    leg_points: Mapping[str, Sequence[Vec3]],
    start: dict[str, Vec3],
    max_step_deg: float,
) -> list[dict[str, Vec3]]:
    """Erzeuge eine Punktfolge mit garantiert begrenztem Gelenksprung.

    Für jeden Bahn-Index wird geprüft, wie viele Zwischenschritte nötig sind,
    damit KEIN Bein einen Gelenksprung > max_step_deg macht. Dann wird für
    alle Beine entsprechend fein interpoliert (gemeinsame Schrittzahl, damit
    die Beine synchron bleiben).
    """
    legs = list(leg_points.keys())
    n = len(next(iter(leg_points.values())))
    prev = dict(start)
    out: list[dict[str, Vec3]] = []

    for i in range(n):
        target = {leg: _v3(leg_points[leg][i]) for leg in legs}

        # Größter Gelenksprung über alle Beine für diesen Abschnitt:
        worst = 0.0
        for leg in legs:
            worst = max(worst, _max_joint_step_deg(robot, leg, prev[leg], target[leg]))

        substeps = max(1, math.ceil(worst / max_step_deg)) if max_step_deg > 0 else 1

        for s in range(1, substeps + 1):
            frac = s / substeps
            pt = {leg: lerp(prev[leg], target[leg], frac) for leg in legs}
            out.append(pt)

        prev = target

    return out


def run_single_leg_trajectory(
    robot: Hexapod,
    leg_name: str,
    points: Sequence[Vec3],
    *,
    rate_hz: float = 50.0,
    max_step_deg: float = 3.0,
    clip: bool = True,
) -> None:
    """Führe eine Trajektorie für ein einzelnes Bein zeitgesteuert aus.

    Args:
        robot: Hexapod-Instanz.
        leg_name: Name des Beins.
        points: Folge von (dx, dy, dz)-Offsets relativ zur Standpose.
        rate_hz: Sende-Frequenz in Hz.
        max_step_deg: Maximaler Gelenkwinkel-Sprung pro Takt (Grad). Größere
            Abschnitte werden automatisch unterteilt. 0 = keine Unterteilung.
        clip: Winkel-Clipping an Servo-Grenzen.
    """
    run_multi_leg_trajectory(
        robot,
        {leg_name: points},
        rate_hz=rate_hz,
        max_step_deg=max_step_deg,
        clip=clip,
    )


def run_multi_leg_trajectory(
    robot: Hexapod,
    leg_points: Mapping[str, Sequence[Vec3]],
    *,
    rate_hz: float = 50.0,
    max_step_deg: float = 3.0,
    clip: bool = True,
    start: Mapping[str, Vec3] | None = None,
) -> None:
    """Führe synchron Trajektorien für mehrere Beine aus.

    Alle Trajektorien müssen gleiche Länge haben. Vor dem Senden wird die
    Bahn so unterteilt, dass kein Bein pro Takt mehr als max_step_deg an
    irgendeinem Gelenk springt (adaptive Schrittbegrenzung gegen Ausfransen
    nahe der Coxa-Singularität).

    Args:
        robot: Hexapod-Instanz.
        leg_points: Bein-Name → Folge von (dx, dy, dz)-Offsets.
        rate_hz: Sende-Frequenz in Hz.
        max_step_deg: Maximaler Gelenksprung pro Takt (Grad). 0 = aus.
        clip: Winkel-Clipping.
    """
    if not leg_points:
        return
    lengths = {len(pts) for pts in leg_points.values()}
    if len(lengths) != 1:
        raise ValueError(
            f"Alle Trajektorien brauchen gleiche Länge, waren {lengths}"
        )

    # Startpunkt = aktuelle Soll-Offsets (nehmen den ersten Bahnpunkt als
    # Referenz für den Sprung vom Stand in die Bahn nicht an; wir starten
    # bei den ersten Bahnpunkten selbst, daher prev=erste Punkte).
    legs = list(leg_points.keys())
    # Startpunkt der Unterteilung: standardmaessig der erste Bahnpunkt; wird
    # ein expliziter ``start`` uebergeben (z.B. die tatsaechliche Ist-Lage der
    # Fuesse), bruecken wir von dort sanft in die Bahn -- so springt nichts bei
    # Geschwindigkeits-/Vorzeichenwechseln.
    if start is None:
        start_pts = {leg: _v3(leg_points[leg][0]) for leg in legs}
    else:
        start_pts = {leg: _v3(start[leg]) for leg in legs}

    if max_step_deg and max_step_deg > 0:
        frames = _subdivide(robot, leg_points, start_pts, max_step_deg)
    else:
        n = lengths.pop()
        frames = [
            {leg: _v3(leg_points[leg][i]) for leg in legs}
            for i in range(n)
        ]

    dt = 1.0 / rate_hz
    for frame in frames:
        t_start = time.perf_counter()
        robot.set_all_foot_offsets(frame, clip=clip)
        elapsed = time.perf_counter() - t_start
        sleep = dt - elapsed
        if sleep > 0:
            time.sleep(sleep)
