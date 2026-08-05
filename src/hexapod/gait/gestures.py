"""Spielerische Gesten/Moves (Winken, Bein heben/Pinkeln, Maennchen/Mantis).

Frame: Fuss-Offset (dx, dy, dz) relativ zur Standpose, koerperparallel
(+x vorne, +y links, +z oben); (0,0,0) = Standpose.
Body-Pose-Konvention: Roll links = negativ / rechts = positiv,
Pitch hoch = negativ / runter = positiv.

Alle Bewegungen sind winkel-geclippt (reichweiten-sicher).
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING

from hexapod.gait.climb import climb_over_box, step_up_front_test
from hexapod.gait.executor import (
    run_multi_leg_trajectory,
    run_single_leg_trajectory,
)
from hexapod.gait.trajectory import Vec3, linear_path, smoothstep

if TYPE_CHECKING:
    from collections.abc import Callable

    from hexapod.robot.hexapod import Hexapod

ORIGIN = (0.0, 0.0, 0.0)


def _max_line_lift(
    robot: Hexapod,
    leg: str,
    dx: float,
    dy: float,
    z_wish: float,
    *,
    margin_us: float = 25.0,
    samples: int = 12,
    z_min: float = 40.0,
) -> float:
    """Hoechstes Ziel-z <= z_wish, sodass die Gerade 0 -> (dx, dy, z) ohne
    Servo-Clipping fahrbar bleibt.

    Beruecksichtigt die tatsaechliche Kalibrierung (min_us/max_us je Servo)
    dieses Roboters -- asymmetrisch kalibrierte Beine bekommen automatisch
    eine niedrigere, aber sichere Hubhoehe. margin_us haelt Reserve fuer die
    gleichzeitig gefahrene Body-Pose vor.
    """
    from hexapod.config import Joint

    joints = (Joint.COXA, Joint.FEMUR, Joint.TIBIA)

    def ok(p: Vec3) -> bool:
        try:
            angles = robot.offset_to_angles(leg, p[0], p[1], p[2])
        except Exception:
            return False
        for j, ang in zip(joints, angles, strict=True):
            mp = robot.mapping_for(robot.config.get_leg_servo(leg, j))
            us = mp.angle_to_us(ang, clip=True)
            if us <= mp.min_us + margin_us or us >= mp.max_us - margin_us:
                return False
        return True

    z = z_wish
    while z >= z_min:
        if all(
            ok((dx * (i / samples), dy * (i / samples), z * (i / samples)))
            for i in range(1, samples + 1)
        ):
            return z
        z -= 5.0
    return z_min


def wave(robot: Hexapod, leg: str = "front_right") -> None:
    """Hebt ein Vorderbein im Bogen hoch und winkt; setzt es danach wieder
    GENAU auf seine Ausgangsposition (Boden) zurueck -- nicht auf hartes 0,
    damit es auch bei gehaltener Body-Pose sauber aufsetzt.

    Der Fuss fuehrt einen Bogen (erst vor, dann hoch entlang des erreichbaren
    Bandes), da er nicht geradlinig nach oben fahren kann (Femur am Anschlag).
    """
    side = -1.0 if leg.endswith("right") else 1.0
    cur = robot.current_offset(leg)
    w1 = (50.0, side * 8.0, 30.0)
    w2 = (78.0, side * 8.0, 100.0)   # aussen hochsteigen: dort ist der Korridor
    top = (76.0, side * 8.0, 175.0)  # voll nach oben gestreckt
    out = (76.0, side * 38.0, 170.0)
    seq = list(linear_path(cur, w1, 8, include_start=True))
    seq += list(linear_path(w1, w2, 8))
    seq += list(linear_path(w2, top, 8))
    for _ in range(3):                       # winken (nur Aussenseite)
        seq += list(linear_path(top, out, 6))
        seq += list(linear_path(out, top, 6))
    seq += list(linear_path(top, w2, 8))     # Bogen zurueck
    seq += list(linear_path(w2, w1, 8))
    seq += list(linear_path(w1, cur, 8))     # zurueck auf Ausgangslage (Boden)
    run_single_leg_trajectory(robot, leg, seq, rate_hz=55.0, max_step_deg=4.0, clip=True)


def lift_leg(robot: Hexapod, leg: str = "back_right") -> None:
    """Hund-pinkelt-Pose: Koerper neigt sich vom Hinterbein weg UND das Bein
    streckt sich gleichzeitig hoch/seitlich aus (eine fluessige Bewegung),
    haelt kurz, dann beides zurueck in die Standpose.

    Tilt-Konvention: Roll links=neg/rechts=pos. Wir neigen vom Bein weg.
    """
    from hexapod.kinematics.body_ik import BodyPose, body_pose_offsets

    side = -1.0 if leg.endswith("right") else 1.0
    roll_abs = math.radians(9.0 * side)  # ABSOLUTE Ziel-Neigung (rechts->negativ/links weg)
    dty = 14.0 * (-side)               # Schwerpunkt zur Gegenseite
    # Hubhoehe an die Kalibrierung DIESES Beins anpassen: so hoch wie ohne
    # Clipping moeglich (Wunsch 150), mindestens aber die alte Hoehe ~50.
    z_lift = _max_line_lift(robot, leg, -12.0, side * 38.0, 150.0, z_min=50.0)
    lift = (-12.0, side * 38.0, z_lift)  # zurueck, seitlich raus, hoch gestreckt
    legs = robot._leg_names
    n = 22

    # Von der AKTUELLEN Body-Pose aus neigen (nicht von Stance) -- so laeuft
    # die Geste aus jeder Pose, ohne vorher in die Standpose zu schnappen.
    start = robot.body_pose
    s_vec = (start.tx, start.ty, start.tz, start.roll, start.pitch, start.yaw)
    e_vec = (start.tx, start.ty + dty, start.tz,
             roll_abs, start.pitch, start.yaw)   # Roll absolut, Rest relativ

    def ramp(forward: bool) -> list[dict[str, Vec3]]:
        rng = range(n + 1) if forward else range(n, -1, -1)
        out = []
        for i in rng:
            f = i / n
            vals = [a + (b - a) * f for a, b in zip(s_vec, e_vec, strict=True)]
            pose = BodyPose(tx=vals[0], ty=vals[1], tz=vals[2],
                            roll=vals[3], pitch=vals[4], yaw=vals[5])
            offs = body_pose_offsets(
                pose, robot._foot_positions_world,
                robot._coxa_positions, robot._neutral_world,
            )
            frame = dict(offs)
            lp = smoothstep(f)
            ox, oy, oz = offs[leg]
            frame[leg] = (ox + lift[0] * lp, oy + lift[1] * lp, oz + lift[2] * lp)
            out.append(frame)
        return out

    seq = ramp(True)
    seq += [seq[-1]] * 12          # Pose halten (pinkeln)
    seq += ramp(False)            # gleichzeitig zurueck zur Ausgangspose

    leg_points: dict[str, list[Vec3]] = {leg_name: [] for leg_name in legs}
    for frame in seq:
        for leg_name in legs:
            leg_points[leg_name].append(frame[leg_name])
    run_multi_leg_trajectory(robot, leg_points, rate_hz=55.0, max_step_deg=3.0, clip=True)
    robot._body_pose = start


def mantis(robot: Hexapod) -> None:
    """Maennchen: zuerst Gewicht nach hinten, dann Vorderbeine als 'Arme' heben
    UND Pitch 'Nase hoch'. Der Pitch wird aufgeteilt: Mittelbeine gehen etwas
    nach UNTEN (heben den Rumpf vorne mit), Hinterbeine etwas nach OBEN (senken
    den Hintern) -- zusammen ~pitch_deg. Kurz halten, danach umgekehrt zurueck
    in die Standpose.
    """
    mid = ["mid_left", "mid_right"]
    back = ["back_left", "back_right"]
    front = ["front_left", "front_right"]
    legs = mid + back + front
    shift, fdx, lift = 34.0, 30.0, 90.0
    pitch_deg = 15.0      # Gesamt-Pitch 'Nase hoch'
    MID_SHARE = 0.4       # Anteil ueber Mittelbeine (nach unten); Rest ueber Hinterbeine
    side = {"front_left": 12.0, "front_right": -12.0}
    n_shift, n_lift, n_hold = 16, 16, 14

    # Gesamter Z-Versatz Mitte<->Hinten fuer pitch_deg, aus echter Geometrie.
    nf = robot.neutral_foot_xy
    mid_x = (nf["mid_left"][0] + nf["mid_right"][0]) / 2.0
    back_x = (nf["back_left"][0] + nf["back_right"][0]) / 2.0
    total = abs(mid_x - back_x) * math.tan(math.radians(pitch_deg))
    mid_dz = -total * MID_SHARE          # Mittelbeine etwas nach unten
    back_dz = total * (1.0 - MID_SHARE)  # Hinterbeine etwas weniger nach oben

    flat = (shift, 0.0, 0.0)
    mid_hold = (shift, 0.0, mid_dz)
    back_hold = (shift, 0.0, back_dz)
    front_up = {leg: (fdx, side[leg], lift) for leg in front}

    leg_points: dict[str, list[Vec3]] = {leg: [] for leg in legs}

    def add(seg: dict[str, list[Vec3]]) -> None:
        for leg in legs:
            leg_points[leg].extend(seg[leg])

    # 1) Gewicht nach hinten: ALLE sechs Fuesse schieben synchron -> reine
    #    Koerpertranslation. Vorher blieben die Vorderbeine auf ihrem Offset
    #    und wurden beim Koerperversatz ueber den Boden gezogen (schleift,
    #    auf Teppich gefaehrlich fuer die Servos).
    add({leg: list(linear_path(ORIGIN, flat, n_shift, include_start=True)) for leg in legs})
    # 2) Vorderbeine heben (aus der VERSCHOBENEN Lage) + Pitch
    add({**{leg: list(linear_path(flat, mid_hold, n_lift)) for leg in mid},
         **{leg: list(linear_path(flat, back_hold, n_lift)) for leg in back},
         **{leg: list(linear_path(flat, front_up[leg], n_lift)) for leg in front}})
    # 3) Halten
    add({**{leg: [mid_hold] * n_hold for leg in mid},
         **{leg: [back_hold] * n_hold for leg in back},
         **{leg: [front_up[leg]] * n_hold for leg in front}})
    # 4) Vorderbeine ablegen (auf die VERSCHOBENE Lage) + Pitch zurueck
    add({**{leg: list(linear_path(mid_hold, flat, n_lift)) for leg in mid},
         **{leg: list(linear_path(back_hold, flat, n_lift)) for leg in back},
         **{leg: list(linear_path(front_up[leg], flat, n_lift)) for leg in front}})
    # 5) Gewicht zurueck nach vorne: wieder alle sechs synchron -> Standpose
    add({leg: list(linear_path(flat, ORIGIN, n_shift)) for leg in legs})

    run_multi_leg_trajectory(robot, leg_points, rate_hz=45.0, max_step_deg=3.0, clip=True)


GESTURES: dict[str, Callable[[Hexapod], None]] = {
    "climb_over": lambda r: climb_over_box(r),
    "step_up_test": lambda r: step_up_front_test(r),
    "wave": lambda r: wave(r),
    "lift_leg": lambda r: lift_leg(r),
    "mantis": mantis,
}
