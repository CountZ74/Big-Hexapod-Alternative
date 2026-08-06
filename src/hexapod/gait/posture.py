"""Aufsteh- und Hinlege-Sequenzen für den Hexapod.

Diese Bewegungen arbeiten in Leg-Frame-Koordinaten (nicht in körper-
parallelen Offsets), weil die Kalibrierposition (Beine radial flach
gestreckt) weit außerhalb des normalen Offset-Bereichs liegt.

Geometrie (relativ zum Coxa-Gelenk, das beim aufliegenden Bauch ~20mm
über dem Boden sitzt):
    - Kalibrierposition: Fuß bei (233, 0, 0)   — radial flach gestreckt
    - Schwebehöhe:       Fuß bei (96.6, 0, -12) — eingeklappt, 8mm über Boden
    - Bodenkontakt:      Fuß bei (96.6, 0, -20) — unter Körper, am Boden
    - Standpose:         Fuß bei (96.6, 0, -46.4) — Körper angehoben
"""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import TYPE_CHECKING

from hexapod.gait.contact import make_contact_freeze
from hexapod.gait.trajectory import Vec3, linear_path
from hexapod.kinematics.leg_ik import forward_kinematics

if TYPE_CHECKING:
    from hexapod.kinematics.body_ik import BodyPose
    from hexapod.robot.hexapod import Hexapod

# Standard-Geometrie (mm im Leg Frame). Anpassbar über stand_up()-Parameter.
STANCE_X = 96.6      # horizontaler Fußabstand in Standpose
STANCE_Z = -46.4     # Fuß-Z in Standpose (Körper angehoben)
GROUND_Z = -20.0     # Boden relativ Coxa (gemessen: Bauch liegt auf)
HOVER_Z = -12.0      # Schwebehöhe (8mm über Boden)
CALIB_X = 233.0      # radialer Fußabstand in Kalibrierposition
LIFT_X = 220.0       # erster erreichbarer X beim Abheben auf Schwebehöhe


def _interp_legframe(
    robot: Hexapod,
    starts: dict[str, Vec3],
    ends: dict[str, Vec3],
    *,
    rate_hz: float,
    max_step_deg: float,
    clip: bool,
) -> None:
    """Interpoliere alle Beine synchron im Leg Frame von starts nach ends.

    Unterteilt adaptiv, sodass kein Gelenk pro Takt mehr als max_step_deg
    springt (gleiche Sicherung wie der Gait-Executor, aber in Leg-Frame-
    Koordinaten statt Offsets).
    """
    legs = list(starts.keys())

    # Größten Gelenksprung über den gesamten Weg bestimmen
    def angles_at(leg: str, p: Vec3) -> tuple[float, float, float]:
        from hexapod.kinematics.leg_ik import inverse_kinematics
        return inverse_kinematics(p[0], p[1], p[2], robot._leg_lengths)

    worst = 0.0
    for leg in legs:
        a0 = angles_at(leg, starts[leg])
        a1 = angles_at(leg, ends[leg])
        import math
        worst = max(worst, max(abs(math.degrees(a1[k] - a0[k])) for k in range(3)))

    import math
    substeps = max(1, math.ceil(worst / max_step_deg)) if max_step_deg > 0 else 1

    dt = 1.0 / rate_hz
    for s in range(1, substeps + 1):
        frac = s / substeps
        targets: dict[str, Vec3] = {}
        for leg in legs:
            p0, p1 = starts[leg], ends[leg]
            targets[leg] = (
                p0[0] + (p1[0] - p0[0]) * frac,
                p0[1] + (p1[1] - p0[1]) * frac,
                p0[2] + (p1[2] - p0[2]) * frac,
            )
        t_start = time.perf_counter()
        robot.set_all_foot_positions(targets, clip=clip)
        elapsed = time.perf_counter() - t_start
        if dt - elapsed > 0:
            time.sleep(dt - elapsed)


def power_up(
    robot: Hexapod,
    *,
    stance_x: float = STANCE_X,
    stance_z: float = STANCE_Z,
    ground_z: float = GROUND_Z,
    hover_z: float = HOVER_Z,
    calib_x: float = CALIB_X,
    lift_x: float = LIFT_X,
    rate_hz: float = 20.0,
    max_step_deg: float = 1.0,
    pause: float = 0.5,
    clip: bool = True,
) -> None:
    """KALTSTART: steht aus der Kalibrierposition (Bauch am Boden) auf.

    Der vollständige Einschalt-Vorgang inkl. P1-Schutz für den ungebremsten
    ersten Maestro-Zug. Für den normalen Übergang abgesetzt<->stehend im
    laufenden Betrieb siehe stand_up / lie_down.

    Voraussetzung: Die Beine stehen physisch in der Kalibrierposition
    (radial flach gestreckt), und der Aufrufer hat VORHER bereits einmal
    diese Position kommandiert (ungebremster erster Maestro-Zug), z.B.:

        robot.set_all_foot_positions({leg: (calib_x, 0, 0) for leg in legs})

    Ablauf (alle Phasen gebremst):
        P2: Füße anheben + nach innen auf Schwebehöhe
        P3: Füße absenken auf Bodenkontakt
        P4: Körper anheben (Füße auf Standpose-Z drücken)
    """
    legs = robot._leg_names
    z_trim = getattr(robot, "_z_trim", {})

    def all_at(x: float, z: float) -> dict[str, Vec3]:
        return {leg: (x, 0.0, z) for leg in legs}

    def all_at_trim(x: float, z: float) -> dict[str, Vec3]:
        # Pro Bein um den Z-Trim korrigiert (positiv = Fuß tiefer).
        return {leg: (x, 0.0, z - z_trim.get(leg, 0.0)) for leg in legs}

    # P2a: kurz abheben — von Kalibrier (calib_x,0,0) auf ersten erreichbaren
    # Schwebepunkt (lift_x, 0, hover_z). Kleine Diagonale, Fuß kommt vom Boden.
    _interp_legframe(
        robot,
        all_at(calib_x, 0.0),
        all_at(lift_x, hover_z),
        rate_hz=rate_hz, max_step_deg=max_step_deg, clip=clip,
    )
    time.sleep(pause)

    # P2b: auf KONSTANTER Schwebehöhe nach innen einschwenken bis über den
    # Standpunkt (stance_x). Fuß bleibt die ganze Zeit hover über dem Boden.
    _interp_legframe(
        robot,
        all_at(lift_x, hover_z),
        all_at(stance_x, hover_z),
        rate_hz=rate_hz, max_step_deg=max_step_deg, clip=clip,
    )
    time.sleep(pause)

    # P3: jetzt rein senkrecht absenken auf Bodenkontakt
    _interp_legframe(
        robot,
        all_at(stance_x, hover_z),
        all_at(stance_x, ground_z),
        rate_hz=rate_hz, max_step_deg=max_step_deg, clip=clip,
    )
    time.sleep(pause)

    # P4: rein senkrecht weiter — Körper hebt sich auf Standpose-Höhe
    _interp_legframe(
        robot,
        all_at(stance_x, ground_z),
        all_at_trim(stance_x, stance_z),
        rate_hz=rate_hz, max_step_deg=max_step_deg, clip=clip,
    )


def stand_up(
    robot: Hexapod,
    *,
    stance_x: float = STANCE_X,
    stance_z: float = STANCE_Z,
    ground_z: float = GROUND_Z,
    rate_hz: float = 20.0,
    max_step_deg: float = 1.0,
    clip: bool = True,
) -> None:
    """Hebt den Körper aus der abgesetzten Lage in die Standpose.

    Gegenpart zu lie_down: reiner senkrechter Körperhub. Voraussetzung ist,
    dass der Roboter abgesetzt ist (Bauch auf dem Boden, Füße in
    Standpose-X/Y am Boden) — also der Zustand nach lie_down. Es findet KEIN
    Strecken und KEIN P1-Schutz statt; dafür ist power_up zuständig (Kaltstart
    aus der Kalibrierposition).
    """
    legs = robot._leg_names
    z_trim = getattr(robot, "_z_trim", {})

    def all_at(x: float, z: float) -> dict[str, Vec3]:
        return {leg: (x, 0.0, z) for leg in legs}

    def all_at_trim(x: float, z: float) -> dict[str, Vec3]:
        return {leg: (x, 0.0, z - z_trim.get(leg, 0.0)) for leg in legs}

    # Reiner Körperhub: Füße von Bodenhöhe auf (getrimmte) Standpose-Z drücken.
    _interp_legframe(
        robot,
        all_at(stance_x, ground_z),
        all_at_trim(stance_x, stance_z),
        rate_hz=rate_hz, max_step_deg=max_step_deg, clip=clip,
    )


def lie_down(
    robot: Hexapod,
    *,
    stance_x: float = STANCE_X,
    stance_z: float = STANCE_Z,
    ground_z: float = GROUND_Z,
    hover_z: float = HOVER_Z,
    calib_x: float = CALIB_X,
    rate_hz: float = 20.0,
    max_step_deg: float = 1.0,
    pause: float = 0.5,
    clip: bool = True,
    settle_first: bool = True,
) -> None:
    """Setzt den Roboter sanft ab: Körper absenken bis der Bauch aufliegt.

    Gegenpart zum Körperhub (P4) von stand_up. Die Beine bleiben in der
    eingeklappten Standpose-X/Y-Position — sie werden NICHT radial
    ausgestreckt. Für einen Kaltstart bringt man die Beine ohnehin von Hand
    in die Kalibrierposition; ein automatisches Ausstrecken ist daher nicht
    nötig.
    """
    legs = robot._leg_names
    z_trim = getattr(robot, "_z_trim", {})

    def all_at(x: float, z: float) -> dict[str, Vec3]:
        return {leg: (x, 0.0, z) for leg in legs}

    def all_at_trim(x: float, z: float) -> dict[str, Vec3]:
        return {leg: (x, 0.0, z - z_trim.get(leg, 0.0)) for leg in legs}

    # Erst sauber in die Standpose (lift-and-place), damit der Abstieg aus
    # einer DEFINIERTEN Lage startet. Ohne das nimmt _interp_legframe die
    # Standpose als Startpunkt an und der erste Takt springt die Servos hart
    # dorthin (sichtbar bei gehaltener Pose oder abweichenden Fuessen). Loest
    # zugleich eine gehaltene Body-Pose auf.
    if settle_first:
        move_to_stance(robot, clip=clip)

    # Körper absenken: Füße von Standpose-Z zurück auf Bodenhöhe.
    # Der Körper sinkt, bis der Bauch aufliegt. Beine bleiben eingeklappt.
    _interp_legframe(robot, all_at_trim(stance_x, stance_z), all_at(stance_x, ground_z),
                     rate_hz=rate_hz, max_step_deg=max_step_deg, clip=clip)


def settle_to_stance(
    robot: Hexapod,
    *,
    lift: float = 15.0,
    order: list[str] | None = None,
    rate_hz: float = 60.0,
    max_step_deg: float = 2.0,
    pause: float = 0.1,
    clip: bool = True,
    force: bool = False,
    touch_level: float | None = None,
    touch_margin_mm: float = 5.0,
) -> dict[str, float]:
    """Bringt alle Beine EINZELN nacheinander sauber in die Standpose.

    Vertrag: Der Roboter steht bereits auf allen sechs Beinen ungefähr auf
    Standpose-Höhe; nur die Fuß-X/Y-Positionen weichen ab. Jedes Bein wird
    einzeln angehoben, über die Standpose-Position geführt und abgesetzt,
    während die anderen fünf tragen (stabil). So schleift kein Fuß über den
    Boden — auch auf Teppich.

    Diese Funktion fängt KEINE wilden Posen ein (mehrere Beine in der Luft
    o.ä.); dafür ist der Aufrufer verantwortlich.

    Args:
        robot: Hexapod-Instanz.
        lift: Anhebung über Standpose-Niveau in mm (Schwebehöhe).
        order: Reihenfolge der Beine. Default: eine Reihenfolge, die
            gegenüberliegende Beine abwechselt (stabiler Schwerpunkt).
        rate_hz: Sende-Frequenz.
        max_step_deg: Max. Gelenksprung pro Takt.
        pause: Pause zwischen den Beinen in Sekunden.
        clip: Winkel-Clipping.
        touch_level: Aufsetz-Erkennung. Ist ein Wert gesetzt und hat das Bein
            einen kalibrierten Fußsensor, bricht die Absetzbewegung ab, sobald
            der Federweg diese Schwelle erreicht. Ohne Wert (Default) bleibt
            das Verhalten exakt wie bisher.

            Sinnvolle Groessenordnung: im Sechsbeinstand liegen die Beine bei
            12 bis 27 % Federweg, das Rauschen bei rund 2 %. Etwa 5 % trennt
            also sauber zwischen "beruehrt" und "traegt".
        touch_margin_mm: Ab welcher Hoehe ueber der Standpose der Kontakt
            ueberhaupt als "zu frueh" gilt. Naeher dran faehrt das Bein
            normal zu Ende.

            Das ist wesentlich, nicht kosmetisch: die Schwelle allein greift
            schon beim Antippen, lange bevor sich das Bein in die Standpose
            drueckt. Ohne Mindesthoehe stoppt deshalb JEDES Bein zu frueh,
            der Koerper sinkt nie ganz ab, und der Roboter steht auf sechs
            kaum eingefederten Beinen -- schlechter als ohne Erkennung.
            Gemessen wurde auf ebenem Boden ein erster Kontakt bis 4 mm ueber
            der Standpose; 5 mm laesst das normale Absetzen also durch und
            faengt echte Hindernisse ab.
        force: Auch Beine anheben, die schon (fast) in der Standpose stehen.
            Normalerweise werden die übersprungen — das spart Bewegung, wenn
            ohnehin nichts zu tun ist. Für den Lastabgleich ist genau das
            aber falsch: dort geht es nicht ums Erreichen der Position,
            sondern ums LÖSEN der Reibung, damit sich die Last neu verteilen
            kann. Und die Korrekturen liegen typischerweise unter der
            Überspring-Toleranz, kämen also nie am Roboter an.
    """
    import time

    from hexapod.gait.executor import run_single_leg_trajectory

    if order is None:
        # Diagonal abwechselnd: hält den Schwerpunkt mittig.
        order = [
            "front_left", "back_right", "mid_left",
            "front_right", "back_left", "mid_right",
        ]
        # Nur Beine nehmen, die es wirklich gibt (Robustheit).
        order = [leg for leg in order if leg in robot._leg_names]
        for leg in robot._leg_names:
            if leg not in order:
                order.append(leg)

    frueh: dict[str, float] = {}
    for leg in order:
        cx, cy, cz = robot.current_offset(leg)

        # Schon in Standpose? (kleine Toleranz) -> überspringen.
        # Mit force=True trotzdem anheben: siehe Docstring.
        if not force and abs(cx) < 0.5 and abs(cy) < 0.5 and abs(cz) < 0.5:
            continue

        # Hubhoehe RELATIV zur aktuellen Fusshoehe: so hebt der Fuss immer
        # wirklich ab, egal wie hoch er durch eine gehaltene Pose schon steht.
        # (Ein fester Ziel-dz wuerde Beine mit hohem Ist-dz nach UNTEN in den
        # Boden druecken und schleifen lassen.)
        lift_z = max(cz, 0.0) + lift
        # Phase 1: anheben (x/y bleiben, dz auf Schwebehöhe)
        p_lift = (cx, cy, lift_z)
        # Phase 2: horizontal zur Standpose-X/Y bei Schwebehöhe
        p_over = (0.0, 0.0, lift_z)
        # Phase 3: absetzen auf Standpose
        p_down = (0.0, 0.0, 0.0)

        seg1 = linear_path((cx, cy, cz), p_lift, max(2, int(abs(lift_z - cz))))
        seg2 = linear_path(p_lift, p_over, max(2, int(abs(cx) + abs(cy))))
        seg3 = linear_path(p_over, p_down, max(2, int(lift_z)))

        # Hin- und Absetzbahn getrennt: die Kontaktabfrage darf erst beim
        # Absenken greifen. Beim Anheben liegt das Bein ja noch auf und wuerde
        # sofort ausloesen.
        run_single_leg_trajectory(
            robot, leg, seg1 + seg2,
            rate_hz=rate_hz, max_step_deg=max_step_deg, clip=clip,
        )

        halt: Callable[[str], bool] | None = None
        if touch_level is not None:
            halt = make_contact_freeze(
                robot, touch_level=touch_level, margin_mm=touch_margin_mm,
                legs=[leg],
            )

        run_single_leg_trajectory(
            robot, leg, seg3,
            rate_hz=rate_hz, max_step_deg=max_step_deg, clip=clip,
            freeze=halt,
        )

        # Wo ist das Bein tatsaechlich stehengeblieben? Bei erkanntem Kontakt
        # ueber der Standpose -- genau die Information, die der Aufrufer
        # braucht, wenn er wissen will, wie uneben der Boden war.
        rest_z = robot.current_offset(leg)[2]
        if touch_level is not None and abs(rest_z) > 0.5:
            frueh[leg] = rest_z
        time.sleep(pause)

    return frueh


def move_to_body_pose(
    robot: Hexapod,
    pose: BodyPose,
    *,
    steps: int = 30,
    rate_hz: float = 50.0,
    max_step_deg: float = 3.0,
    clip: bool = True,
) -> None:
    """Fahre den Körper weich von der aktuellen Pose in die Zielpose.

    Die Füße bleiben am Boden; der Körper wird über sie translatiert und
    rotiert. Der Übergang läuft über denselben adaptiven Trajektorien-
    Executor wie der Gait (Gelenksprung-Begrenzung gegen Ausfransen), startet
    also ruckfrei bei der aktuellen Pose und rampt sanft in die Zielpose.

    Args:
        robot: Hexapod-Instanz.
        pose: Ziel-BodyPose.
        steps: Interpolationsschritte für den Pose-Übergang (Grobraster; der
            Executor unterteilt zusätzlich adaptiv).
        rate_hz: Sende-Frequenz.
        max_step_deg: Max. Gelenksprung pro Takt.
        clip: Winkel-Clipping.
    """
    from hexapod.gait.executor import run_multi_leg_trajectory
    from hexapod.kinematics.body_ik import BodyPose, body_pose_offsets

    start = robot.body_pose
    s_vec = (start.tx, start.ty, start.tz, start.roll, start.pitch, start.yaw)
    e_vec = (pose.tx, pose.ty, pose.tz, pose.roll, pose.pitch, pose.yaw)

    n = max(1, steps)
    leg_points: dict[str, list[Vec3]] = {leg: [] for leg in robot._leg_names}
    for i in range(n + 1):
        frac = i / n
        vals = [a + (b - a) * frac for a, b in zip(s_vec, e_vec, strict=True)]
        p_i = BodyPose(
            tx=vals[0], ty=vals[1], tz=vals[2],
            roll=vals[3], pitch=vals[4], yaw=vals[5],
        )
        offs = body_pose_offsets(
            p_i,
            robot._foot_positions_world,
            robot._coxa_positions,
            robot._neutral_world,
        )
        for leg in robot._leg_names:
            leg_points[leg].append(offs[leg])

    run_multi_leg_trajectory(
        robot, leg_points, rate_hz=rate_hz, max_step_deg=max_step_deg, clip=clip
    )
    robot._body_pose = pose

def level_body(
    robot: Hexapod,
    *,
    steps: int = 30,
    rate_hz: float = 40.0,
    clip: bool = True,
) -> None:
    """Loest eine gehaltene Body-Pose auf, ohne die Fuesse zu verschieben.

    Der Koerper wird sanft von der aktuellen Pose ins Level (Identitaet)
    gefahren, waehrend jeder Fuss exakt an seiner aktuellen WELT-Position
    bleibt (planted). Es bewegt sich also nur der Koerper -- kein Fuss wird
    ueber den Boden gezogen. Danach stehen alle Beine wieder im normalen
    Arbeitsbereich (die Auslenkung kam ja von der Pose), sodass ein
    anschliessendes settle_to_stance fuer JEDES Bein vollen Hub-Spielraum hat.
    """
    import math
    import time

    import numpy as np

    from hexapod.kinematics.body_ik import BodyPose, body_ik, rotation_matrix

    pose = robot.body_pose
    if (pose.tx == 0.0 and pose.ty == 0.0 and pose.tz == 0.0
            and pose.roll == 0.0 and pose.pitch == 0.0 and pose.yaw == 0.0):
        return

    # Aktuelle Fuss-Positionen im INERTIAL-/Bodenframe aus den Ist-Winkeln.
    # Body-Center-Position B aus der Vorwaertskinematik, dann mit der AKTUELLEN
    # Pose transformieren: world = R(pose) @ B + t(pose). (Nur B zu nehmen waere
    # falsch -- das ist die koerperfeste Position, nicht die am Boden.)
    R = rotation_matrix(pose.roll, pose.pitch, pose.yaw)
    t = np.array([pose.tx, pose.ty, pose.tz], dtype=np.float64)
    world: dict[str, tuple[float, float, float]] = {}
    for leg in robot._leg_names:
        st = robot._leg_states[leg]
        fx, fy, fz = forward_kinematics(
            st.theta1, st.theta2, st.theta3, robot._leg_lengths
        )
        ma = robot._mount_angles[leg]
        cx = float(robot._coxa_positions[leg][0])
        cy = float(robot._coxa_positions[leg][1])
        cz = float(robot._coxa_positions[leg][2])
        b = np.array([
            fx * math.cos(ma) - fy * math.sin(ma) + cx,
            fx * math.sin(ma) + fy * math.cos(ma) + cy,
            fz + cz,
        ], dtype=np.float64)
        w = R @ b + t
        world[leg] = (float(w[0]), float(w[1]), float(w[2]))

    sv = (pose.tx, pose.ty, pose.tz, pose.roll, pose.pitch, pose.yaw)
    coxa = robot._coxa_positions
    n = max(1, steps)
    dt = 1.0 / rate_hz
    for i in range(n + 1):
        frac = 1.0 - i / n  # von 1 (aktuelle Pose) nach 0 (Level)
        p_i = BodyPose(
            tx=sv[0] * frac, ty=sv[1] * frac, tz=sv[2] * frac,
            roll=sv[3] * frac, pitch=sv[4] * frac, yaw=sv[5] * frac,
        )
        targets = body_ik(p_i, world, coxa)
        robot._body_pose = p_i  # Pose konsistent mitfuehren
        t0 = time.perf_counter()
        robot.set_all_foot_positions_world(targets, clip=clip)
        el = time.perf_counter() - t0
        if dt - el > 0:
            time.sleep(dt - el)


def move_to_stance(
    robot: Hexapod,
    *,
    lift: float = 15.0,
    rate_hz: float = 80.0,
    max_step_deg: float = 2.0,
    pause: float = 0.05,
    clip: bool = True,
) -> None:
    """Bringt den Roboter sanft und OHNE Schleifen in die Standpose.

    Hebt jedes Bein einzeln an, fuehrt es ueber die Standpose-Position und
    setzt es ab (lift-and-place via settle_to_stance), sodass KEIN Fuss
    horizontal ueber den Boden gezogen wird -- funktioniert auch auf Teppich
    und schont die Servos. Setzt anschliessend die Body-Pose auf Identitaet
    zurueck, damit der naechste Pose-Befehl sauber von der Standpose startet.

    Gegenstueck zum harten robot.stance(): gleicher Endzustand (alle Offsets
    null), aber gehoben statt geschliffen.
    """
    from hexapod.kinematics.body_ik import BodyPose

    # 1) Pose aufloesen: Koerper ins Level, Fuesse bleiben am Boden (kein
    #    Schleifen). Danach hat jedes Bein wieder vollen Hub-Spielraum.
    level_body(robot, rate_hz=rate_hz, clip=clip)
    robot._body_pose = BodyPose()
    # 2) Fuesse einzeln sauber in die Standpose (lift-and-place).
    settle_to_stance(
        robot,
        lift=lift,
        rate_hz=rate_hz,
        max_step_deg=max_step_deg,
        pause=pause,
        clip=clip,
    )
    robot._body_pose = BodyPose()
