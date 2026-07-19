"""Kanten-/Stufen-Klettern — Stufe 1: Vorderbeine auf ein Podest.

step_up_front_test() setzt beide Vorderbeine nacheinander auf eine Kante
bekannter Hoehe (Default 68 mm), haelt die Pose kurz und steigt kontrolliert
rueckwaerts wieder ab. Dient als risikoarmer Geometrie-Test fuer den
spaeteren vollen Ueberstieg.

Aufstellung: Roboter gerade vor der Kante, Vorderfussspitzen ca. 2 cm vor
der Kante. Die Fuesse landen dann ~2,5 cm auf dem Podest.

Geometrie (Offsets koerperparallel, relativ zur Standpose):
  * Koerper wird um body_raise angehoben (Fuesse dz=-body_raise), damit
    Haltepunkt (height-body_raise) und Schwung-Apex im dauerhaft
    erreichbaren Band BEIDER Vorderbeine liegen (front_left ist durch
    seine Servo-Kalibrierung der Engpass: max ~+64 bei dx=45).
  * Schwung-Apex = height + edge_clearance - body_raise; der Fuss
    ueberquert die Kante mit edge_clearance Luft.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

from hexapod.gait.executor import run_multi_leg_trajectory, run_single_leg_trajectory
from hexapod.gait.trajectory import Vec3, linear_path

if TYPE_CHECKING:
    from hexapod.robot.hexapod import Hexapod

# Verifizierte Grenze: front_left haelt bei dx=45 maximal ~+64 dz.
_MAX_HOLD_DZ = 55.0
_MIN_APEX_DZ_MARGIN = 8.0


def _swing_points(dz_start: float, apex: float, land_dx: float, land_dz: float) -> list[Vec3]:
    """Bahn: senkrecht hoch, vor auf Apex-Hoehe, absenken auf den Landepunkt."""
    p0: Vec3 = (0.0, 0.0, dz_start)
    p1: Vec3 = (0.0, 0.0, apex * 0.5)    # erst halb hoch (steil, nahe Koerper)
    p2: Vec3 = (20.0, 0.0, apex)         # diagonal auf volle Hoehe (weiter
    #   aussen: nahe dx=10 ist der front_left-Arbeitsraum oben zu eng)
    p3: Vec3 = (land_dx - 8.0, 0.0, apex)   # ueber die Kante
    p4: Vec3 = (land_dx, 0.0, land_dz)   # absetzen auf dem Podest
    pts = list(linear_path(p0, p1, 10, include_start=True))
    pts += list(linear_path(p1, p2, 8))
    pts += list(linear_path(p2, p3, 12))
    pts += list(linear_path(p3, p4, 8))
    return pts


def step_up_front_test(
    robot: Hexapod,
    *,
    height: float = 68.0,
    body_raise: float = 55.0,
    land_dx: float = 45.0,
    edge_clearance: float = 15.0,
    hold_s: float = 2.0,
    rate_hz: float = 50.0,
    max_step_deg: float = 2.0,
    clip: bool = True,
) -> None:
    """Vorderbeine auf eine Kante der Hoehe ``height`` setzen und zurueck.

    Erwartet den Roboter stehend (Standpose), gerade vor der Kante,
    Vorderfussspitzen ~2 cm davor. Endet wieder in der Standpose.
    """
    hold_dz = height - body_raise
    apex = height + edge_clearance - body_raise
    if hold_dz > _MAX_HOLD_DZ:
        raise ValueError(
            f"height-body_raise={hold_dz:.0f} ueber sicherem Haltebereich "
            f"({_MAX_HOLD_DZ:.0f}) -- body_raise erhoehen"
        )
    if apex < hold_dz + _MIN_APEX_DZ_MARGIN:
        raise ValueError("edge_clearance zu klein")

    legs = robot.leg_names
    fronts = [leg for leg in legs if leg.startswith("front")]
    ground: Vec3 = (0.0, 0.0, -body_raise)

    # 1) Koerper anheben: alle Fuesse synchron nach unten druecken.
    run_multi_leg_trajectory(
        robot,
        {leg: linear_path(robot.current_offset(leg), ground, 20, include_start=True)
         for leg in legs},
        rate_hz=rate_hz, max_step_deg=max_step_deg, clip=clip,
    )
    time.sleep(0.3)

    # 2) Vorderbeine nacheinander auf das Podest.
    up = _swing_points(-body_raise, apex, land_dx, hold_dz)
    for leg in fronts:
        run_single_leg_trajectory(
            robot, leg, up, rate_hz=rate_hz, max_step_deg=max_step_deg, clip=clip
        )
        time.sleep(0.3)

    # 3) Pose halten (Gewicht vorne oben).
    time.sleep(hold_s)

    # 4) Rueckwaerts wieder herunter (umgekehrte Bahn, umgekehrte Reihenfolge).
    down = list(reversed(up))
    for leg in reversed(fronts):
        run_single_leg_trajectory(
            robot, leg, down, rate_hz=rate_hz, max_step_deg=max_step_deg, clip=clip
        )
        time.sleep(0.3)

    # 5) Koerper zurueck auf Standhoehe.
    run_multi_leg_trajectory(
        robot,
        {leg: linear_path(robot.current_offset(leg), (0.0, 0.0, 0.0), 20,
                          include_start=True) for leg in legs},
        rate_hz=rate_hz, max_step_deg=max_step_deg, clip=clip,
    )


# ---------------------------------------------------------------------
# Stufe 2: voller Ueberstieg ueber ein Podest begrenzter Tiefe
# ---------------------------------------------------------------------

_CYCLE_ORDER = (
    "front_right", "mid_left", "back_right",
    "front_left", "mid_right", "back_left",
)


def plan_climb_over(
    neutral_x: dict[str, float],
    *,
    height: float = 68.0,
    depth: float = 210.0,
    edge_ahead: float = 20.0,
    body_raise: float = 85.0,
    stride: float = 50.0,
    clearance: float = 28.0,
    edge_margin_before: float = 30.0,
    edge_margin_after: float = 55.0,
) -> list[dict[str, list[Vec3]]]:
    """Erzeugt die komplette Ueberstieg-Choreographie als Offset-Plan.

    Kriech-Gang mit Hoehenprofil: jedes Bein schwingt reihum um ``stride``
    vorwaerts und bekommt seine Zielhoehe aus der Lage relativ zum Podest
    (0 -> height -> 0); nach jeder Runde schiebt der Koerper nach. Der
    Koerper bleibt konstant um ``body_raise`` erhoeht -- damit liegen ALLE
    Haltelagen (Boden -70, Podest -2) tief im sicheren Arbeitsband und der
    Bauch behaelt Luft ueber dem Podest.

    Landungen halten Abstand zu beiden Kanten -- ASYMMETRISCH: vor einer
    Kante reichen ``edge_margin_before``, HINTER einer Kante wird mit
    ``edge_margin_after`` deutlich weiter gelandet. Das puffert den real
    akkumulierten Schlupf (der Roboter kommt physisch weniger weit als das
    Modell annimmt) und haelt die Tibia beim Abstieg von der Boxwand weg.
    ``clearance`` enthaelt Reserve fuer Servo-Durchhang unter Last.

    Rein geometrisch (keine Roboter-Zugriffe) -- dadurch kann der ganze
    Plan VOR der Ausfuehrung Punkt fuer Punkt per IK verifiziert werden.

    Args:
        neutral_x: Neutral-Fuss-X je Bein (robot.neutral_foot_xy[leg][0]).
        edge_ahead: Abstand Vorderfussspitzen -> nahe Kante bei Start (mm).
    """
    r = body_raise
    legs = [leg for leg in _CYCLE_ORDER if leg in neutral_x]
    edge_a = max(neutral_x[leg] for leg in legs) + edge_ahead + 25.0
    edge_b = edge_a + depth
    if depth < edge_margin_before + edge_margin_after + stride / 2:
        raise ValueError("Podest zu flach fuer die Kantenabstaende")

    def terrain(x: float) -> float:
        return height if edge_a < x < edge_b else 0.0

    foot_x = dict(neutral_x)
    foot_h = dict.fromkeys(legs, 0.0)
    body_x = 0.0
    plan: list[dict[str, list[Vec3]]] = []

    # 1) Koerper anheben (alle Fuesse synchron nach unten)
    plan.append({
        leg: linear_path((0.0, 0.0, 0.0), (0.0, 0.0, -r), 20, include_start=True)
        for leg in legs
    })

    guard = 0
    while min(foot_x[leg] for leg in legs) < edge_b + edge_margin_after - 1.0 and guard < 40:
        guard += 1
        for leg in legs:
            tgt = foot_x[leg] + stride
            for e in (edge_a, edge_b):
                after = edge_margin_after
                if e == edge_b and leg.startswith("back"):
                    # Hinterbein-Tibia lehnt hinter der Box zurueck zur
                    # Boxwand -- deutlich weiter hinten landen.
                    after += 25.0
                if e - edge_margin_before < tgt < e + after:
                    tgt = (e - edge_margin_before) if tgt < e else (e + after)
            if tgt <= foot_x[leg] + 5.0:
                continue  # Anpassung liesse das Bein quasi stehen
            dxf = foot_x[leg] - body_x - neutral_x[leg]
            dxt = tgt - body_x - neutral_x[leg]
            if dxt > 60.0:
                # Ziel (noch) zu weit vorn -- Schwung um einen Zyklus
                # aufschieben; nach dem naechsten Koerper-Schub passt er.
                continue
            zf = foot_h[leg] - r
            zt = terrain(tgt) - r
            apex = max(foot_h[leg], terrain(tgt)) + clearance - r
            pts = list(linear_path((dxf, 0.0, zf), (dxf + 4.0, 0.0, apex), 10,
                                   include_start=True))
            pts += list(linear_path((dxf + 4.0, 0.0, apex), (dxt - 4.0, 0.0, apex), 10))
            pts += list(linear_path((dxt - 4.0, 0.0, apex), (dxt, 0.0, zt), 8))
            plan.append({leg: pts})
            foot_x[leg] = tgt
            foot_h[leg] = terrain(tgt)
        # Koerper-Schub: alle sechs planted, Offsets -stride (reine Translation)
        shift: dict[str, list[Vec3]] = {}
        for leg in legs:
            dx = foot_x[leg] - body_x - neutral_x[leg]
            z = foot_h[leg] - r
            shift[leg] = list(linear_path((dx, 0.0, z), (dx - stride, 0.0, z), 15,
                                          include_start=True))
        plan.append(shift)
        body_x += stride

    # 3) Koerper absenken (alle Fuesse zurueck auf Bodenniveau-Offsets)
    plan.append({
        leg: linear_path(
            (foot_x[leg] - body_x - neutral_x[leg], 0.0, foot_h[leg] - r),
            (foot_x[leg] - body_x - neutral_x[leg], 0.0, foot_h[leg]),
            20, include_start=True,
        )
        for leg in legs
    })
    return plan


def climb_over_box(
    robot: Hexapod,
    *,
    height: float = 68.0,
    depth: float = 210.0,
    rate_hz: float = 50.0,
    max_step_deg: float = 2.0,
    clip: bool = True,
) -> None:
    """Steigt ueber ein Podest (Default: 68mm hoch, 210mm tief) hinweg.

    Aufstellung wie bei step_up_front_test: Roboter gerade vor der Kante,
    Vorderfussspitzen ~2cm davor. Dauer ca. 1,5-2 Minuten; die Sequenz
    ist waehrenddessen NICHT unterbrechbar (Not-Aus: Dienst-Neustart).
    Endet hinter dem Podest sauber in der Standpose.
    """
    from hexapod.gait.posture import move_to_stance

    neutral_x = {leg: xy[0] for leg, xy in robot.neutral_foot_xy.items()}
    plan = plan_climb_over(neutral_x, height=height, depth=depth)
    for op in plan:
        run_multi_leg_trajectory(
            robot, op, rate_hz=rate_hz, max_step_deg=max_step_deg, clip=clip
        )
        time.sleep(0.15)
    # Offsets sauber auf null (lift-and-place, kein Schleifen)
    move_to_stance(robot)
