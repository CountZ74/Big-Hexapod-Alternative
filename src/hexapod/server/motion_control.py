"""Kontinuierliche Velocity-Steuerung fuer den Hexapod-Webserver.

Liefert das WalkRequest-Modell (API) und den MotionController (thread-confined,
laeuft ausschliesslich im Worker-Thread). Der Controller treibt pro Aufruf von
``step_once`` genau EINEN Tripod-Halbzyklus mit der aktuell gelatchten
Geschwindigkeit. Dadurch wirken Geschwindigkeitsaenderungen an jeder
Halbzyklus-Grenze, und ``halt`` greift ebenfalls an der naechsten Grenze.
"""

from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING

from pydantic import BaseModel

from hexapod.config import Joint
from hexapod.gait.body_motion import Vec2, clamp_command, stride_vectors
from hexapod.gait.continuous import CONTINUOUS_GAITS, cycle_targets
from hexapod.gait.executor import run_multi_leg_trajectory
from hexapod.gait.gaits import TRIPOD, Gait, get_gait, phase_end_offsets
from hexapod.gait.trajectory import Vec3, smoothstep, stance_path, swing_path
from hexapod.gait.tripod import GROUP_A, GROUP_B

if TYPE_CHECKING:
    from hexapod.kinematics.body_ik import BodyPose
    from hexapod.robot.hexapod import Hexapod

logger = logging.getLogger(__name__)

# Kontinuierliche Gangarten (Ripple) werden pro step_once um EINE Slice des
# Zyklus vorgetrieben. Mehr Slices = feinere halt-/Geschwindigkeits-Granular-
# itaet (Werte werden je Slice frisch gelatcht), gleiche Sichtbarkeit wie ein
# Wave-Phasenschritt bei sechs Slices.
_CONT_SLICES_PER_CYCLE = 6


class WalkRequest(BaseModel):
    """Befehl fuer kontinuierliches Laufen im Velocity-Modus.

    vx: Translation vorwaerts(+)/rueckwaerts(-) pro Schritt (mm).
    vy: Translation links(+)/rechts(-) pro Schritt (mm).
    omega_deg: Drehung CCW(+) pro Schritt (Grad).
    height: Schwung-Hubhoehe (mm).
    steps: Punkte pro Halbzyklus.
    rate_hz: Sende-Frequenz der Trajektorie.
    """

    vx: float = 0.0
    vy: float = 0.0
    omega_deg: float = 0.0
    height: float = 30.0
    steps: int = 30
    rate_hz: float = 40.0


class MotionController:
    """Treibt das kontinuierliche Laufen, ein Halbzyklus pro ``step_once``.

    Thread-confined: wird nur aus dem Worker-Thread heraus benutzt.
    Kein Locking noetig -- set_walk/halt werden vom Worker zwischen
    zwei Halbzyklen aufgerufen (nach _drain_commands, vor step_once).
    """

    def __init__(self, robot: Hexapod) -> None:
        self._robot = robot
        self._mode = "idle"
        self._vx = 0.0
        self._vy = 0.0
        self._omega = 0.0  # rad/Schritt
        self._height = 30.0
        self._steps = 30
        self._rate_hz = 40.0
        self._swing_is_a = True
        self._gait: Gait = TRIPOD
        self._phase = 0
        self._cycle_t = 0.0  # kontinuierliche Zyklus-Phase [0,1) fuer Ripple

    def set_walk(
        self,
        vx: float,
        vy: float,
        omega_deg: float,
        *,
        height: float = 30.0,
        steps: int = 30,
        rate_hz: float = 40.0,
    ) -> None:
        """Latcht eine neue Geschwindigkeit und schaltet in den Walk-Modus."""
        self._vx = vx
        self._vy = vy
        self._omega = math.radians(omega_deg)
        self._height = height
        self._steps = steps
        self._rate_hz = rate_hz
        self._mode = "walking"

    def set_gait(self, name: str) -> None:
        """Waehlt die Gangart und setzt den Phasenzaehler zurueck.

        Sollte zwischen zwei Schritten aufgerufen werden (der Aufrufer setzt
        den Roboter ueber die Standpose ab, bevor er die neue Gangart startet).
        """
        self._gait = get_gait(name)
        self._phase = 0
        self._swing_is_a = True
        self._cycle_t = 0.0

    @property
    def gait_name(self) -> str:
        return self._gait.name

    def halt(self) -> None:
        """Stoppt das Laufen an der naechsten Halbzyklus-Grenze."""
        self._mode = "idle"

    @property
    def is_walking(self) -> bool:
        return self._mode == "walking"

    _JOINTS = (Joint.COXA, Joint.FEMUR, Joint.TIBIA)

    def _reachable(self, leg: str, off: Vec3, *, margin_us: float = 20.0) -> bool:
        """True, wenn der Fuss-Offset OHNE Clipping erreichbar ist.

        Prueft die drei Gelenke gegen ihre Servo-Grenzen (min_us/max_us) mit
        einem kleinen Sicherheitsabstand. Geometrisch unmoegliche Punkte
        (ausserhalb der Beinreichweite) gelten ebenfalls als nicht erreichbar.
        """
        r = self._robot
        try:
            angles = r.offset_to_angles(leg, off[0], off[1], off[2])
        except Exception:
            return False
        for joint, ang in zip(self._JOINTS, angles, strict=True):
            ch = r.config.get_leg_servo(leg, joint).channel
            mp = r._mappings[ch]
            us = mp.angle_to_us(ang, clip=True)
            if us <= mp.min_us + margin_us or us >= mp.max_us - margin_us:
                return False
        return True

    def _limit_stride_for_pose(
        self, strides: dict[str, Vec2], pose: BodyPose
    ) -> dict[str, Vec2]:
        """Begrenzt die Schrittweite, damit Pose + Schritt erreichbar bleiben.

        Bei gehaltener Pose verbraucht der Koerper bereits einen Teil der
        Beinreichweite. Ein voller Schritt wuerde dann an die Gelenkgrenze
        clippen -> die Bahn franst aus. Hier wird der groesste noch erreichbare
        Skalierungsfaktor des Schritts gesucht (Binaersuche) und angewandt.
        Bei neutraler Pose unveraendert (Gang ist bereits sicher geclamped).
        """
        if (pose.tx == 0.0 and pose.ty == 0.0 and pose.tz == 0.0
                and pose.roll == 0.0 and pose.pitch == 0.0 and pose.yaw == 0.0):
            return strides
        legs = list(strides.keys())

        def reachable_at(sf: float) -> bool:
            for leg in legs:
                dx, dy = strides[leg]
                hx, hy = dx * 0.5 * sf, dy * 0.5 * sf
                for off in ((hx, hy, 0.0), (-hx, -hy, 0.0)):
                    c = self._compose_offsets({leg: off}, pose)[leg]
                    if not self._reachable(leg, c):
                        return False
            return True

        if reachable_at(1.0):
            return strides
        if not reachable_at(0.0):
            logger.warning(
                "Pose allein an der Gelenkgrenze -- Schritt auf 0 begrenzt"
            )
            return {leg: (0.0, 0.0) for leg in legs}
        lo, hi = 0.0, 1.0
        for _ in range(14):
            mid = (lo + hi) / 2.0
            if reachable_at(mid):
                lo = mid
            else:
                hi = mid
        sf = lo * 0.95  # kleiner Sicherheitsabstand
        logger.info("Schrittweite wegen Pose auf %.0f%% begrenzt", sf * 100.0)
        return {leg: (dx * sf, dy * sf) for leg, (dx, dy) in strides.items()}

    def _compose_offsets(
        self, off: dict[str, Vec3], pose: BodyPose
    ) -> dict[str, Vec3]:
        """Legt die gehaltene Body-Pose ueber einen Satz Gang-Offsets.

        Nimmt {leg: (dx, dy, dz)} (koerperparallel, relativ zur Standpose) und
        liefert pose-komponierte Standpose-Offsets zurueck: der Fuss plant
        seinen Punkt am Boden, der Koerper behaelt Translation/Neigung/Gier.
        Bei neutraler Pose unveraendert (mathematisches No-op).
        """
        if (pose.tx == 0.0 and pose.ty == 0.0 and pose.tz == 0.0
                and pose.roll == 0.0 and pose.pitch == 0.0 and pose.yaw == 0.0):
            return off
        from hexapod.kinematics.body_ik import body_pose_offsets
        robot = self._robot
        fpw = robot._foot_positions_world
        coxa = robot._coxa_positions
        neutral = robot._neutral_world
        world = {
            leg: (
                float(fpw[leg][0]) + off[leg][0],
                float(fpw[leg][1]) + off[leg][1],
                float(fpw[leg][2]) + off[leg][2],
            )
            for leg in off
        }
        return body_pose_offsets(pose, world, coxa, neutral)

    def step_once(self) -> None:
        """Laeuft genau einen Halbzyklus. Bei Fehler -> idle (Sicherheit).

        Jeder Halbzyklus startet aus der TATSAECHLICHEN Ist-Lage der Fuesse:
        Schwungbeine schwingen per Bogen (smoothstep -> vertikales Abheben) aus
        ihrer aktuellen Position zum neuen Ziel, Standbeine schieben am Boden.
        Dadurch wird bei Geschwindigkeits-/Vorzeichenwechseln kein Fuss ueber
        den Boden zur neuen Startposition gezogen -- er hebt ab. Die gehaltene
        Body-Pose wird in die Zielpunkte komponiert.
        """
        if self._mode != "walking":
            return
        try:
            robot = self._robot
            feet = robot.neutral_foot_xy
            foot_radius = max((fx * fx + fy * fy) ** 0.5 for fx, fy in feet.values())
            cvx, cvy, comega = clamp_command(
                self._vx, self._vy, self._omega,
                max_translation=50.0, max_rotation=0.30, foot_radius=foot_radius,
            )
            strides = stride_vectors(feet, cvx, cvy, comega)
            # Schrittweite an die gehaltene Pose anpassen, damit Pose +
            # Schritt nicht an die Gelenkgrenze clippen (kein Ausfransen).
            strides = self._limit_stride_for_pose(strides, robot.body_pose)
            if self._gait.name == "tripod":
                self._step_tripod(strides)
            elif self._gait.name in CONTINUOUS_GAITS:
                self._step_continuous(strides)
            else:
                self._step_phase(strides)
        except Exception as e:
            logger.warning("Schritt fehlgeschlagen, stoppe: %s", e)
            self._mode = "idle"

    def _step_tripod(self, strides: dict[str, Vec2]) -> None:
        """Klassischer Tripod-Halbzyklus (zentriert, +/-v/2). Unveraendert."""
        robot = self._robot
        swing_group = GROUP_A if self._swing_is_a else GROUP_B
        stance_group = GROUP_B if self._swing_is_a else GROUP_A
        height = self._height
        steps = self._steps
        pose = robot.body_pose

        end_unposed: dict[str, Vec3] = {}
        for leg in swing_group:
            dx, dy = strides[leg]
            end_unposed[leg] = (-dx / 2.0, -dy / 2.0, 0.0)
        for leg in stance_group:
            dx, dy = strides[leg]
            end_unposed[leg] = (dx / 2.0, dy / 2.0, 0.0)
        end = self._compose_offsets(end_unposed, pose)

        start = {
            leg: robot.current_offset(leg)
            for leg in (*swing_group, *stance_group)
        }

        paths: dict[str, list[Vec3]] = {}
        for leg in swing_group:
            paths[leg] = swing_path(
                start[leg], end[leg], height, steps,
                include_start=True, ease=smoothstep,
            )
        for leg in stance_group:
            paths[leg] = stance_path(
                start[leg], end[leg], steps,
                include_start=True, ease=smoothstep,
            )

        run_multi_leg_trajectory(
            robot, paths, rate_hz=self._rate_hz, max_step_deg=3.0,
            clip=True, start=start,
        )
        self._swing_is_a = not self._swing_is_a

    def _step_phase(self, strides: dict[str, Vec2]) -> None:
        """Eine Phase einer mehrphasigen Gangart (tetrapod/ripple/wave).

        Schwungbeine dieser Phase heben ab und holen ihre ueber die
        Standphasen zurueckgelegte Strecke auf; alle anderen schieben am Boden
        ein Inkrement weiter. Ziel-Offsets liefert die akkumulierende
        Phasen-Mechanik (phase_end_offsets); der Start ist die tatsaechliche
        Ist-Lage (Bruecke), sodass nichts ueber den Boden gezogen wird.
        """
        robot = self._robot
        gait = self._gait
        k = self._phase
        pose = robot.body_pose
        swing = set(gait.phases[k])

        end_unposed = phase_end_offsets(gait, k, strides)
        end = self._compose_offsets(end_unposed, pose)
        start = {leg: robot.current_offset(leg) for leg in end_unposed}

        height = self._height
        steps = self._steps
        paths: dict[str, list[Vec3]] = {}
        for leg in end_unposed:
            if leg in swing:
                paths[leg] = swing_path(
                    start[leg], end[leg], height, steps,
                    include_start=True, ease=smoothstep,
                )
            else:
                paths[leg] = stance_path(
                    start[leg], end[leg], steps,
                    include_start=True, ease=smoothstep,
                )

        run_multi_leg_trajectory(
            robot, paths, rate_hz=self._rate_hz, max_step_deg=3.0,
            clip=True, start=start,
        )
        self._phase = (k + 1) % gait.n_phases

    def _step_continuous(self, strides: dict[str, Vec2]) -> None:
        """Eine Slice einer ueberlappenden Gangart (Ripple).

        Treibt die globale Zyklus-Phase um ``1/_CONT_SLICES_PER_CYCLE`` vor.
        Die Fuss-Bahn jedes Beins wird laengs des kontinuierlichen Modells
        (continuous.cycle_targets) fein abgetastet -- so entsteht der echte
        Bogen mit Z-Hub auch dann, wenn ein Bein quer ueber Slice-Grenzen
        schwingt (zwei Beine sind bei Duty 2/3 staendig gleichzeitig in der
        Luft). Gestartet wird aus der tatsaechlichen Ist-Lage (Bruecke), die
        gehaltene Body-Pose wird in die Ziele komponiert.
        """
        robot = self._robot
        gait = CONTINUOUS_GAITS[self._gait.name]
        pose = robot.body_pose
        height = self._height
        steps = self._steps
        legs = list(strides.keys())

        n = _CONT_SLICES_PER_CYCLE
        dtc = 1.0 / n
        t0 = self._cycle_t

        paths: dict[str, list[Vec3]] = {leg: [] for leg in legs}
        for j in range(1, steps + 1):
            tj = (t0 + (j / steps) * dtc) % 1.0
            targ = self._compose_offsets(cycle_targets(gait, strides, height, tj), pose)
            for leg in legs:
                paths[leg].append(targ[leg])

        start = {leg: robot.current_offset(leg) for leg in legs}
        run_multi_leg_trajectory(
            robot, paths, rate_hz=self._rate_hz, max_step_deg=3.0,
            clip=True, start=start,
        )
        self._cycle_t = (t0 + dtc) % 1.0
