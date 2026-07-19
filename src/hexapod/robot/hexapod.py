"""Die zentrale Hexapod-Klasse.

Bringt alle Schichten zusammen:
  Config → Driver → ServoMapping → IK → Servos

Design-Prinzip: diese Klasse enthält KEINE Mathematik und KEINE
Hardware-Details. Sie delegiert alles an die Fachmodule.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from pathlib import Path
from types import TracebackType

import numpy as np

from hexapod.config import (
    CameraAxis,
    Joint,
    RobotConfig,
)
from hexapod.config.loader import load_robot_config
from hexapod.drivers.base import ServoDriver
from hexapod.drivers.maestro import MaestroDriver
from hexapod.drivers.simulator import SimulatorDriver
from hexapod.kinematics import (
    BodyPose,
    LegLengths,
    body_ik,
    forward_kinematics,
    inverse_kinematics,
)
from hexapod.kinematics.body_ik import Vec3
from hexapod.servo_mapper import ServoMapping

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------
# Datenklasse für den Zustand eines einzelnen Beins
# ---------------------------------------------------------------------


@dataclass
class LegState:
    """Aktueller Zustand eines Beins (Position + Winkel)."""

    name: str
    foot_x: float = 0.0
    foot_y: float = 0.0
    foot_z: float = 0.0
    theta1: float = 0.0  # coxa [rad]
    theta2: float = 0.0  # femur [rad]
    theta3: float = 0.0  # tibia [rad]


# ---------------------------------------------------------------------
# Die Hexapod-Klasse
# ---------------------------------------------------------------------


class Hexapod:
    """High-Level-Steuerung für den gesamten Hexapod.

    Typische Nutzung::

        with Hexapod.from_config("config/robot.yaml") as robot:
            robot.home()
            robot.set_body_pose(tz=20)

    Oder mit Simulator (kein Maestro nötig)::

        config = load_robot_config("config/robot.yaml")
        data = config.model_dump()
        data["driver"] = {"type": "simulator", ...}
        with Hexapod(config.model_validate(data)) as robot:
            ...
    """

    def __init__(self, config: RobotConfig) -> None:
        self._config = config
        self._config_path: Path | None = None
        self._driver = self._create_driver()

        # Bein-Geometrie (für alle Beine gleich)
        geo = config.body.leg_geometry
        self._leg_lengths = LegLengths(
            coxa=geo.coxa_length,
            femur=geo.femur_length,
            tibia=geo.tibia_length,
        )

        # Pro Servo ein Mapping erzeugen (channel → ServoMapping)
        self._mappings: dict[int, ServoMapping] = {}
        for servo in config.servos:
            self._mappings[servo.channel] = ServoMapping(
                center_us=servo.center_us,
                range_us=servo.range_us,
                direction=servo.direction,
                min_us=servo.min_us,
                max_us=servo.max_us,
            )

        # Bein-Zustand tracken
        self._leg_states: dict[str, LegState] = {
            leg.name: LegState(name=leg.name)
            for leg in config.body.legs
        }

        # Bein-Namen in der konfigurierten Reihenfolge
        self._leg_names = [leg.name for leg in config.body.legs]

        # Coxa-Positionen im Body-Frame (aus Konfig)
        self._coxa_positions: dict[str, Vec3] = {
            leg.name: np.array([leg.mount_x, leg.mount_y, 0.0], dtype=np.float64)
            for leg in config.body.legs
        }

        # Mount-Winkel pro Bein (Radiant) — für Weltframe↔Leg-Frame Transformation
        self._mount_angles: dict[str, float] = {
            leg.name: math.radians(leg.mount_angle_deg)
            for leg in config.body.legs
        }

        # Standpose-Parameter: der Fuß steht in Ruhe bei θ1=0 (Coxa-Mitte),
        # also in Mount-Richtung. Wir berechnen den Leg-Frame-Standpunkt einmal
        # per FK und leiten daraus den körperparallelen Neutralpunkt ab.
        self._stance_femur = math.radians(45.0)
        self._stance_tibia = math.radians(135.0)
        from hexapod.kinematics.leg_ik import forward_kinematics
        sx, _sy, sz = forward_kinematics(
            0.0, self._stance_femur, self._stance_tibia, self._leg_lengths
        )
        # sx = horizontaler Abstand (Leg-X), sy ≈ 0, sz = Höhe
        self._stance_radius = sx       # horizontaler Abstand Coxa→Fuß
        self._stance_z = sz            # Fuß-Höhe relativ zur Coxa
        # Z-Trim pro Bein (mm) zum Ausgleich ungleichen Bodenkontakts.
        # Positiv = Fuß tiefer (Standpose-Z weiter nach unten).
        self._z_trim: dict[str, float] = {
            leg.name: leg.z_trim for leg in config.body.legs
        }

        # Neutralpunkt im körperparallelen Frame (Ursprung = Coxa):
        # liegt in Mount-Richtung im Abstand stance_radius. Der Z-Trim
        # verschiebt die Standpose-Höhe individuell pro Bein.
        self._neutral_world: dict[str, tuple[float, float, float]] = {}
        for name, ma in self._mount_angles.items():
            nx = self._stance_radius * math.cos(ma)
            ny = self._stance_radius * math.sin(ma)
            nz = self._stance_z - self._z_trim[name]
            self._neutral_world[name] = (nx, ny, nz)

        # Aktuelle Körper-Pose (startet neutral)
        self._body_pose = BodyPose.neutral()

        # Neutral-Fußpositionen im Body-Frame, abgeleitet aus DERSELBEN
        # Standpose wie Stance/Gait. _neutral_world ist coxa-relativ und
        # körperparallel; plus die Coxa-Position ergibt die absolute Lage
        # im Body-Frame (inkl. Z-Trim). So haben Körperpose und Gait genau
        # EINEN gemeinsamen Nullpunkt.
        self._foot_positions_world: dict[str, Vec3] = {
            name: self._coxa_positions[name]
            + np.array(self._neutral_world[name], dtype=np.float64)
            for name in self._leg_names
        }

        logger.info(
            "Hexapod initialisiert: %s, %d Beine, %d Servos, Driver=%s",
            config.name,
            len(config.body.legs),
            len(config.servos),
            config.driver.type,
        )

    # ---- Factory-Methoden ----

    @classmethod
    def from_config(cls, path: str | Path) -> Hexapod:
        """Erzeugt einen Hexapod aus einer YAML-Konfiguration."""
        config = load_robot_config(path)
        robot = cls(config)
        robot._config_path = Path(path)
        return robot

    # ---- Driver-Erzeugung ----

    def _create_driver(self) -> ServoDriver:
        dc = self._config.driver
        if dc.type == "maestro":
            lim = self._config.servo_limits
            return MaestroDriver(
                port=dc.port,
                num_channels=dc.num_channels,
                timeout=dc.timeout,
                min_pulse_us=lim.absolute_min_us,
                max_pulse_us=lim.absolute_max_us,
            )
        elif dc.type == "simulator":
            lim = self._config.servo_limits
            return SimulatorDriver(
                num_channels=dc.num_channels,
                verbose=True,
                min_pulse_us=lim.absolute_min_us,
                max_pulse_us=lim.absolute_max_us,
            )
        else:
            raise ValueError(f"Unbekannter Driver-Typ: {dc.type!r}")

    # ---- Properties ----

    @property
    def config(self) -> RobotConfig:
        return self._config

    @property
    def driver(self) -> ServoDriver:
        return self._driver

    @property
    def leg_names(self) -> list[str]:
        return list(self._leg_names)

    @property
    def leg_lengths(self) -> LegLengths:
        return self._leg_lengths

    @property
    def neutral_foot_xy(self) -> dict[str, tuple[float, float]]:
        """Neutral-Fußposition (x, y) je Bein relativ zum Körperzentrum (mm).

        Das sind die Standpunkte, um die herum die Gait-Trajektorien als
        Offsets schwingen — genau die Eingabe für ``stride_vectors``.
        """
        # _neutral_world ist coxa-zentriert (reine Mount-Rotation, keine
        # Translation). Für stride_vectors muss r vom KÖRPERZENTRUM aus zählen,
        # sonst ist der Rotationsterm omega×r zu klein → Drehung rutscht.
        return {
            name: (nx + self._coxa_positions[name][0],
                   ny + self._coxa_positions[name][1])
            for name, (nx, ny, _nz) in self._neutral_world.items()
        }

    @property
    def body_pose(self) -> BodyPose:
        """Aktuelle Körper-Pose."""
        return self._body_pose

    @property
    def foot_positions_world(self) -> dict[str, Vec3]:
        """Aktuelle Fußpositionen im Welt-Frame (Kopie)."""
        return dict(self._foot_positions_world)

    def get_leg_state(self, leg_name: str) -> LegState:
        """Gibt den aktuellen Zustand eines Beins zurück."""
        if leg_name not in self._leg_states:
            raise KeyError(f"Unbekanntes Bein: {leg_name!r}")
        return self._leg_states[leg_name]

    # ---- Einzelnes Bein steuern ----

    def set_leg_angles(
        self,
        leg_name: str,
        theta1: float,
        theta2: float,
        theta3: float,
        *,
        clip: bool = False,
    ) -> None:
        """Setze die drei Gelenkwinkel eines Beins direkt (in Radiant)."""
        joints = [
            (Joint.COXA, theta1),
            (Joint.FEMUR, theta2),
            (Joint.TIBIA, theta3),
        ]

        positions: dict[int, float] = {}
        for joint, angle in joints:
            servo_cfg = self._config.get_leg_servo(leg_name, joint)
            mapping = self._mappings[servo_cfg.channel]
            us = mapping.angle_to_us(angle, clip=clip)
            positions[servo_cfg.channel] = us

        self._driver.set_positions(positions)

        state = self._leg_states[leg_name]
        state.theta1 = theta1
        state.theta2 = theta2
        state.theta3 = theta3
        state.foot_x, state.foot_y, state.foot_z = forward_kinematics(
            theta1, theta2, theta3, self._leg_lengths,
        )

    def set_foot_position(
        self,
        leg_name: str,
        x: float,
        y: float,
        z: float,
        *,
        clip: bool = False,
    ) -> None:
        """Setze die Fußposition eines Beins im Leg Frame (mm)."""
        theta1, theta2, theta3 = inverse_kinematics(x, y, z, self._leg_lengths)
        self.set_leg_angles(leg_name, theta1, theta2, theta3, clip=clip)

    def set_foot_position_world(
        self,
        leg_name: str,
        x: float,
        y: float,
        z: float,
        *,
        clip: bool = False,
    ) -> None:
        """Setze die Fußposition eines Beins im Weltframe (mm).

        Der Ursprung liegt am Coxa-Gelenk des Beins. Die Achsen sind
        parallel zum Body-Frame (X = vorwärts, Y = links, Z = oben).
        Die Transformation in den Leg Frame rotiert um den negativen
        Mount-Winkel des Beins.

        Args:
            leg_name: Name des Beins.
            x: Fußposition in Weltframe-X (vorwärts) relativ zur Coxa, mm.
            y: Fußposition in Weltframe-Y (links) relativ zur Coxa, mm.
            z: Fußposition in Weltframe-Z (oben) relativ zur Coxa, mm.
            clip: Wenn True, werden Winkel auf den zulässigen Bereich geclippt.
        """
        ma = self._mount_angles[leg_name]
        # Rotation um -mount_angle (Weltframe → Leg Frame)
        lx =  x * math.cos(ma) + y * math.sin(ma)
        ly = -x * math.sin(ma) + y * math.cos(ma)
        lz = z
        self.set_foot_position(leg_name, lx, ly, lz, clip=clip)

    def set_all_foot_positions_world(
        self,
        targets: dict[str, tuple[float, float, float]],
        *,
        clip: bool = False,
    ) -> None:
        """Setze Fußpositionen für mehrere Beine im Weltframe.

        Wie set_foot_position_world, aber für mehrere Beine gleichzeitig.
        Alle Servo-Befehle werden in einem einzigen USB-Paket gesendet.
        """
        leg_frame_targets: dict[str, tuple[float, float, float]] = {}
        for leg_name, (x, y, z) in targets.items():
            ma = self._mount_angles[leg_name]
            lx =  x * math.cos(ma) + y * math.sin(ma)
            ly = -x * math.sin(ma) + y * math.cos(ma)
            leg_frame_targets[leg_name] = (lx, ly, z)
        self.set_all_foot_positions(leg_frame_targets, clip=clip)

    # ---- Offset-basiert relativ zur Standpose ----

    def set_foot_offset(
        self,
        leg_name: str,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        *,
        clip: bool = True,
    ) -> None:
        """Bewege den Fuß relativ zur Standpose im körperparallelen Frame.

        (dx, dy, dz) = (0, 0, 0) ist immer die Standpose (Coxa-Mitte, θ1=0).
        dx = Bewegung in Körper-X (vorwärts), dy = Körper-Y (links),
        dz = Körper-Z (oben). So bleibt der Coxa automatisch um 0° zentriert
        und man muss nie über Mount-Winkel nachdenken.
        """
        nx, ny, nz = self._neutral_world[leg_name]
        self.set_foot_position_world(
            leg_name, nx + dx, ny + dy, nz + dz, clip=clip
        )

    def set_all_foot_offsets(
        self,
        offsets: dict[str, tuple[float, float, float]],
        *,
        clip: bool = True,
    ) -> None:
        """Wie set_foot_offset, aber für mehrere Beine in einem USB-Paket."""
        targets: dict[str, tuple[float, float, float]] = {}
        for leg_name, (dx, dy, dz) in offsets.items():
            nx, ny, nz = self._neutral_world[leg_name]
            targets[leg_name] = (nx + dx, ny + dy, nz + dz)
        self.set_all_foot_positions_world(targets, clip=clip)

    def stance(self, *, clip: bool = True) -> None:
        """Fahre alle Beine in die Standpose (alle Offsets = 0)."""
        self.set_all_foot_offsets({n: (0.0, 0.0, 0.0) for n in self._leg_names}, clip=clip)

    def current_offset(self, leg_name: str) -> tuple[float, float, float]:
        """Aktueller Fuß-Offset (dx, dy, dz) relativ zur Standpose.

        Rekonstruiert aus dem getrackten Gelenk-Zustand. Gibt (0,0,0)
        zurück, wenn das Bein exakt in der Standpose steht.
        """
        state = self._leg_states[leg_name]
        # Aktuelle Fußposition im Leg Frame aus den Winkeln:
        fx, fy, fz = forward_kinematics(
            state.theta1, state.theta2, state.theta3, self._leg_lengths
        )
        # Leg Frame -> körperparalleler Frame (Rotation um +mount_angle):
        ma = self._mount_angles[leg_name]
        wx = fx * math.cos(ma) - fy * math.sin(ma)
        wy = fx * math.sin(ma) + fy * math.cos(ma)
        # Offset = aktuelle Weltposition - Neutralpunkt:
        nx, ny, nz = self._neutral_world[leg_name]
        return (wx - nx, wy - ny, fz - nz)

    def get_z_trim(self, leg_name: str) -> float:
        """Aktueller Z-Trim eines Beins (mm)."""
        return self._z_trim[leg_name]

    def set_z_trim(self, leg_name: str, value: float) -> None:
        """Setzt den Z-Trim eines Beins (mm) und aktualisiert den Neutralpunkt.

        Positiv = Fuß tiefer (Bein drückt mehr nach unten / trägt mehr).
        Wirkt sofort auf alle Offset-basierten Bewegungen (Standpose, Gait).
        Wird NICHT automatisch gespeichert — dafür save_z_trims() nutzen.
        """
        self._z_trim[leg_name] = value
        nx, ny, _ = self._neutral_world[leg_name]
        self._neutral_world[leg_name] = (nx, ny, self._stance_z - value)

    def save_z_trims(self, path: str | Path | None = None) -> Path:
        """Schreibt die aktuellen Z-Trims zurück in die robot.yaml.

        Args:
            path: Zielpfad. Default: die Datei, aus der geladen wurde.

        Returns:
            Der geschriebene Pfad.
        """
        import yaml

        target = Path(path) if path is not None else self._config_path
        if target is None:
            raise ValueError("Kein Config-Pfad bekannt; bitte path angeben.")

        data = yaml.safe_load(target.read_text())
        for leg in data["body"]["legs"]:
            leg["z_trim"] = round(float(self._z_trim[leg["name"]]), 2)
        target.write_text(
            yaml.dump(data, default_flow_style=False, allow_unicode=True, sort_keys=False)
        )
        logger.info("Z-Trims gespeichert nach %s", target)
        return target

    def prime(self) -> int:
        """Bereite sanftes Anfahren vor (siehe MaestroDriver.prime).

        Schreibt die aktuelle Servoposition einmal zurück, damit der
        ungebremste erste Maestro-Zug keine sichtbare Bewegung erzeugt.
        Bei Drivern ohne prime() (z.B. Simulator) eine No-Op.

        Returns:
            Anzahl geprimeter Kanäle (0 wenn der Driver es nicht unterstützt).
        """
        prime_fn = getattr(self._driver, "prime", None)
        if prime_fn is None:
            return 0
        n: int = prime_fn()
        logger.info("prime(): %d Kanäle vorbereitet", n)
        return n


    def reconstruct_state_from_hardware(self) -> None:
        """Liest alle Servo-Positionen vom Maestro, konvertiert zu Winkeln."""
        for leg_name in self._leg_names:
            try:
                angles: dict[Joint, float] = {}
                for joint in (Joint.COXA, Joint.FEMUR, Joint.TIBIA):
                    servo_cfg = self._config.get_leg_servo(leg_name, joint)
                    channel = servo_cfg.channel
                    us = self._driver.get_position(channel)
                    angles[joint] = self._mappings[channel].us_to_angle(us)
                theta1 = angles[Joint.COXA]
                theta2 = angles[Joint.FEMUR]
                theta3 = angles[Joint.TIBIA]
                lx, ly, lz = forward_kinematics(theta1, theta2, theta3, self._leg_lengths)
                ma = self._mount_angles[leg_name]
                cos_ma = math.cos(ma)
                sin_ma = math.sin(ma)
                body_x = lx * cos_ma - ly * sin_ma
                body_y = lx * sin_ma + ly * cos_ma
                body_z = lz
                coxa_pos = self._coxa_positions[leg_name]
                world_x = body_x + coxa_pos[0]
                world_y = body_y + coxa_pos[1]
                world_z = body_z + coxa_pos[2]
                state = self._leg_states[leg_name]
                # LegState speichert Leg-Frame-Koordinaten (wie set_leg_angles):
                # rohe FK-Ausgabe, OHNE Mount-Rotation/Coxa-Offset.
                state.foot_x = lx
                state.foot_y = ly
                state.foot_z = lz
                state.theta1 = theta1
                state.theta2 = theta2
                state.theta3 = theta3
                self._foot_positions_world[leg_name] = np.array([world_x, world_y, world_z], dtype=np.float64)
                logger.debug("reconstruct %s: th1=%.2f°, th2=%.2f°, th3=%.2f° → world(%.1f, %.1f, %.1f) mm", leg_name, math.degrees(theta1), math.degrees(theta2), math.degrees(theta3), world_x, world_y, world_z)
            except Exception as e:
                logger.error("Konnte Zustand von Bein %r nicht rekonstruieren: %s", leg_name, e)
                raise
        logger.info("Hardware-Zustand erfolgreich rekonstruiert (%d Beine)", len(self._leg_names))

    def read_servo_state(self) -> dict[int, dict[str, float | None]]:
        """Rein lesender Snapshot aller Servo-Kanaele (mutiert NICHTS).

        Liest pro Kanal die zuletzt gehaltene Pulsweite vom Treiber zurueck
        und rechnet sie (wo moeglich) in einen Gelenkwinkel um. Deaktivierte
        Kanaele (us<=0) oder Konvertierungsfehler liefern angle_rad=None,
        statt eine Exception zu werfen -- damit ist die Methode fuer eine
        Telemetrie-Schleife robust.

        Returns:
            {channel: {"us": float|None, "angle_rad": float|None}}
        """
        out: dict[int, dict[str, float | None]] = {}
        for servo in self._config.servos:
            ch = servo.channel
            try:
                us = self._driver.get_position(ch)
            except Exception:
                out[ch] = {"us": None, "angle_rad": None}
                continue
            angle: float | None = None
            if us and us > 0:
                try:
                    angle = self._mappings[ch].us_to_angle(us)
                except Exception:
                    angle = None
            out[ch] = {"us": us, "angle_rad": angle}
        return out

    def goto_stance(
        self,
        *,
        speed_mm_s: float = 60.0,
        rate_hz: float = 40.0,
        max_step_deg: float = 3.0,
        clip: bool = True,
    ) -> None:
        """Fahre sanft (interpoliert) aus der aktuellen Pose in die Standpose.

        Anders als stance() (ein harter Sprung) interpoliert diese Methode
        über den Trajektorien-Executor, sodass die Bewegung kontrolliert
        langsam ist — auch wenn Speed/Acceleration am Maestro freigegeben sind.

        Args:
            speed_mm_s: Ungefähre Fußgeschwindigkeit in mm/s (bestimmt die
                Anzahl der Zwischenschritte).
            rate_hz: Sende-Frequenz.
            max_step_deg: Max. Gelenksprung pro Takt.
            clip: Winkel-Clipping.
        """
        from hexapod.gait.executor import run_multi_leg_trajectory
        from hexapod.gait.trajectory import linear_path

        # Größte zurückzulegende Distanz bestimmt die Schrittzahl.
        max_dist = 0.0
        starts: dict[str, tuple[float, float, float]] = {}
        for leg in self._leg_names:
            off = self.current_offset(leg)
            starts[leg] = off
            dist = math.sqrt(off[0] ** 2 + off[1] ** 2 + off[2] ** 2)
            max_dist = max(max_dist, dist)

        if max_dist < 0.5:
            return  # schon in Standpose

        duration = max_dist / speed_mm_s
        steps = max(2, int(duration * rate_hz))

        # Lineare Bahn von aktuellem Offset nach (0,0,0) für jedes Bein.
        leg_points = {
            leg: linear_path(starts[leg], (0.0, 0.0, 0.0), steps)
            for leg in self._leg_names
        }
        run_multi_leg_trajectory(
            self, leg_points, rate_hz=rate_hz, max_step_deg=max_step_deg, clip=clip
        )

    def offset_to_angles(
        self,
        leg_name: str,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
    ) -> tuple[float, float, float]:
        """Rechnet einen Fuß-Offset in Gelenkwinkel um, OHNE zu senden.

        Nützlich um vorauszuschauen, wie groß ein Gelenksprung zwischen
        zwei Bahnpunkten wäre (z.B. für adaptive Schrittunterteilung).

        Returns:
            (theta1, theta2, theta3) in Radiant.
        """
        nx, ny, nz = self._neutral_world[leg_name]
        x, y, z = nx + dx, ny + dy, nz + dz
        ma = self._mount_angles[leg_name]
        lx = x * math.cos(ma) + y * math.sin(ma)
        ly = -x * math.sin(ma) + y * math.cos(ma)
        return inverse_kinematics(lx, ly, z, self._leg_lengths)

    # ---- Alle Beine gleichzeitig ----

    def set_all_foot_positions(
        self,
        targets: dict[str, tuple[float, float, float]],
        *,
        clip: bool = False,
    ) -> None:
        """Setze Fußpositionen für mehrere Beine gleichzeitig."""
        all_positions: dict[int, float] = {}

        for leg_name, (x, y, z) in targets.items():
            theta1, theta2, theta3 = inverse_kinematics(x, y, z, self._leg_lengths)

            joints = [
                (Joint.COXA, theta1),
                (Joint.FEMUR, theta2),
                (Joint.TIBIA, theta3),
            ]

            for joint, angle in joints:
                servo_cfg = self._config.get_leg_servo(leg_name, joint)
                mapping = self._mappings[servo_cfg.channel]
                us = mapping.angle_to_us(angle, clip=clip)
                all_positions[servo_cfg.channel] = us

            state = self._leg_states[leg_name]
            state.theta1 = theta1
            state.theta2 = theta2
            state.theta3 = theta3
            state.foot_x, state.foot_y, state.foot_z = x, y, z

        self._driver.set_positions(all_positions)

    # ---- Body-Pose ----

    def set_body_pose(
        self,
        *,
        tx: float = 0.0,
        ty: float = 0.0,
        tz: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        clip: bool = True,
    ) -> None:
        """Setze die Körper-Pose (Translation + Rotation).

        Berechnet für jedes Bein die neue Fußposition im Leg-Frame
        und schickt alle Servos in einem Batch.

        Args:
            tx, ty, tz: Translation in mm.
            roll, pitch, yaw: Rotation in Radiant.
            clip: Servo-Grenzen still klemmen statt Fehler werfen.
        """
        pose = BodyPose(tx=tx, ty=ty, tz=tz, roll=roll, pitch=pitch, yaw=yaw)
        leg_targets = body_ik(pose, self._foot_positions_world, self._coxa_positions)
        self.set_all_foot_positions_world(leg_targets, clip=clip)
        self._body_pose = pose

    def set_foot_positions_world(
        self, positions: dict[str, tuple[float, float, float]]
    ) -> None:
        """Aktualisiert die Welt-Frame-Fußpositionen (z.B. beim Gehen)."""
        for name, (x, y, z) in positions.items():
            self._foot_positions_world[name] = np.array(
                [x, y, z], dtype=np.float64
            )

    # ---- Kamera ----

    def set_camera(
        self, pan_deg: float = 0.0, tilt_deg: float = 0.0, *, clip: bool = True
    ) -> None:
        """Setzt Pan/Tilt der Kamera in Grad."""
        positions: dict[int, float] = {}
        for axis, angle_deg in [
            (CameraAxis.PAN, pan_deg),
            (CameraAxis.TILT, tilt_deg),
        ]:
            try:
                servo_cfg = self._config.get_camera_servo(axis)
            except KeyError:
                continue
            mapping = self._mappings[servo_cfg.channel]
            us = mapping.angle_to_us(math.radians(angle_deg), clip=clip)
            positions[servo_cfg.channel] = us
        if positions:
            self._driver.set_positions(positions)

    # ---- Convenience-Methoden ----

    def home(self, *, clip: bool = False) -> None:
        """Alle Beine in die Neutralstellung (alle Winkel = 0)."""
        for name in self._leg_names:
            self.set_leg_angles(name, 0.0, 0.0, 0.0, clip=clip)

    def safe_start(
        self,
        *,
        speed: int = 10,
        acceleration: int = 3,
        delay_per_leg: float = 4.0,
    ) -> None:
        """Fährt alle Beine sicher und langsam in die Neutralstellung.

        Setzt zuerst Speed und Acceleration, dann fährt jedes Bein
        einzeln nacheinander auf 0°. Geeignet als erster Aufruf nach
        dem Start, wenn die Servo-Positionen unbekannt sind.

        Args:
            speed: Maestro-Speed in 0.25µs/10ms (0 = unbegrenzt).
            acceleration: Maestro-Acceleration in 0.25µs/10ms/80ms (0 = unbegrenzt).
            delay_per_leg: Wartezeit pro Bein in Sekunden.
        """
        import time
        num_ch = self._config.driver.num_channels
        self._driver.set_speed_all(num_ch, speed)
        self._driver.set_acceleration_all(num_ch, acceleration)
        # Warten bis alle Speed/Acceleration-Befehle verarbeitet sind:
        # 24 Kanäle × 2 Befehle × 10ms = ~480ms + Puffer
        time.sleep(1.0)
        for name in self._leg_names:
            self.set_leg_angles(name, 0.0, 0.0, 0.0, clip=True)
            time.sleep(delay_per_leg)

    def disable_all(self) -> None:
        """Alle Servos stromlos schalten."""
        self._driver.disable_all(self._config.driver.num_channels)

    def set_servo_us(self, channel: int, microseconds: float) -> None:
        """Direkt µs an einen Kanal schicken (für Kalibrierung)."""
        self._driver.set_position(channel, microseconds)

    def get_servo_us(self, channel: int) -> float:
        """Aktuelle Soll-Position eines Kanals in µs lesen."""
        return self._driver.get_position(channel)

    # ---- Context Manager ----

    def close(self, *, disable: bool = True) -> None:
        """Driver schließen. disable=True deaktiviert vorher alle Servos.

        disable=False lässt die Servos unter Signal, sodass der Roboter
        seine Pose hält (z.B. nach power_up/stand_up). Die serielle
        Verbindung wird in beiden Fällen geschlossen.
        """
        if disable:
            try:
                self.disable_all()
            except Exception as e:
                logger.warning("Fehler beim Deaktivieren: %s", e)
        self._driver.close(disable=disable)

    def __enter__(self) -> Hexapod:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        self.close()
