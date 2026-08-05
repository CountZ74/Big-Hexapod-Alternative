"""Roboter-Worker-Thread: alleiniger Besitzer der Hexapod-Instanz.

Der Worker laeuft in einem eigenen Thread, oeffnet den Roboter genau einmal
und besitzt ihn exklusiv. Der Webserver greift NIE direkt auf den Roboter zu:
- lesend  ueber den thread-sicheren Telemetrie-Snapshot
- schreibend ueber eine thread-sichere Kommando-Queue (diskrete Befehle)

Iteration 2: diskrete Befehle (stance, pose). Schnelle Einzelaktionen, kein
mehrsekuendiges Blockieren -- werden pro Schleifentakt abgearbeitet.
"""

from __future__ import annotations

import logging
import math
import os
import queue
import threading
import time
from typing import Any, ClassVar

from hexapod.config import Joint
from hexapod.config.loader import load_robot_config
from hexapod.drivers.adc import (
    ADS7830,
    PI_CRIT_ACTION,
    SERVO_CRIT_ACTION,
    BatteryMonitor,
)
from hexapod.drivers.mpu6050 import MPU6050
from hexapod.gait.posture import (
    lie_down,
    move_to_body_pose,
    move_to_stance,
    power_up,
    settle_to_stance,
    stand_up,
)
from hexapod.kinematics import forward_kinematics
from hexapod.kinematics.body_ik import BodyPose
from hexapod.robot.hexapod import Hexapod

from .camera_thread import CameraThread
from .models import LegTelemetry, TelemetrySnapshot
from .motion_control import MotionController
from .sonar_thread import SonarThread, decide_free_dir

logger = logging.getLogger(__name__)

_JOINTS = (Joint.COXA, Joint.FEMUR, Joint.TIBIA)


class RobotWorker:
    """Besitzt die Hexapod-Instanz, pflegt Telemetrie und arbeitet Befehle ab."""

    def __init__(self, config_path: str, *, poll_hz: float = 5.0) -> None:
        self._config_path = config_path
        self._poll_hz = poll_hz
        self._snapshot: TelemetrySnapshot | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._robot: Hexapod | None = None
        self._robot_name = "?"
        self._driver_type = "?"
        self._num_legs = 0
        self._cmd_queue: queue.Queue[dict[str, Any]] = queue.Queue()
        self._cmd_counter = 0
        self._last_command: dict[str, Any] | None = None
        self._motion: MotionController | None = None
        self._camera: CameraThread | None = None
        self._sonar: SonarThread | None = None
        self._robot_state: str = "off"  # off | standing | walking | lying
        self._state_file = os.environ.get(
            "HEXAPOD_STATE_FILE", "/tmp/hexapod_robot_state"
        )
        # Batterie-Ueberwachung (ADS7830 ueber I2C)
        self._adc = ADS7830()
        self._imu = MPU6050(swap_axes=True, invert_roll=True)
        self._leveling = False
        self._prev_walking = False
        self._level_roll = 0.0
        self._level_pitch = 0.0
        self._tilt_f = (0.0, 0.0)
        self._obstacle_guard = False
        self._scanning = False
        self._fwd_blocked = False
        self._obstacle_free_dir: str | None = None
        self._obstacle_profile: dict[float, float | None] = {}
        self._walk_params: dict[str, Any] | None = None
        self._battmon = BatteryMonitor(crit_confirm=3)
        self._battery: dict[str, float | None] = {"pi": None, "servo": None}
        self._battery_state: dict[str, str] = {"pi": "absent", "servo": "absent"}
        self._batt_read_t = 0.0
        self._gather_t = 0.0  # Telemetrie-Throttle (monotonic)

    # ---- Lebenszyklus ----

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("Worker laeuft bereits")
        self._thread = threading.Thread(
            target=self._run, name="robot-worker", daemon=True
        )
        self._thread.start()

    def stop(self, timeout: float = 5.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ---- Kommando-Schnittstelle (vom Webserver aufgerufen) ----

    # Erlaubte Aktionen je Zustand
    _ALLOWED: ClassVar[dict[str, set[str]]] = {
        "off":      {"power_up", "assume_standing", "camera", "set_gait", "obstacle"},
        "standing": {
            "stance", "settle", "pose", "walk", "halt",
            "stand_up", "lie_down", "camera", "set_gait", "gesture", "level", "obstacle",
        },
        "walking":  {"walk", "halt", "pose", "camera", "set_gait", "obstacle"},
        "lying":    {"stand_up", "camera", "obstacle"},
    }

    def enqueue(self, cmd: dict[str, Any]) -> tuple[int, int]:
        """Legt einen Befehl in die Queue. Gibt (id, wartende_anzahl) zurueck.

        Lehnt Befehle ab die im aktuellen Roboter-Zustand nicht erlaubt sind.
        """
        action = cmd.get("action", "")
        with self._lock:
            # Pruefung und Einreihen atomar: sonst kann sich der Zustand
            # zwischen Check und put aendern (TOCTOU).
            state = self._robot_state
            allowed = self._ALLOWED.get(state, set())
            if action not in allowed:
                raise ValueError(
                    f"Aktion {action!r} im Zustand {state!r} nicht erlaubt. "
                    f"Erlaubt: {sorted(allowed)}"
                )
            self._cmd_counter += 1
            cmd_id = self._cmd_counter
        cmd = {**cmd, "id": cmd_id}
        self._cmd_queue.put(cmd)
        return cmd_id, self._cmd_queue.qsize()

    # ---- Zustands-Persistenz ----

    def _set_state(self, state: str) -> None:
        """Setzt den Roboter-Zustand und schreibt ihn persistent weg.

        Datei in /tmp: reiner Dienst-Neustart behaelt den Zustand (Roboter
        bleibt gestromt stehen), echter Reboot/Stromausfall leert /tmp ->
        Zustand faellt korrekt auf 'off' zurueck.
        """
        with self._lock:
            self._robot_state = state
        try:
            with open(self._state_file, "w", encoding="utf-8") as f:
                f.write(state)
        except Exception as e:
            logger.warning("Zustand nicht persistiert: %s", e)

    def _restore_state(self) -> None:
        """Liest den letzten Zustand beim Worker-Start (nach Neustart).

        'walking' wird aus Sicherheit zu 'standing' verflacht.
        """
        try:
            with open(self._state_file, encoding="utf-8") as f:
                saved = f.read().strip()
        except FileNotFoundError:
            return
        except Exception as e:
            logger.warning("Zustand nicht gelesen: %s", e)
            return
        if saved in ("off", "standing", "walking", "lying"):
            restored = "standing" if saved == "walking" else saved
            with self._lock:
                self._robot_state = restored
            logger.info("Roboter-Zustand wiederhergestellt: %s", restored)

    # ---- Thread-Body ----

    def _open_robot(self) -> Hexapod:
        config = load_robot_config(self._config_path)
        # Dev-Override: HEXAPOD_DRIVER=simulator erlaubt gefahrloses Testen.
        override = os.environ.get("HEXAPOD_DRIVER")
        if override == "simulator":
            data = config.model_dump()
            # Alle Busse gleichzeitig simulieren — sonst wuerde der Roboter
            # halb echte Hardware ansprechen.
            data["buses"] = {
                name: {"type": "simulator", "num_channels": bus.num_channels}
                for name, bus in config.buses.items()
            }
            config = config.model_validate(data)
        elif override:
            logger.warning(
                "HEXAPOD_DRIVER=%r wird ignoriert (unterstuetzt: 'simulator')", override
            )
        return Hexapod(config)

    def _run(self) -> None:
        logger.info("Worker startet, oeffne Roboter aus %s", self._config_path)
        self._robot = self._open_robot()
        self._robot_name = self._robot.config.name
        self._driver_type = ",".join(
            f"{name}={bus.type}" for name, bus in self._robot.config.buses.items()
        )
        self._num_legs = len(self._robot.leg_names)
        self._motion = MotionController(self._robot)
        self._camera = CameraThread(self._robot)
        self._camera.start()
        self._sonar = SonarThread()
        self._sonar.start()
        self._restore_state()
        period = 1.0 / self._poll_hz
        try:
            while not self._stop.is_set():
                self._drain_commands()
                if self._motion.is_walking:
                    self._motion.step_once()
                    self._prev_walking = True
                    self._guard_step()
                else:
                    if self._prev_walking:
                        # Lauf gerade beendet: einmal sauber (lift-and-place) in
                        # die Standpose, damit die Nivellierung nicht hart aus der
                        # Schrittlage in die Default-Pose snappt.
                        self._prev_walking = False
                        if self._leveling and self._robot_state == "standing":
                            move_to_stance(self._robot)
                            self._level_roll = 0.0
                            self._level_pitch = 0.0
                            self._tilt_f = (0.0, 0.0)
                    if self._leveling and self._robot_state == "standing":
                        self._level_step()
                    else:
                        self._stop.wait(period)
                # Telemetrie throtteln: _gather() macht pro Aufruf 24 serielle
                # Roundtrips -- beim Gehen wuerde das den 40-Hz-Gait stoeren.
                now = time.monotonic()
                if now - self._gather_t >= 0.2:
                    self._gather_t = now
                    snap = self._gather()
                    with self._lock:
                        self._snapshot = snap
                self._poll_battery()
        finally:
            if self._camera is not None:
                self._camera.stop()
            if self._sonar is not None:
                self._sonar.stop()
            # WICHTIG: disable=False! Ein stehender Roboter darf beim Schliessen
            # NICHT stromlos werden, sonst sackt er zusammen.
            try:
                self._robot.close(disable=False)
            except Exception as e:
                logger.warning("Fehler beim Schliessen des Roboters: %s", e)
            logger.info("Worker beendet")

    def _drain_commands(self) -> None:
        robot = self._robot
        assert robot is not None
        # Alle aktuell wartenden Befehle ziehen.
        pending: list[dict[str, Any]] = []
        while True:
            try:
                pending.append(self._cmd_queue.get_nowait())
            except queue.Empty:
                break
        if not pending:
            return
        # Aufeinanderfolgende pose-Befehle zusammenfassen: nur der jeweils
        # letzte einer Serie wird ausgefuehrt. Der Slider schickt bei jeder
        # Zwischenstellung einen Befehl, der aber bereits ALLE Slider-Werte
        # enthaelt -- nur die aktuelle Zielpose zaehlt. Andere Aktionen
        # bleiben erhalten und in Reihenfolge.
        coalesced: list[dict[str, Any]] = []
        dropped = 0
        for cmd in pending:
            a = cmd.get("action")
            if (a in ("pose", "camera") and coalesced
                    and coalesced[-1].get("action") == a):
                coalesced[-1] = cmd
                dropped += 1
            else:
                coalesced.append(cmd)
        if dropped:
            logger.debug("%d veraltete pose-Befehle uebersprungen", dropped)
        for cmd in coalesced:
            result = {"id": cmd.get("id"), "action": cmd.get("action"),
                      "ts": time.time(), "ok": True, "error": None}
            try:
                self._execute(robot, cmd)
            except Exception as e:
                result["ok"] = False
                result["error"] = str(e)
                logger.warning("Befehl %s fehlgeschlagen: %s", cmd, e)
            with self._lock:
                self._last_command = result

    def _execute(self, robot: Hexapod, cmd: dict[str, Any]) -> None:
        action = cmd.get("action")
        if action == "stance":
            move_to_stance(robot)
            self._set_state("standing")
        elif action == "assume_standing":
            # Zustands-Sync OHNE Servo-Bewegung: der Roboter steht physisch
            # schon. WICHTIG: die Ist-Lage der Servos einlesen, damit das
            # Bewegungs-Modell der Realitaet entspricht -- sonst steht es auf
            # theta=0 (Beine gestreckt) und der erste walk faehrt die Beine
            # physisch dorthin (Bellyflop). prime() vermeidet einen ruckartigen
            # ersten Zug.
            robot.prime()
            robot.reconstruct_state_from_hardware()
            self._set_state("standing")
        elif action == "camera":
            # Kamera-Schwenk/Neige (Pan/Tilt). Unabhaengig vom Roboter-Zustand.
            # NICHT-BLOCKIEREND: nur die Zielvorgabe setzen -- der eigene
            # Kamera-Thread slewt dorthin, parallel zum Gang (kein Ruckeln).
            assert self._camera is not None
            if not self._scanning:
                self._camera.set_target(
                    float(cmd.get("pan_deg", 0.0)),
                    float(cmd.get("tilt_deg", 0.0)),
                )
        elif action == "level":
            # Selbst-Nivellierung ein/aus. Nur im Stehen sinnvoll.
            on = bool(cmd.get("on", True))
            self._leveling = on
            if on:
                self._level_roll = self._level_pitch = 0.0
                self._tilt_f = (0.0, 0.0)
            else:
                cur = robot.body_pose
                move_to_body_pose(robot, BodyPose(tx=cur.tx, ty=cur.ty, tz=cur.tz),
                                  steps=12, rate_hz=60.0)
        elif action == "obstacle":
            # Hindernis-Waechter ein/aus (Sonar-Vorwaertssperre + Scan).
            on = bool(cmd.get("on", True))
            self._obstacle_guard = on
            if self._sonar is not None:
                self._sonar.set_enabled(on)
            if not on:
                self._fwd_blocked = False
                self._obstacle_free_dir = None
        elif action == "pose":
            target = BodyPose(
                tx=float(cmd.get("tx", 0.0)),
                ty=float(cmd.get("ty", 0.0)),
                tz=float(cmd.get("tz", 0.0)),
                roll=math.radians(float(cmd.get("roll_deg", 0.0))),
                pitch=math.radians(float(cmd.get("pitch_deg", 0.0))),
                yaw=math.radians(float(cmd.get("yaw_deg", 0.0))),
            )
            if self._motion is not None and self._motion.is_walking:
                # Im Gehen KEIN eigener Bewegungslauf: nur die gehaltene Pose
                # aktualisieren. Der naechste Gang-Halbzyklus komponiert sie
                # ueber die Ist-Lage-Bruecke sanft ein -- der Koerper neigt sich
                # ueber die naechsten Schritte weich in die neue Pose.
                robot._body_pose = target
            else:
                move_to_body_pose(robot, target, steps=12, rate_hz=60.0)
        elif action == "walk":
            assert self._motion is not None
            vx = float(cmd.get("vx", 0.0))
            vy = float(cmd.get("vy", 0.0))
            omega = float(cmd.get("omega_deg", 0.0))
            height = float(cmd.get("height", 30.0))
            steps = int(cmd.get("steps", 30))
            rate = float(cmd.get("rate_hz", 40.0))
            self._walk_params = {"vx": vx, "vy": vy, "omega": omega,
                                 "height": height, "steps": steps, "rate": rate}
            blocked = (self._obstacle_guard and self._sonar is not None
                       and self._sonar.blocked)
            if not blocked:
                self._fwd_blocked = False
            eff_vx = 0.0 if (blocked and vx > 0.0) else vx
            self._motion.set_walk(
                vx=eff_vx, vy=vy, omega_deg=omega,
                height=height, steps=steps, rate_hz=rate,
            )
            # Zustand erst NACH erfolgreichem set_walk setzen -- wirft es,
            # bleibt der bisherige Zustand korrekt erhalten.
            self._set_state("walking")
        elif action == "gesture":
            from hexapod.gait.gestures import GESTURES
            name = str(cmd.get("gesture", ""))
            fn = GESTURES.get(name)
            if fn is None:
                raise ValueError(f"Unbekannte Geste {name!r}. Verfuegbar: {sorted(GESTURES)}")
            fn(robot)
            # Geste endet in der Standpose -> Zustand bleibt 'standing'
        elif action == "set_gait":
            assert self._motion is not None
            name = str(cmd.get("gait", "tripod"))
            if self._motion.is_walking:
                # Wechsel IMMER ueber die Standpose: anhalten, sauber absetzen,
                # dann neue Gangart aktivieren. Der naechste walk-Befehl der UI
                # (Joystick gehalten) nimmt mit der neuen Gangart wieder auf.
                self._motion.halt()
                move_to_stance(robot)
                self._set_state("standing")
            self._motion.set_gait(name)
        elif action == "halt":
            assert self._motion is not None
            self._motion.halt()
            self._set_state("standing")
        elif action == "power_up":
            assert self._motion is not None
            self._motion.halt()
            power_up(robot)
            self._set_state("standing")
        elif action == "stand_up":
            assert self._motion is not None
            self._motion.halt()
            # stand_up() faehrt bedingungslos aus der abgesetzten Lage (Fuesse am
            # Boden) hoch -- ruft man es im Stehen, springt der Koerper zuerst in
            # die Liegepose. Daher NUR aus 'lying' die Aufsteh-Sequenz fahren;
            # steht der Roboter schon, nur sauber in die Standpose (kein Absacken).
            if self._robot_state == "lying":
                stand_up(robot)
            else:
                move_to_stance(robot)
            self._set_state("standing")
        elif action == "lie_down":
            assert self._motion is not None
            self._motion.halt()
            lie_down(robot)
            self._set_state("lying")
        elif action == "settle":
            assert self._motion is not None
            self._motion.halt()
            settle_to_stance(robot)
        else:
            raise ValueError(f"Unbekannte action: {action!r}")

    # ---- Selbst-Nivellierung (MPU6050) ----
    _LEVEL_KI = 0.35     # Regelverstaerkung pro Schritt
    _LEVEL_MAX = 12.0    # max. Ausgleichs-Neigung (Grad)
    _LEVEL_DEAD = 0.6    # Totzone (Grad)
    _LEVEL_ALPHA = 0.3   # Tiefpass auf die Messung

    def _level_step(self) -> None:
        """Ein Regelschritt: gemessene Neigung -> Ausgleichs-Pose (integrierend)."""
        robot = self._robot
        assert robot is not None
        t = self._imu.tilt()
        if t is None:
            self._stop.wait(0.1)
            return
        fr = self._LEVEL_ALPHA * t[0] + (1 - self._LEVEL_ALPHA) * self._tilt_f[0]
        fp = self._LEVEL_ALPHA * t[1] + (1 - self._LEVEL_ALPHA) * self._tilt_f[1]
        self._tilt_f = (fr, fp)
        er = 0.0 if abs(fr) < self._LEVEL_DEAD else fr
        ep = 0.0 if abs(fp) < self._LEVEL_DEAD else fp
        nr = max(-self._LEVEL_MAX, min(self._LEVEL_MAX, self._level_roll - self._LEVEL_KI * er))
        npi = max(-self._LEVEL_MAX, min(self._LEVEL_MAX, self._level_pitch - self._LEVEL_KI * ep))
        # Nur bewegen, wenn sich die Zielneigung merklich aendert -- sonst bleibt
        # der Worker responsiv (kein Dauer-move_to_body_pose, das ihn blockiert).
        if abs(nr - self._level_roll) > 0.1 or abs(npi - self._level_pitch) > 0.1:
            self._level_roll, self._level_pitch = nr, npi
            cur = robot.body_pose
            target = BodyPose(tx=cur.tx, ty=cur.ty, tz=cur.tz,
                              roll=math.radians(nr), pitch=math.radians(npi), yaw=cur.yaw)
            move_to_body_pose(robot, target, steps=8, rate_hz=60.0)
        else:
            self._stop.wait(0.05)

    # ---- Hindernis-Waechter ----

    _SCAN_ANGLES = (-40.0, -20.0, 0.0, 20.0, 40.0)

    def _guard_step(self) -> None:
        """Waehrend des Gehens: Vorwaerts bei Blockade kappen + einmal scannen."""
        if self._sonar is None or self._motion is None:
            return
        wp = self._walk_params or {}
        want_fwd = wp.get("vx", 0.0) > 0.0
        if self._obstacle_guard and self._sonar.blocked and want_fwd:
            if not self._fwd_blocked:
                self._fwd_blocked = True
                self._motion.set_walk(
                    vx=0.0, vy=wp.get("vy", 0.0), omega_deg=wp.get("omega", 0.0),
                    height=wp.get("height", 30.0), steps=int(wp.get("steps", 30)),
                    rate_hz=wp.get("rate", 40.0),
                )
                self._obstacle_scan()
        elif not self._sonar.blocked:
            self._fwd_blocked = False

    def _obstacle_scan(self) -> None:
        """Kopf schwenken, Sonar je Winkel messen, freie Richtung melden."""
        if self._camera is None or self._sonar is None:
            return
        prev_pan, prev_tilt = self._camera.pan, self._camera.tilt
        profile: dict[float, float | None] = {}
        self._scanning = True
        try:
            for ang in self._SCAN_ANGLES:
                self._camera.set_target(ang, 0.0)
                t0 = time.monotonic()
                while abs(self._camera.pan - ang) > 1.5 and time.monotonic() - t0 < 1.0:
                    self._stop.wait(0.02)
                self._stop.wait(0.08)
                profile[ang] = self._sonar.read_now(samples=5)
            free = decide_free_dir(profile)
        finally:
            self._camera.set_target(prev_pan, prev_tilt)
            self._scanning = False
        with self._lock:
            self._obstacle_profile = profile
            self._obstacle_free_dir = free
        logger.info("Sonar-Scan %s -> frei: %s",
                    {int(k): (round(v, 2) if v else None) for k, v in profile.items()}, free)

    # ---- Telemetrie ----

    def _gather(self) -> TelemetrySnapshot:
        robot = self._robot
        assert robot is not None
        raw = robot.read_servo_state()
        legs: list[LegTelemetry] = []
        ok = True
        for name in robot.leg_names:
            try:
                svs = [robot.config.get_leg_servo(name, j) for j in _JOINTS]
                keys = [f"{s.bus}:{s.channel}" for s in svs]
                us = [raw.get(k, {}).get("us") for k in keys]
                a0, a1, a2 = (raw.get(k, {}).get("angle_rad") for k in keys)
                if a0 is None or a1 is None or a2 is None:
                    ok = False
                    legs.append(LegTelemetry(
                        name=name, angles_deg=None,
                        foot_leg_frame_mm=None, servo_us=us,
                    ))
                    continue
                foot = forward_kinematics(a0, a1, a2, robot.leg_lengths)
                legs.append(LegTelemetry(
                    name=name,
                    angles_deg=[round(math.degrees(a), 2) for a in (a0, a1, a2)],
                    foot_leg_frame_mm=[round(v, 2) for v in foot],
                    servo_us=us,
                ))
            except Exception as e:
                ok = False
                logger.debug("Telemetrie fuer %s fehlgeschlagen: %s", name, e)
                legs.append(LegTelemetry(
                    name=name, angles_deg=None,
                    foot_leg_frame_mm=None, servo_us=[None, None, None],
                ))
        return TelemetrySnapshot(timestamp=time.time(), ok=ok, legs=legs)

    # ---- Batterie ----

    def _poll_battery(self) -> None:
        now = time.time()
        if now - self._batt_read_t < 2.0:
            return
        self._batt_read_t = now
        volts = self._adc.read_batteries()
        res = self._battmon.update(volts)
        with self._lock:
            self._battery = volts
            self._battery_state = {p: r['state'] for p, r in res.items()}
        for pack, r in res.items():
            if r['changed'] and r['state'] in ('warn', 'critical'):
                logger.warning('Batterie %s: %s (%.2f V)', pack, r['state'],
                               r['voltage'] or 0.0)
            if r['fire_action']:
                self._battery_action(pack, r['voltage'])

    def _battery_action(self, pack: str, voltage: float | None) -> None:
        action = PI_CRIT_ACTION if pack == 'pi' else SERVO_CRIT_ACTION
        logger.error('KRITISCHE Unterspannung %s = %.2f V -> Aktion: %s',
                     pack, voltage or 0.0, action)
        try:
            if action == 'shutdown':
                import subprocess
                subprocess.Popen(['sudo', 'shutdown', '-h', 'now'])
            elif action == 'lie_down':
                self.enqueue({'action': 'halt'})
                self.enqueue({'action': 'lie_down'})
            elif action == 'disable':
                if self._robot is not None:
                    self._robot.disable_all()
        except Exception as e:
            logger.warning('Batterie-Schutzaktion fehlgeschlagen: %s', e)

    # ---- Lese-Zugriff fuer den Webserver ----

    def snapshot(self) -> TelemetrySnapshot | None:
        with self._lock:
            return self._snapshot

    def status(self) -> dict[str, Any]:
        with self._lock:
            snap = self._snapshot
            last = self._last_command
            free_dir = self._obstacle_free_dir
            guard = self._obstacle_guard
            scanning = self._scanning
        return {
            "robot_name": self._robot_name,
            "driver_type": self._driver_type,
            "num_legs": self._num_legs,
            "worker_running": self.running,
            "poll_rate_hz": self._poll_hz,
            "last_update": snap.timestamp if snap else None,
            "queued_commands": self._cmd_queue.qsize(),
            "last_command": last,
            "robot_state": self._robot_state,
            "gait": self._motion.gait_name if self._motion is not None else "tripod",
            "battery": dict(self._battery),
            "battery_state": dict(self._battery_state),
            "obstacle": {
                "guard": guard,
                "distance": self._sonar.distance if self._sonar is not None else None,
                "blocked": self._sonar.blocked if self._sonar is not None else False,
                "free_dir": free_dir,
                "scanning": scanning,
            },
        }
