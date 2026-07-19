"""Eigenstaendiger Kamera-Thread: slewt Pan/Tilt unabhaengig vom Gang-Worker.

Frueher liefen Kamera-Nachfuehrung und Gangart im selben Worker-Thread; ein
Lauf-Halbzyklus blockierte die Kamera fuer mehrere hundert Millisekunden, was
die Verfolgung beim Laufen grob und ruckartig machte.

Jetzt besitzt dieser Thread die Kamera-Servos und faehrt in fester Taktrate
slew-ratenbegrenzt zur jeweils NEUESTEN Zielvorgabe. Gang-Worker und
Kamera-Thread teilen sich denselben Servo-Treiber; dessen interner Lock haelt
die seriellen Schreibzugriffe sauber getrennt. ``set_target`` ist
nicht-blockierend -- der Aufrufer (Worker) gibt nur das Ziel vor.
"""
from __future__ import annotations

import logging
import threading
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from hexapod.robot.hexapod import Hexapod

logger = logging.getLogger(__name__)

DEFAULT_MAX_SPEED_DEG_S = 150.0
DEFAULT_RATE_HZ = 80.0


def _approach(current: float, target: float, max_step: float) -> float:
    """Naehert ``current`` um hoechstens ``max_step`` an ``target`` an."""
    delta = target - current
    if delta > max_step:
        return current + max_step
    if delta < -max_step:
        return current - max_step
    return target


class CameraThread:
    """Faehrt Pan/Tilt in eigenem Thread ruckfrei zur neuesten Zielvorgabe."""

    def __init__(
        self,
        robot: Hexapod,
        *,
        max_speed_deg_s: float = DEFAULT_MAX_SPEED_DEG_S,
        rate_hz: float = DEFAULT_RATE_HZ,
    ) -> None:
        if max_speed_deg_s <= 0.0:
            raise ValueError("max_speed_deg_s muss positiv sein")
        if rate_hz <= 0.0:
            raise ValueError("rate_hz muss positiv sein")
        self._robot = robot
        self._max_step = max_speed_deg_s / rate_hz
        self._dt = 1.0 / rate_hz
        self._lock = threading.Lock()
        self._target_pan = 0.0
        self._target_tilt = 0.0
        self._pan = 0.0
        self._tilt = 0.0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def pan(self) -> float:
        return self._pan

    @property
    def tilt(self) -> float:
        return self._tilt

    def set_target(self, pan_deg: float, tilt_deg: float) -> None:
        """Nicht-blockierend: legt die neue Zielvorgabe fest."""
        with self._lock:
            self._target_pan = float(pan_deg)
            self._target_tilt = float(tilt_deg)

    def sync(self, pan_deg: float, tilt_deg: float) -> None:
        """Setzt Ist- UND Ziel-Lage OHNE Bewegung (z. B. Start/Reset)."""
        with self._lock:
            self._pan = self._target_pan = float(pan_deg)
            self._tilt = self._target_tilt = float(tilt_deg)

    def _advance(self) -> bool:
        """Ein Tick Richtung Ziel. True, wenn ein Servo-Schreibzugriff erfolgte."""
        with self._lock:
            tp, tt = self._target_pan, self._target_tilt
        new_pan = _approach(self._pan, tp, self._max_step)
        new_tilt = _approach(self._tilt, tt, self._max_step)
        if new_pan == self._pan and new_tilt == self._tilt:
            return False
        self._pan, self._tilt = new_pan, new_tilt
        self._robot.set_camera(pan_deg=self._pan, tilt_deg=self._tilt, clip=True)
        return True

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("CameraThread laeuft bereits")
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="camera", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None

    def _run(self) -> None:
        logger.info("Kamera-Thread gestartet (max_step=%.2f deg, %.0f Hz)",
                    self._max_step, 1.0 / self._dt)
        while not self._stop.is_set():
            try:
                self._advance()
            except Exception as e:
                logger.warning("Kamera-Tick fehlgeschlagen: %s", e)
            self._stop.wait(self._dt)
        logger.info("Kamera-Thread beendet")
