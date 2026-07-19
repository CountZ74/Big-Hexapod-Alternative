"""Sanfte, slew-ratenbegrenzte Pan/Tilt-Steuerung der Kamera.

Die Kamera-Servos (Pan/Tilt) sollen nicht hart auf den Zielwinkel springen,
sondern ruckfrei dorthin fahren. ``CameraController`` haelt die zuletzt
geschriebene Ist-Lage und faehrt bei jedem ``move_to`` mit begrenzter
Winkelgeschwindigkeit in kleinen, gleichmaessig getakteten Schritten zum Ziel.

Thread-confined: wird ausschliesslich aus dem Worker-Thread benutzt (wie der
MotionController). Kein Locking noetig.
"""
from __future__ import annotations

import time
from collections.abc import Callable
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from hexapod.robot.hexapod import Hexapod

# Maximale Winkelgeschwindigkeit (Grad/s) und Taktrate (Hz) der Glaettung.
DEFAULT_MAX_SPEED_DEG_S = 150.0
DEFAULT_RATE_HZ = 80.0


class CameraController:
    """Faehrt Pan/Tilt ruckfrei (slew-ratenbegrenzt) zum Zielwinkel."""

    def __init__(
        self,
        robot: Hexapod,
        *,
        max_speed_deg_s: float = DEFAULT_MAX_SPEED_DEG_S,
        rate_hz: float = DEFAULT_RATE_HZ,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        if max_speed_deg_s <= 0.0:
            raise ValueError("max_speed_deg_s muss positiv sein")
        if rate_hz <= 0.0:
            raise ValueError("rate_hz muss positiv sein")
        self._robot = robot
        self._max_speed = max_speed_deg_s
        self._rate_hz = rate_hz
        self._sleep = sleep
        self._pan = 0.0
        self._tilt = 0.0

    @property
    def pan(self) -> float:
        return self._pan

    @property
    def tilt(self) -> float:
        return self._tilt

    def sync(self, pan_deg: float, tilt_deg: float) -> None:
        """Setzt die angenommene Ist-Lage OHNE Servo-Bewegung."""
        self._pan = float(pan_deg)
        self._tilt = float(tilt_deg)

    def move_to(self, pan_deg: float, tilt_deg: float) -> None:
        """Faehrt sanft von der aktuellen Lage zum Ziel.

        Die Anzahl der Zwischenschritte ergibt sich aus der groesseren der
        beiden Winkeldifferenzen geteilt durch die erlaubte Schrittweite
        (max_speed / rate). Beide Achsen werden synchron interpoliert, sodass
        sie gemeinsam starten und ankommen -- eine gerade, ruckfreie Bahn.
        """
        pan_deg = float(pan_deg)
        tilt_deg = float(tilt_deg)
        start_pan, start_tilt = self._pan, self._tilt
        d_pan = pan_deg - start_pan
        d_tilt = tilt_deg - start_tilt
        dist = max(abs(d_pan), abs(d_tilt))

        max_step = self._max_speed / self._rate_hz
        n = int(dist / max_step) if max_step > 0.0 else 0
        if n <= 1:
            # Schon am Ziel oder nur ein einziger (kleiner) Schritt noetig.
            self._apply(pan_deg, tilt_deg)
            return

        dt = 1.0 / self._rate_hz
        for i in range(1, n + 1):
            f = i / n
            self._apply(start_pan + d_pan * f, start_tilt + d_tilt * f)
            if i < n:
                self._sleep(dt)

    def _apply(self, pan_deg: float, tilt_deg: float) -> None:
        self._robot.set_camera(pan_deg=pan_deg, tilt_deg=tilt_deg, clip=True)
        self._pan = pan_deg
        self._tilt = tilt_deg
