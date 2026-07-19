"""Hintergrund-Thread fuer den HC-SR04-Sonar (Hindernis voraus).

Analog zum CameraThread: misst kontinuierlich (~8 Hz) den Abstand in
Blickrichtung des Kopfes, haelt den letzten (median-gefilterten) Wert und
ein entprelltes `blocked`-Flag bereit. Der Worker liest beides ohne zu
blockieren und klemmt bei Blockade die Vorwaertsgeschwindigkeit.

Der eigentliche Ping (Busy-Wait) laeuft hier im Thread, damit der Worker-Loop
und der Gait taktgenau bleiben.
"""
from __future__ import annotations

import logging
import threading

from hexapod.drivers.sonar import Sonar

logger = logging.getLogger(__name__)

DEFAULT_RATE_HZ = 8.0
DEFAULT_THRESHOLD_M = 0.25
# Entprellung: so viele aufeinanderfolgende Messungen muessen die Schwelle
# unter-/ueberschreiten, bevor `blocked` umschaltet (gegen Ausreisser).
BLOCK_CONFIRM = 2
CLEAR_CONFIRM = 3


class SonarThread:
    def __init__(
        self,
        sonar: Sonar | None = None,
        *,
        threshold_m: float = DEFAULT_THRESHOLD_M,
        rate_hz: float = DEFAULT_RATE_HZ,
        samples: int = 3,
    ) -> None:
        if rate_hz <= 0.0:
            raise ValueError("rate_hz muss positiv sein")
        self._sonar = sonar if sonar is not None else Sonar()
        self._threshold = float(threshold_m)
        self._dt = 1.0 / rate_hz
        self._samples = int(samples)
        self._lock = threading.Lock()
        self._distance: float | None = None
        self._blocked = False
        self._enabled = False
        self._block_cnt = 0
        self._clear_cnt = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def distance(self) -> float | None:
        with self._lock:
            return self._distance

    @property
    def blocked(self) -> bool:
        with self._lock:
            return self._blocked and self._enabled

    @property
    def enabled(self) -> bool:
        with self._lock:
            return self._enabled

    def set_enabled(self, on: bool) -> None:
        with self._lock:
            self._enabled = bool(on)
            if not on:
                self._blocked = False
                self._block_cnt = self._clear_cnt = 0

    def set_threshold(self, m: float) -> None:
        with self._lock:
            self._threshold = float(m)

    def read_now(self, samples: int = 5) -> float | None:
        """Blockierende Einzelmessung (fuer den Sweep-Scan)."""
        return self._sonar.distance(samples=samples)

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("SonarThread laeuft bereits")
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="sonar", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None
        self._sonar.close()

    def _run(self) -> None:
        logger.info("Sonar-Thread gestartet (Schwelle=%.2f m, %.0f Hz)",
                    self._threshold, 1.0 / self._dt)
        while not self._stop.is_set():
            with self._lock:
                enabled = self._enabled
            if enabled:
                thr = self._threshold
                d = self._sonar.distance(samples=self._samples)
                self._update(d, thr)
            self._stop.wait(self._dt)
        logger.info("Sonar-Thread beendet")

    def _update(self, d: float | None, thr: float) -> None:
        with self._lock:
            self._distance = d
            near = d is not None and d <= thr
            if near:
                self._block_cnt += 1
                self._clear_cnt = 0
                if self._block_cnt >= BLOCK_CONFIRM:
                    self._blocked = True
            else:
                self._clear_cnt += 1
                self._block_cnt = 0
                if self._clear_cnt >= CLEAR_CONFIRM:
                    self._blocked = False


def decide_free_dir(profile: dict[float, float | None], clear_m: float = 0.5) -> str:
    """Freie Richtung aus einem Winkel->Distanz(m)-Profil.

    None-Distanz = kein Echo = frei (weit). Rueckgabe: 'left' | 'right' | 'none'.
    Negative Winkel = links, positive = rechts.
    """
    def clearance(angles: list[float]) -> float:
        ds: list[float] = []
        for a in angles:
            v = profile.get(a)
            ds.append(999.0 if v is None else v)
        return min(ds) if ds else 999.0
    left = clearance([-40.0, -20.0])
    right = clearance([20.0, 40.0])
    left_ok = left >= clear_m
    right_ok = right >= clear_m
    if not left_ok and not right_ok:
        return "none"
    if left_ok and not right_ok:
        return "left"
    if right_ok and not left_ok:
        return "right"
    return "left" if left >= right else "right"
