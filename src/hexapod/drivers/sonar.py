"""HC-SR04 Ultraschall-Abstandssensor (GPIO, lgpio) -- Hindernis voraus.

Der Freenove-Sonar sitzt auf dem Pan/Tilt-Kopf, misst also in Blickrichtung
der Kamera. Trigger/Echo haengen direkt an GPIO (BCM): Trig=27, Echo=22.

Robust wie die anderen Treiber: fehlendes lgpio / nicht oeffnbarer gpiochip /
Timeout -> None statt Absturz. Die Telemetrie laeuft dann ohne Abstandswerte.

Timing per Busy-Wait mit time.perf_counter; auf dem Pi 3 leicht verrauscht,
darum liefert distance() den Median mehrerer Pings.
"""
from __future__ import annotations

import logging
import time
from typing import Any

logger = logging.getLogger(__name__)

SOUND_MM_PER_S = 343000.0   # Schallgeschwindigkeit ~20 grad C, mm/s
TRIG_PULSE_S = 1.1e-5       # 11 us Trigger-Impuls (>= 10 us)


class Sonar:
    """Minimaler HC-SR04-Treiber ueber lgpio.

    Args:
        trigger_pin: BCM-GPIO des Trigger-Pins.
        echo_pin: BCM-GPIO des Echo-Pins.
        max_distance: Messbereich in Metern (bestimmt das Ping-Timeout).
    """

    def __init__(self, trigger_pin: int = 27, echo_pin: int = 22,
                 max_distance: float = 3.0, chip: int = 0) -> None:
        self._trig = int(trigger_pin)
        self._echo = int(echo_pin)
        self._max_m = float(max_distance)
        self._chipno = int(chip)
        self._h: int | None = None
        self._lg: Any = None
        self._failed = False
        # Timeout je Wartephase: Laufzeit fuer 2x max_distance + Reserve.
        self._timeout = (2.0 * self._max_m) / (SOUND_MM_PER_S / 1000.0) * 1.6 + 0.005

    def _ensure(self) -> int | None:
        if self._h is not None or self._failed:
            return self._h
        try:
            import lgpio
            self._lg = lgpio
            self._h = lgpio.gpiochip_open(self._chipno)
            lgpio.gpio_claim_output(self._h, self._trig, 0)
            lgpio.gpio_claim_input(self._h, self._echo)
            logger.info("Sonar: lgpio bereit (Trig=%d, Echo=%d)", self._trig, self._echo)
        except Exception as e:  # lgpio fehlt / Pins belegt / kein Chip
            logger.warning("Sonar: GPIO nicht verfuegbar (%s) -- keine Abstandswerte", e)
            self._failed = True
            self._h = None
        return self._h

    def _ping_mm(self) -> float | None:
        """Ein Einzel-Ping -> Distanz in mm, oder None bei Timeout."""
        h = self._ensure()
        if h is None:
            return None
        lg = self._lg
        try:
            lg.gpio_write(h, self._trig, 0)
            time.sleep(2e-6)
            lg.gpio_write(h, self._trig, 1)
            time.sleep(TRIG_PULSE_S)
            lg.gpio_write(h, self._trig, 0)
            # Auf steigende Flanke warten
            deadline = time.monotonic() + self._timeout
            while lg.gpio_read(h, self._echo) == 0:
                if time.monotonic() > deadline:
                    return None
            t_start = time.perf_counter()
            # Auf fallende Flanke warten
            deadline = time.monotonic() + self._timeout
            while lg.gpio_read(h, self._echo) == 1:
                if time.monotonic() > deadline:
                    return None
            t_end = time.perf_counter()
        except Exception as e:
            logger.debug("Sonar-Ping-Fehler: %s", e)
            return None
        dist_mm = (t_end - t_start) * SOUND_MM_PER_S / 2.0
        if dist_mm <= 0.0 or dist_mm > self._max_m * 1000.0 * 1.2:
            return None
        return dist_mm

    def distance(self, samples: int = 5, gap: float = 0.012) -> float | None:
        """Median-gefilterter Abstand in Metern (None, wenn kein gueltiger Ping)."""
        vals: list[float] = []
        for i in range(max(1, samples)):
            d = self._ping_mm()
            if d is not None:
                vals.append(d)
            if i + 1 < samples:
                time.sleep(gap)
        if not vals:
            return None
        vals.sort()
        return round(vals[len(vals) // 2] / 1000.0, 3)

    def close(self) -> None:
        if self._h is not None and self._lg is not None:
            import contextlib
            with contextlib.suppress(Exception):
                self._lg.gpio_free(self._h, self._trig)
                self._lg.gpio_free(self._h, self._echo)
                self._lg.gpiochip_close(self._h)
        self._h = None
