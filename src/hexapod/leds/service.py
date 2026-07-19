"""LED-Ring-Dienst fuer den Hexapod (WS2812 an GPIO18).

Laeuft als ROOT (GPIO18/PWM braucht /dev/mem) -- getrennt vom Webserver, der
als unprivilegierter Nutzer laeuft. Der Dienst:
  * pollt den Roboter-Zustand vom Webserver (/status) und zeigt Statusfarben,
  * liest eine kleine JSON-Override-Datei, die der Webserver bei manueller
    Steuerung schreibt (mode=off|manual|rainbow|auto).

Kein Maestro/Roboter-Zugriff -- rein Anzeige.
"""
from __future__ import annotations

import colorsys
import json
import math
import os
import time
import urllib.request
from typing import Any

from rpi_ws281x import Adafruit_NeoPixel, Color

LED_COUNT = int(os.environ.get("HEXAPOD_LED_COUNT", "8"))
LED_PIN = int(os.environ.get("HEXAPOD_LED_PIN", "18"))
LED_BRIGHT = int(os.environ.get("HEXAPOD_LED_BRIGHT", "120"))
STATUS_URL = os.environ.get("HEXAPOD_STATUS_URL", "http://127.0.0.1:8000/status")
OVERRIDE_FILE = os.environ.get("HEXAPOD_LED_FILE", "/tmp/hexapod_led.json")

# Statusfarben (R, G, B)
STATE_COLORS = {
    "off":      (40, 0, 60),    # gedimmtes Violett
    "standing": (0, 160, 40),   # gruen
    "walking":  (0, 110, 200),  # teal/blau
    "lying":    (200, 110, 0),  # amber
}
NO_SERVER = (120, 0, 0)         # Server nicht erreichbar -> dunkelrot


def _poll_state() -> str | None:
    try:
        with urllib.request.urlopen(STATUS_URL, timeout=1.0) as r:
            data = json.load(r)
            return str(data.get("robot_state", "off"))
    except Exception:
        return None


def _read_override() -> dict[str, Any]:
    try:
        with open(OVERRIDE_FILE, encoding="utf-8") as f:
            data: dict[str, Any] = json.load(f)
            return data
    except Exception:
        return {"mode": "auto"}


def main() -> None:
    strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, 800000, 10, False, LED_BRIGHT, 0)
    strip.begin()
    t = 0.0
    try:
        while True:
            ov = _read_override()
            mode = ov.get("mode", "auto")

            if mode == "off":
                cols = [(0, 0, 0)] * LED_COUNT
            elif mode == "manual":
                c = (int(ov.get("r", 0)), int(ov.get("g", 0)), int(ov.get("b", 0)))
                cols = [c] * LED_COUNT
            elif mode == "rainbow":
                cols = []
                for i in range(LED_COUNT):
                    h = ((i * 256 // LED_COUNT) + int(t * 60)) % 256 / 255.0
                    rr, gg, bb = colorsys.hsv_to_rgb(h, 1, 1)
                    cols.append((int(rr * 255), int(gg * 255), int(bb * 255)))
            else:  # auto -> Statusfarbe
                st = _poll_state()
                base = STATE_COLORS.get(st, (60, 60, 60)) if st else NO_SERVER
                # beim Laufen leicht pulsieren
                if st == "walking":
                    f = 0.55 + 0.45 * (0.5 + 0.5 * math.sin(t * 3.0))
                    base = (int(base[0] * f), int(base[1] * f), int(base[2] * f))
                cols = [base] * LED_COUNT

            for i in range(LED_COUNT):
                r, g, b = cols[i]
                strip.setPixelColor(i, Color(r, g, b))
            strip.show()
            t += 0.05
            time.sleep(0.05)
    except KeyboardInterrupt:
        for i in range(LED_COUNT):
            strip.setPixelColor(i, Color(0, 0, 0))
        strip.show()


if __name__ == "__main__":
    main()
