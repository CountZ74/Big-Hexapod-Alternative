#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11,<3.14"
# dependencies = ["pygame", "requests"]
# ///
"""Hexapod-Controller-Daemon (laeuft auf dem PC, steuert ueber WLAN).

Liest einen USB-Joystick/Gamepad (z.B. RadioMaster TX16S im USB-Joystick-Modus)
mit pygame und schickt die Achsen/Knoepfe als Befehle an die Hexapod-Web-API
(POST /command) -- genau die Schnittstelle, die auch das Webinterface nutzt.

Der Sender haengt per USB am PC, der Roboter laeuft frei im WLAN.

SETUP TX16S (EdgeTX):
  Model Setup -> USB Joystick: Mode = Joystick/Gamepad, Channel-Reihenfolge nach
  Wunsch. Beim Anstecken "USB Joystick (HID)" waehlen.

ZUERST das Achsen-/Knopf-Mapping herausfinden:
  uv run hexapod_controller.py --monitor
  -> Knueppel/Schalter bewegen, die angezeigten Indizes unten in MAP eintragen.

DANN steuern:
  uv run hexapod_controller.py --host http://hexapod.local:8000

Abhaengigkeiten stehen im /// script Block oben -- "uv run" installiert sie
automatisch (kein pip noetig). Ohne uv:  pip install pygame requests
"""
from __future__ import annotations

import argparse
import sys
import time

import pygame
import requests

# ===================== MAPPING (ggf. via --monitor anpassen) =====================
# Achsen-Indizes (pygame). EdgeTX-Standard Mode 2 (kann je Treiber abweichen!):
#   Right-X=Aileron, Right-Y=Elevator, Left-Y=Throttle, Left-X=Rudder
AX_VX     = 1   # rechter Knueppel hoch/runter  -> vx (selbstzentrierend)
AX_VY     = 3   # rechter Knueppel links/rechts -> vy (seitwaerts)
AX_OMEGA  = 0   # linker Knueppel links/rechts  -> drehen
AX_HEIGHT = 2   # linker Knueppel hoch/runter (Throttle, haelt) -> Hoehe; -1 = fix
AX_GAIT   = 4   # 6-Pos-Schalter -> Gangart; -1 = aus

# Falls eine Richtung verkehrt herum ist: einfach True<->False tauschen.
INV_VX = False      # Knueppel oben = vorwaerts
INV_VY = True       # Knueppel rechts = nach rechts
INV_OMEGA = True    # Knueppel rechts = Drehung nach rechts (CW)
INV_HEIGHT = False  # Throttle oben = hoeher

AX_TZ   = 5     # Drehregler 1 -> Koerperhoehe tz (Bodenfreiheit); -1 = aus
INV_TZ  = False # Drehregler im Uhrzeigersinn = hoeher
AX_TX   = 6     # Schieberegler 1 -> Koerper vor/zurueck (tx); -1 = aus
AX_TY   = 7     # Drehregler 2 -> Koerper links/rechts (ty); -1 = aus
INV_TX  = False
INV_TY  = False
BTN_MODE = 6    # Schalter 6 gehalten = Pose-Modus (Knueppel -> roll/pitch/yaw)

# Knopf-Indizes (pygame) fuer Aktionen; -1 = deaktiviert.
BTN_GAIT    = -1  # Gangart ueber 6-Pos-Schalter (AX_GAIT)
BTN_POSTURE = -1  # 2-Pos-Schalter: EIN = stand_up, AUS = lie_down; -1 = aus
BTN_STAND   = -1  # optionaler momentaner Knopf -> stand_up
BTN_SIT     = -1  # optionaler momentaner Knopf -> lie_down
BTN_STANCE  = -1  # stance
BTN_HALT    = 8   # Taster 8 = Not-Halt
BTN_POSE_HOLD = 5 # haelt die aktuelle Stick-Pose fest (auch im Bewegungsmodus)
BTN_POWERUP = -1  # power_up selten -> Webinterface

# Wertebereiche (passend zum Webinterface):
VX_MAX = 40.0       # mm/Schritt
VY_MAX = 40.0
OMEGA_MAX = 30.0    # Grad/Schritt
HEIGHT_MIN, HEIGHT_MAX = 15.0, 50.0
TZ_MIN, TZ_MAX = -20.0, 40.0          # Koerperhoehe-Bereich (Drehregler)
ROLL_MAX = PITCH_MAX = YAW_MAX = 18.0 # Pose-Modus, Grad
TX_MIN, TX_MAX = -30.0, 30.0          # Koerper vor/zurueck
TY_MIN, TY_MAX = -30.0, 30.0          # Koerper links/rechts
WALK_STEPS = 30
WALK_RATE_HZ = 40.0
# ================================================================================

GAITS = ["tripod", "tetrapod", "ripple", "wave"]


def apply(v: float, inv: bool, deadzone: float) -> float:
    if inv:
        v = -v
    if abs(v) < deadzone:
        return 0.0
    # Deadzone herausrechnen und auf 0..1 reskalieren (sanfter Einsatz)
    s = (abs(v) - deadzone) / (1.0 - deadzone)
    return (1.0 if v > 0 else -1.0) * s


def axis(js, idx: int) -> float:
    return js.get_axis(idx) if 0 <= idx < js.get_numaxes() else 0.0


def pot(js, idx: int, inv: bool, lo: float, hi: float) -> float:
    """Pot/Schieber-Achse -> Wert in [lo, hi] (Achsenmitte = Mitte)."""
    if not 0 <= idx < js.get_numaxes():
        return 0.0
    return lo + (apply(axis(js, idx), inv, 0.0) + 1.0) / 2.0 * (hi - lo)


def post(session, url: str, payload: dict) -> None:
    try:
        session.post(url, json=payload, timeout=0.4)
    except requests.RequestException as e:
        print(f"  [WARN] Roboter nicht erreichbar: {e}", file=sys.stderr)


def monitor(js) -> None:
    print("Achsen-/Knopf-Monitor -- Strg-C zum Beenden.\n")
    try:
        while True:
            pygame.event.pump()
            ax = [f"{i}:{js.get_axis(i):+.2f}" for i in range(js.get_numaxes())]
            bt = [i for i in range(js.get_numbuttons()) if js.get_button(i)]
            hats = [js.get_hat(i) for i in range(js.get_numhats())]
            print(f"\rAchsen {'  '.join(ax)} | Knoepfe {bt} | Hats {hats}      ",
                  end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n")


def main() -> int:
    ap = argparse.ArgumentParser(description="Hexapod USB-Controller -> WLAN")
    ap.add_argument("--host", default="http://hexapod.local:8000",
                    help="Basis-URL der Hexapod-API")
    ap.add_argument("--rate", type=float, default=15.0, help="Sende-Frequenz Hz")
    ap.add_argument("--deadzone", type=float, default=0.08)
    ap.add_argument("--index", type=int, default=0, help="Joystick-Index")
    ap.add_argument("--monitor", action="store_true",
                    help="Nur Achsen/Knoepfe anzeigen (Mapping finden)")
    ap.add_argument("--no-failsafe", action="store_true")
    args = ap.parse_args()

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("Kein Joystick gefunden. TX16S im USB-Joystick-Modus anstecken.",
              file=sys.stderr)
        return 1
    js = pygame.joystick.Joystick(args.index)
    js.init()
    print(f"Joystick: {js.get_name()}  Achsen={js.get_numaxes()} "
          f"Knoepfe={js.get_numbuttons()}")

    if args.monitor:
        monitor(js)
        return 0

    url = args.host.rstrip("/") + "/command"
    session = requests.Session()
    period = 1.0 / args.rate
    gait_idx = 0
    walking = False
    last_pose = None
    held_rpy = (0.0, 0.0, 0.0)
    prev_btn: dict[int, bool] = {}

    def pressed(idx: int) -> bool:
        """True nur im Moment des Druecks (Flanke)."""
        if idx < 0 or idx >= js.get_numbuttons():
            return False
        now = js.get_button(idx) == 1
        was = prev_btn.get(idx, False)
        prev_btn[idx] = now
        return now and not was

    def edge(idx: int) -> int:
        """+1 = gerade EIN, -1 = gerade AUS, 0 = keine Aenderung."""
        if idx < 0 or idx >= js.get_numbuttons():
            return 0
        now = js.get_button(idx) == 1
        was = prev_btn.get(idx, False)
        prev_btn[idx] = now
        return 1 if (now and not was) else -1 if (was and not now) else 0

    print(f"Steuere {url} @ {args.rate:.0f} Hz. Strg-C beendet (Failsafe: halt).")
    try:
        while True:
            pygame.event.pump()

            # --- Gangart per 6-Pos-Schalter (Achse) ---
            if 0 <= AX_GAIT < js.get_numaxes():
                gv = js.get_axis(AX_GAIT)
                gi = max(0, min(len(GAITS) - 1,
                                int(round((gv + 1.0) / 2.0 * (len(GAITS) - 1)))))
                if gi != gait_idx:
                    gait_idx = gi
                    post(session, url, {"action": "set_gait", "gait": GAITS[gi]})
                    print(f"  Gangart -> {GAITS[gi]}")

            # --- Aktions-Knoepfe (flankengetriggert) ---
            if pressed(BTN_GAIT):
                gait_idx = (gait_idx + 1) % len(GAITS)
                post(session, url, {"action": "set_gait", "gait": GAITS[gait_idx]})
                print(f"  Gangart -> {GAITS[gait_idx]}")
            if pressed(BTN_STAND):
                post(session, url, {"action": "stand_up"}); print("  stand_up")
            if pressed(BTN_SIT):
                post(session, url, {"action": "lie_down"}); print("  lie_down")
            if pressed(BTN_STANCE):
                post(session, url, {"action": "stance"}); print("  stance")
            if pressed(BTN_HALT):
                post(session, url, {"action": "halt"}); walking = False; print("  halt")
            if pressed(BTN_POWERUP):
                post(session, url, {"action": "power_up"}); print("  power_up")

            # --- Haltung per 2-Pos-Schalter: EIN=aufstehen, AUS=hinlegen ---
            pe = edge(BTN_POSTURE)
            if pe > 0:
                post(session, url, {"action": "stand_up"}); print("  stand_up")
            elif pe < 0:
                post(session, url, {"action": "lie_down"}); print("  lie_down")

            # --- Translation/Hoehe: immer von den Reglern ---
            tz = pot(js, AX_TZ, INV_TZ, TZ_MIN, TZ_MAX)
            tx = pot(js, AX_TX, INV_TX, TX_MIN, TX_MAX)
            ty = pot(js, AX_TY, INV_TY, TY_MIN, TY_MAX)

            pose_mode = (0 <= BTN_MODE < js.get_numbuttons()
                         and js.get_button(BTN_MODE) == 1)

            if pose_mode:
                # Knueppel steuern die Koerper-Neigung
                lr = apply(axis(js, AX_VY), INV_VY, args.deadzone) * ROLL_MAX
                lp = apply(axis(js, AX_VX), INV_VX, args.deadzone) * PITCH_MAX
                ly = apply(axis(js, AX_OMEGA), INV_OMEGA, args.deadzone) * YAW_MAX
                if edge(BTN_POSE_HOLD) > 0:         # aktuelle Stick-Pose festhalten
                    held_rpy = (lr, lp, ly)
                    print("  Pose gehalten")
                if abs(lr) > 0 or abs(lp) > 0 or abs(ly) > 0:
                    roll, pitch, yaw = lr, lp, ly   # live, solange ausgelenkt
                else:
                    roll, pitch, yaw = held_rpy     # losgelassen -> gehaltene Pose
                if walking:
                    post(session, url, {"action": "halt"})
                    walking = False
            else:
                # Bewegungs-Modus: gehaltene Neigung beibehalten, Knueppel -> walk
                roll, pitch, yaw = held_rpy
                vx = apply(axis(js, AX_VX), INV_VX, args.deadzone) * VX_MAX
                vy = apply(axis(js, AX_VY), INV_VY, args.deadzone) * VY_MAX
                om = apply(axis(js, AX_OMEGA), INV_OMEGA, args.deadzone) * OMEGA_MAX
                height = 30.0
                if 0 <= AX_HEIGHT < js.get_numaxes():
                    height = HEIGHT_MIN + (apply(axis(js, AX_HEIGHT), INV_HEIGHT, 0.0) + 1.0) / 2.0 * (HEIGHT_MAX - HEIGHT_MIN)
                if abs(vx) > 0 or abs(vy) > 0 or abs(om) > 0:
                    post(session, url, {
                        "action": "walk", "vx": round(vx, 1), "vy": round(vy, 1),
                        "omega_deg": round(om, 1), "height": round(height, 1),
                        "steps": WALK_STEPS, "rate_hz": WALK_RATE_HZ,
                    })
                    walking = True
                elif walking:
                    post(session, url, {"action": "halt"})
                    walking = False

            # --- Pose (6-DOF) senden, wenn geaendert ---
            tp = tuple(round(v * 2) / 2 for v in (roll, pitch, yaw, tx, ty, tz))
            if tp != last_pose:
                post(session, url, {
                    "action": "pose", "roll_deg": tp[0], "pitch_deg": tp[1],
                    "yaw_deg": tp[2], "tx": tp[3], "ty": tp[4], "tz": tp[5],
                })
                last_pose = tp

            time.sleep(period)
    except KeyboardInterrupt:
        print("\nBeende...")
    finally:
        if not args.no_failsafe:
            post(session, url, {"action": "halt"})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
