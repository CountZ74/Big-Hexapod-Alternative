"""Interaktives Feinjustieren der Servo-Nulllagen in der Stance (Footprint-Methode).

Der Roboter steht in Standpose. Man waehlt ein Bein und ein Gelenk
(Coxa/Femur/Tibia) und verschiebt dessen ``center_us`` live in Mikrosekunden.
Der zugehoerige Servo bewegt sich sofort -- so schiebt man den Fuss auf seine
Soll-Markierung des Pappe-Templates.

Eine ``center_us``-Verschiebung ist eine ECHTE Kalibrier-Korrektur: sie
verschiebt die Nulllage des Gelenks und damit jede Pose, nicht nur die Stance.

Das Display zeigt zusaetzlich, um wie viele mm sich der Fuss gegenueber der
nominalen Stance verschoben hat -- zerlegt in radial / tangential / Hoehe --,
als Gegenprobe zur am Template gemessenen Abweichung.

Speichern schreibt die neuen ``center_us`` pro Kanal in die robot.yaml.
"""
from __future__ import annotations

import math
import time
from pathlib import Path

import readchar
import yaml
from rich import box
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

from hexapod.config.model import Joint
from hexapod.kinematics.leg_ik import forward_kinematics
from hexapod.robot.hexapod import Hexapod
from hexapod.servo_mapper.mapper import MAX_ANGLE_RAD

console = Console()

STEP_SMALL = 1.0   # µs (Pfeil links/rechts)
STEP_LARGE = 5.0   # µs (Pfeil hoch/runter)
JOINTS = (Joint.COXA, Joint.FEMUR, Joint.TIBIA)
JOINT_KEYS = {"c": 0, "f": 1, "t": 2}


def _patch_yaml(config_path: Path, channel: int, center_us: float) -> None:
    raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    for servo in raw.get("servos", []):
        if servo.get("channel") == channel:
            servo["center_us"] = round(center_us, 1)
            break
    config_path.write_text(
        yaml.dump(raw, allow_unicode=True, sort_keys=False, default_flow_style=False),
        encoding="utf-8",
    )


def run_stance_trim(config: Path, simulator: bool = False, do_power_up: bool = False) -> None:
    from hexapod.config.loader import load_robot_config

    cfg = load_robot_config(config)
    if simulator:
        cfg = cfg.model_copy(
            update={"driver": cfg.driver.model_copy(update={"type": "simulator"})}
        )
    robot = Hexapod(cfg)
    legs = robot._leg_names
    leg_idx = 0
    joint_idx = 0

    # Kanal + Original-Mapping je (Bein, Gelenk); Stance-Winkel je Bein.
    chan = {
        (leg, j): robot._config.get_leg_servo(leg, j).channel
        for leg in legs for j in JOINTS
    }
    stance_angles = {leg: robot.offset_to_angles(leg, 0.0, 0.0, 0.0) for leg in legs}
    delta: dict[int, float] = {chan[k]: 0.0 for k in chan}

    def k_per_us(ch: int) -> float:
        m = robot._mappings[ch]
        return m.range_us / MAX_ANGLE_RAD  # µs pro rad

    def send(leg: str) -> None:
        for ji, j in enumerate(JOINTS):
            ch = chan[(leg, j)]
            m = robot._mappings[ch]
            ang = stance_angles[leg][ji]
            us = m.center_us + delta[ch] + m.direction * ang * k_per_us(ch)
            us = max(m.min_us, min(m.max_us, us))
            robot.set_servo_us(ch, us)

    def foot_shift_mm(leg: str) -> tuple[float, float, float]:
        """Fuss-Verschiebung ggue. nominaler Stance: (radial, tangential, hoehe)."""
        ma = robot._mount_angles[leg]
        cx, cy, _ = robot._coxa_positions[leg]
        # effektive Winkel unter ORIGINAL-Mapping aus den gesendeten Pulsen
        ang_eff = []
        for ji, j in enumerate(JOINTS):
            ch = chan[(leg, j)]
            m = robot._mappings[ch]
            ang = stance_angles[leg][ji]
            ang_eff.append(ang + delta[ch] / (m.direction * k_per_us(ch)))
        lx, ly, lz = forward_kinematics(
            ang_eff[0], ang_eff[1], ang_eff[2], robot.leg_lengths
        )
        wx = lx * math.cos(ma) - ly * math.sin(ma) + cx
        wy = lx * math.sin(ma) + ly * math.cos(ma) + cy
        tx, ty = robot.neutral_foot_xy[leg]
        dx, dy = wx - tx, wy - ty
        radial = dx * math.cos(ma) + dy * math.sin(ma)
        tangential = -dx * math.sin(ma) + dy * math.cos(ma)
        _, _, nz = robot._neutral_world[leg]
        return radial, tangential, lz - nz

    def render(msg: str = "") -> None:
        console.clear()
        leg = legs[leg_idx]
        console.print(Panel(
            f"[bold]Stance-Trim[/bold]  ·  Bein [cyan]{leg}[/cyan]",
            style="blue", box=box.ROUNDED,
        ))
        t = Table(box=box.SIMPLE, show_header=True)
        t.add_column("Gelenk")
        t.add_column("Kanal", justify="right")
        t.add_column("center_us", justify="right")
        t.add_column("Δ µs", justify="right")
        for ji, j in enumerate(JOINTS):
            ch = chan[(leg, j)]
            m = robot._mappings[ch]
            sel = "▶ " if ji == joint_idx else "  "
            d = delta[ch]
            t.add_row(
                f"{sel}{j.value}", str(ch),
                f"{m.center_us + d:.1f}",
                f"[yellow]{d:+.1f}[/yellow]" if abs(d) > 0.05 else "[dim]0[/dim]",
            )
        console.print(t)
        rad, tan, hgt = foot_shift_mm(leg)
        console.print(
            f"Fuss-Verschiebung ggue. Soll:  "
            f"radial [bold]{rad:+.1f}[/bold] mm   "
            f"tangential [bold]{tan:+.1f}[/bold] mm   "
            f"Hoehe [bold]{hgt:+.1f}[/bold] mm"
        )
        console.print(Panel(
            "  [bold]c/f/t[/bold] Gelenk   [bold]n/p[/bold] od. [bold]1-6[/bold] Bein\n"
            "  [bold]←/→[/bold] ±1 µs    [bold]↑/↓[/bold] ±5 µs    [bold]r[/bold] Reset Gelenk\n"
            "  [bold]s[/bold] speichern   [bold]q[/bold] beenden",
            title="Tasten", style="dim", box=box.ROUNDED,
        ))
        if msg:
            console.print(f"\n{msg}")

    try:
        n = robot._config.driver.num_channels
        # Gedaempfte Servo-Geschwindigkeit: erstes Anfahren der Stance und jeder
        # Trim-Schritt laeuft sanft statt ruckartig.
        try:
            robot._driver.set_speed_all(n, 25)
            robot._driver.set_acceleration_all(n, 4)
        except Exception:
            pass
        if do_power_up:
            from hexapod.gait.posture import CALIB_X, power_up
            console.print("Beine in Kalibrierposition, dann Enter...")
            input()
            robot.set_all_foot_positions({leg: (CALIB_X, 0.0, 0.0) for leg in legs}, clip=True)
            time.sleep(2)
            power_up(robot)
        else:
            # Roboter steht bereits in der Stance: nur Ist-Lage erfassen. KEIN
            # settle_to_stance -- das startet intern am ausgestreckten Calib-Punkt
            # (233 mm = Reichweitenlimit) und wirft dort UnreachableError.
            robot.prime()
            time.sleep(0.2)
        for leg in legs:
            send(leg)
        render()
        while True:
            key = readchar.readkey()
            leg = legs[leg_idx]
            ch = chan[(leg, JOINTS[joint_idx])]
            m = robot._mappings[ch]
            lo, hi = m.min_us - m.center_us, m.max_us - m.center_us
            if key in JOINT_KEYS:
                joint_idx = JOINT_KEYS[key]
            elif key in ("n", "N"):
                leg_idx = (leg_idx + 1) % len(legs)
            elif key in ("p", "P"):
                leg_idx = (leg_idx - 1) % len(legs)
            elif key in tuple("123456")[: len(legs)]:
                leg_idx = int(key) - 1
            elif key == readchar.key.RIGHT:
                delta[ch] = min(hi, delta[ch] + STEP_SMALL)
                send(leg)
            elif key == readchar.key.LEFT:
                delta[ch] = max(lo, delta[ch] - STEP_SMALL)
                send(leg)
            elif key == readchar.key.UP:
                delta[ch] = min(hi, delta[ch] + STEP_LARGE)
                send(leg)
            elif key == readchar.key.DOWN:
                delta[ch] = max(lo, delta[ch] - STEP_LARGE)
                send(leg)
            elif key in ("r", "R"):
                delta[ch] = 0.0
                send(leg)
            elif key in ("s", "S"):
                changed = {c: robot._mappings[c].center_us + d
                           for c, d in delta.items() if abs(d) > 0.05}
                for c, newc in sorted(changed.items()):
                    _patch_yaml(config, c, newc)
                render(f"[green]✓ {len(changed)} Kanal/Kanaele gespeichert.[/green]"
                       if changed else "[yellow]Keine Aenderung.[/yellow]")
                readchar.readkey()
            elif key in ("q", "Q", readchar.key.CTRL_C):
                break
            render()
    finally:
        robot.close(disable=False)
        console.clear()
        console.print("Beendet (Pose gehalten).")
