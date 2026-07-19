"""Interaktives Einstellen des Z-Trim pro Bein.

Der Roboter steht in Standpose. Mit Pfeiltasten wählt man ein Bein und
verschiebt seinen Fuß in Z (hoch/runter), bis alle Beine gleichmäßig
Bodenkontakt haben. Speichern schreibt die Werte in die robot.yaml.
"""

from __future__ import annotations

from pathlib import Path

import readchar
from rich.console import Console
from rich.table import Table

from hexapod.robot.hexapod import Hexapod

console = Console()

STEP = 1.0  # mm pro Tastendruck


def run_trim(config: Path, do_power_up: bool = False) -> None:
    import time
    robot = Hexapod.from_config(config)
    legs = robot._leg_names
    idx = 0

    def render() -> None:
        console.clear()
        table = Table(title="Z-Trim Einstellung")
        table.add_column("Bein")
        table.add_column("Z-Trim (mm)", justify="right")
        table.add_column("", justify="center")
        for i, leg in enumerate(legs):
            marker = "◀" if i == idx else ""
            table.add_row(leg, f"{robot.get_z_trim(leg):+.1f}", marker)
        console.print(table)
        console.print(
            "\n[bold]↑/↓[/bold] Bein wählen   "
            "[bold]←/→[/bold] Trim −/+   "
            "[bold]s[/bold] speichern   [bold]q[/bold] beenden"
        )
        console.print(
            "[dim]Positiver Trim = Fuß tiefer (Bein trägt mehr).[/dim]"
        )

    try:
        if do_power_up:
            # Kaltstart aus Kalibrierposition (alles in EINER Sitzung).
            from hexapod.gait.posture import CALIB_X, power_up
            console.print("Kaltstart: Beine in Kalibrierposition richten, dann Enter...")
            input()
            robot.set_all_foot_positions(
                {leg: (CALIB_X, 0.0, 0.0) for leg in legs}, clip=True
            )
            time.sleep(2)
            n = robot._config.driver.num_channels
            robot._driver.set_speed_all(n, 0)
            robot._driver.set_acceleration_all(n, 0)
            time.sleep(0.3)
            console.print("Aufstehen...")
            power_up(robot)
        else:
            # Roboter steht schon (z.B. nach separatem power-up): nur sanft
            # in die Standpose settlen, NICHT hart ziehen.
            from hexapod.gait.posture import settle_to_stance
            robot.prime()
            time.sleep(0.3)
            settle_to_stance(robot)
        render()
        while True:
            key = readchar.readkey()
            if key == readchar.key.UP:
                idx = (idx - 1) % len(legs)
            elif key == readchar.key.DOWN:
                idx = (idx + 1) % len(legs)
            elif key == readchar.key.RIGHT:
                leg = legs[idx]
                robot.set_z_trim(leg, robot.get_z_trim(leg) + STEP)
                robot.set_foot_offset(leg, 0, 0, 0, clip=True)
            elif key == readchar.key.LEFT:
                leg = legs[idx]
                robot.set_z_trim(leg, robot.get_z_trim(leg) - STEP)
                robot.set_foot_offset(leg, 0, 0, 0, clip=True)
            elif key in ("s", "S"):
                path = robot.save_z_trims()
                console.print(f"\n[green]Gespeichert nach {path}[/green]")
                readchar.readkey()
            elif key in ("q", "Q", readchar.key.CTRL_C):
                break
            render()
    finally:
        # Pose halten, Servos nicht abschalten.
        robot.close(disable=False)
        console.print("Beendet (Pose gehalten).")
