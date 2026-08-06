"""Beine einzeln anheben und absetzen — mit optionaler Aufsetz-Erkennung.

Zum Ausprobieren gedacht: leg etwas unter einen Fuß und schau, ob das Bein
dort stehenbleibt, statt blind auf Standpose-Höhe durchzudrücken.

BEWEGT SERVOS. Der Roboter muss auf allen sechs Beinen stehen.
"""

from __future__ import annotations

from pathlib import Path

import typer
from rich.console import Console
from rich.table import Table

from hexapod.robot.hexapod import Hexapod

console = Console()


def run_settle(
    config_path: Path,
    *,
    touch_percent: float | None = None,
    lift_mm: float = 15.0,
) -> None:
    from hexapod.gait.posture import settle_to_stance

    robot = Hexapod.from_config(config_path)
    try:
        if not robot.sync_state_from_hardware():
            console.print(
                "[red]Kein Puls auf den Servokanälen — der Roboter muss stehen "
                "und bestromt sein.[/red]"
            )
            raise typer.Exit(code=1)

        schwelle = None if touch_percent is None else touch_percent / 100.0
        if schwelle is None:
            console.print("[dim]Ohne Aufsetz-Erkennung — jedes Bein fährt auf "
                          "Standpose-Höhe.[/dim]")
        else:
            sensors = robot.foot_sensors
            anzahl = 0 if sensors is None else len(sensors.legs)
            console.print(
                f"[dim]Aufsetz-Erkennung bei {touch_percent:.1f} % Federweg, "
                f"{anzahl} Beine mit Sensor.[/dim]"
            )

        frueh = settle_to_stance(
            robot, force=True, lift=lift_mm, touch_level=schwelle
        )

        if not frueh:
            console.print(
                "\n[green]Alle Beine haben die Standpose erreicht.[/green]"
                + ("" if schwelle is None else
                   " Kein Bein hat vorher Boden gefunden.")
            )
            return

        table = Table(title="Beine, die früher aufgesetzt haben")
        table.add_column("Bein")
        table.add_column("über Standpose", justify="right")
        for leg, dz in sorted(frueh.items(), key=lambda kv: -kv[1]):
            table.add_row(leg, f"{dz:+.1f} mm")
        console.print(table)
        console.print(
            "[dim]Positive Werte heißen: der Boden lag dort höher als "
            "erwartet.[/dim]"
        )
    finally:
        robot.close(disable=False)
