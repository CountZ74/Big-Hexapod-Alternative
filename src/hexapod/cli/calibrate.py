"""Interaktives Kalibrier-CLI.

Globale Servo-Grenzen kommen aus robot.yaml (servo_limits),
nicht aus hardcodierten Konstanten.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import readchar
import yaml
from rich import box
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

from hexapod.config.loader import load_robot_config
from hexapod.drivers.maestro import MaestroDriver
from hexapod.drivers.simulator import SimulatorDriver

console = Console()

STEP_SMALL = 1
STEP_LARGE = 10


@dataclass
class ServoEntry:
    label: str
    channel: int
    current_us: float
    original_us: float
    min_us: float
    max_us: float


def patch_yaml(config_path: Path, channel: int, center_us: float) -> None:
    raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    for servo in raw.get("servos", []):
        if servo.get("channel") == channel:
            servo["center_us"] = round(center_us, 1)
            break
    config_path.write_text(
        yaml.dump(raw, allow_unicode=True, sort_keys=False, default_flow_style=False),
        encoding="utf-8",
    )


def render_screen(
    entry: ServoEntry,
    index: int,
    total: int,
    message: str = "",
) -> None:
    console.clear()
    changed = abs(entry.current_us - entry.original_us) > 0.05
    console.print(Panel(
        f"[bold]Hexapod Kalibrierung[/bold]  [dim]Servo {index + 1} / {total}[/dim]",
        style="blue", box=box.ROUNDED,
    ))
    table = Table(box=box.SIMPLE, show_header=False, padding=(0, 2))
    table.add_column(style="dim")
    table.add_column(style="bold")
    table.add_row("Servo:", entry.label)
    table.add_row("Kanal:", str(entry.channel))
    table.add_row("Aktuell:", f"[cyan]{entry.current_us:.1f} µs[/cyan]")
    table.add_row("Original:", f"[dim]{entry.original_us:.1f} µs[/dim]")
    table.add_row("Grenzen:", f"{entry.min_us:.0f} – {entry.max_us:.0f} µs")
    table.add_row("Geändert:", "[yellow]✎ Ja[/yellow]" if changed else "[dim]Nein[/dim]")
    console.print(table)
    console.print(Panel(
        "  [bold]←/→[/bold]  ±1 µs        [bold]↑/↓[/bold]  ±10 µs\n"
        "  [bold]R[/bold]    Reset        [bold]N[/bold]    Nächster\n"
        "  [bold]P[/bold]    Voriger      [bold]Q[/bold]    Beenden",
        title="Tasten", style="dim", box=box.ROUNDED,
    ))
    if message:
        console.print(f"\n{message}")


def calibrate_servo(
    entry: ServoEntry,
    index: int,
    total: int,
    driver: MaestroDriver | SimulatorDriver,
) -> str:
    """Gibt zurück: next, prev, save_quit, discard_quit."""
    driver.set_position(entry.channel, entry.current_us)

    while True:
        render_screen(entry, index, total)
        key = readchar.readkey()

        if key == readchar.key.RIGHT:
            entry.current_us = min(entry.max_us, entry.current_us + STEP_SMALL)
            driver.set_position(entry.channel, entry.current_us)
        elif key == readchar.key.LEFT:
            entry.current_us = max(entry.min_us, entry.current_us - STEP_SMALL)
            driver.set_position(entry.channel, entry.current_us)
        elif key == readchar.key.UP:
            entry.current_us = min(entry.max_us, entry.current_us + STEP_LARGE)
            driver.set_position(entry.channel, entry.current_us)
        elif key == readchar.key.DOWN:
            entry.current_us = max(entry.min_us, entry.current_us - STEP_LARGE)
            driver.set_position(entry.channel, entry.current_us)
        elif key in ("r", "R"):
            entry.current_us = entry.original_us
            driver.set_position(entry.channel, entry.current_us)
        elif key in ("n", "N"):
            return "next"
        elif key in ("p", "P"):
            return "prev"
        elif key in ("q", "Q"):
            render_screen(
                entry, index, total,
                "[bold yellow]Beenden?[/bold yellow]  "
                "[bold][S][/bold] Alle Änderungen speichern    "
                "[bold][Q][/bold] Verwerfen    "
                "[bold][andere Taste][/bold] Weiter",
            )
            confirm = readchar.readkey()
            if confirm in ("s", "S"):
                return "save_quit"
            elif confirm in ("q", "Q"):
                return "discard_quit"


def run_calibration(
    config: Path = Path("config/robot.yaml"),
    simulator: bool = False,
    start_channel: int = 0,
) -> None:
    """Interaktive Servo-Kalibrierung."""
    if not config.exists():
        console.print(f"[red]Fehler: {config} nicht gefunden.[/red]")
        return

    robot_config = load_robot_config(config)

    # Grenzen aus der Konfig — eine einzige Quelle der Wahrheit
    abs_min = robot_config.servo_limits.absolute_min_us
    abs_max = robot_config.servo_limits.absolute_max_us

    if simulator:
        driver: MaestroDriver | SimulatorDriver = SimulatorDriver(
            num_channels=robot_config.driver.num_channels,
            verbose=False,
        )
        console.print("[yellow]Simulator-Modus — kein Servo bewegt sich.[/yellow]\n")
    else:
        driver = MaestroDriver(
            port=robot_config.driver.port,
            num_channels=robot_config.driver.num_channels,
            # Manuelles Jog-Tool: bewusst langsam/sanft, damit ein weit
            # entfernter Zielwert den Servo nicht ruckartig springen laesst.
            # (Der Laufbetrieb nutzt den unbegrenzten Default.)
            initial_speed=10,
            initial_acceleration=3,
        )

    entries: list[ServoEntry] = []
    for servo in sorted(robot_config.servos, key=lambda s: s.channel):
        if hasattr(servo, "leg") and hasattr(servo, "joint"):
            label = f"{servo.leg} · {servo.joint}"
        elif hasattr(servo, "axis"):
            label = f"kamera · {servo.axis}"
        else:
            label = f"kanal {servo.channel}"

        entries.append(ServoEntry(
            label=label,
            channel=servo.channel,
            current_us=servo.center_us,
            original_us=servo.center_us,
            # Per-Servo Grenzen, aber innerhalb der absoluten Grenzen
            min_us=max(abs_min, servo.min_us),
            max_us=min(abs_max, servo.max_us),
        ))

    index = max(0, min(start_channel, len(entries) - 1))
    save = False

    try:
        with driver:
            while 0 <= index < len(entries):
                result = calibrate_servo(entries[index], index, len(entries), driver)

                if result == "next":
                    index = min(len(entries) - 1, index + 1)
                elif result == "prev":
                    index = max(0, index - 1)
                elif result == "save_quit":
                    save = True
                    break
                elif result == "discard_quit":
                    break

    except KeyboardInterrupt:
        pass
    finally:
        console.clear()

    if save:
        changed = {
            e.channel: e.current_us
            for e in entries
            if abs(e.current_us - e.original_us) > 0.05
        }
        if changed:
            console.print(
                f"\n[bold]Speichere {len(changed)} geänderte Wert(e) in {config}...[/bold]"
            )
            for channel, us in sorted(changed.items()):
                patch_yaml(config, channel, us)
                console.print(f"  Kanal {channel:2d}: center_us = {us:.1f} µs")
            console.print("\n[green]✓ Fertig.[/green]")
        else:
            console.print("\n[yellow]Keine Änderungen — nichts gespeichert.[/yellow]")
    else:
        console.print("\n[yellow]Abgebrochen — keine Werte gespeichert.[/yellow]")
