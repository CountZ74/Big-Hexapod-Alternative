"""Typer-App — Entry-Point für alle CLI-Befehle."""

from __future__ import annotations

from pathlib import Path

import typer

app = typer.Typer(
    name="hexapod",
    help="Hexapod-Steuerung und Kalibrierung.",
    add_completion=False,
    no_args_is_help=True,
)


@app.command("calibrate")
def calibrate(
    config: Path = typer.Option(
        Path("config/robot.yaml"),
        "--config", "-c",
        help="Pfad zur robot.yaml",
    ),
    simulator: bool = typer.Option(
        False,
        "--simulator", "-s",
        help="Simulator statt echtem Maestro nutzen",
    ),
    start_channel: int = typer.Option(
        0,
        "--start", "-n",
        help="Mit diesem Kanal beginnen (0-basiert)",
    ),
    bus: str | None = typer.Option(
        None, "--bus", "-b",
        help="Welcher Controller (Default: erster Maestro der Konfiguration)",
    ),
) -> None:
    """Interaktive Servo-Kalibrierung (immer genau ein Controller)."""
    from .calibrate import run_calibration
    run_calibration(
        config=config, simulator=simulator, start_channel=start_channel, bus=bus
    )


_CONFIG_OPT = typer.Option(
    Path("config/robot.yaml"), "--config", "-c", help="Pfad zur robot.yaml"
)


@app.command("power-up")
def power_up_cmd(
    config: Path = _CONFIG_OPT,
    yes: bool = typer.Option(
        False, "--yes", "-y", help="Ohne Rückfrage starten"
    ),
) -> None:
    """Kaltstart: aus der Kalibrierposition aufstehen (mit P1-Schutz)."""
    from .motion import cmd_power_up
    cmd_power_up(config, confirm=not yes)


@app.command("stand-up")
def stand_up_cmd(config: Path = _CONFIG_OPT) -> None:
    """Körper aus abgesetzter Lage in die Standpose heben."""
    from .motion import cmd_stand_up
    cmd_stand_up(config)


@app.command("lie-down")
def lie_down_cmd(config: Path = _CONFIG_OPT) -> None:
    """Körper in die abgesetzte Lage senken."""
    from .motion import cmd_lie_down
    cmd_lie_down(config)


@app.command("stance")
def stance_cmd(config: Path = _CONFIG_OPT) -> None:
    """Alle Beine in die Standpose fahren."""
    from .motion import cmd_stance
    cmd_stance(config)


@app.command("walk")
def walk_cmd(
    config: Path = _CONFIG_OPT,
    cycles: int = typer.Option(3, "--cycles", "-n", help="Anzahl Zyklen"),
    stride: float = typer.Option(40.0, "--stride", help="Schrittlänge mm"),
    height: float = typer.Option(30.0, "--height", help="Hubhöhe mm"),
    rate_hz: float = typer.Option(40.0, "--rate", help="Sende-Frequenz Hz"),
    direction: float = typer.Option(
        1.0, "--direction", "-d", help="+1 vorwärts, -1 rückwärts"
    ),
) -> None:
    """Tripod-Gait laufen."""
    from .motion import cmd_walk
    cmd_walk(config, cycles, stride, height, rate_hz, direction)


@app.command("move")
def move_cmd(
    config: Path = _CONFIG_OPT,
    vx: float = typer.Option(0.0, "--vx", help="Translation vorwärts(+)/rückwärts(-) mm/Schritt"),
    vy: float = typer.Option(0.0, "--vy", help="Translation links(+)/rechts(-) mm/Schritt"),
    omega: float = typer.Option(0.0, "--omega", "-w", help="Drehung CCW(+) rad/Schritt"),
    cycles: int = typer.Option(3, "--cycles", "-n", help="Anzahl Zyklen"),
    height: float = typer.Option(30.0, "--height", help="Hubhöhe mm"),
    rate_hz: float = typer.Option(40.0, "--rate", help="Sende-Frequenz Hz"),
) -> None:
    """Verallgemeinerter Gait: vorwärts, seitwärts, drehen und Mischungen."""
    from .motion import cmd_move
    cmd_move(config, vx, vy, omega, cycles, height, rate_hz)


@app.command("pose")
def pose_cmd(
    config: Path = _CONFIG_OPT,
    tx: float = typer.Option(0.0, "--tx", help="K\u00f6rper vor(+)/zur\u00fcck(-) mm"),
    ty: float = typer.Option(0.0, "--ty", help="K\u00f6rper links(+)/rechts(-) mm"),
    tz: float = typer.Option(0.0, "--tz", help="K\u00f6rper hoch(+)/runter(-) mm"),
    roll: float = typer.Option(0.0, "--roll", "-r", help="Rollen um X-Achse, Grad"),
    pitch: float = typer.Option(0.0, "--pitch", "-p", help="Nicken um Y-Achse, Grad"),
    yaw: float = typer.Option(0.0, "--yaw", help="Gieren um Z-Achse, Grad"),
    steps: int = typer.Option(30, "--steps", help="Interpolationsschritte"),
    rate_hz: float = typer.Option(50.0, "--rate", help="Sende-Frequenz Hz"),
) -> None:
    """K\u00f6rperpose \u00fcber stehenden F\u00fc\u00dfen einnehmen (Translation + Rotation)."""
    from .motion import cmd_pose
    cmd_pose(config, tx, ty, tz, roll, pitch, yaw, steps, rate_hz)


@app.command("trim")
def trim_cmd(
    config: Path = _CONFIG_OPT,
    power_up: bool = typer.Option(
        False, "--power-up", "-p",
        help="Vorher aus Kalibrierposition aufstehen (alles in einer Sitzung)",
    ),
) -> None:
    """Interaktiv den Z-Trim pro Bein einstellen (Bodenkontakt angleichen)."""
    from .trim import run_trim
    run_trim(config, do_power_up=power_up)


@app.command("stance-trim")
def stance_trim_cmd(
    config: Path = _CONFIG_OPT,
    simulator: bool = typer.Option(
        False, "--simulator", "-s", help="Simulator statt echtem Maestro nutzen"
    ),
    power_up: bool = typer.Option(
        False, "--power-up", "-p",
        help="Vorher aus Kalibrierposition aufstehen (alles in einer Sitzung)",
    ),
) -> None:
    """Servo-Nulllagen in der Stance fein justieren (Footprint-Methode)."""
    from .stance_trim import run_stance_trim
    run_stance_trim(config, simulator=simulator, do_power_up=power_up)


@app.command("foot-monitor")
def foot_monitor_cmd(
    config: Path = _CONFIG_OPT,
    simulator: bool = typer.Option(
        False, "--simulator", "-s", help="Simulator statt echtem Maestro nutzen"
    ),
    rate_hz: float = typer.Option(8.0, "--rate", help="Aktualisierungen pro Sekunde"),
) -> None:
    """Live-Anzeige der Fußsensoren mit Min/Max-Gedächtnis (rein lesend)."""
    from .foot import run_foot_monitor
    run_foot_monitor(config, simulator=simulator, rate_hz=rate_hz)


@app.command("foot-calibrate")
def foot_calibrate_cmd(
    config: Path = _CONFIG_OPT,
    leg: str | None = typer.Option(
        None, "--leg", "-l", help="Nur dieses Bein messen (Default: alle)"
    ),
    simulator: bool = typer.Option(
        False, "--simulator", "-s", help="Simulator statt echtem Maestro nutzen"
    ),
) -> None:
    """Messbereich der Fußsensoren aufnehmen (rein lesend, bewegt keinen Servo)."""
    from .foot import run_foot_calibration
    run_foot_calibration(config, simulator=simulator, only_leg=leg)


@app.command("auto-trim")
def auto_trim_cmd(
    config: Path = _CONFIG_OPT,
    rounds: int = typer.Option(4, "--rounds", "-n", help="Anzahl Mess-/Korrekturrunden"),
    level_per_mm: float | None = typer.Option(
        None, "--level-per-mm",
        help="Federweg-Anteil je mm z_trim (Default: aus robot.yaml oder geschätzt)",
    ),
    damping: float = typer.Option(0.8, "--damping", help="Anteil der Korrektur je Runde"),
    deadband: float = typer.Option(
        3.0, "--deadband",
        help="Residuen unter diesem Prozentwert nicht mehr korrigieren",
    ),
    imu: bool = typer.Option(
        True, "--imu/--no-imu",
        help="IMU mitverwenden (nur auf waagerechtem Boden sinnvoll)",
    ),
) -> None:
    """z_trim automatisch aus Fußsensoren und IMU bestimmen (BEWEGT SERVOS)."""
    from .autotrim import run_auto_trim
    run_auto_trim(
        config, rounds=rounds, level_per_mm=level_per_mm, damping=damping,
        deadband=deadband / 100.0, use_imu=imu,
    )


@app.command("foot-linearity")
def foot_linearity_cmd(
    config: Path = _CONFIG_OPT,
    lift_mm: float = typer.Option(
        30.0, "--lift-mm", help="Hubhöhe der angehobenen Tripod-Gruppe"
    ),
) -> None:
    """Linearität der Fußsensoren prüfen: 6 gegen 3 Beine (BEWEGT SERVOS)."""
    from .linearity import run_foot_linearity
    run_foot_linearity(config, lift_mm=lift_mm)


@app.command("settle")
def settle_cmd(
    config: Path = _CONFIG_OPT,
    touch: float | None = typer.Option(
        None, "--touch",
        help="Aufsetz-Erkennung ab diesem Federweg in Prozent (z.B. 5)",
    ),
    touch_margin_mm: float = typer.Option(
        5.0, "--touch-margin",
        help="Kontakt gilt erst oberhalb dieser Höhe über der Standpose als zu früh",
    ),
    lift_mm: float = typer.Option(15.0, "--lift-mm", help="Hubhöhe je Bein"),
) -> None:
    """Beine einzeln anheben und absetzen, optional mit Aufsetz-Erkennung."""
    from .settle import run_settle
    run_settle(config, touch_percent=touch, touch_margin_mm=touch_margin_mm,
               lift_mm=lift_mm)


@app.command("foot-sweep")
def foot_sweep_cmd(
    config: Path = _CONFIG_OPT,
    leg: str = typer.Option("front_left", "--leg", "-l", help="Welches Bein"),
    max_mm: float = typer.Option(12.0, "--max-mm", help="Maximale Auslenkung nach unten"),
    schritt_mm: float = typer.Option(1.0, "--step-mm", help="Schrittweite"),
) -> None:
    """Kennlinie eines Fußsensors unter Last aufnehmen (BEWEGT SERVOS)."""
    from .sweep import run_foot_sweep
    run_foot_sweep(config, leg=leg, max_mm=max_mm, schritt_mm=schritt_mm)


@app.command("status")
def status() -> None:
    """Zeigt den aktuellen Roboter-Status."""
    typer.echo("Status: noch nicht implementiert.")


if __name__ == "__main__":
    app()
