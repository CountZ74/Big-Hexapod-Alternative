"""Kalibrierung und Live-Anzeige der Fußsensoren.

Die Fußsensoren (Hall-Sensor + federbelastete Schubstange) liefern einen
Rohwert 0..1023 vom Analogeingang des Maestro. Damit daraus ein brauchbares
"Fuß hat Boden"-Signal wird, brauchen wir pro Bein zwei Referenzpunkte:

  1. Rohwert, wenn das Bein frei in der Luft hängt   → raw_released
  2. Rohwert, wenn der Fuß voll aufgesetzt ist       → raw_contact

Dazwischen wird linear normiert. Ob der Wert beim Aufsetzen steigt oder
fällt, ist egal — das ergibt sich aus den beiden Messungen.

SICHERHEIT: Dieses Werkzeug bewegt KEINEN Servo. Es öffnet den Maestro
bewusst ohne Speed-/Acceleration-Initialisierung und sendet ausschließlich
Lesebefehle. Man kann es also gefahrlos laufen lassen, während der Roboter
steht oder in Kalibrierposition liegt.
"""

from __future__ import annotations

import time
from pathlib import Path

import typer
from rich.console import Console
from rich.live import Live
from rich.table import Table

from hexapod.config.loader import load_robot_config, save_foot_sensor_calibrations
from hexapod.config.model import (
    ANALOG_MAX,
    MIN_CALIBRATION_SPAN,
    FootSensorCalibration,
    RobotConfig,
)
from hexapod.drivers.foot_sensor import FootSensorArray, endpoints_from_samples
from hexapod.drivers.maestro import MaestroDriver
from hexapod.drivers.simulator import SimulatorDriver

console = Console()

# Messreihe pro Referenzpunkt: 40 Werte in ~1,2 s. Genug, um Zittern und
# einzelne ADC-Ausreißer über den Median wegzumitteln, kurz genug, dass man
# den Fuß problemlos so lange still hält.
CALIB_SAMPLES = 40
CALIB_INTERVAL = 0.03

BAR_WIDTH = 24


# ---------------------------------------------------------------------
# Treiber-Aufbau — bewusst OHNE Hexapod-Klasse
# ---------------------------------------------------------------------


ReadOnlyDriver = MaestroDriver | SimulatorDriver


def _open_driver(config: RobotConfig, *, simulator: bool) -> ReadOnlyDriver:
    """Öffnet einen rein lesenden Treiber.

    `initial_speed=None` und `initial_acceleration=None` sind hier der Kern
    der Sicherheit: der MaestroDriver schreibt dann beim Öffnen kein einziges
    Byte auf die Servokanäle.
    """
    if simulator or config.driver.type == "simulator":
        return SimulatorDriver(num_channels=config.driver.num_channels)
    return MaestroDriver(
        port=config.driver.port,
        num_channels=config.driver.num_channels,
        timeout=config.driver.timeout,
        min_pulse_us=config.servo_limits.absolute_min_us,
        max_pulse_us=config.servo_limits.absolute_max_us,
        initial_speed=None,
        initial_acceleration=None,
    )


def _close(driver: ReadOnlyDriver) -> None:
    # disable=False: keinen einzigen Kanal abschalten — der Roboter soll
    # seine Pose behalten, auch wenn er gerade steht.
    driver.close(disable=False)


def _load(config_path: Path) -> RobotConfig:
    config = load_robot_config(config_path)
    if not config.foot_sensors.active:
        console.print(
            "[red]In der robot.yaml ist kein aktiver Fußsensor eingetragen.[/red]\n"
            "Erwartet wird ein Block wie:\n\n"
            "[dim]foot_sensors:\n"
            "  sensors:\n"
            "    - leg: front_left\n"
            "      channel: 0[/dim]"
        )
        raise typer.Exit(code=1)
    return config


# ---------------------------------------------------------------------
# Anzeige-Bausteine
# ---------------------------------------------------------------------


def _bar(fraction: float, width: int = BAR_WIDTH) -> str:
    filled = round(max(0.0, min(1.0, fraction)) * width)
    return "█" * filled + "·" * (width - filled)


def _live_table(sensors: FootSensorArray, *, title: str) -> Table:
    table = Table(title=title)
    table.add_column("Bein")
    table.add_column("Kanal", justify="right")
    table.add_column("Roh", justify="right")
    table.add_column("Volt", justify="right")
    table.add_column("Pegel", justify="right")
    table.add_column("")
    table.add_column("Kontakt", justify="center")

    for leg, r in sensors.read_all().items():
        if r.level is None:
            level_txt, bar, contact = "—", _bar(r.raw / ANALOG_MAX), "[dim]unkal.[/dim]"
        else:
            level_txt = f"{r.level:.2f}"
            bar = _bar(r.level)
            contact = "[green]BODEN[/green]" if r.contact else "[dim]frei[/dim]"
        table.add_row(
            leg, str(r.channel), f"{r.raw:.0f}", f"{r.volts:.2f}", level_txt, bar, contact
        )
    return table


def _measure(driver: ReadOnlyDriver, channel: int, *, label: str) -> list[float]:
    """Eine Messreihe aufnehmen und dabei live anzeigen."""
    values: list[float] = []
    with Live(console=console, refresh_per_second=12) as live:
        for i in range(CALIB_SAMPLES):
            value = float(driver.read_analog(channel))
            values.append(value)
            live.update(
                f"{label}  [bold]{value:4.0f}[/bold]  "
                f"{_bar(value / ANALOG_MAX)}  ({i + 1}/{CALIB_SAMPLES})"
            )
            time.sleep(CALIB_INTERVAL)
    return values


def _spread(values: list[float]) -> str:
    lo, hi = min(values), max(values)
    return f"min {lo:.0f} / max {hi:.0f} (Streuung {hi - lo:.0f})"


# ---------------------------------------------------------------------
# Befehl: foot-monitor
# ---------------------------------------------------------------------


def run_foot_monitor(
    config_path: Path,
    *,
    simulator: bool = False,
    rate_hz: float = 8.0,
) -> None:
    """Live-Ansicht aller Fußsensoren, bis Strg-C."""
    config = _load(config_path)
    driver = _open_driver(config, simulator=simulator)
    sensors = FootSensorArray(driver, config.foot_sensors)
    period = 1.0 / max(0.5, rate_hz)
    console.print("[dim]Strg-C beendet die Anzeige.[/dim]")
    try:
        with Live(console=console, refresh_per_second=12) as live:
            while True:
                live.update(_live_table(sensors, title="Fußsensoren"))
                time.sleep(period)
    except KeyboardInterrupt:
        console.print("\nBeendet.")
    finally:
        _close(driver)


# ---------------------------------------------------------------------
# Befehl: foot-calibrate
# ---------------------------------------------------------------------


def run_foot_calibration(
    config_path: Path,
    *,
    simulator: bool = False,
    only_leg: str | None = None,
    threshold: float = 0.40,
    hysteresis: float = 0.15,
) -> None:
    """Interaktive Kalibrierung — pro Bein zwei Messungen, dann Live-Test."""
    config = _load(config_path)

    todo = [s for s in config.foot_sensors.active if only_leg in (None, s.leg)]
    if not todo:
        console.print(f"[red]Kein aktiver Fußsensor für Bein {only_leg!r}.[/red]")
        raise typer.Exit(code=1)

    console.print(
        "\n[bold]Fußsensor-Kalibrierung[/bold]\n"
        "[dim]Es wird ausschließlich gelesen — kein Servo bewegt sich.\n"
        "Läuft der Webserver (hexapod-web), belegt er die serielle Schnittstelle:\n"
        "  sudo systemctl stop hexapod-web[/dim]\n"
    )

    driver = _open_driver(config, simulator=simulator)
    results: dict[str, FootSensorCalibration] = {}

    try:
        for sensor in todo:
            console.rule(f"[bold]{sensor.leg}[/bold]  (Maestro-Kanal {sensor.channel})")

            answer = typer.prompt(
                "[Enter] kalibrieren, [s] überspringen, [q] beenden",
                default="",
                show_default=False,
            ).strip().lower()
            if answer == "q":
                break
            if answer == "s":
                continue

            calib = _calibrate_one(
                driver,
                sensor.channel,
                sensor.leg,
                threshold=threshold,
                hysteresis=hysteresis,
            )
            if calib is None:
                continue
            results[sensor.leg] = calib

            _live_check(driver, config, sensor.leg, calib)

        if not results:
            console.print("\nNichts kalibriert, nichts gespeichert.")
            return

        console.rule("Ergebnis")
        table = Table()
        table.add_column("Bein")
        table.add_column("frei", justify="right")
        table.add_column("Kontakt", justify="right")
        table.add_column("Spanne", justify="right")
        table.add_column("Schwelle", justify="right")
        for leg, calib in results.items():
            table.add_row(
                leg,
                f"{calib.raw_released:.0f}",
                f"{calib.raw_contact:.0f}",
                f"{calib.span:+.0f}",
                f"{calib.threshold:.2f} (-{calib.hysteresis:.2f})",
            )
        console.print(table)

        if typer.confirm(f"In {config_path} speichern?", default=True):
            path = save_foot_sensor_calibrations(config_path, results)
            console.print(f"[green]Gespeichert nach {path}[/green]")
        else:
            console.print("Nicht gespeichert.")
    finally:
        _close(driver)


def _calibrate_one(
    driver: ReadOnlyDriver,
    channel: int,
    leg: str,
    *,
    threshold: float,
    hysteresis: float,
) -> FootSensorCalibration | None:
    """Die beiden Referenzmessungen für ein Bein. None = verworfen."""
    while True:
        console.print(
            f"\n[bold]1/2 — frei:[/bold] {leg} anheben, sodass der Fuß nichts "
            f"berührt und die Feder ganz ausgefahren ist."
        )
        typer.prompt("Bereit? [Enter]", default="", show_default=False)
        released = _measure(driver, channel, label="frei    ")
        console.print(f"  Median [bold]{sorted(released)[len(released) // 2]:.0f}[/bold]"
                      f"   {_spread(released)}")

        console.print(
            f"\n[bold]2/2 — Kontakt:[/bold] Fuß von {leg} fest aufsetzen, bis die "
            f"Feder spürbar eingedrückt ist — so fest wie im Stand."
        )
        typer.prompt("Bereit? [Enter]", default="", show_default=False)
        contact = _measure(driver, channel, label="Kontakt ")
        console.print(f"  Median [bold]{sorted(contact)[len(contact) // 2]:.0f}[/bold]"
                      f"   {_spread(contact)}")

        raw_released, raw_contact = endpoints_from_samples(released, contact)
        span = raw_contact - raw_released
        console.print(
            f"\n  frei {raw_released:.0f} → Kontakt {raw_contact:.0f}  "
            f"(Spanne {span:+.0f}, "
            f"{'steigend' if span > 0 else 'fallend'})"
        )

        if abs(span) < MIN_CALIBRATION_SPAN:
            console.print(
                f"[red]Zu wenig Signal ({abs(span):.0f} < {MIN_CALIBRATION_SPAN:.0f} "
                f"Zähler).[/red] Mögliche Ursachen: Magnet zu weit vom Hall-Sensor, "
                f"Kanal {channel} noch nicht als 'Input' im Maestro konfiguriert, "
                f"oder die Feder wurde nicht wirklich eingedrückt."
            )
            if typer.confirm("Nochmal versuchen?", default=True):
                continue
            return None

        # Rauschabschätzung: Wie sicher trennt die Schwelle die beiden Zustände?
        noise = max(
            max(released) - min(released),
            max(contact) - min(contact),
        )
        if noise > abs(span) * 0.5:
            console.print(
                f"[yellow]Achtung: Streuung ({noise:.0f}) ist groß gegenüber der "
                f"Spanne ({abs(span):.0f}) — das Signal wird wackelig.[/yellow]"
            )

        try:
            return FootSensorCalibration(
                raw_released=raw_released,
                raw_contact=raw_contact,
                threshold=threshold,
                hysteresis=hysteresis,
            )
        except ValueError as e:
            console.print(f"[red]Unbrauchbar: {e}[/red]")
            if typer.confirm("Nochmal versuchen?", default=True):
                continue
            return None


def _live_check(
    driver: ReadOnlyDriver,
    config: RobotConfig,
    leg: str,
    calib: FootSensorCalibration,
) -> None:
    """Direkt nach der Messung prüfen, ob die Schwelle im Alltag passt."""
    console.print(
        "\n[bold]Test:[/bold] Fuß mehrfach auf- und absetzen — springt "
        "'BODEN' sauber um? [dim](Strg-C beendet den Test)[/dim]"
    )
    # Eine Wegwerf-Konfiguration nur für diesen einen Sensor, damit die
    # Auswertung exakt dieselbe Code-Bahn nimmt wie später im Betrieb.
    patched = config.foot_sensors.model_copy(
        update={
            "sensors": [
                s.model_copy(update={"calibration": calib}) if s.leg == leg else s
                for s in config.foot_sensors.sensors
                if s.leg == leg
            ]
        }
    )
    sensors = FootSensorArray(driver, patched)
    try:
        with Live(console=console, refresh_per_second=12) as live:
            while True:
                live.update(_live_table(sensors, title=f"Test {leg}"))
                time.sleep(0.08)
    except KeyboardInterrupt:
        console.print("")
