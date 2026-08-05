"""Kalibrierung und Live-Anzeige der Fußsensoren.

Der Fußsensor (Hall-Sensor + federbelastete Schubstange) ist ein Wegaufnehmer:
Er misst, wie weit die Stange gegen die Feder eingedrückt ist — also
näherungsweise die Auflagekraft. Kalibriert wird deshalb nur der
**mechanische Vollweg**, die beiden festen Endpunkte:

  1. Schubstange ganz ausgefahren (Bein frei in der Luft)  → raw_unloaded
  2. Schubstange ganz eingedrückt (mechanischer Anschlag)  → raw_full

Alles, was der Roboter im Betrieb macht — antippen, stehen, laufen, klettern —
liegt irgendwo dazwischen. Wo genau, findet man mit `hexapod foot-monitor`
heraus: Die Anzeige merkt sich den kleinsten und größten gesehenen Wert.

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

# Messreihe pro Endpunkt: 40 Werte in ~1,2 s. Genug, um Zittern und einzelne
# ADC-Ausreißer über den Median wegzumitteln, kurz genug, dass man die Stange
# problemlos so lange in Position hält.
CALIB_SAMPLES = 40
CALIB_INTERVAL = 0.03

BAR_WIDTH = 24

ReadOnlyDriver = MaestroDriver | SimulatorDriver


# ---------------------------------------------------------------------
# Treiber-Aufbau — bewusst OHNE Hexapod-Klasse
# ---------------------------------------------------------------------


def _open_drivers(
    config: RobotConfig, *, simulator: bool
) -> dict[str, ReadOnlyDriver]:
    """Öffnet je Sensor-Bus einen rein lesenden Treiber.

    Die Fußsensoren sitzen lokal am Controller ihrer Körperseite, es sind
    also mehrere Busse im Spiel.

    `initial_speed=None` und `initial_acceleration=None` sind hier der Kern
    der Sicherheit: der MaestroDriver schreibt dann beim Öffnen kein einziges
    Byte auf die Servokanäle.
    """
    drivers: dict[str, ReadOnlyDriver] = {}
    for name in config.foot_sensors.active_buses:
        bus = config.get_bus(name)
        if simulator or bus.type == "simulator":
            drivers[name] = SimulatorDriver(num_channels=bus.num_channels)
        elif bus.type == "maestro":
            drivers[name] = MaestroDriver(
                port=bus.port,
                num_channels=bus.num_channels,
                timeout=bus.timeout,
                min_pulse_us=config.servo_limits.absolute_min_us,
                max_pulse_us=config.servo_limits.absolute_max_us,
                initial_speed=None,
                initial_acceleration=None,
            )
        else:
            raise typer.BadParameter(
                f"Bus {name!r} vom Typ {bus.type!r} hat keine Analogeingänge."
            )
    return drivers


def _close(drivers: dict[str, ReadOnlyDriver]) -> None:
    # disable=False: keinen einzigen Kanal abschalten — der Roboter soll
    # seine Pose behalten, auch wenn er gerade steht.
    for driver in drivers.values():
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


def _hinweis() -> None:
    console.print(
        "[dim]Es wird ausschließlich gelesen — kein Servo bewegt sich.\n"
        "Läuft der Webserver (hexapod-web), belegt er die serielle Schnittstelle:\n"
        "  sudo systemctl stop hexapod-web[/dim]\n"
    )


# ---------------------------------------------------------------------
# Anzeige-Bausteine
# ---------------------------------------------------------------------


def _bar(fraction: float, width: int = BAR_WIDTH) -> str:
    filled = round(max(0.0, min(1.0, fraction)) * width)
    return "█" * filled + "·" * (width - filled)


def _spread(values: list[float]) -> str:
    lo, hi = min(values), max(values)
    return f"min {lo:.0f} / max {hi:.0f} (Streuung {hi - lo:.0f})"


class _PeakHold:
    """Merkt sich den kleinsten und größten Rohwert je Bein.

    Genau dafür ist der Monitor da: Der mechanische Vollweg steht nach der
    Kalibrierung fest, aber wo *im* Bereich Antippen, Stand und Gait liegen,
    weiß man erst, wenn man es gesehen hat.
    """

    def __init__(self) -> None:
        self._min: dict[str, float] = {}
        self._max: dict[str, float] = {}

    def update(self, leg: str, raw: float) -> None:
        self._min[leg] = min(self._min.get(leg, raw), raw)
        self._max[leg] = max(self._max.get(leg, raw), raw)

    def seen(self, leg: str) -> tuple[float, float] | None:
        if leg not in self._min:
            return None
        return self._min[leg], self._max[leg]


def _monitor_table(sensors: FootSensorArray, peaks: _PeakHold) -> Table:
    table = Table(title="Fußsensoren — Federweg")
    table.add_column("Bein")
    table.add_column("Kanal", justify="right")
    table.add_column("Roh", justify="right")
    table.add_column("Volt", justify="right")
    table.add_column("Federweg", justify="right")
    table.add_column("")
    table.add_column("bisher gesehen", justify="right")

    for leg, r in sensors.read_all().items():
        peaks.update(leg, r.raw)
        seen = peaks.seen(leg)
        seen_txt = f"{seen[0]:.0f} … {seen[1]:.0f}" if seen else "—"
        if r.percent is None:
            weg_txt, bar = "[dim]unkal.[/dim]", _bar(r.raw / ANALOG_MAX)
        else:
            weg_txt = f"{r.percent:5.1f} %"
            bar = _bar(r.level or 0.0)
        table.add_row(
            leg, str(r.channel), f"{r.raw:.0f}", f"{r.volts:.2f}", weg_txt, bar, seen_txt
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


# ---------------------------------------------------------------------
# Befehl: foot-monitor
# ---------------------------------------------------------------------


def run_foot_monitor(
    config_path: Path,
    *,
    simulator: bool = False,
    rate_hz: float = 8.0,
) -> None:
    """Live-Ansicht aller Fußsensoren mit Min/Max-Gedächtnis, bis Strg-C."""
    config = _load(config_path)
    drivers = _open_drivers(config, simulator=simulator)
    sensors = FootSensorArray(drivers, config.foot_sensors)
    peaks = _PeakHold()
    period = 1.0 / max(0.5, rate_hz)
    _hinweis()
    console.print("[dim]Strg-C beendet die Anzeige.[/dim]")
    try:
        with Live(console=console, refresh_per_second=12) as live:
            while True:
                live.update(_monitor_table(sensors, peaks))
                time.sleep(period)
    except KeyboardInterrupt:
        console.print("\nBeendet.")
    finally:
        _close(drivers)


# ---------------------------------------------------------------------
# Befehl: foot-calibrate
# ---------------------------------------------------------------------


def run_foot_calibration(
    config_path: Path,
    *,
    simulator: bool = False,
    only_leg: str | None = None,
) -> None:
    """Messbereich pro Bein aufnehmen: die beiden mechanischen Endpunkte."""
    config = _load(config_path)

    todo = [s for s in config.foot_sensors.active if only_leg in (None, s.leg)]
    if not todo:
        console.print(f"[red]Kein aktiver Fußsensor für Bein {only_leg!r}.[/red]")
        raise typer.Exit(code=1)

    console.print(
        "\n[bold]Fußsensor-Messbereich aufnehmen[/bold]\n"
        "[dim]Gemessen wird der mechanische Vollweg der Schubstange — nicht,\n"
        "ab wann der Fuß 'Boden hat'. Diese Grenze hängt davon ab, was der\n"
        "Roboter gerade tut, und wird später im Gait gezogen.[/dim]\n"
    )
    _hinweis()

    drivers = _open_drivers(config, simulator=simulator)
    results: dict[str, FootSensorCalibration] = {}

    try:
        for sensor in todo:
            console.rule(f"[bold]{sensor.leg}[/bold]  (Maestro-Kanal {sensor.channel})")

            answer = typer.prompt(
                "[Enter] messen, [s] überspringen, [q] beenden",
                default="",
                show_default=False,
            ).strip().lower()
            if answer == "q":
                break
            if answer == "s":
                continue

            calib = _calibrate_one(
                drivers[sensor.bus], sensor.channel, sensor.leg
            )
            if calib is None:
                continue
            results[sensor.leg] = calib

            _live_check(drivers, config, sensor.leg, calib)

        if not results:
            console.print("\nNichts gemessen, nichts gespeichert.")
            return

        console.rule("Ergebnis")
        table = Table()
        table.add_column("Bein")
        table.add_column("unbelastet", justify="right")
        table.add_column("Anschlag", justify="right")
        table.add_column("Messbereich", justify="right")
        table.add_column("Auflösung", justify="right")
        for leg, calib in results.items():
            table.add_row(
                leg,
                f"{calib.raw_unloaded:.0f}",
                f"{calib.raw_full:.0f}",
                f"{calib.span:+.0f} Zähler",
                f"{calib.counts_per_percent:.1f} / %",
            )
        console.print(table)

        if typer.confirm(f"In {config_path} speichern?", default=True):
            path = save_foot_sensor_calibrations(config_path, results)
            console.print(f"[green]Gespeichert nach {path}[/green]")
            console.print(
                "[dim]Nächster Schritt: 'hexapod foot-monitor' laufen lassen und "
                "schauen, wo Antippen, Stand und Gait im Bereich landen.[/dim]"
            )
        else:
            console.print("Nicht gespeichert.")
    finally:
        _close(drivers)


def _calibrate_one(
    driver: ReadOnlyDriver,
    channel: int,
    leg: str,
) -> FootSensorCalibration | None:
    """Die beiden Endpunkt-Messungen für ein Bein. None = verworfen."""
    while True:
        console.print(
            f"\n[bold]1/2 — unbelastet:[/bold] {leg} anheben, sodass der Fuß nichts "
            f"berührt und die Feder die Schubstange ganz ausfährt."
        )
        typer.prompt("Bereit? [Enter]", default="", show_default=False)
        unloaded = _measure(driver, channel, label="unbelastet")
        console.print(
            f"  Median [bold]{sorted(unloaded)[len(unloaded) // 2]:.0f}[/bold]"
            f"   {_spread(unloaded)}"
        )

        console.print(
            f"\n[bold]2/2 — Anschlag:[/bold] Schubstange von {leg} von Hand ganz "
            f"eindrücken, bis sie mechanisch nicht weiter geht."
        )
        typer.prompt("Bereit? [Enter]", default="", show_default=False)
        full = _measure(driver, channel, label="Anschlag  ")
        console.print(
            f"  Median [bold]{sorted(full)[len(full) // 2]:.0f}[/bold]"
            f"   {_spread(full)}"
        )

        raw_unloaded, raw_full = endpoints_from_samples(unloaded, full)
        span = raw_full - raw_unloaded
        console.print(
            f"\n  unbelastet {raw_unloaded:.0f} → Anschlag {raw_full:.0f}  "
            f"(Messbereich {span:+.0f} Zähler, "
            f"{'steigend' if span > 0 else 'fallend'})"
        )

        if abs(span) < MIN_CALIBRATION_SPAN:
            console.print(
                f"[red]Zu wenig Signal ({abs(span):.0f} < {MIN_CALIBRATION_SPAN:.0f} "
                f"Zähler).[/red] Mögliche Ursachen: Magnet zu weit vom Hall-Sensor, "
                f"Kanal {channel} noch nicht als 'Input' im Maestro konfiguriert, "
                f"oder die Stange ist nicht bis zum Anschlag gekommen."
            )
            if typer.confirm("Nochmal versuchen?", default=True):
                continue
            return None

        # Rauschabschätzung: Wieviel vom Messbereich frisst das Rauschen?
        noise = max(max(unloaded) - min(unloaded), max(full) - min(full))
        noise_percent = noise / abs(span) * 100.0
        console.print(
            f"  Auflösung {abs(span) / 100.0:.1f} Zähler pro % Federweg, "
            f"Rauschen ~{noise_percent:.1f} % des Bereichs"
        )
        if noise_percent > 10.0:
            console.print(
                "[yellow]Achtung: Das Rauschen frisst über 10 % des Messbereichs — "
                "feine Lastunterschiede werden schwer zu unterscheiden sein.[/yellow]"
            )

        try:
            return FootSensorCalibration(raw_unloaded=raw_unloaded, raw_full=raw_full)
        except ValueError as e:
            console.print(f"[red]Unbrauchbar: {e}[/red]")
            if typer.confirm("Nochmal versuchen?", default=True):
                continue
            return None


def _live_check(
    drivers: dict[str, ReadOnlyDriver],
    config: RobotConfig,
    leg: str,
    calib: FootSensorCalibration,
) -> None:
    """Direkt nach der Messung sehen, wie sich der Federweg anfühlt."""
    console.print(
        "\n[bold]Test:[/bold] Fuß unterschiedlich stark belasten — leicht "
        "antippen, dann fest aufdrücken. Der Federweg sollte sauber "
        "mitlaufen. [dim](Strg-C beendet den Test)[/dim]"
    )
    # Eine Wegwerf-Konfiguration nur für diesen einen Sensor, damit die
    # Auswertung exakt dieselbe Code-Bahn nimmt wie später im Betrieb.
    patched = config.foot_sensors.model_copy(
        update={
            "sensors": [
                s.model_copy(update={"calibration": calib})
                for s in config.foot_sensors.sensors
                if s.leg == leg
            ]
        }
    )
    sensors = FootSensorArray(drivers, patched)
    peaks = _PeakHold()
    try:
        with Live(console=console, refresh_per_second=12) as live:
            while True:
                live.update(_monitor_table(sensors, peaks))
                time.sleep(0.08)
    except KeyboardInterrupt:
        console.print("")
