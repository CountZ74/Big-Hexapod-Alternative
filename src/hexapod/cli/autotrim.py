"""Automatischer Bein-Höhenabgleich aus Fußsensoren und IMU.

Der Roboter misst sich selbst ein. Pro Runde:

  1. Beine einzeln anheben und absetzen, damit sich die Last überhaupt neu
     verteilen kann — Reibung an den Füßen friert sonst den alten Zustand ein.
  2. Federweg aller sechs Füße messen und die Körperneigung ablesen.
  3. Ebene durch die Lastwerte legen. Die Residuen sind die Bein-Einzelfehler;
     Schwerpunkt und Bodenneigung stecken in der Ebene und fallen heraus.
  4. Aus der IMU den Anteil ergänzen, den die Lastsensoren prinzipiell nicht
     sehen können (siehe `hexapod.calib.tilt_corrections`).
  5. z_trim nachziehen, mittelwertfrei.

Nach der ersten Runde rechnet das Werkzeug den mechanischen Vollweg der
Schubstange aus der beobachteten Reaktion selbst zurück — der Wert muss also
nicht bekannt sein.

BEWEGT SERVOS. Der Roboter muss auf allen sechs Beinen stehen, und für den
IMU-Anteil auf einer waagerechten Fläche.
"""

from __future__ import annotations

import json
import statistics
import time
from datetime import datetime
from pathlib import Path

import typer
from rich.console import Console
from rich.table import Table

from hexapod.calib import (
    BalanceResult,
    estimate_level_per_mm,
    fit_load_plane,
    tilt_corrections,
    z_trim_corrections,
)
from hexapod.drivers.foot_sensor import FootSensorArray
from hexapod.robot.hexapod import Hexapod

console = Console()

# Startwert fuer die Empfindlichkeit, bis die erste Runde einen echten
# Wert liefert. Gemessen wurden am Arbeitspunkt rund 0,068 Federweg je mm.
DEFAULT_LEVEL_PER_MM = 0.068
# Messdauer je Runde. Lang genug, dass sich der Aufbau beruhigt.
MEASURE_S = 2.0
MEASURE_INTERVAL = 0.05
# Groesser als das darf z_trim nie werden (auch das Modell begrenzt auf 30).
MAX_Z_TRIM_MM = 25.0
# Standard-Totband. Gemessene Wiederholbarkeit der Lastverteilung liegt bei
# rund 2 % Federweg -- darunter korrigiert man nur noch Rauschen.
DEFAULT_DEADBAND = 0.03
# Rundenprotokoll. Ohne das verschwinden die Messwerte im Terminal -- und
# genau die braucht man, um hinterher zu beurteilen, ob eine Korrektur
# ueberhaupt gewirkt hat.
LOG_PATH = Path("logs/autotrim.jsonl")


def _measure(sensors: FootSensorArray) -> dict[str, float]:
    """Federweg je Bein, Median über mehrere Sekunden."""
    serien: dict[str, list[float]] = {leg: [] for leg in sensors.legs}
    ende = time.time() + MEASURE_S
    while time.time() < ende:
        for leg, reading in sensors.read_all().items():
            if reading.level is not None:
                serien[leg].append(reading.level)
        time.sleep(MEASURE_INTERVAL)
    return {leg: statistics.median(v) for leg, v in serien.items() if v}


def _read_tilt(robot: Hexapod) -> tuple[float, float] | None:
    """Körperneigung in Grad (roll, pitch), oder None ohne IMU."""
    try:
        from hexapod.drivers.mpu6050 import MPU6050
    except Exception:
        return None
    imu = MPU6050(swap_axes=True, invert_roll=True)
    werte = [imu.tilt() for _ in range(20)]
    gueltig = [t for t in werte if t is not None]
    if not gueltig:
        return None
    return (
        statistics.median([t[0] for t in gueltig]),
        statistics.median([t[1] for t in gueltig]),
    )


def _report(runde: int, result: BalanceResult, tilt: tuple[float, float] | None,
            korrektur: dict[str, float], level_per_mm: float,
            vorrunde: dict[str, float] | None = None) -> None:
    table = Table(title=f"Runde {runde}   ({level_per_mm * 100:.1f} % je mm)")
    table.add_column("Bein")
    table.add_column("Federweg", justify="right")
    if vorrunde is not None:
        # Wieviel hat sich seit der letzten Runde bewegt? Das ist die
        # Wiederholbarkeit -- und damit die Grenze dessen, was das
        # Verfahren ueberhaupt aufloesen kann.
        table.add_column("Δ Vorrunde", justify="right")
    table.add_column("Residuum", justify="right")
    table.add_column("Δ z_trim", justify="right")
    for leg in sorted(result.residuals, key=lambda k: -result.residuals[k]):
        zeile = [leg, f"{result.levels[leg] * 100:5.1f} %"]
        if vorrunde is not None:
            d = (result.levels[leg] - vorrunde.get(leg, result.levels[leg])) * 100
            zeile.append(f"{d:+5.1f} %")
        zeile += [
            f"{result.residuals[leg] * 100:+5.1f} %",
            f"{korrektur.get(leg, 0.0):+6.2f} mm",
        ]
        table.add_row(*zeile)
    console.print(table)
    z0, a, b = result.plane
    console.print(
        f"[dim]Ebene: Grundlast {z0 * 100:.1f} %, Längsneigung {a * 1000:+.2f} %/100mm, "
        f"Querneigung {b * 1000:+.2f} %/100mm  (Schwerpunkt + Boden)[/dim]"
    )
    if tilt is not None:
        console.print(f"[dim]IMU: Roll {tilt[0]:+.2f}°, Nick {tilt[1]:+.2f}°[/dim]")


def _log(lauf_id: str, runde: int, result: BalanceResult,
         tilt: tuple[float, float] | None, korrektur: dict[str, float],
         level_per_mm: float, robot: Hexapod) -> None:
    """Eine Zeile je Runde als JSON wegschreiben.

    Bewusst maschinenlesbar und anhaengend: so bleibt die Historie mehrerer
    Trimmlaeufe erhalten und laesst sich hinterher auswerten.
    """
    eintrag = {
        "lauf": lauf_id,
        "zeit": datetime.now().isoformat(timespec="seconds"),
        "runde": runde,
        "level_per_mm": round(level_per_mm, 5),
        "levels": {k: round(v, 5) for k, v in result.levels.items()},
        "residuen": {k: round(v, 5) for k, v in result.residuals.items()},
        "ebene": [round(v, 6) for v in result.plane],
        "tilt_deg": [round(v, 3) for v in tilt] if tilt else None,
        "korrektur_mm": {k: round(v, 4) for k, v in korrektur.items()},
        "z_trim_mm": {leg: round(robot.get_z_trim(leg), 3) for leg in robot.leg_names},
    }
    try:
        LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
        with LOG_PATH.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(eintrag, ensure_ascii=False) + "\n")
    except OSError as e:
        console.print(f"[yellow]Protokoll nicht geschrieben: {e}[/yellow]")


def run_auto_trim(
    config_path: Path,
    *,
    rounds: int = 4,
    level_per_mm: float | None = None,
    damping: float = 0.8,
    deadband: float = DEFAULT_DEADBAND,
    use_imu: bool = True,
) -> None:
    """Misst und korrigiert z_trim in mehreren Runden."""
    import math

    from hexapod.gait.posture import settle_to_stance

    robot = Hexapod.from_config(config_path)
    try:
        if not robot.sync_state_from_hardware():
            console.print(
                "[red]Kein Puls auf den Servokanälen — der Roboter muss stehen "
                "und bestromt sein.[/red]"
            )
            raise typer.Exit(code=1)

        sensors = robot.foot_sensors
        if sensors is None or len(sensors.legs) < 4:
            console.print(
                "[red]Zu wenige kalibrierte Fußsensoren.[/red] Mindestens vier "
                "werden gebraucht, um Schwerpunkt und Bein-Einzelfehler zu trennen.\n"
                "Vorher: [bold]hexapod foot-calibrate[/bold]"
            )
            raise typer.Exit(code=1)

        positions = robot.neutral_foot_xy
        # Reihenfolge: CLI schlaegt Konfiguration schlaegt Schaetzung.
        aus_config = robot.config.foot_sensors.level_per_mm
        aktuell = level_per_mm or aus_config or DEFAULT_LEVEL_PER_MM
        gelernt = level_per_mm is not None or aus_config is not None
        if gelernt:
            quelle = "CLI" if level_per_mm is not None else "robot.yaml"
            console.print(
                f"[dim]Vollweg der Schubstange: {aktuell:.2f} mm ({quelle})[/dim]"
            )
        else:
            console.print(
                "[yellow]Kein level_per_mm in foot_sensors hinterlegt — es wird "
                "geschaetzt, was deutlich unzuverlaessiger ist.[/yellow]"
            )
        letzte_neigung: float | None = None

        console.print(
            "\n[bold]Automatischer Bein-Höhenabgleich[/bold]\n"
            "[dim]Der Roboter hebt in jeder Runde jedes Bein einzeln an und setzt es "
            "wieder ab.\nBleib in Reichweite und schalte im Zweifel den Servostrom ab."
            "[/dim]\n"
        )
        console.print(
            f"[dim]Totband {deadband * 100:.1f} % — darunter wird nicht mehr "
            f"korrigiert.[/dim]"
        )
        if not typer.confirm("Roboter steht auf allen sechs Beinen. Starten?", default=False):
            return

        vorrunde: dict[str, float] | None = None
        lauf_id = datetime.now().strftime("%Y%m%d-%H%M%S")

        for runde in range(1, rounds + 1):
            # force=True ist hier entscheidend: ohne das ueberspringt
            # settle_to_stance jedes Bein, das schon in der Standpose steht --
            # und die Korrekturen liegen fast immer unter dessen 0,5-mm-
            # Toleranz. Die Schleife wuerde dann z_trim weiterrechnen, ohne
            # dass der Roboter die Werte je einnimmt.
            settle_to_stance(robot, force=True)
            time.sleep(0.4)

            levels = _measure(sensors)
            tilt = _read_tilt(robot) if use_imu else None
            result = fit_load_plane(levels, positions)

            korrektur = z_trim_corrections(
                result.residuals, level_per_mm=aktuell,
                damping=damping, deadband=deadband,
            )
            if tilt is not None:
                for leg, d in tilt_corrections(
                    math.radians(tilt[0]), math.radians(tilt[1]), positions
                ).items():
                    korrektur[leg] = korrektur.get(leg, 0.0) + d * damping

            _report(runde, result, tilt, korrektur, aktuell, vorrunde)
            _log(lauf_id, runde, result, tilt, korrektur, aktuell, robot)
            vorrunde = dict(levels)

            # Fertig, wenn nichts mehr ueber der Messgrenze liegt. Ohne dieses
            # Kriterium ruehrt die Schleife weiter im Rauschen und laesst
            # z_trim zufaellig wandern.
            if result.max_abs_residual < deadband:
                console.print(
                    f"[green]Konvergiert: alle Residuen unter dem Totband von "
                    f"{deadband * 100:.1f} %. Weitere Runden wuerden nur Rauschen "
                    f"korrigieren.[/green]"
                )
                break

            # Sicherung: wird die Neigung schlechter statt besser, stimmt
            # vermutlich das Vorzeichen nicht -- dann lieber abbrechen, als
            # den Roboter immer schiefer zu stellen.
            if tilt is not None:
                betrag = abs(tilt[0]) + abs(tilt[1])
                if letzte_neigung is not None and betrag > letzte_neigung * 1.5 + 0.5:
                    console.print(
                        f"[red]Neigung hat sich verschlechtert "
                        f"({letzte_neigung:.2f}° → {betrag:.2f}°).[/red] "
                        "Abbruch — vermutlich stimmt das Vorzeichen der IMU-Achsen nicht.\n"
                        "Nochmal mit [bold]--no-imu[/bold] versuchen."
                    )
                    return
                letzte_neigung = betrag

            vorher = dict(levels)
            angewendet: dict[str, float] = {}
            for leg, d in korrektur.items():
                neu = max(-MAX_Z_TRIM_MM, min(MAX_Z_TRIM_MM, robot.get_z_trim(leg) + d))
                angewendet[leg] = neu - robot.get_z_trim(leg)
                robot.set_z_trim(leg, neu)

            if not gelernt and runde == 1:
                settle_to_stance(robot, force=True)
                time.sleep(0.4)
                nachher = _measure(sensors)
                geschaetzt = estimate_level_per_mm(angewendet, vorher, nachher)
                if geschaetzt is not None and 0.005 < geschaetzt < 0.5:
                    aktuell = geschaetzt
                    gelernt = True
                    console.print(
                        f"[green]Empfindlichkeit aus der Reaktion gemessen: "
                        f"{geschaetzt * 100:.1f} % je mm[/green]"
                    )
                else:
                    console.print(
                        "[yellow]Empfindlichkeit ließ sich nicht bestimmen — die Schritte "
                        "waren zu klein. Rechne weiter mit dem Startwert.[/yellow]"
                    )

        console.rule("Ergebnis")
        table = Table()
        table.add_column("Bein")
        table.add_column("z_trim", justify="right")
        for leg in robot.leg_names:
            table.add_row(leg, f"{robot.get_z_trim(leg):+6.2f} mm")
        console.print(table)

        if typer.confirm(f"In {config_path} speichern?", default=True):
            console.print(f"[green]Gespeichert nach {robot.save_z_trims()}[/green]")
        else:
            console.print("Nicht gespeichert.")
    finally:
        robot.close(disable=False)
