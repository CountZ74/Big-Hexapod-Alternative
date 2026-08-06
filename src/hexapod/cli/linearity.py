"""Prüft, ob der Fußsensor linear misst — Sechs- gegen Dreibeinstand.

Die gesamte Auswertung im Bein-Höhenabgleich setzt voraus, dass die
Einfederung proportional zur Last ist. Nur dann gilt

    federweg_i = z0 + a*x_i + b*y_i + e_i

und nur dann trennt der Ebenen-Fit Schwerpunkt und Bein-Einzelfehler sauber.
Diese Annahme ist bisher nie geprüft worden.

Der Roboter liefert die Prüfung selbst: Hebt man eine Tripod-Gruppe an,
tragen drei Beine statt sechs — also jedes ungefähr das Doppelte. Ist die
Feder linear, verdoppelt sich der Federweg. Zwei sauber definierte Lastpunkte
mit bekanntem Verhältnis, ganz ohne Prüfgewichte.

Abweichungen sind ihrerseits aussagekräftig:
  Verhältnis < 2   Feder wird mit zunehmendem Weg steifer (progressiv)
  Verhältnis > 2   Feder wird weicher, oder der Sensor läuft aus dem
                   linearen Bereich des Hall-Elements
  stark streuend   einzelne Beine mechanisch unterschiedlich

BEWEGT SERVOS. Der Roboter steht dabei zeitweise auf drei Beinen — das ist
derselbe Zustand wie mitten im Tripod-Gang, aber er sollte auf ebenem Boden
und in Reichweite stehen.
"""

from __future__ import annotations

import statistics
import time
from pathlib import Path

import typer
from rich.console import Console
from rich.table import Table

from hexapod.drivers.foot_sensor import FootSensorArray
from hexapod.gait.tripod import GROUP_A, GROUP_B
from hexapod.robot.hexapod import Hexapod

console = Console()

MESSDAUER_S = 3.0
MESSINTERVALL = 0.05
# Hubhoehe der angehobenen Gruppe. Gross genug, dass die Fuesse sicher frei
# sind -- ihr Federweg muss auf null gehen, sonst tragen sie noch mit.
LIFT_MM = 30.0
LIFT_SCHRITTE = 40
LIFT_RATE_HZ = 60.0


def _messen(sensors: FootSensorArray, dauer: float = MESSDAUER_S) -> dict[str, float]:
    serien: dict[str, list[float]] = {leg: [] for leg in sensors.legs}
    ende = time.time() + dauer
    while time.time() < ende:
        for leg, r in sensors.read_all().items():
            if r.level is not None:
                serien[leg].append(r.level)
        time.sleep(MESSINTERVALL)
    return {leg: statistics.median(v) for leg, v in serien.items() if v}


def _gruppe_bewegen(robot: Hexapod, gruppe: tuple[str, ...], hoehe: float) -> None:
    """Die drei Beine der Gruppe sanft auf `hoehe` bringen (0 = abgesetzt)."""
    start = robot.current_offset(gruppe[0])[2]
    for i in range(1, LIFT_SCHRITTE + 1):
        z = start + (hoehe - start) * i / LIFT_SCHRITTE
        robot.set_all_foot_offsets({leg: (0.0, 0.0, z) for leg in gruppe}, clip=True)
        time.sleep(1.0 / LIFT_RATE_HZ)


def run_foot_linearity(
    config_path: Path,
    *,
    lift_mm: float = LIFT_MM,
) -> None:
    from hexapod.gait.posture import settle_to_stance

    robot = Hexapod.from_config(config_path)
    try:
        if not robot.sync_state_from_hardware():
            console.print("[red]Kein Puls — der Roboter muss stehen und bestromt sein.[/red]")
            raise typer.Exit(code=1)
        sensors = robot.foot_sensors
        if sensors is None or len(sensors.legs) < 6:
            console.print("[red]Es braucht sechs kalibrierte Fußsensoren.[/red]")
            raise typer.Exit(code=1)

        console.print(
            "\n[bold]Linearitätsprüfung der Fußsensoren[/bold]\n"
            "[dim]Der Roboter hebt nacheinander beide Tripod-Gruppen an und steht\n"
            "dabei jeweils auf drei Beinen. Jedes tragende Bein sollte dann etwa\n"
            "den doppelten Federweg zeigen wie im Sechsbeinstand.[/dim]\n"
        )
        if not typer.confirm("Steht auf ebenem Boden. Starten?", default=False):
            return

        settle_to_stance(robot, force=True)
        time.sleep(0.5)
        sechs = _messen(sensors)

        drei: dict[str, float] = {}
        frei: dict[str, float] = {}
        for gruppe in (GROUP_A, GROUP_B):
            traeger = [leg for leg in sensors.legs if leg not in gruppe]
            console.print(f"[dim]Hebe {', '.join(gruppe)} …[/dim]")
            _gruppe_bewegen(robot, gruppe, lift_mm)
            time.sleep(0.5)
            gemessen = _messen(sensors)
            drei.update({leg: gemessen[leg] for leg in traeger if leg in gemessen})
            frei.update({leg: gemessen[leg] for leg in gruppe if leg in gemessen})
            _gruppe_bewegen(robot, gruppe, 0.0)
            time.sleep(0.5)

        settle_to_stance(robot, force=True)

        table = Table(title="Federweg bei halber und ganzer Beinzahl")
        table.add_column("Bein")
        table.add_column("6 Beine", justify="right")
        table.add_column("3 Beine", justify="right")
        table.add_column("Verhältnis", justify="right")
        table.add_column("angehoben", justify="right")
        verhaeltnisse: list[float] = []
        for leg in sensors.legs:
            s6, s3 = sechs.get(leg), drei.get(leg)
            if s6 and s3 and s6 > 0.02:
                v = s3 / s6
                verhaeltnisse.append(v)
                v_txt = f"{v:.2f}"
            else:
                v_txt = "—"
            table.add_row(
                leg,
                f"{s6 * 100:.1f} %" if s6 is not None else "—",
                f"{s3 * 100:.1f} %" if s3 is not None else "—",
                v_txt,
                f"{frei[leg] * 100:.1f} %" if leg in frei else "—",
            )
        console.print(table)

        if not verhaeltnisse:
            console.print("[red]Keine auswertbaren Beine — zu wenig Last im Stand?[/red]")
            return

        med = statistics.median(verhaeltnisse)
        console.print(
            f"\n[bold]Median-Verhältnis {med:.2f}[/bold]  "
            f"(Streuung {min(verhaeltnisse):.2f} … {max(verhaeltnisse):.2f}, "
            f"erwartet 2,00 bei linearer Feder)"
        )
        if 1.8 <= med <= 2.2:
            console.print(
                "[green]Im erwarteten Bereich — die lineare Annahme des "
                "Höhenabgleichs traegt.[/green]"
            )
        elif med < 1.8:
            console.print(
                "[yellow]Deutlich unter 2: die Feder wird mit zunehmendem Weg "
                "steifer. Der Höhenabgleich unterschätzt dann grosse Fehler und "
                "braucht mehr Runden, bleibt aber richtig in der Richtung.[/yellow]"
            )
        else:
            console.print(
                "[yellow]Deutlich über 2: die Feder wird weicher, oder der "
                "Hall-Sensor verlaesst seinen linearen Bereich. Bei grossen "
                "Korrekturen ist mit Ueberschwingen zu rechnen.[/yellow]"
            )

        rest = [v for v in frei.values() if v > 0.03]
        if rest:
            console.print(
                f"[yellow]Achtung: {len(rest)} angehobene Beine zeigen noch "
                f"Federweg — sie waren nicht wirklich frei. Hub erhoehen "
                f"(--lift-mm).[/yellow]"
            )
    finally:
        robot.close(disable=False)
