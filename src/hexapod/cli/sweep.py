"""Kennlinie eines Fußsensors unter echter Last aufnehmen.

Ein Bein wird schrittweise weiter nach unten gefahren und hebt dabei seine
Ecke des Roboters an. Bei jedem Schritt wird der Federweg gemessen. Das
Ergebnis ist die Kennlinie des Sensors über der Auslenkung — und damit die
Antwort auf die Frage, die zwei Messungen allein nicht klären konnten:

  * Läuft der Wert gegen 100 % und bleibt dort stehen, obwohl das Bein
    weiter drückt, ist die Schubstange mechanisch am Anschlag. Dann ist der
    Messbereich erschöpft und für höhere Lasten braucht es härtere Federn
    oder mehr Weg.

  * Steigt er dagegen weiter, nur immer flacher, ist nichts angeschlagen —
    dann ist es die Kennlinie des Magnetfelds, das mit dem Abstand steil
    abfällt. Der Sensor misst dann durchaus noch, nur mit schlechter
    Auflösung im oberen Bereich.

Der Unterschied ist wichtig: das eine ist ein Hardware-Limit, das andere
eine Frage der Umrechnung.

BEWEGT SERVOS. Der Roboter steht dabei zeitweise schief, weil eine Ecke
angehoben wird.
"""

from __future__ import annotations

import json
import statistics
import time
from datetime import datetime
from itertools import pairwise
from pathlib import Path

import typer
from rich.console import Console
from rich.table import Table

from hexapod.robot.hexapod import Hexapod

console = Console()

LOG_PATH = Path("logs/foot_sweep.jsonl")
MESSUNGEN = 12          # Einzelmessungen je Stuetzstelle
RATE_HZ = 60.0
SCHRITT_TAKTE = 8       # Takte je Millimeter-Schritt (sanftes Fahren)


def _messen(robot: Hexapod, leg: str) -> dict[str, float]:
    """Federweg aller Beine, Median. Die anderen interessieren als Gegenprobe."""
    sensors = robot.foot_sensors
    assert sensors is not None
    serien: dict[str, list[float]] = {b: [] for b in sensors.legs}
    for _ in range(MESSUNGEN):
        for b, r in sensors.read_all(samples=1).items():
            if r.level is not None:
                serien[b].append(r.level)
        time.sleep(0.02)
    return {b: statistics.median(v) for b, v in serien.items() if v}


def _fahre_auf(robot: Hexapod, leg: str, dz: float) -> None:
    """Das Bein sanft auf den Offset dz bringen (negativ = tiefer)."""
    start = robot.current_offset(leg)[2]
    for i in range(1, SCHRITT_TAKTE + 1):
        z = start + (dz - start) * i / SCHRITT_TAKTE
        robot.set_foot_offset(leg, 0.0, 0.0, z, clip=True)
        time.sleep(1.0 / RATE_HZ)


def run_foot_sweep(
    config_path: Path,
    *,
    leg: str,
    max_mm: float = 12.0,
    schritt_mm: float = 1.0,
) -> None:
    robot = Hexapod.from_config(config_path)
    try:
        if not robot.sync_state_from_hardware():
            console.print("[red]Kein Puls — der Roboter muss stehen und bestromt sein.[/red]")
            raise typer.Exit(code=1)
        sensors = robot.foot_sensors
        if sensors is None or not sensors.has_sensor(leg):
            console.print(f"[red]{leg} hat keinen kalibrierten Fußsensor.[/red]")
            raise typer.Exit(code=1)

        console.print(
            f"\n[bold]Kennlinie {leg}[/bold]\n"
            f"[dim]Das Bein drückt sich in {schritt_mm:.1f}-mm-Schritten bis "
            f"{max_mm:.0f} mm nach unten und hebt dabei seine Ecke an.\n"
            f"Bleib in Reichweite — bei ungewöhnlichen Geräuschen Servostrom "
            f"abschalten.[/dim]\n"
        )
        if not typer.confirm("Starten?", default=False):
            return

        lauf = datetime.now().strftime("%Y%m%d-%H%M%S")
        punkte: list[tuple[float, dict[str, float]]] = []
        start_offset = robot.current_offset(leg)[2]

        tiefe = 0.0
        while tiefe <= max_mm + 1e-9:
            _fahre_auf(robot, leg, start_offset - tiefe)
            time.sleep(0.25)
            werte = _messen(robot, leg)
            punkte.append((tiefe, werte))
            if werte.get(leg, 0.0) >= 0.995:
                console.print("[dim]Vollausschlag erreicht — Abbruch.[/dim]")
                break
            tiefe += schritt_mm

        _fahre_auf(robot, leg, start_offset)

        table = Table(title=f"Federweg von {leg} über der Auslenkung")
        table.add_column("Auslenkung", justify="right")
        table.add_column("Federweg", justify="right")
        table.add_column("Zuwachs", justify="right")
        table.add_column("übrige Beine", justify="right")
        vorher: float | None = None
        for tiefe, werte in punkte:
            v = werte.get(leg)
            zuwachs = "—" if vorher is None or v is None else f"{(v - vorher) * 100:+5.1f} %"
            rest = sum(x for b, x in werte.items() if b != leg)
            table.add_row(
                f"{tiefe:5.1f} mm",
                f"{v * 100:5.1f} %" if v is not None else "—",
                zuwachs,
                f"{rest * 100:5.0f} %",
            )
            vorher = v
        console.print(table)

        # Auswertung: wird der Zuwachs am Ende null, ist die Stange am Anschlag.
        letzte = [b - a for (_, wa), (_, wb) in pairwise(punkte)
                  if (a := wa.get(leg)) is not None and (b := wb.get(leg)) is not None]
        if len(letzte) >= 3:
            spaet = letzte[-3:]
            mittel = sum(spaet) / len(spaet)
            console.print(
                f"\nZuwachs über die letzten drei Schritte: "
                f"[bold]{mittel * 100:+.2f} % je {schritt_mm:.1f} mm[/bold]"
            )
            if mittel * 100 < 0.5:
                console.print(
                    "[yellow]Praktisch null — die Schubstange steht am mechanischen "
                    "Anschlag. Der Messbereich ist bei dieser Last erschöpft; für "
                    "mehr braucht es härtere Federn oder mehr Weg.[/yellow]"
                )
            else:
                console.print(
                    "[green]Der Wert steigt weiter. Nichts angeschlagen — die "
                    "Abflachung kommt von der Kennlinie des Magnetfelds. Der "
                    "Sensor misst noch, nur mit schlechterer Auflösung.[/green]"
                )

        try:
            LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
            with LOG_PATH.open("a", encoding="utf-8") as fh:
                for tiefe, werte in punkte:
                    fh.write(json.dumps({
                        "lauf": lauf, "bein": leg,
                        "zeit": datetime.now().isoformat(timespec="seconds"),
                        "auslenkung_mm": round(tiefe, 2),
                        "levels": {k: round(v, 5) for k, v in werte.items()},
                    }, ensure_ascii=False) + "\n")
            console.print(f"[dim]Protokoll: {LOG_PATH}[/dim]")
        except OSError as e:
            console.print(f"[yellow]Protokoll nicht geschrieben: {e}[/yellow]")
    finally:
        robot.close(disable=False)
