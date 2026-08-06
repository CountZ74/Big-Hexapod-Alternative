"""Einfrieren pro Bein im Executor.

Im Tripod schwingen drei Beine gleichzeitig. Findet eines frueher Boden,
soll genau dieses stehenbleiben -- die anderen beiden muessen ihre Bahn zu
Ende fahren. Ein globaler Abbruch waere hier falsch: er wuerde die ganze
Gruppe mitten in der Bewegung anhalten.
"""

from __future__ import annotations

import pytest

from hexapod.config.loader import load_robot_config
from hexapod.gait.executor import run_multi_leg_trajectory, run_single_leg_trajectory
from hexapod.robot import Hexapod

CONFIG_PATH = "config/robot.yaml"
GRUPPE = ["front_right", "mid_left", "back_right"]


@pytest.fixture
def sim_hexapod() -> Hexapod:
    config = load_robot_config(CONFIG_PATH)
    data = config.model_dump()
    data["buses"] = {
        n: {"type": "simulator", "num_channels": b["num_channels"]}
        for n, b in data["buses"].items()
    }
    robot = Hexapod(config.model_validate(data))
    robot.stance()
    return robot


def _bahn(n: int = 12) -> list[tuple[float, float, float]]:
    """Gerade nach unten, von +12 mm bis 0."""
    return [(0.0, 0.0, 12.0 - 12.0 * i / (n - 1)) for i in range(n)]


def test_ohne_freeze_erreichen_alle_das_ende(sim_hexapod: Hexapod) -> None:
    gehalten = run_multi_leg_trajectory(
        sim_hexapod, {leg: _bahn() for leg in GRUPPE}, rate_hz=500.0
    )
    assert gehalten == {}
    for leg in GRUPPE:
        assert sim_hexapod.current_offset(leg)[2] == pytest.approx(0.0, abs=0.01)


def test_ein_bein_friert_ein_die_anderen_laufen_weiter(sim_hexapod: Hexapod) -> None:
    """Der Kernfall: nur mid_left meldet Kontakt, und zwar nach einigen Takten.

    Das Kriterium haengt bewusst am Takt und nicht an current_offset: dessen
    Wert stammt vom ZULETZT gesendeten Frame und ist beim ersten Takt noch
    die Lage vor der Bahn. Im Echtbetrieb ist das unkritisch, weil dort der
    Sensorwert entscheidet -- im Test wuerde es nur verschleiern, was geprueft
    werden soll.
    """
    takt = [0]

    def halt(leg: str) -> bool:
        if leg == "mid_left":
            takt[0] += 1
            return takt[0] > 4
        return False

    gehalten = run_multi_leg_trajectory(
        sim_hexapod, {leg: _bahn() for leg in GRUPPE}, rate_hz=500.0, freeze=halt
    )
    assert set(gehalten) == {"mid_left"}
    # Eingefroren irgendwo zwischen Start und Ziel ...
    z = sim_hexapod.current_offset("mid_left")[2]
    assert 0.5 < z < 12.0, z
    # ... die anderen beiden sind aber unten angekommen.
    for leg in ("front_right", "back_right"):
        assert sim_hexapod.current_offset(leg)[2] == pytest.approx(0.0, abs=0.01)


def test_eingefrorenes_bein_bewegt_sich_nicht_mehr(sim_hexapod: Hexapod) -> None:
    """Nach dem Einfrieren darf kein weiterer Takt es tiefer druecken."""
    takt = [0]
    gesendet: list[float] = []

    def halt(leg: str) -> bool:
        if leg != "mid_left":
            return False
        takt[0] += 1
        gesendet.append(sim_hexapod.current_offset("mid_left")[2])
        return takt[0] > 4

    run_multi_leg_trajectory(
        sim_hexapod, {leg: _bahn(20) for leg in GRUPPE}, rate_hz=500.0, freeze=halt
    )
    # Nach dem Ausloesen wird nicht mehr gefragt -- also selbst pruefen, dass
    # die Endlage der Halteposition entspricht und nicht dem Bahnende.
    assert sim_hexapod.current_offset("mid_left")[2] == pytest.approx(
        gesendet[-1], abs=0.01
    )
    assert sim_hexapod.current_offset("mid_left")[2] > 0.5


def test_nach_dem_einfrieren_wird_nicht_mehr_gefragt(sim_hexapod: Hexapod) -> None:
    """Sonst kostet die Sensorabfrage in jedem Takt unnoetig Zeit."""
    fragen = {leg: 0 for leg in GRUPPE}

    def halt(leg: str) -> bool:
        fragen[leg] += 1
        return leg == "mid_left"

    run_multi_leg_trajectory(
        sim_hexapod, {leg: _bahn(30) for leg in GRUPPE}, rate_hz=500.0, freeze=halt
    )
    assert fragen["mid_left"] == 1
    assert fragen["front_right"] > 5


def test_alle_eingefroren_beendet_die_bahn(sim_hexapod: Hexapod) -> None:
    """Sind alle Beine gehalten, gibt es nichts mehr zu fahren."""
    takte = [0]

    def halt(leg: str) -> bool:
        takte[0] += 1
        return True

    gehalten = run_multi_leg_trajectory(
        sim_hexapod, {leg: _bahn(50) for leg in GRUPPE}, rate_hz=500.0, freeze=halt
    )
    assert set(gehalten) == set(GRUPPE)
    # Nach dem ersten Takt ist Schluss -- nicht 50 Takte lang weitergefragt.
    assert takte[0] == len(GRUPPE)


def test_einzelbein_gibt_halteposition_zurueck(sim_hexapod: Hexapod) -> None:
    takt = [0]

    def halt(leg: str) -> bool:
        takt[0] += 1
        return takt[0] > 3

    gehalten = run_single_leg_trajectory(
        sim_hexapod, "front_left", _bahn(), rate_hz=500.0, freeze=halt
    )
    assert "front_left" in gehalten
    assert gehalten["front_left"][2] > 0.5
