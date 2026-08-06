"""Aufsetz-Erkennung im Tripod-Gang.

Anders als beim Absetzen einzelner Beine schwingen hier drei gleichzeitig.
Findet eines frueher Boden, muss genau dieses halten -- die anderen beiden
duerfen sich davon nicht stoeren lassen, sonst bricht der Gang zusammen.
"""

from __future__ import annotations

import pytest

from hexapod.config.loader import load_robot_config
from hexapod.drivers.foot_sensor import FootSensorReading
from hexapod.gait.tripod import GROUP_A, GROUP_B, walk
from hexapod.robot import Hexapod

CONFIG_PATH = "config/robot.yaml"


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


def _boden(robot: Hexapod, hoehen: dict[str, float],
           einfederweg_mm: float = 1.5) -> None:
    """Simulierter Boden: Last entsteht erst beim Eindringen.

    hoehen: Bein -> Bodenhoehe als Offset zur Standpose. Positive Werte sind
    Hindernisse, 0.0 ist ebener Boden auf Standpose-Hoehe.
    """
    array = robot.foot_sensors
    assert array is not None

    def read(leg: str, *, samples: int | None = None) -> FootSensorReading:
        z = robot.current_offset(leg)[2]
        tiefe = hoehen.get(leg, 0.0) - z
        level = max(0.0, min(1.0, tiefe / einfederweg_mm))
        return FootSensorReading(leg=leg, channel=0, raw=0.0, level=level)

    array.read = read  # type: ignore[method-assign]


def test_ohne_erkennung_unveraendert(sim_hexapod: Hexapod) -> None:
    treffer = walk(sim_hexapod, cycles=1, rate_hz=500.0, steps=8)
    assert treffer == {}


def test_ebener_boden_loest_nicht_aus(sim_hexapod: Hexapod) -> None:
    """Beim normalen Aufsetzen meldet jedes Bein Kontakt -- das ist kein Grund
    anzuhalten, sonst sackt der Roboter nie in die Standpose."""
    _boden(sim_hexapod, dict.fromkeys(sim_hexapod.leg_names, 0.0))
    treffer = walk(sim_hexapod, cycles=1, rate_hz=500.0, steps=8, touch_level=0.05)
    assert treffer == {}


def test_hindernis_haelt_nur_das_betroffene_bein(sim_hexapod: Hexapod) -> None:
    hoehen = dict.fromkeys(sim_hexapod.leg_names, 0.0)
    hoehen["mid_left"] = 12.0          # mid_left gehoert zu GROUP_A
    _boden(sim_hexapod, hoehen)

    treffer = walk(sim_hexapod, cycles=1, rate_hz=500.0, steps=10, touch_level=0.05)

    assert "mid_left" in treffer
    assert treffer["mid_left"] > 5.0
    # Die beiden anderen der Gruppe sind unbehelligt geblieben
    for leg in GROUP_A:
        if leg != "mid_left":
            assert leg not in treffer


def test_gemessene_hoehe_passt_zum_hindernis(sim_hexapod: Hexapod) -> None:
    """Die zurueckgegebene Hoehe ist die Gelaendehoehe an dieser Stelle."""
    hoehen = dict.fromkeys(sim_hexapod.leg_names, 0.0)
    hoehen["front_left"] = 10.0        # front_left gehoert zu GROUP_B
    _boden(sim_hexapod, hoehen)

    treffer = walk(sim_hexapod, cycles=1, rate_hz=500.0, steps=12, touch_level=0.05)

    assert "front_left" in treffer
    assert 7.0 < treffer["front_left"] < 13.0, treffer["front_left"]


def test_beide_gruppen_werden_geprueft(sim_hexapod: Hexapod) -> None:
    hoehen = dict.fromkeys(sim_hexapod.leg_names, 0.0)
    hoehen[GROUP_A[0]] = 11.0
    hoehen[GROUP_B[0]] = 11.0
    _boden(sim_hexapod, hoehen)

    treffer = walk(sim_hexapod, cycles=1, rate_hz=500.0, steps=10, touch_level=0.05)

    assert GROUP_A[0] in treffer and GROUP_B[0] in treffer


def test_gang_laeuft_nach_dem_hindernis_weiter(sim_hexapod: Hexapod) -> None:
    """Ein eingefrorenes Bein darf den Zyklus nicht abbrechen."""
    hoehen = dict.fromkeys(sim_hexapod.leg_names, 0.0)
    hoehen["mid_left"] = 12.0
    _boden(sim_hexapod, hoehen)

    walk(sim_hexapod, cycles=2, rate_hz=500.0, steps=8, touch_level=0.05)

    # Alle uebrigen Beine stehen am Ende wieder sauber in der Standpose
    for leg in sim_hexapod.leg_names:
        if leg == "mid_left":
            continue
        assert abs(sim_hexapod.current_offset(leg)[2]) < 0.5, leg
