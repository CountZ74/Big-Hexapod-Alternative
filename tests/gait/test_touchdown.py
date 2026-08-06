"""Aufsetz-Erkennung: das Bein hoert auf zu sinken, wenn es Boden findet.

Ohne die Erkennung faehrt jedes Bein blind auf Standpose-Hoehe. Steht dort
etwas im Weg -- eine Bodenwelle, ein Kabel, eine Stufe --, drueckt es weiter
und hebt dabei den Koerper an oder verspannt den Roboter. Mit Erkennung
bleibt es stehen, wo der Boden tatsaechlich ist.
"""

from __future__ import annotations

import pytest

from hexapod.config.loader import load_robot_config
from hexapod.drivers.simulator import SimulatorDriver
from hexapod.gait.posture import settle_to_stance
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
    return Hexapod(config.model_validate(data))


def _alle_unbelastet(robot: Hexapod) -> None:
    """Alle Sensoren auf 0 setzen.

    Wichtig, weil ein Simulatorkanal per Default 0 liefert -- und bei einer
    invertierten Kennlinie (front_left: raw_unloaded 482, raw_full 184)
    bedeutet ein Rohwert von 0 gerade VOLLEN Kontakt. Derselbe Effekt
    traefe an der Hardware einen abgezogenen Stecker.
    """
    for sensor in robot.config.foot_sensors.active:
        _sensor_auf(robot, sensor.leg, 0.0)


def _sensor_auf(robot: Hexapod, leg: str, level: float) -> None:
    """Den simulierten Analogeingang so setzen, dass er `level` meldet."""
    sensor = robot.config.foot_sensors.get(leg)
    calib = sensor.calibration
    assert calib is not None, f"{leg} ist nicht kalibriert"
    roh = calib.raw_unloaded + level * calib.span
    driver = robot.bus_driver(sensor.bus)
    assert isinstance(driver, SimulatorDriver)
    driver.set_analog(sensor.channel, round(roh))


def _boden_modell(robot: Hexapod, hoehen: dict[str, float],
                  einfederweg_mm: float = 1.5) -> None:
    """Ersetzt die Sensorlesung durch einen simulierten Boden.

    Ein fester Analogwert waere unphysikalisch: ein Bein in der Luft meldet
    keine Last. Hier waechst der Federweg erst, wenn der Fuss unter die
    Bodenhoehe faehrt -- ueber `einfederweg_mm` linear von 0 auf 1.

    hoehen: Bein -> Bodenhoehe als Offset zur Standpose. 0.0 heisst "Boden
    genau auf Standpose-Hoehe", positive Werte sind Hindernisse.
    """
    from hexapod.drivers.foot_sensor import FootSensorReading

    array = robot.foot_sensors
    assert array is not None

    def read(leg: str, *, samples: int | None = None) -> FootSensorReading:
        z = robot.current_offset(leg)[2]
        tiefe = hoehen.get(leg, 0.0) - z
        level = max(0.0, min(1.0, tiefe / einfederweg_mm))
        return FootSensorReading(leg=leg, channel=0, raw=0.0, level=level)

    array.read = read  # type: ignore[method-assign]


def test_ohne_schwelle_faehrt_alles_in_die_standpose(sim_hexapod: Hexapod) -> None:
    """Default-Verhalten bleibt unveraendert -- auch wenn Last anliegt."""
    sim_hexapod.stance()
    for leg in sim_hexapod.leg_names:
        _sensor_auf(sim_hexapod, leg, 0.9)     # alle melden kraeftig Kontakt
    frueh = settle_to_stance(sim_hexapod, force=True, rate_hz=500.0, pause=0.0)
    assert frueh == {}
    for leg in sim_hexapod.leg_names:
        assert abs(sim_hexapod.current_offset(leg)[2]) < 0.01, leg


def test_kontakt_stoppt_das_absenken(sim_hexapod: Hexapod) -> None:
    sim_hexapod.stance()
    _alle_unbelastet(sim_hexapod)
    # back_left steht auf einem Klotz, die anderen auf normalem Boden.
    # Ein fester Sensorwert waere hier falsch: das Bein wird erst angehoben,
    # und ein angehobener Fuss meldet keine Last. Die Erkennung wird
    # absichtlich erst scharf, nachdem der Fuss einmal frei war -- sonst
    # friert sie das Bein schon beim Abheben ein.
    hoehen = dict.fromkeys(sim_hexapod.leg_names, -50.0)
    hoehen["back_left"] = 8.0
    _boden_modell(sim_hexapod, hoehen)
    frueh = settle_to_stance(
        sim_hexapod, force=True, rate_hz=500.0, pause=0.0, touch_level=0.05,
        touch_margin_mm=0.0
    )
    assert "back_left" in frueh
    # Es blieb ueber der Standpose stehen, nicht darunter
    assert frueh["back_left"] > 0.5
    assert sim_hexapod.current_offset("back_left")[2] == pytest.approx(
        frueh["back_left"]
    )


def test_unbelastete_beine_erreichen_die_standpose(sim_hexapod: Hexapod) -> None:
    sim_hexapod.stance()
    _alle_unbelastet(sim_hexapod)
    _sensor_auf(sim_hexapod, "back_left", 0.9)
    frueh = settle_to_stance(
        sim_hexapod, force=True, rate_hz=500.0, pause=0.0, touch_level=0.05,
        touch_margin_mm=0.0
    )
    for leg in sim_hexapod.leg_names:
        if leg == "back_left":
            continue
        assert leg not in frueh
        assert abs(sim_hexapod.current_offset(leg)[2]) < 0.01, leg


def test_schwelle_ueber_dem_messwert_loest_nicht_aus(sim_hexapod: Hexapod) -> None:
    """Rauschen darf die Bewegung nicht abbrechen."""
    sim_hexapod.stance()
    for leg in sim_hexapod.leg_names:
        _sensor_auf(sim_hexapod, leg, 0.02)    # unterhalb der Schwelle
    frueh = settle_to_stance(
        sim_hexapod, force=True, rate_hz=500.0, pause=0.0, touch_level=0.05,
        touch_margin_mm=0.0
    )
    assert frueh == {}


def test_bein_ohne_sensor_faehrt_normal(sim_hexapod: Hexapod) -> None:
    """Ein Bein ohne Sensor darf die Erkennung nicht blockieren."""
    cfg = sim_hexapod.config
    ohne = [s for s in cfg.foot_sensors.sensors if s.leg == "mid_right"]
    assert ohne, "Testannahme: mid_right hat einen Sensor"
    ohne[0].enabled = False
    robot = Hexapod(cfg)
    robot.stance()
    _alle_unbelastet(robot)
    frueh = settle_to_stance(robot, force=True, rate_hz=500.0, pause=0.0, touch_level=0.05,
        touch_margin_mm=0.0)
    assert abs(robot.current_offset("mid_right")[2]) < 0.01
    assert "mid_right" not in frueh


# ---------------------------------------------------------------------
# Mindesthoehe: Kontakt zaehlt nur, wenn er FRUEHER kommt als erwartet
# ---------------------------------------------------------------------


def test_normales_aufsetzen_laeuft_trotz_kontakt_durch(sim_hexapod: Hexapod) -> None:
    """Der Fall, der am Roboter aufgefallen ist.

    Auf ebenem Boden meldet jedes Bein beim Absetzen Kontakt -- es soll sich
    aber trotzdem sauber in die Standpose einfedern. Ohne Mindesthoehe stoppt
    sonst JEDES Bein zu frueh, der Koerper sinkt nie ganz ab, und der Roboter
    steht auf sechs kaum eingefederten Beinen. Genau das war am echten
    Roboter zu sehen: alle sechs blieben 1 bis 4 mm ueber der Standpose.
    """
    sim_hexapod.stance()
    _boden_modell(sim_hexapod, dict.fromkeys(sim_hexapod.leg_names, 0.0))
    frueh = settle_to_stance(
        sim_hexapod, force=True, rate_hz=500.0, pause=0.0,
        touch_level=0.05, touch_margin_mm=5.0,
    )
    assert frueh == {}
    for leg in sim_hexapod.leg_names:
        assert abs(sim_hexapod.current_offset(leg)[2]) < 0.01, leg


def test_hindernis_oberhalb_der_mindesthoehe_stoppt(sim_hexapod: Hexapod) -> None:
    """Ein 10 mm hohes Hindernis unter einem Fuss muss die Bahn anhalten."""
    sim_hexapod.stance()
    hoehen = dict.fromkeys(sim_hexapod.leg_names, 0.0)
    hoehen["mid_left"] = 10.0
    _boden_modell(sim_hexapod, hoehen)
    frueh = settle_to_stance(
        sim_hexapod, force=True, rate_hz=500.0, pause=0.0,
        touch_level=0.05, touch_margin_mm=5.0,
    )
    assert "mid_left" in frueh
    # Es bleibt ungefaehr auf Hindernishoehe stehen, nicht darunter
    assert 8.0 < frueh["mid_left"] < 11.0, frueh["mid_left"]
    for leg in sim_hexapod.leg_names:
        if leg != "mid_left":
            assert abs(sim_hexapod.current_offset(leg)[2]) < 0.01, leg


def test_flaches_hindernis_unter_der_mindesthoehe_wird_durchgedrueckt(
    sim_hexapod: Hexapod,
) -> None:
    """Eine 2-mm-Unebenheit ist kein Hindernis, sondern normale Toleranz."""
    sim_hexapod.stance()
    hoehen = dict.fromkeys(sim_hexapod.leg_names, 0.0)
    hoehen["back_right"] = 2.0
    _boden_modell(sim_hexapod, hoehen)
    frueh = settle_to_stance(
        sim_hexapod, force=True, rate_hz=500.0, pause=0.0,
        touch_level=0.05, touch_margin_mm=5.0,
    )
    assert frueh == {}
