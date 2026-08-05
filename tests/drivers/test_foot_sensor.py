"""Tests für den Fußsensor-Treiber (Federweg-Auswertung, Maestro-Protokoll)."""

from __future__ import annotations

import pytest

from hexapod.config.model import FootSensorsConfig
from hexapod.drivers.foot_sensor import (
    FootSensorArray,
    describe,
    endpoints_from_samples,
)
from hexapod.drivers.maestro import CMD_GET_POSITION, MaestroDriver
from hexapod.drivers.simulator import SimulatorDriver

from .fake_serial import FakeSerial

# Vollweg: unbelastet 400, Anschlag 700 -> 300 Zähler Messbereich.
CALIB = {"raw_unloaded": 400.0, "raw_full": 700.0}


def _config(**overrides: object) -> FootSensorsConfig:
    sensor: dict[str, object] = {
        "leg": "front_left",
        "channel": 0,
        "calibration": dict(CALIB),
    }
    sensor.update(overrides)
    return FootSensorsConfig.model_validate({"samples": 1, "sensors": [sensor]})


def _array(raw: int, **overrides: object) -> tuple[FootSensorArray, SimulatorDriver]:
    driver = SimulatorDriver(num_channels=24)
    driver.set_analog(0, raw)
    return FootSensorArray({"main": driver}, _config(**overrides)), driver


# ---------------------------------------------------------------------
# Grundverhalten
# ---------------------------------------------------------------------


def test_rohwert_wird_durchgereicht() -> None:
    sensors, _ = _array(512)
    assert sensors.read_raw("front_left") == pytest.approx(512.0)


def test_median_filtert_ausreisser() -> None:
    """Drei Messungen, davon eine als Ausreißer — der Median gewinnt."""

    class ZackigerTreiber(SimulatorDriver):
        def __init__(self) -> None:
            super().__init__(num_channels=24)
            self._folge = iter([500, 9, 502])

        def read_analog(self, channel: int) -> int:
            return next(self._folge)

    config = FootSensorsConfig.model_validate(
        {"samples": 3, "sensors": [{"leg": "front_left", "channel": 0}]}
    )
    sensors = FootSensorArray({"main": ZackigerTreiber()}, config)
    assert sensors.read_raw("front_left") == pytest.approx(500.0)


def test_unbekanntes_bein_wirft() -> None:
    sensors, _ = _array(500)
    with pytest.raises(KeyError, match="mid_left"):
        sensors.read_raw("mid_left")


def test_abgeschalteter_sensor_taucht_nicht_auf() -> None:
    sensors, _ = _array(500, enabled=False)
    assert sensors.legs == []
    assert not sensors.any_configured


def test_ohne_kalibrierung_nur_rohwert() -> None:
    sensors, _ = _array(500, calibration=None)
    reading = sensors.read("front_left")
    assert reading.raw == pytest.approx(500.0)
    assert reading.level is None
    assert reading.percent is None
    assert not reading.calibrated


def test_volt_umrechnung() -> None:
    sensors, _ = _array(1023)
    assert sensors.read("front_left").volts == pytest.approx(5.0)


# ---------------------------------------------------------------------
# Federweg als kontinuierliche Größe
# ---------------------------------------------------------------------


def test_federweg_ueber_den_ganzen_bereich() -> None:
    """Der Sensor misst einen Bereich, keine zwei Zustände."""
    sensors, driver = _array(400)
    for raw, erwartet in [
        (400, 0.00),   # Feder entspannt
        (430, 0.10),   # gerade angetippt
        (475, 0.25),
        (550, 0.50),   # halbe Federkraft
        (625, 0.75),
        (700, 1.00),   # mechanischer Anschlag
    ]:
        driver.set_analog(0, raw)
        assert sensors.read("front_left").level == pytest.approx(erwartet)


def test_federweg_ist_monoton() -> None:
    """Mehr Last darf nie einen kleineren Pegel ergeben."""
    sensors, driver = _array(400)
    vorher = -1.0
    for raw in range(400, 701, 10):
        driver.set_analog(0, raw)
        level = sensors.read("front_left").level
        assert level is not None
        assert level >= vorher
        vorher = level


def test_prozent_ist_derselbe_wert_lesbarer() -> None:
    sensors, _ = _array(550)
    assert sensors.read("front_left").percent == pytest.approx(50.0)


def test_ausserhalb_des_vollwegs_wird_begrenzt() -> None:
    """Über den Anschlag hinaus geht es mechanisch nicht."""
    sensors, driver = _array(200)
    assert sensors.read("front_left").level == 0.0
    driver.set_analog(0, 1023)
    assert sensors.read("front_left").level == 1.0


def test_fallende_kennlinie_funktioniert_genauso() -> None:
    """Magnet andersherum gepolt: Rohwert sinkt beim Eindrücken."""
    sensors, driver = _array(700, calibration={"raw_unloaded": 700.0, "raw_full": 400.0})
    assert sensors.read("front_left").level == pytest.approx(0.0)
    driver.set_analog(0, 550)
    assert sensors.read("front_left").level == pytest.approx(0.5)
    driver.set_analog(0, 400)
    assert sensors.read("front_left").level == pytest.approx(1.0)


def test_auswertung_ist_zustandslos() -> None:
    """Derselbe Rohwert liefert denselben Pegel, egal was vorher war."""
    sensors, driver = _array(700)
    assert sensors.read("front_left").level == pytest.approx(1.0)
    driver.set_analog(0, 475)
    erst = sensors.read("front_left").level
    driver.set_analog(0, 400)
    sensors.read("front_left")
    driver.set_analog(0, 475)
    assert sensors.read("front_left").level == erst


def test_levels_liefert_nur_kalibrierte() -> None:
    driver = SimulatorDriver(num_channels=24)
    driver.set_analog(0, 700)
    driver.set_analog(1, 700)
    config = FootSensorsConfig.model_validate(
        {
            "samples": 1,
            "sensors": [
                {"leg": "front_left", "channel": 0, "calibration": dict(CALIB)},
                {"leg": "mid_left", "channel": 1},
            ],
        }
    )
    sensors = FootSensorArray({"main": driver}, config)
    assert sensors.levels() == {"front_left": pytest.approx(1.0)}


# ---------------------------------------------------------------------
# Lastverteilung
# ---------------------------------------------------------------------


def _zwei_beine(raw_a: int, raw_b: int) -> FootSensorArray:
    driver = SimulatorDriver(num_channels=24)
    driver.set_analog(0, raw_a)
    driver.set_analog(1, raw_b)
    config = FootSensorsConfig.model_validate(
        {
            "samples": 1,
            "sensors": [
                {"leg": "front_left", "channel": 0, "calibration": dict(CALIB)},
                {"leg": "mid_left", "channel": 1, "calibration": dict(CALIB)},
            ],
        }
    )
    return FootSensorArray({"main": driver}, config)


def test_gleiche_last_haelfte_haelfte() -> None:
    sensors = _zwei_beine(550, 550)
    anteile = sensors.load_share()
    assert anteile["front_left"] == pytest.approx(0.5)
    assert anteile["mid_left"] == pytest.approx(0.5)


def test_ungleiche_last_wird_sichtbar() -> None:
    """Ein Bein am Anschlag, eines halb: 2/3 zu 1/3."""
    sensors = _zwei_beine(700, 550)
    anteile = sensors.load_share()
    assert anteile["front_left"] == pytest.approx(2 / 3)
    assert anteile["mid_left"] == pytest.approx(1 / 3)


def test_alle_beine_in_der_luft_ergibt_keine_verteilung() -> None:
    sensors = _zwei_beine(400, 400)
    assert sensors.load_share() == {}


# ---------------------------------------------------------------------
# Hilfsfunktionen
# ---------------------------------------------------------------------


def test_endpunkte_aus_messreihen() -> None:
    unloaded, full = endpoints_from_samples([400, 402, 398], [700, 705, 695])
    assert unloaded == pytest.approx(400.0)
    assert full == pytest.approx(700.0)


def test_endpunkte_brauchen_werte() -> None:
    with pytest.raises(ValueError, match="mindestens einen Wert"):
        endpoints_from_samples([], [700])


def test_describe_zeigt_prozent() -> None:
    sensors, _ = _array(550)
    text = describe(sensors.read_all())
    assert "front_left" in text
    assert "50%" in text


def test_describe_markiert_unkalibrierte() -> None:
    sensors, _ = _array(550, calibration=None)
    assert "unkalibriert" in describe(sensors.read_all())


# ---------------------------------------------------------------------
# Maestro-Protokoll
# ---------------------------------------------------------------------


def test_maestro_read_analog_sendet_get_position() -> None:
    ser = FakeSerial()
    driver = MaestroDriver(
        ser=ser, num_channels=24, initial_speed=None, initial_acceleration=None
    )
    # 700 = 0x02BC -> low 0xBC, high 0x02
    ser.queue_response(0xBC, 0x02)
    value = driver.read_analog(3)
    assert value == 700
    assert bytes(ser.written) == bytes([CMD_GET_POSITION, 3])


def test_maestro_read_analog_teilt_nicht_durch_vier() -> None:
    """get_position rechnet in µs um, read_analog liefert den ADC-Rohwert."""
    ser = FakeSerial()
    driver = MaestroDriver(
        ser=ser, num_channels=24, initial_speed=None, initial_acceleration=None
    )
    ser.queue_response(0xFF, 0x03)  # 1023
    assert driver.read_analog(0) == 1023
    ser.queue_response(0xFF, 0x03)
    assert driver.get_position(0) == pytest.approx(1023 / 4)


def test_maestro_read_analog_prueft_kanal() -> None:
    ser = FakeSerial()
    driver = MaestroDriver(
        ser=ser, num_channels=24, initial_speed=None, initial_acceleration=None
    )
    with pytest.raises(ValueError, match="außerhalb"):
        driver.read_analog(24)


# ---------------------------------------------------------------------
# Simulator
# ---------------------------------------------------------------------


def test_simulator_analog_default_null() -> None:
    driver = SimulatorDriver(num_channels=24)
    assert driver.read_analog(5) == 0


def test_simulator_analog_grenzen() -> None:
    driver = SimulatorDriver(num_channels=24)
    with pytest.raises(ValueError, match="ausserhalb"):
        driver.set_analog(0, 1024)
