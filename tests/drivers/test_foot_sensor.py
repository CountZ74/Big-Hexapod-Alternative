"""Tests für den Fußsensor-Treiber (Auswertung, Hysterese, Maestro-Protokoll)."""

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

CALIB = {"raw_released": 400.0, "raw_contact": 700.0, "threshold": 0.4, "hysteresis": 0.15}


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
    return FootSensorArray(driver, _config(**overrides)), driver


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
    sensors = FootSensorArray(ZackigerTreiber(), config)
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
    assert reading.contact is None
    assert not reading.calibrated


def test_volt_umrechnung() -> None:
    sensors, _ = _array(1023)
    assert sensors.read("front_left").volts == pytest.approx(5.0)


# ---------------------------------------------------------------------
# Schwelle und Hysterese
# ---------------------------------------------------------------------


def test_kein_kontakt_unterhalb_der_schwelle() -> None:
    # Pegel 0.2 < threshold 0.4
    sensors, _ = _array(460)
    assert sensors.read("front_left").contact is False


def test_kontakt_ab_der_schwelle() -> None:
    # Pegel 0.4 == threshold
    sensors, _ = _array(520)
    assert sensors.read("front_left").contact is True


def test_hysterese_haelt_den_kontakt() -> None:
    """Einmal aufgesetzt, bleibt der Kontakt bis unter threshold - hysteresis."""
    sensors, driver = _array(700)  # Pegel 1.0 -> Kontakt
    assert sensors.read("front_left").contact is True

    driver.set_analog(0, 490)  # Pegel 0.30: unter 0.4, aber über 0.25
    assert sensors.read("front_left").contact is True

    driver.set_analog(0, 460)  # Pegel 0.20: unter 0.25 -> löst aus
    assert sensors.read("front_left").contact is False

    driver.set_analog(0, 490)  # wieder 0.30, aber jetzt aus dem Ruhezustand
    assert sensors.read("front_left").contact is False


def test_reset_state_setzt_kontakt_zurueck() -> None:
    sensors, driver = _array(700)
    assert sensors.read("front_left").contact is True
    sensors.reset_state()
    driver.set_analog(0, 490)  # 0.30 — ohne Vorgeschichte kein Kontakt
    assert sensors.read("front_left").contact is False


def test_contacts_liefert_nur_kalibrierte() -> None:
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
    sensors = FootSensorArray(driver, config)
    assert sensors.contacts() == {"front_left": True}


# ---------------------------------------------------------------------
# Hilfsfunktionen
# ---------------------------------------------------------------------


def test_endpunkte_aus_messreihen() -> None:
    released, contact = endpoints_from_samples([400, 402, 398], [700, 705, 695])
    assert released == pytest.approx(400.0)
    assert contact == pytest.approx(700.0)


def test_endpunkte_brauchen_werte() -> None:
    with pytest.raises(ValueError, match="mindestens einen Wert"):
        endpoints_from_samples([], [700])


def test_describe_ist_lesbar() -> None:
    sensors, _ = _array(700)
    text = describe(sensors.read_all())
    assert "front_left" in text
    assert "KONTAKT" in text


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
