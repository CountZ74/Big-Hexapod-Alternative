"""Tests für den foot_sensors-Block der Konfiguration."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest
import yaml
from pydantic import ValidationError

from hexapod.config.loader import load_robot_config, save_foot_sensor_calibrations
from hexapod.config.model import (
    FootSensorCalibration,
    FootSensorsConfig,
    RobotConfig,
)

# Mechanischer Vollweg: unbelastet 400, Anschlag 700.
VOLLWEG = {"raw_unloaded": 400.0, "raw_full": 700.0}


# ---------------------------------------------------------------------
# Messbereich
# ---------------------------------------------------------------------


def test_federweg_normiert_zwischen_endpunkten() -> None:
    calib = FootSensorCalibration(**VOLLWEG)
    assert calib.level(400.0) == pytest.approx(0.0)
    assert calib.level(550.0) == pytest.approx(0.5)
    assert calib.level(700.0) == pytest.approx(1.0)


def test_federweg_wird_hart_begrenzt() -> None:
    calib = FootSensorCalibration(**VOLLWEG)
    assert calib.level(100.0) == 0.0
    assert calib.level(1023.0) == 1.0


def test_fallende_kennlinie_funktioniert_genauso() -> None:
    """Magnet andersherum gepolt: Rohwert sinkt beim Eindrücken."""
    calib = FootSensorCalibration(raw_unloaded=700.0, raw_full=400.0)
    assert calib.level(700.0) == pytest.approx(0.0)
    assert calib.level(550.0) == pytest.approx(0.5)
    assert calib.level(400.0) == pytest.approx(1.0)
    assert calib.span == pytest.approx(-300.0)


def test_aufloesung_wird_ausgewiesen() -> None:
    calib = FootSensorCalibration(**VOLLWEG)
    assert calib.counts_per_percent == pytest.approx(3.0)


def test_zu_kleiner_messbereich_wird_abgelehnt() -> None:
    with pytest.raises(ValidationError, match="Messbereich"):
        FootSensorCalibration(raw_unloaded=500.0, raw_full=505.0)


def test_kalibrierung_kennt_keine_schwelle() -> None:
    """Die Entscheidung 'hat Boden' gehoert nicht in die Kalibrierung."""
    felder = set(FootSensorCalibration.model_fields)
    assert felder == {"raw_unloaded", "raw_full"}


# ---------------------------------------------------------------------
# Sensor-Liste
# ---------------------------------------------------------------------


def test_nur_ein_sensor_pro_bein() -> None:
    with pytest.raises(ValidationError, match="Doppelt"):
        FootSensorsConfig.model_validate(
            {
                "sensors": [
                    {"leg": "front_left", "channel": 0},
                    {"leg": "front_left", "channel": 1},
                ]
            }
        )


def test_kanaele_muessen_eindeutig_sein() -> None:
    with pytest.raises(ValidationError, match="eindeutig"):
        FootSensorsConfig.model_validate(
            {
                "sensors": [
                    {"leg": "front_left", "channel": 0},
                    {"leg": "mid_left", "channel": 0},
                ]
            }
        )


def test_nur_analogfaehige_kanaele() -> None:
    """Kanaele 12..23 des Mini Maestro haben keinen ADC."""
    with pytest.raises(ValidationError):
        FootSensorsConfig.model_validate({"sensors": [{"leg": "front_left", "channel": 12}]})


def test_active_filtert_abgeschaltete_sensoren() -> None:
    cfg = FootSensorsConfig.model_validate(
        {
            "sensors": [
                {"leg": "front_left", "channel": 0},
                {"leg": "mid_left", "channel": 1, "enabled": False},
            ]
        }
    )
    assert [s.leg for s in cfg.active] == ["front_left"]


# ---------------------------------------------------------------------
# Zusammenspiel mit der Gesamt-Konfiguration
# ---------------------------------------------------------------------


def test_sensor_auf_servokanal_wird_abgelehnt(minimal_config_dict: dict[str, Any]) -> None:
    """Die wichtigste Pruefung: ein Servo-Kanal als Eingang laesst das Bein fallen."""
    servo_channel = minimal_config_dict["servos"][0]["channel"]
    minimal_config_dict["foot_sensors"] = {
        "sensors": [{"leg": "front_right", "channel": servo_channel}]
    }
    with pytest.raises(ValidationError, match="bereits durch einen Servo belegt"):
        RobotConfig.model_validate(minimal_config_dict)


def test_sensor_auf_unbekanntem_bein(minimal_config_dict: dict[str, Any]) -> None:
    minimal_config_dict["foot_sensors"] = {
        "sensors": [{"leg": "gibt_es_nicht", "channel": 0}]
    }
    with pytest.raises(ValidationError, match="unbekanntes"):
        RobotConfig.model_validate(minimal_config_dict)


def test_freier_kanal_ist_erlaubt(minimal_config_dict: dict[str, Any]) -> None:
    belegt = {s["channel"] for s in minimal_config_dict["servos"]}
    frei = next(ch for ch in range(12) if ch not in belegt)
    minimal_config_dict["foot_sensors"] = {
        "sensors": [
            {"leg": "front_right", "channel": frei, "calibration": dict(VOLLWEG)}
        ]
    }
    config = RobotConfig.model_validate(minimal_config_dict)
    sensor = config.foot_sensors.get("front_right")
    assert sensor.channel == frei
    assert sensor.calibration is not None


def test_ohne_foot_sensors_bleibt_alles_wie_bisher(minimal_config_dict: dict[str, Any]) -> None:
    config = RobotConfig.model_validate(minimal_config_dict)
    assert config.foot_sensors.sensors == []
    assert config.foot_sensors.active == []


# ---------------------------------------------------------------------
# Kamera auf eigenem Bus
# ---------------------------------------------------------------------


def test_kamera_auf_eigenem_bus_darf_kanal_wiederverwenden(
    minimal_config_dict: dict[str, Any],
) -> None:
    """Maestro-Kanal 6 und PCA9685-Anschluss 6 sind verschiedene Anschluesse."""
    leg_channel = minimal_config_dict["servos"][0]["channel"]
    minimal_config_dict["camera_driver"] = {"type": "pca9685"}
    minimal_config_dict["servos"].append(
        {
            "kind": "camera",
            "axis": "pan",
            "channel": leg_channel,
            "center_us": 1500.0,
            "min_us": 900.0,
            "max_us": 2100.0,
            "range_us": 700.0,
        }
    )
    config = RobotConfig.model_validate(minimal_config_dict)
    assert config.camera_on_own_bus
    assert len(config.camera_bus_servos) == 1
    assert all(s.kind == "leg" for s in config.main_bus_servos)


def test_ohne_kamera_bus_kollidieren_kanaele_weiterhin(
    minimal_config_dict: dict[str, Any],
) -> None:
    leg_channel = minimal_config_dict["servos"][0]["channel"]
    minimal_config_dict["servos"].append(
        {
            "kind": "camera",
            "axis": "pan",
            "channel": leg_channel,
            "center_us": 1500.0,
            "min_us": 900.0,
            "max_us": 2100.0,
            "range_us": 700.0,
        }
    )
    with pytest.raises(ValidationError, match="eindeutig"):
        RobotConfig.model_validate(minimal_config_dict)


# ---------------------------------------------------------------------
# Zurueckschreiben in die YAML
# ---------------------------------------------------------------------


def test_kalibrierung_speichern_laesst_rest_unangetastet(
    tmp_path: Path, minimal_config_dict: dict[str, Any]
) -> None:
    minimal_config_dict["foot_sensors"] = {
        "sensors": [{"leg": "front_right", "channel": 0, "enabled": True}]
    }
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.dump(minimal_config_dict, sort_keys=False), encoding="utf-8")

    calib = FootSensorCalibration(raw_unloaded=412.0, raw_full=688.0)
    save_foot_sensor_calibrations(path, {"front_right": calib})

    reloaded = load_robot_config(path)
    sensor = reloaded.foot_sensors.get("front_right")
    assert sensor.calibration is not None
    assert sensor.calibration.raw_unloaded == pytest.approx(412.0)
    assert sensor.calibration.raw_full == pytest.approx(688.0)
    # Der Rest der Datei ist unveraendert geblieben.
    assert reloaded.body.legs[0].mount_x == minimal_config_dict["body"]["legs"][0]["mount_x"]


def test_speichern_fuer_unbekanntes_bein_wirft(
    tmp_path: Path, minimal_config_dict: dict[str, Any]
) -> None:
    minimal_config_dict["foot_sensors"] = {"sensors": [{"leg": "front_right", "channel": 0}]}
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.dump(minimal_config_dict, sort_keys=False), encoding="utf-8")

    calib = FootSensorCalibration(**VOLLWEG)
    with pytest.raises(KeyError, match="mid_left"):
        save_foot_sensor_calibrations(path, {"mid_left": calib})
