"""Lädt RobotConfig aus YAML-Dateien.

Trennt das *Wo* (Dateipfade, IO) vom *Was* (Datenmodell in model.py).
Wenn wir später Konfig aus anderen Quellen lesen wollen (z.B. JSON
über MQTT), kommt das hier dazu, ohne das Modell anzufassen.
"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

import yaml

from .model import FootSensorCalibration, RobotConfig


def load_robot_config(path: str | Path) -> RobotConfig:
    """Liest eine YAML-Datei und gibt eine validierte RobotConfig zurück.

    Raises:
        FileNotFoundError: wenn `path` nicht existiert.
        yaml.YAMLError: bei kaputter YAML-Syntax.
        pydantic.ValidationError: bei Verstößen gegen das Schema.
    """
    p = Path(path)
    with p.open("r", encoding="utf-8") as f:
        raw: Any = yaml.safe_load(f)

    if not isinstance(raw, dict):
        raise ValueError(
            f"Erwarte ein YAML-Mapping auf Top-Level in {path}, "
            f"erhalten: {type(raw).__name__}"
        )

    return RobotConfig.model_validate(raw)


def dump_robot_config(config: RobotConfig, path: str | Path) -> None:
    """Schreibt eine RobotConfig als YAML.

    Nützlich für Generatoren (z.B. Kalibrier-Tool, das beim Speichern
    eine aktualisierte Konfig rausschreibt).
    """
    p = Path(path)
    data = config.model_dump(mode="json")
    with p.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=False, allow_unicode=True)


def save_foot_sensor_calibrations(
    path: str | Path,
    calibrations: Mapping[str, FootSensorCalibration],
) -> Path:
    """Schreibt Fußsensor-Kalibrierungen zurück in eine bestehende robot.yaml.

    Bewusst als gezielter Patch auf dem rohen YAML-Baum (wie `save_z_trims`)
    statt als kompletter Neu-Dump: so bleiben Reihenfolge und alle nicht
    betroffenen Werte der Datei unangetastet.

    Args:
        path: Die zu ändernde robot.yaml.
        calibrations: Bein-Name → neue Kalibrierung.

    Returns:
        Der geschriebene Pfad.

    Raises:
        KeyError: wenn für ein Bein gar kein Sensor in der Datei steht.
    """
    target = Path(path)
    data: Any = yaml.safe_load(target.read_text(encoding="utf-8"))

    sensors = data.get("foot_sensors", {}).get("sensors", [])
    by_leg = {s["leg"]: s for s in sensors}

    unknown = set(calibrations) - set(by_leg)
    if unknown:
        raise KeyError(
            f"Für diese Beine steht kein Fußsensor in {target}: {sorted(unknown)}"
        )

    for leg, calib in calibrations.items():
        by_leg[leg]["calibration"] = {
            "raw_unloaded": round(calib.raw_unloaded, 1),
            "raw_full": round(calib.raw_full, 1),
        }

    target.write_text(
        yaml.dump(data, default_flow_style=False, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    return target
