"""Lädt RobotConfig aus YAML-Dateien.

Trennt das *Wo* (Dateipfade, IO) vom *Was* (Datenmodell in model.py).
Wenn wir später Konfig aus anderen Quellen lesen wollen (z.B. JSON
über MQTT), kommt das hier dazu, ohne das Modell anzufassen.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from .model import RobotConfig


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
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=False)
