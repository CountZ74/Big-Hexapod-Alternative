"""Fixtures für Hexapod-Tests."""

from __future__ import annotations

import pytest

from hexapod.config.loader import load_robot_config
from hexapod.robot import Hexapod

CONFIG_PATH = "config/robot.yaml"


@pytest.fixture
def sim_hexapod() -> Hexapod:
    """Hexapod mit Simulator-Driver — kein Maestro nötig."""
    config = load_robot_config(CONFIG_PATH)
    data = config.model_dump()
    data["driver"] = {
        "type": "simulator",
        "port": "/dev/null",
        "num_channels": 24,
        "timeout": 1.0,
    }
    # Die Kamera-Servos haengen am PCA9685 — im Test ebenfalls simuliert,
    # sonst wuerde der Test echten I2C-Verkehr versuchen.
    if data.get("camera_driver") is not None:
        data["camera_driver"] = {**data["camera_driver"], "type": "simulator"}
    sim_config = config.model_validate(data)
    return Hexapod(sim_config)
