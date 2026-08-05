"""Fixtures für Hexapod-Tests."""

from __future__ import annotations

import pytest

from hexapod.config.loader import load_robot_config
from hexapod.robot import Hexapod

CONFIG_PATH = "config/robot.yaml"


@pytest.fixture
def sim_hexapod() -> Hexapod:
    """Hexapod mit Simulator auf ALLEN Bussen — keine Hardware nötig."""
    config = load_robot_config(CONFIG_PATH)
    data = config.model_dump()
    data["buses"] = {
        name: {"type": "simulator", "num_channels": bus.num_channels}
        for name, bus in config.buses.items()
    }
    return Hexapod(config.model_validate(data))


@pytest.fixture
def leg_bus(sim_hexapod: Hexapod) -> str:
    """Der Bus, an dem die Beinservos des ersten Beins hängen."""
    from hexapod.config import Joint
    first = sim_hexapod.leg_names[0]
    return sim_hexapod.config.get_leg_servo(first, Joint.COXA).bus
