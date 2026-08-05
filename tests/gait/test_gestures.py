"""Tests fuer die Gesten: laufen auf dem Simulator durch und enden in Stance."""
from __future__ import annotations

import pytest

from hexapod.config.loader import load_robot_config
from hexapod.gait import gestures
from hexapod.robot import Hexapod

CONFIG_PATH = "config/robot.yaml"


@pytest.fixture
def sim_robot() -> Hexapod:
    cfg = load_robot_config(CONFIG_PATH)
    data = cfg.model_dump()
    data["buses"] = {
        n: {"type": "simulator", "num_channels": b["num_channels"]}
        for n, b in data["buses"].items()
    }
    r = Hexapod(cfg.model_validate(data))
    r.stance()
    return r


@pytest.mark.parametrize("name", sorted(gestures.GESTURES))
def test_gesture_runs_and_returns_to_stance(sim_robot, monkeypatch, name) -> None:
    monkeypatch.setattr("time.sleep", lambda *_a, **_k: None)
    gestures.GESTURES[name](sim_robot)
    # Endet wieder nahe der Standpose (alle Offsets ~0)
    for leg in sim_robot.leg_names:
        dx, dy, dz = sim_robot.current_offset(leg)
        assert abs(dx) < 1.0 and abs(dy) < 1.0 and abs(dz) < 1.0, (
            f"{name}/{leg}: nicht in Stance zurueck ({dx:.1f},{dy:.1f},{dz:.1f})"
        )


def test_wave_left_leg(sim_robot, monkeypatch) -> None:
    monkeypatch.setattr("time.sleep", lambda *_a, **_k: None)
    gestures.wave(sim_robot, leg="front_left")
    dx, dy, dz = sim_robot.current_offset("front_left")
    assert abs(dx) < 1.0 and abs(dy) < 1.0 and abs(dz) < 1.0
