"""Tests fuer das Stufen-/Podest-Klettern (climb.py)."""
from __future__ import annotations

from itertools import pairwise

import pytest

from hexapod.config import Joint
from hexapod.config.loader import load_robot_config
from hexapod.gait.climb import climb_over_box, plan_climb_over
from hexapod.robot import Hexapod

CONFIG_PATH = "config/robot.yaml"
_JOINTS = (Joint.COXA, Joint.FEMUR, Joint.TIBIA)


@pytest.fixture
def sim_robot() -> Hexapod:
    cfg = load_robot_config(CONFIG_PATH)
    data = cfg.model_dump()
    data["driver"] = {"type": "simulator", "port": "/dev/null",
                      "num_channels": 24, "timeout": 1.0}
    r = Hexapod(cfg.model_validate(data))
    r.stance()
    return r


def _reachable(r: Hexapod, leg: str, p: tuple[float, float, float],
               margin: float = 12.0) -> bool:
    try:
        angles = r.offset_to_angles(leg, p[0], p[1], p[2])
    except Exception:
        return False
    for j, ang in zip(_JOINTS, angles, strict=True):
        ch = r.config.get_leg_servo(leg, j).channel
        mp = r._mappings[ch]
        us = mp.angle_to_us(ang, clip=True)
        if us <= mp.min_us + margin or us >= mp.max_us - margin:
            return False
    return True


def test_plan_climb_over_all_points_reachable(sim_robot: Hexapod) -> None:
    """Jeder Punkt der Ueberstieg-Choreographie muss ohne Clipping fahrbar sein."""
    neutral_x = {leg: xy[0] for leg, xy in sim_robot.neutral_foot_xy.items()}
    plan = plan_climb_over(neutral_x)
    assert len(plan) > 10
    for i, op in enumerate(plan):
        for leg, pts in op.items():
            for a, b in pairwise(pts):
                for k in range(4):
                    f = k / 3
                    p = (a[0] + (b[0] - a[0]) * f,
                         a[1] + (b[1] - a[1]) * f,
                         a[2] + (b[2] - a[2]) * f)
                    assert _reachable(sim_robot, leg, p), (
                        f"op{i} {leg}: {p} nicht erreichbar"
                    )


def test_plan_climb_over_advances_all_legs(sim_robot: Hexapod) -> None:
    """Der Plan endet mit allen Fuessen auf Bodenniveau (letzte Op senkt ab)."""
    neutral_x = {leg: xy[0] for leg, xy in sim_robot.neutral_foot_xy.items()}
    plan = plan_climb_over(neutral_x)
    final = plan[-1]
    assert set(final) == set(neutral_x)
    for pts in final.values():
        assert pts[-1][2] == pytest.approx(0.0)  # dz zurueck auf Standniveau


def test_climb_over_box_runs_and_returns_to_stance(sim_robot: Hexapod, monkeypatch) -> None:
    monkeypatch.setattr("time.sleep", lambda *_a, **_k: None)
    climb_over_box(sim_robot)
    for leg in sim_robot.leg_names:
        dx, dy, dz = sim_robot.current_offset(leg)
        assert abs(dx) < 1.0 and abs(dy) < 1.0 and abs(dz) < 1.0, (
            f"{leg}: nicht in Stance ({dx:.1f},{dy:.1f},{dz:.1f})"
        )
