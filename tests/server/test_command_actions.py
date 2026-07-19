"""Regression: die Action-Whitelist des /command-Endpoints muss mit den im
Worker je Zustand erlaubten Actions uebereinstimmen (sonst werden gueltige
Befehle wie set_gait faelschlich mit 422 abgelehnt)."""
from __future__ import annotations

from hexapod.server.app import VALID_ACTIONS
from hexapod.server.worker import RobotWorker


def test_set_gait_is_accepted() -> None:
    assert "set_gait" in VALID_ACTIONS


def test_whitelist_matches_worker_allowed() -> None:
    union = set().union(*RobotWorker._ALLOWED.values())
    assert set(VALID_ACTIONS) == union


def test_core_actions_present() -> None:
    for a in ("walk", "halt", "pose", "camera", "set_gait", "stance",
              "lie_down", "stand_up", "power_up", "assume_standing", "settle"):
        assert a in VALID_ACTIONS, a
