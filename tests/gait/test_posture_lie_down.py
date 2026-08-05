"""lie_down soll ruckfrei (lift-and-place) absetzen, nicht hart in die
Standpose springen -- auch aus einer gehaltenen Body-Pose heraus.

Der harte Sprung (altes Verhalten) tritt im UEBERGANG von der Ist-Lage in den
ersten lie_down-Frame auf. Deshalb wird der Driver-Snapshot UNMITTELBAR vor
dem Aufruf als Basis-Frame genommen.
"""
from __future__ import annotations

import pytest

from hexapod.config.loader import load_robot_config
from hexapod.gait.posture import lie_down, move_to_body_pose
from hexapod.kinematics.body_ik import BodyPose
from hexapod.robot import Hexapod

CONFIG_PATH = "config/robot.yaml"


@pytest.fixture
def sim_hexapod() -> Hexapod:
    config = load_robot_config(CONFIG_PATH)
    data = config.model_dump()
    data["buses"] = {
        n: {"type": "simulator", "num_channels": b["num_channels"]}
        for n, b in data["buses"].items()
    }
    return Hexapod(config.model_validate(data))


def _max_jump_during_lie_down(robot, monkeypatch, *, settle_first: bool) -> float:
    monkeypatch.setattr("time.sleep", lambda *_a, **_k: None)
    # In eine gehaltene Pose bringen (Koerper angehoben + geneigt).
    move_to_body_pose(robot, BodyPose(tz=20.0, pitch=0.15), steps=8, rate_hz=200.0)

    # Basis = tatsaechlich anliegende Servo-Lage VOR dem Absetzen.
    # Mit mehreren Bussen ueber alle Treiber sammeln. Die Kanalnummern
    # wiederholen sich, daher (Bus, Kanal) als Schluessel.
    base = {
        (bus, ch): us
        for bus, drv in robot.drivers.items()
        for ch, us in drv.snapshot().items()
        if us > 0
    }
    frames: list[dict[tuple[str, int], float]] = [base]

    def make_spy(bus, orig):
        def spy(positions, *a, **k):
            frames.append({(bus, ch): us for ch, us in positions.items()})
            return orig(positions, *a, **k)
        return spy

    for bus, drv in robot.drivers.items():
        monkeypatch.setattr(drv, "set_positions", make_spy(bus, drv.set_positions))
    lie_down(robot, settle_first=settle_first)

    max_jump = 0.0
    prev = frames[0]
    for fr in frames[1:]:
        for ch, us in fr.items():
            if ch in prev:
                max_jump = max(max_jump, abs(us - prev[ch]))
        prev = fr
    return max_jump


class TestLieDownSmooth:
    def test_no_hard_snap_from_held_pose(self, sim_hexapod, monkeypatch) -> None:
        jump = _max_jump_during_lie_down(sim_hexapod, monkeypatch, settle_first=True)
        print(f"\\nsettle_first=True  -> max jump {jump:.1f} us")
        assert jump < 60.0, f"harter Sprung trotz settle_first: {jump:.1f} us"

    def test_settle_first_false_snaps(self, sim_hexapod, monkeypatch) -> None:
        jump = _max_jump_during_lie_down(sim_hexapod, monkeypatch, settle_first=False)
        print(f"\\nsettle_first=False -> max jump {jump:.1f} us")
        # Belegt, dass der alte Pfad tatsaechlich hart springt (Regression-Wache).
        assert jump > 100.0
