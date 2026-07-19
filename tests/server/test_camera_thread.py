"""Tests fuer den eigenstaendigen Kamera-Thread (CameraThread)."""
from __future__ import annotations

import time

import pytest

from hexapod.server.camera_thread import CameraThread, _approach


class FakeRobot:
    """Faengt set_camera-Aufrufe auf, ohne echte Hardware."""

    def __init__(self) -> None:
        self.calls: list[tuple[float, float]] = []

    def set_camera(self, *, pan_deg: float, tilt_deg: float, clip: bool = True) -> None:
        self.calls.append((pan_deg, tilt_deg))


def make(max_speed_deg_s: float = 150.0, rate_hz: float = 80.0):
    robot = FakeRobot()
    return CameraThread(robot, max_speed_deg_s=max_speed_deg_s, rate_hz=rate_hz), robot


class TestConstruction:
    def test_starts_centered(self) -> None:
        ct, _ = make()
        assert ct.pan == 0.0 and ct.tilt == 0.0

    @pytest.mark.parametrize("speed,rate", [(0.0, 80.0), (-1.0, 80.0), (150.0, 0.0)])
    def test_rejects_nonpositive_params(self, speed: float, rate: float) -> None:
        with pytest.raises(ValueError):
            CameraThread(FakeRobot(), max_speed_deg_s=speed, rate_hz=rate)


class TestApproach:
    def test_clamps_to_max_step(self) -> None:
        assert _approach(0.0, 100.0, 2.0) == 2.0
        assert _approach(0.0, -100.0, 2.0) == -2.0

    def test_snaps_when_within_step(self) -> None:
        assert _approach(0.0, 1.0, 2.0) == 1.0


class TestAdvance:
    def test_no_target_no_write(self) -> None:
        ct, robot = make()
        assert ct._advance() is False
        assert robot.calls == []

    def test_slews_toward_target_rate_limited(self) -> None:
        ct, robot = make(max_speed_deg_s=150.0, rate_hz=80.0)
        max_step = 150.0 / 80.0
        ct.set_target(60.0, 0.0)
        prev = 0.0
        # genug Ticks, um anzukommen
        for _ in range(200):
            if not ct._advance():
                break
            assert abs(ct.pan - prev) <= max_step + 1e-9
            prev = ct.pan
        assert ct.pan == pytest.approx(60.0)
        assert robot.calls[-1] == pytest.approx((60.0, 0.0))

    def test_retargets_to_newest(self) -> None:
        ct, _ = make()
        ct.set_target(60.0, 0.0)
        for _ in range(5):
            ct._advance()
        ct.set_target(-30.0, 0.0)  # neues Ziel mitten in der Fahrt
        for _ in range(200):
            if not ct._advance():
                break
        assert ct.pan == pytest.approx(-30.0)

    def test_sync_sets_without_motion(self) -> None:
        ct, robot = make()
        ct.sync(20.0, -10.0)
        assert ct.pan == 20.0 and ct.tilt == -10.0
        assert ct._advance() is False  # schon am Ziel -> kein Schreibzugriff
        assert robot.calls == []


class TestThreadLifecycle:
    def test_start_stop_reaches_target(self) -> None:
        ct, robot = make(max_speed_deg_s=600.0, rate_hz=200.0)
        ct.start()
        try:
            ct.set_target(30.0, -20.0)
            deadline = time.time() + 2.0
            while time.time() < deadline:
                if robot.calls and robot.calls[-1] == pytest.approx((30.0, -20.0)):
                    break
                time.sleep(0.02)
        finally:
            ct.stop()
        assert ct.pan == pytest.approx(30.0)
        assert ct.tilt == pytest.approx(-20.0)

    def test_double_start_raises(self) -> None:
        ct, _ = make()
        ct.start()
        try:
            with pytest.raises(RuntimeError):
                ct.start()
        finally:
            ct.stop()
