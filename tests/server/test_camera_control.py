"""Tests fuer die slew-ratenbegrenzte Kamera-Glaettung (CameraController)."""
from __future__ import annotations

import pytest

from hexapod.server.camera_control import CameraController


class FakeRobot:
    """Faengt set_camera-Aufrufe auf, ohne echte Hardware."""

    def __init__(self) -> None:
        self.calls: list[tuple[float, float]] = []

    def set_camera(self, *, pan_deg: float, tilt_deg: float, clip: bool = True) -> None:
        self.calls.append((pan_deg, tilt_deg))


def make(
    max_speed_deg_s: float = 150.0, rate_hz: float = 80.0
) -> tuple[CameraController, FakeRobot]:
    robot = FakeRobot()
    ctrl = CameraController(
        robot, max_speed_deg_s=max_speed_deg_s, rate_hz=rate_hz, sleep=lambda _dt: None
    )
    return ctrl, robot


class TestConstruction:
    def test_starts_centered(self) -> None:
        ctrl, _ = make()
        assert ctrl.pan == 0.0 and ctrl.tilt == 0.0

    @pytest.mark.parametrize("speed,rate", [(0.0, 80.0), (-1.0, 80.0), (150.0, 0.0)])
    def test_rejects_nonpositive_params(self, speed: float, rate: float) -> None:
        with pytest.raises(ValueError):
            CameraController(FakeRobot(), max_speed_deg_s=speed, rate_hz=rate)


class TestSlew:
    def test_reaches_target_exactly(self) -> None:
        ctrl, robot = make()
        ctrl.move_to(60.0, -40.0)
        assert robot.calls[-1] == pytest.approx((60.0, -40.0))
        assert ctrl.pan == pytest.approx(60.0)
        assert ctrl.tilt == pytest.approx(-40.0)

    def test_respects_slew_rate(self) -> None:
        # max_step = 150/80 = 1.875 deg pro Schritt auf der schnelleren Achse.
        ctrl, robot = make(max_speed_deg_s=150.0, rate_hz=80.0)
        ctrl.move_to(60.0, 0.0)
        max_step = 150.0 / 80.0
        prev_pan = 0.0
        for pan, _tilt in robot.calls:
            assert abs(pan - prev_pan) <= max_step + 1e-9
            prev_pan = pan

    def test_axes_arrive_together(self) -> None:
        # Beide Achsen werden synchron interpoliert: gleiche Schrittzahl.
        ctrl, robot = make()
        ctrl.move_to(30.0, 10.0)
        # Monoton steigend auf beiden Achsen, gemeinsamer Endpunkt.
        pans = [p for p, _ in robot.calls]
        tilts = [t for _, t in robot.calls]
        assert pans == sorted(pans)
        assert tilts == sorted(tilts)
        assert pans[-1] == pytest.approx(30.0)
        assert tilts[-1] == pytest.approx(10.0)

    def test_uses_many_steps_for_large_move(self) -> None:
        ctrl, robot = make()
        ctrl.move_to(60.0, 0.0)
        # 60 deg / 1.875 = 32 Schritte -- jedenfalls deutlich mehr als 1.
        assert len(robot.calls) >= 10

    def test_small_move_single_step(self) -> None:
        ctrl, robot = make()
        ctrl.move_to(1.0, 0.5)  # < max_step -> ein einziger Apply
        assert len(robot.calls) == 1
        assert robot.calls[0] == pytest.approx((1.0, 0.5))

    def test_consecutive_moves_continue_from_current(self) -> None:
        ctrl, robot = make()
        ctrl.move_to(60.0, 0.0)
        robot.calls.clear()
        ctrl.move_to(0.0, 0.0)  # zurueck zur Mitte
        # Erster Schritt darf nur max_step von 60 entfernt sein, nicht springen.
        max_step = 150.0 / 80.0
        assert 60.0 - robot.calls[0][0] <= max_step + 1e-9
        assert robot.calls[-1][0] == pytest.approx(0.0)


class TestSync:
    def test_sync_sets_position_without_moving(self) -> None:
        ctrl, robot = make()
        ctrl.sync(15.0, -5.0)
        assert ctrl.pan == 15.0 and ctrl.tilt == -5.0
        assert robot.calls == []
