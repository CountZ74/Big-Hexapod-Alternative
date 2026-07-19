"""Tests fuer die MPU6050-Winkelmathematik (reine Funktionen, keine Hardware)."""
from __future__ import annotations

import math
import pytest

from hexapod.drivers.mpu6050 import accel_to_tilt, _s16


class TestS16:
    def test_positive(self) -> None:
        assert _s16(0x01, 0x00) == 256

    def test_negative(self) -> None:
        assert _s16(0xFF, 0xFF) == -1

    def test_zero(self) -> None:
        assert _s16(0, 0) == 0


class TestAccelToTilt:
    def test_level(self) -> None:
        roll, pitch = accel_to_tilt(0.0, 0.0, 1.0)
        assert roll == pytest.approx(0.0, abs=1e-6)
        assert pitch == pytest.approx(0.0, abs=1e-6)

    def test_roll_30(self) -> None:
        # Seitneigung 30 deg: g teilt sich auf ay/az
        ay, az = math.sin(math.radians(30)), math.cos(math.radians(30))
        roll, pitch = accel_to_tilt(0.0, ay, az)
        assert roll == pytest.approx(30.0, abs=0.1)
        assert pitch == pytest.approx(0.0, abs=0.1)

    def test_pitch_20(self) -> None:
        ax, az = -math.sin(math.radians(20)), math.cos(math.radians(20))
        roll, pitch = accel_to_tilt(ax, 0.0, az)
        assert pitch == pytest.approx(20.0, abs=0.1)
        assert roll == pytest.approx(0.0, abs=0.1)
