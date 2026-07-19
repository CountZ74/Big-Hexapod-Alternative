"""Tests für Fuß-Trajektorien."""

import math

import pytest

from hexapod.gait.trajectory import (
    lerp,
    linear_path,
    stance_path,
    swing_path,
)


class TestLerp:
    def test_endpoints(self):
        assert lerp((0, 0, 0), (10, 20, 30), 0.0) == (0, 0, 0)
        assert lerp((0, 0, 0), (10, 20, 30), 1.0) == (10, 20, 30)

    def test_midpoint(self):
        assert lerp((0, 0, 0), (10, 20, 30), 0.5) == (5, 10, 15)


class TestLinearPath:
    def test_count(self):
        pts = linear_path((0, 0, 0), (10, 0, 0), 5)
        assert len(pts) == 5

    def test_ends_at_p1(self):
        pts = linear_path((0, 0, 0), (10, 0, 0), 5)
        assert pts[-1] == pytest.approx((10, 0, 0))

    def test_excludes_p0(self):
        pts = linear_path((0, 0, 0), (10, 0, 0), 5)
        assert pts[0] != (0, 0, 0)

    def test_evenly_spaced(self):
        pts = linear_path((0, 0, 0), (10, 0, 0), 10)
        xs = [p[0] for p in pts]
        diffs = [xs[i + 1] - xs[i] for i in range(len(xs) - 1)]
        for d in diffs:
            assert d == pytest.approx(1.0)

    def test_invalid_steps(self):
        with pytest.raises(ValueError):
            linear_path((0, 0, 0), (1, 1, 1), 0)


class TestSwingPath:
    def test_count(self):
        pts = swing_path((0, 0, 0), (40, 0, 0), height=30, steps=20)
        assert len(pts) == 20

    def test_ends_on_ground(self):
        # Bei s=1 ist sin(pi)=0, also keine Anhebung am Ende
        pts = swing_path((0, 0, 0), (40, 0, 0), height=30, steps=20)
        assert pts[-1] == pytest.approx((40, 0, 0), abs=1e-9)

    def test_lifts_in_middle(self):
        # Der mittlere Punkt sollte deutlich angehoben sein
        pts = swing_path((0, 0, 0), (40, 0, 0), height=30, steps=20)
        mid_z = pts[len(pts) // 2 - 1][2]
        assert mid_z > 25  # nahe 30

    def test_max_height_positive(self):
        pts = swing_path((0, 0, 0), (40, 0, 0), height=30, steps=21)
        max_z = max(p[2] for p in pts)
        assert max_z == pytest.approx(30, abs=2)

    def test_horizontal_linear(self):
        # x-Komponente bleibt linear trotz Z-Anhebung
        pts = swing_path((0, 0, 0), (40, 0, 0), height=30, steps=20)
        assert pts[-1][0] == pytest.approx(40)


class TestStancePath:
    def test_same_as_linear(self):
        a = stance_path((0, 0, 0), (10, 5, 0), 8)
        b = linear_path((0, 0, 0), (10, 5, 0), 8)
        assert a == b


class TestRoundTrip:
    def test_swing_then_stance_continuous(self):
        # Schwung von 0 nach +20, dann Stand zurück nach 0:
        # die Aneinanderreihung soll am Boden geschlossen sein.
        swing = swing_path((-20, 0, 0), (20, 0, 0), height=30, steps=10)
        stance = stance_path((20, 0, 0), (-20, 0, 0), steps=10)
        # Ende Schwung trifft Start Stand (beide bei +20, am Boden)
        assert swing[-1] == pytest.approx((20, 0, 0), abs=1e-9)
        # Ende Stand trifft Start Schwung (beide bei -20)
        assert stance[-1] == pytest.approx((-20, 0, 0), abs=1e-9)
