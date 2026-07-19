"""Tests fuer das kontinuierliche, ueberlappende Gait-Modell (Ripple)."""
from __future__ import annotations

import math

import pytest

from hexapod.gait.continuous import (
    CONTINUOUS_GAITS,
    RIPPLE,
    ContinuousGait,
    cycle_targets,
    foot_offset,
)

LEGS = (
    "front_left", "front_right", "mid_left",
    "mid_right", "back_left", "back_right",
)


class TestRippleDefinition:
    def test_registered(self) -> None:
        assert CONTINUOUS_GAITS["ripple"] is RIPPLE

    def test_all_six_legs_have_offsets(self) -> None:
        assert sorted(RIPPLE.phase_offsets) == sorted(LEGS)

    def test_duty_two_thirds(self) -> None:
        assert RIPPLE.duty == pytest.approx(2.0 / 3.0)

    def test_offsets_evenly_spaced(self) -> None:
        vals = sorted(RIPPLE.phase_offsets.values())
        assert vals == pytest.approx([i / 6.0 for i in range(6)])

    @pytest.mark.parametrize("bad", [-0.1, 0.0, 1.0, 1.5])
    def test_invalid_duty_rejected(self, bad) -> None:
        with pytest.raises(ValueError, match="duty"):
            ContinuousGait("x", {"front_left": 0.0}, duty=bad)

    def test_invalid_offset_rejected(self) -> None:
        with pytest.raises(ValueError, match="phase_offset"):
            ContinuousGait("x", {"front_left": 1.0}, duty=0.5)


class TestOverlap:
    @pytest.mark.parametrize("t", [0.05, 0.21, 0.4, 0.55, 0.72, 0.9])
    def test_exactly_two_legs_swing(self, t) -> None:
        # Duty 2/3 -> Schwungfenster 1/3 = 2 x 1/6 -> stets zwei Beine in Luft
        assert len(RIPPLE.legs_in_swing(t)) == 2

    def test_swing_sets_change_over_cycle(self) -> None:
        sets = {RIPPLE.legs_in_swing(t / 12.0) for t in range(12)}
        # ueber den Zyklus mehrere unterschiedliche Schwung-Paare
        assert len(sets) >= 5


class TestFootOffsetGeometry:
    def test_stance_is_on_ground(self) -> None:
        for u in [0.0, 0.2, 0.4, 0.6]:
            _, _, z = foot_offset((50.0, 0.0), 30.0, u, 2.0 / 3.0)
            assert z == pytest.approx(0.0)

    def test_swing_lifts_foot(self) -> None:
        # Mitte des Schwungfensters (u = duty + (1-duty)/2)
        _, _, z = foot_offset((50.0, 0.0), 30.0, 2.0 / 3.0 + 1.0 / 6.0, 2.0 / 3.0)
        assert z == pytest.approx(30.0, abs=1e-6)

    def test_z_never_negative(self) -> None:
        for i in range(200):
            _, _, z = foot_offset((40.0, 10.0), 25.0, i / 200.0, 2.0 / 3.0)
            assert z >= -1e-9

    def test_continuous_at_stance_swing_boundary(self) -> None:
        duty = 2.0 / 3.0
        before = foot_offset((50.0, 0.0), 30.0, duty - 1e-6, duty)
        after = foot_offset((50.0, 0.0), 30.0, duty + 1e-6, duty)
        for a, b in zip(before, after):
            assert a == pytest.approx(b, abs=1e-3)

    def test_continuous_at_cycle_wrap(self) -> None:
        duty = 2.0 / 3.0
        end = foot_offset((50.0, 0.0), 30.0, 1.0 - 1e-6, duty)
        start = foot_offset((50.0, 0.0), 30.0, 0.0, duty)
        for a, b in zip(end, start):
            assert a == pytest.approx(b, abs=1e-3)

    def test_stance_travel_equals_stride(self) -> None:
        # Stand laeuft von -s/2 nach +s/2 -> Fuss bewegt sich um +stride
        # (schiebt den Koerper um stride in -stride-Richtung pro Zyklus).
        sx = 50.0
        start = foot_offset((sx, 0.0), 30.0, 0.0, 2.0 / 3.0)[0]
        end = foot_offset((sx, 0.0), 30.0, 2.0 / 3.0 - 1e-6, 2.0 / 3.0)[0]
        assert end - start == pytest.approx(sx, abs=1e-3)


class TestCycleTargets:
    def test_covers_all_legs(self) -> None:
        strides = {leg: (50.0, 0.0) for leg in LEGS}
        out = cycle_targets(RIPPLE, strides, 30.0, 0.3)
        assert sorted(out) == sorted(LEGS)

    def test_two_legs_lifted(self) -> None:
        strides = {leg: (50.0, 0.0) for leg in LEGS}
        out = cycle_targets(RIPPLE, strides, 30.0, 0.4)
        lifted = [leg for leg, (_, _, z) in out.items() if z > 1e-6]
        assert len(lifted) == 2

    def test_full_cycle_returns_to_stance(self) -> None:
        strides = {leg: (50.0, 0.0) for leg in LEGS}
        a = cycle_targets(RIPPLE, strides, 30.0, 0.0)
        b = cycle_targets(RIPPLE, strides, 30.0, 1.0 - 1e-9)
        for leg in LEGS:
            for x, y in zip(a[leg], b[leg]):
                assert x == pytest.approx(y, abs=1e-2)
