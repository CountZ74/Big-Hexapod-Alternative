"""Tests für den Tripod-Gait."""

import pytest

from hexapod.gait.tripod import half_cycle_paths, GROUP_A, GROUP_B


class TestGroups:
    def test_groups_disjoint(self):
        assert set(GROUP_A).isdisjoint(set(GROUP_B))

    def test_groups_cover_all_legs(self):
        all_legs = set(GROUP_A) | set(GROUP_B)
        expected = {
            "front_right", "front_left", "mid_right",
            "mid_left", "back_right", "back_left",
        }
        assert all_legs == expected

    def test_each_group_has_three(self):
        assert len(GROUP_A) == 3
        assert len(GROUP_B) == 3


class TestHalfCyclePaths:
    def test_all_legs_present(self):
        paths = half_cycle_paths(GROUP_A, GROUP_B, stride=50, height=30, steps=10)
        assert set(paths.keys()) == set(GROUP_A) | set(GROUP_B)

    def test_equal_length(self):
        paths = half_cycle_paths(GROUP_A, GROUP_B, stride=50, height=30, steps=10)
        lengths = {len(p) for p in paths.values()}
        assert len(lengths) == 1

    def test_swing_group_lifts(self):
        paths = half_cycle_paths(GROUP_A, GROUP_B, stride=50, height=30, steps=10)
        # Schwung-Gruppe (A) muss in der Mitte angehoben sein
        for leg in GROUP_A:
            max_z = max(p[2] for p in paths[leg])
            assert max_z > 20

    def test_stance_group_stays_down(self):
        paths = half_cycle_paths(GROUP_A, GROUP_B, stride=50, height=30, steps=10)
        # Stand-Gruppe (B) bleibt am Boden (z=0)
        for leg in GROUP_B:
            for p in paths[leg]:
                assert p[2] == pytest.approx(0.0)

    def test_continuity_between_half_cycles(self):
        # Ende H1 == Start H2 (mit include_start)
        h1 = half_cycle_paths(GROUP_A, GROUP_B, stride=50, height=30,
                              steps=10, include_start=True)
        h2 = half_cycle_paths(GROUP_B, GROUP_A, stride=50, height=30,
                              steps=10, include_start=True)
        for leg in set(GROUP_A) | set(GROUP_B):
            assert h1[leg][-1] == pytest.approx(h2[leg][0], abs=1e-9)

    def test_direction_reverses(self):
        fwd = half_cycle_paths(GROUP_A, GROUP_B, stride=50, height=30,
                              steps=10, direction=1.0)
        bwd = half_cycle_paths(GROUP_A, GROUP_B, stride=50, height=30,
                              steps=10, direction=-1.0)
        # Schwung-Endpunkt in x ist gespiegelt
        leg = GROUP_A[0]
        assert fwd[leg][-1][0] == pytest.approx(-bwd[leg][-1][0])
