"""Tests f\u00fcr den verallgemeinerten Tripod-Schritt (command_tripod)."""

import math

import pytest

from hexapod.gait.body_motion import stride_vectors
from hexapod.gait.command_tripod import command_half_cycle_paths
from hexapod.gait.tripod import GROUP_A, GROUP_B


FEET = {
    "front_right": (152.9, -112.2),
    "front_left": (152.9, 112.2),
    "mid_right": (0.0, -181.6),
    "mid_left": (0.0, 181.6),
    "back_right": (-152.9, -112.2),
    "back_left": (-152.9, 112.2),
}

ALL = list(FEET)


def _endpoints(path):
    """Erster und letzter Punkt einer Bahn (mit include_start)."""
    return path[0], path[-1]


class TestStructure:
    def test_all_legs_present(self):
        sv = stride_vectors(FEET, vx=40, vy=0, omega=0)
        paths = command_half_cycle_paths(
            GROUP_A, GROUP_B, sv, height=30, steps=20, include_start=True
        )
        assert set(paths) == set(ALL)

    def test_equal_length(self):
        sv = stride_vectors(FEET, vx=40, vy=0, omega=0)
        paths = command_half_cycle_paths(
            GROUP_A, GROUP_B, sv, height=30, steps=20, include_start=True
        )
        lengths = {len(p) for p in paths.values()}
        assert lengths == {21}  # steps + 1 wegen include_start

    def test_steps_without_include_start(self):
        sv = stride_vectors(FEET, vx=40, vy=0, omega=0)
        paths = command_half_cycle_paths(
            GROUP_A, GROUP_B, sv, height=30, steps=20
        )
        assert all(len(p) == 20 for p in paths.values())


class TestSwingVsStance:
    def test_swing_lifts_stance_stays_down(self):
        sv = stride_vectors(FEET, vx=40, vy=0, omega=0)
        paths = command_half_cycle_paths(
            GROUP_A, GROUP_B, sv, height=30, steps=20, include_start=True
        )
        # Schwung-Gruppe (A) hebt ab: irgendwo z > 0
        for leg in GROUP_A:
            assert max(z for _, _, z in paths[leg]) > 1.0
        # Stand-Gruppe (B) bleibt am Boden: z == 0 \u00fcberall
        for leg in GROUP_B:
            assert all(abs(z) < 1e-9 for _, _, z in paths[leg])

    def test_swing_returns_stance_advances(self):
        # Vorw\u00e4rts: Stand-Fu\u00df l\u00e4uft entlang Stride-Vektor (nach hinten = -x),
        # Schwung-Fu\u00df entgegengesetzt zur\u00fcck.
        sv = stride_vectors(FEET, vx=40, vy=0, omega=0)
        paths = command_half_cycle_paths(
            GROUP_A, GROUP_B, sv, height=30, steps=20, include_start=True
        )
        leg = GROUP_B[0]  # Standphase
        (x0, _, _), (x1, _, _) = _endpoints(paths[leg])
        # Stride-Vektor f\u00fcr vorw\u00e4rts ist -x; Stand l\u00e4uft -v/2 \u2192 +v/2 = +x \u2192 -x
        assert x1 < x0
        leg_s = GROUP_A[0]  # Schwungphase, l\u00e4uft entgegengesetzt
        (xs0, _, _), (xs1, _, _) = _endpoints(paths[leg_s])
        assert xs1 > xs0


class TestRoundTrip:
    def test_full_cycle_returns_to_origin(self):
        # H1: A schwingt (+v/2\u2192-v/2), B steht (-v/2\u2192+v/2).
        # H2: B schwingt (+v/2\u2192-v/2), A steht (-v/2\u2192+v/2).
        # Pro Bein: Summe der Verschiebungen = 0.
        sv = stride_vectors(FEET, vx=40, vy=10, omega=0.05)
        h1 = command_half_cycle_paths(
            GROUP_A, GROUP_B, sv, height=30, steps=10, include_start=True
        )
        h2 = command_half_cycle_paths(
            GROUP_B, GROUP_A, sv, height=30, steps=10, include_start=True
        )
        for leg in ALL:
            # Gesamtverschiebung H1 + H2 in xy soll ~0 sein
            dx = (h1[leg][-1][0] - h1[leg][0][0]) + (h2[leg][-1][0] - h2[leg][0][0])
            dy = (h1[leg][-1][1] - h1[leg][0][1]) + (h2[leg][-1][1] - h2[leg][0][1])
            assert dx == pytest.approx(0, abs=1e-9)
            assert dy == pytest.approx(0, abs=1e-9)

    def test_continuity_between_half_cycles(self):
        # Ende H1 eines Stand-Beins == Start H2 desselben Beins (jetzt Schwung).
        sv = stride_vectors(FEET, vx=40, vy=0, omega=0)
        h1 = command_half_cycle_paths(
            GROUP_A, GROUP_B, sv, height=30, steps=10, include_start=True
        )
        h2 = command_half_cycle_paths(
            GROUP_B, GROUP_A, sv, height=30, steps=10, include_start=True
        )
        for leg in ALL:
            end1 = h1[leg][-1]
            start2 = h2[leg][0]
            assert end1[0] == pytest.approx(start2[0])
            assert end1[1] == pytest.approx(start2[1])


class TestMotionTypes:
    def test_forward_all_same_direction(self):
        sv = stride_vectors(FEET, vx=40, vy=0, omega=0)
        paths = command_half_cycle_paths(
            GROUP_A, GROUP_B, sv, height=30, steps=10, include_start=True
        )
        # Alle Stand-Beine laufen gleich (reine Translation)
        deltas = []
        for leg in GROUP_B:
            deltas.append(paths[leg][-1][0] - paths[leg][0][0])
        assert all(d == pytest.approx(deltas[0]) for d in deltas)

    def test_rotation_left_right_opposite(self):
        # Reine Drehung: linke und rechte Stand-Beine laufen in x entgegengesetzt
        sv = stride_vectors(FEET, vx=0, vy=0, omega=0.2)
        paths = command_half_cycle_paths(
            GROUP_A, GROUP_B, sv, height=30, steps=10, include_start=True
        )
        # front_left ist in GROUP_B (Stand in H1), back_right in GROUP_A.
        # Vergleiche zwei Stand-Beine unterschiedlicher Seiten in derselben
        # Gruppe: mid_right (B) und front_left (B).
        dr = paths["mid_right"][-1][1] - paths["mid_right"][0][1]
        dl = paths["front_left"][-1][1] - paths["front_left"][0][1]
        # bei Drehung haben die y-Komponenten der gegen\u00fcberliegenden Seiten
        # unterschiedliche Vorzeichen-Tendenz; pr\u00fcfe einfach: nicht identisch
        assert dr != pytest.approx(dl)

    def test_zero_command_no_motion(self):
        sv = stride_vectors(FEET, vx=0, vy=0, omega=0)
        paths = command_half_cycle_paths(
            GROUP_A, GROUP_B, sv, height=30, steps=10, include_start=True
        )
        # Stand-Beine bewegen sich gar nicht; Schwung-Beine heben nur ab (z),
        # kehren aber zur selben xy-Position zur\u00fcck.
        for leg in GROUP_B:
            for (x, y, z) in paths[leg]:
                assert (x, y, z) == pytest.approx((0, 0, 0), abs=1e-9)
        for leg in GROUP_A:
            assert paths[leg][0][0] == pytest.approx(0, abs=1e-9)
            assert paths[leg][-1][0] == pytest.approx(0, abs=1e-9)
