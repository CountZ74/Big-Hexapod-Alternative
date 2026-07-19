"""Tests für die verallgemeinerte Körperbewegung."""

import math

import pytest

from hexapod.gait.body_motion import (
    foot_ground_vector,
    stride_vectors,
    clamp_command,
)


FEET = {
    "front_right": (152.9, -112.2),
    "front_left": (152.9, 112.2),
    "mid_right": (0.0, -181.6),
    "mid_left": (0.0, 181.6),
    "back_right": (-152.9, -112.2),
    "back_left": (-152.9, 112.2),
}


class TestTranslation:
    def test_forward_all_parallel(self):
        v = stride_vectors(FEET, vx=40, vy=0, omega=0)
        # Alle Füße bewegen sich identisch -40 in x
        for dx, dy in v.values():
            assert dx == pytest.approx(-40)
            assert dy == pytest.approx(0, abs=1e-9)

    def test_sideways_all_parallel(self):
        v = stride_vectors(FEET, vx=0, vy=30, omega=0)
        for dx, dy in v.values():
            assert dx == pytest.approx(0, abs=1e-9)
            assert dy == pytest.approx(-30)

    def test_backward(self):
        v = stride_vectors(FEET, vx=-40, vy=0, omega=0)
        for dx, dy in v.values():
            assert dx == pytest.approx(40)


class TestRotation:
    def test_pure_rotation_tangential(self):
        v = stride_vectors(FEET, vx=0, vy=0, omega=0.2)
        # Rechte Füße (negatives y) bewegen sich anders als linke
        # Symmetrie: front_right und front_left haben entgegengesetztes dx
        assert v["front_right"][0] == pytest.approx(-v["front_left"][0])

    def test_rotation_magnitude_scales_with_radius(self):
        v = stride_vectors(FEET, vx=0, vy=0, omega=0.2)
        # Eckbeine weiter außen -> größerer Betrag als Mittelbeine
        corner = math.hypot(*v["front_right"])
        middle = math.hypot(*v["mid_right"])
        assert corner > middle

    def test_zero_command_zero_motion(self):
        v = stride_vectors(FEET, vx=0, vy=0, omega=0)
        for dx, dy in v.values():
            assert dx == pytest.approx(0, abs=1e-9)
            assert dy == pytest.approx(0, abs=1e-9)


class TestMixed:
    def test_curve_asymmetric(self):
        # Vorwärts + Drehung: linke und rechte Seite unterschiedlich weit
        v = stride_vectors(FEET, vx=40, vy=0, omega=0.1)
        right = abs(v["front_right"][0])
        left = abs(v["front_left"][0])
        assert right != left

    def test_superposition(self):
        # Mischung = Summe der Einzelbewegungen
        trans = stride_vectors(FEET, vx=40, vy=0, omega=0)
        rot = stride_vectors(FEET, vx=0, vy=0, omega=0.1)
        mix = stride_vectors(FEET, vx=40, vy=0, omega=0.1)
        for name in FEET:
            assert mix[name][0] == pytest.approx(trans[name][0] + rot[name][0])
            assert mix[name][1] == pytest.approx(trans[name][1] + rot[name][1])


class TestClamp:
    def test_translation_clamped(self):
        vx, vy, om = clamp_command(
            100, 0, 0, max_translation=40, max_rotation=0.3, foot_radius=190
        )
        assert math.hypot(vx, vy) == pytest.approx(40)

    def test_rotation_clamped(self):
        vx, vy, om = clamp_command(
            0, 0, 1.0, max_translation=40, max_rotation=0.3, foot_radius=190
        )
        assert om == pytest.approx(0.3)

    def test_within_limits_unchanged(self):
        vx, vy, om = clamp_command(
            20, 0, 0.1, max_translation=40, max_rotation=0.3, foot_radius=190
        )
        assert vx == pytest.approx(20)
        assert om == pytest.approx(0.1)

    def test_direction_preserved(self):
        # Skalierung erhält die Richtung
        vx, vy, om = clamp_command(
            0, 100, 0, max_translation=40, max_rotation=0.3, foot_radius=190
        )
        assert vx == pytest.approx(0)
        assert vy == pytest.approx(40)
