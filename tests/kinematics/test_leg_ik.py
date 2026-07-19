"""Tests für die Bein-Vorwärts- und Rückwärtskinematik."""

from __future__ import annotations

import math
from itertools import product

import pytest

from hexapod.kinematics import (
    LegLengths,
    UnreachableError,
    forward_kinematics,
    inverse_kinematics,
)


# Standard-Geometrie für den Freenove Big Hexapod
L = LegLengths(coxa=33.0, femur=90.0, tibia=110.0)


# ============================================================
# LegLengths-Validierung
# ============================================================


class TestLegLengths:
    def test_valid_construction(self) -> None:
        lengths = LegLengths(coxa=33.0, femur=90.0, tibia=110.0)
        assert lengths.coxa == 33.0
        assert lengths.femur == 90.0
        assert lengths.tibia == 110.0

    def test_frozen(self) -> None:
        """LegLengths ist unveränderlich."""
        lengths = LegLengths(coxa=33.0, femur=90.0, tibia=110.0)
        with pytest.raises((AttributeError, Exception)):
            lengths.coxa = 50.0  # type: ignore[misc]

    @pytest.mark.parametrize("field", ["coxa", "femur", "tibia"])
    def test_rejects_zero_or_negative(self, field: str) -> None:
        kwargs = {"coxa": 33.0, "femur": 90.0, "tibia": 110.0}
        kwargs[field] = 0.0
        with pytest.raises(ValueError, match=field):
            LegLengths(**kwargs)

        kwargs[field] = -10.0
        with pytest.raises(ValueError, match=field):
            LegLengths(**kwargs)


# ============================================================
# Forward Kinematics: Bekannte Posen
# ============================================================


class TestForwardKinematicsKnownPoses:
    """Wenn alle Winkel = 0, sollte das Bein gestreckt auf +X liegen."""

    def test_zero_pose_stretched_along_x(self) -> None:
        x, y, z = forward_kinematics(0.0, 0.0, 0.0, L)
        assert x == pytest.approx(L.coxa + L.femur + L.tibia)
        assert y == pytest.approx(0.0)
        assert z == pytest.approx(0.0)

    def test_coxa_90deg_rotates_to_y(self) -> None:
        """θ₁ = 90° dreht das ausgestreckte Bein auf die +Y-Achse."""
        x, y, z = forward_kinematics(math.pi / 2, 0.0, 0.0, L)
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(L.coxa + L.femur + L.tibia)
        assert z == pytest.approx(0.0)

    def test_femur_up_90deg_with_knee_stretched(self) -> None:
        """θ₂ = 90°, θ₃ = 0: Femur und Tibia zeigen senkrecht nach oben."""
        x, y, z = forward_kinematics(0.0, math.pi / 2, 0.0, L)
        assert x == pytest.approx(L.coxa)
        assert y == pytest.approx(0.0)
        assert z == pytest.approx(L.femur + L.tibia)

    def test_knee_fully_bent(self) -> None:
        """θ₂ = 0, θ₃ = 180°: Tibia faltet sich zurück über den Femur.

        Tibia liegt absolut bei (theta2 - theta3) = -180° (entlang -X).
        Fußspitze: Coxa + Femur - Tibia = 33 + 90 - 110 = 13.
        """
        x, y, z = forward_kinematics(0.0, 0.0, math.pi, L)
        assert x == pytest.approx(L.coxa + L.femur - L.tibia)
        assert y == pytest.approx(0.0)
        assert z == pytest.approx(0.0, abs=1e-9)


# ============================================================
# Inverse Kinematics: Bekannte Posen
# ============================================================


class TestInverseKinematicsKnownPoses:
    def test_stretched_along_x(self) -> None:
        t1, t2, t3 = inverse_kinematics(L.coxa + L.femur + L.tibia, 0.0, 0.0, L)
        assert t1 == pytest.approx(0.0)
        assert t2 == pytest.approx(0.0, abs=1e-9)
        assert t3 == pytest.approx(0.0, abs=1e-9)

    def test_negative_x_means_coxa_180(self) -> None:
        """Punkt direkt hinter der Coxa: θ₁ ≈ ±π, Bein wieder gestreckt."""
        t1, _, _ = inverse_kinematics(-(L.coxa + L.femur + L.tibia), 0.0, 0.0, L)
        assert abs(t1) == pytest.approx(math.pi)


# ============================================================
# Roundtrip: FK → IK → gleiche Winkel
# ============================================================


# Wir testen über einen Bereich realistischer Gelenkwinkel.
# θ₁ schwingt das Bein in seinem normalen Arbeitsbereich.
# θ₂ hebt das Bein zwischen "unten" und "oben".
# θ₃ beugt das Knie zwischen "gestreckt" und "stark gebeugt".
#
# Diese Bereiche sind absichtlich konservativ — wir vermeiden
# Singularitäten (z.B. θ3 = 0 exakt führt zu d = max_reach,
# wo Float-Toleranzen knapp werden).
_T1_VALUES = [-1.2, -0.7, -0.3, 0.0, 0.3, 0.7, 1.2]
_T2_VALUES = [-0.6, -0.3, 0.0, 0.3, 0.6, 0.9]
_T3_VALUES = [0.1, 0.4, 0.7, 1.0, 1.3]


def _angles_grid() -> list[tuple[float, float, float]]:
    return list(product(_T1_VALUES, _T2_VALUES, _T3_VALUES))


class TestRoundtrip:
    """Der goldene Test: FK auf Winkel, dann IK auf Punkt zurück,
    muss wieder zum Ausgangs-Tripel führen."""

    @pytest.mark.parametrize("t1, t2, t3", _angles_grid())
    def test_fk_then_ik_returns_same_angles(
        self, t1: float, t2: float, t3: float
    ) -> None:
        x, y, z = forward_kinematics(t1, t2, t3, L)
        t1r, t2r, t3r = inverse_kinematics(x, y, z, L)
        assert t1r == pytest.approx(t1, abs=1e-6)
        assert t2r == pytest.approx(t2, abs=1e-6)
        assert t3r == pytest.approx(t3, abs=1e-6)

    @pytest.mark.parametrize("t1, t2, t3", _angles_grid())
    def test_ik_then_fk_returns_same_position(
        self, t1: float, t2: float, t3: float
    ) -> None:
        """Symmetrischer Test: nimm einen Punkt aus FK, lass IK→FK das
        Tripel verarbeiten, und prüfe dass die Position wieder stimmt."""
        x, y, z = forward_kinematics(t1, t2, t3, L)
        t1r, t2r, t3r = inverse_kinematics(x, y, z, L)
        xr, yr, zr = forward_kinematics(t1r, t2r, t3r, L)
        assert xr == pytest.approx(x, abs=1e-6)
        assert yr == pytest.approx(y, abs=1e-6)
        assert zr == pytest.approx(z, abs=1e-6)


# ============================================================
# Reichweiten-Grenzen und Unreachable-Fehler
# ============================================================


class TestUnreachable:
    def test_far_point_raises(self) -> None:
        """Punkt weit jenseits der maximalen Reichweite."""
        # max_reach ab Coxa-Drehpunkt = L1 + L2 + L3 = 233.
        # Ein Punkt bei x=500 ist klar unerreichbar.
        with pytest.raises(UnreachableError, match="zu weit"):
            inverse_kinematics(500.0, 0.0, 0.0, L)

    def test_near_point_raises(self) -> None:
        """Punkt zu nah am Coxa-Drehpunkt für das Femur-Tibia-Dreieck."""
        # min_reach (vom Femur-Gelenk aus) = |L2 - L3| = 20.
        # Wenn d < 20, geht's nicht.
        # Wir wählen einen Punkt direkt über dem Coxa-Drehpunkt:
        # r=0, also r_eff = -33, d = sqrt(33²+0²) = 33 (geht gerade).
        # Stattdessen: nah am Femur-Gelenk → x ≈ L1, z klein.
        with pytest.raises(UnreachableError):
            inverse_kinematics(L.coxa, 0.0, 5.0, L)  # d ≈ 5, unter min_reach=20

    def test_just_at_max_reach_works(self) -> None:
        """Punkt exakt auf der maximalen Reichweite ist erreichbar."""
        max_reach = L.coxa + L.femur + L.tibia
        t1, t2, t3 = inverse_kinematics(max_reach, 0.0, 0.0, L)
        assert t1 == pytest.approx(0.0)
        assert t2 == pytest.approx(0.0, abs=1e-6)
        assert t3 == pytest.approx(0.0, abs=1e-6)


# ============================================================
# Spezialfall: Punkt direkt unter der Coxa
# ============================================================


class TestVerticalDownPosition:
    """Ein häufiger Anwendungsfall: Bein steht senkrecht nach unten,
    Fuß ist nicht ganz unter dem Coxa, sondern leicht außen."""

    def test_foot_below_and_outside(self) -> None:
        # 120 mm außen, 80 mm unter Coxa → realistische Standpose
        t1, t2, t3 = inverse_kinematics(120.0, 0.0, -80.0, L)

        # θ₁ muss 0 sein, da y=0
        assert t1 == pytest.approx(0.0)

        # Wenn wir mit den Winkeln FK rechnen, müssen wir zurückkommen
        x, y, z = forward_kinematics(t1, t2, t3, L)
        assert x == pytest.approx(120.0, abs=1e-6)
        assert y == pytest.approx(0.0, abs=1e-6)
        assert z == pytest.approx(-80.0, abs=1e-6)
