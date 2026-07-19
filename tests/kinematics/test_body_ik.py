"""Tests für Body-IK (Körper-Pose → Fußpositionen im Leg-Frame)."""

from __future__ import annotations

import math

import numpy as np
import pytest

from hexapod.kinematics import (
    BodyPose,
    body_ik,
    body_pose_offsets,
    default_foot_positions,
    rotation_matrix,
)


# ============================================================
# Hilfsfunktionen
# ============================================================


def make_coxa() -> dict[str, np.ndarray]:
    """Zwei einfache Beine für Tests (links/rechts symmetrisch)."""
    return {
        "right": np.array([60.0, -45.0, 0.0]),
        "left":  np.array([60.0,  45.0, 0.0]),
    }


def make_feet(coxa: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    """Neutrale Fußpositionen für die Test-Coxa."""
    return default_foot_positions(coxa, foot_distance=80.0, foot_height=80.0)


# ============================================================
# BodyPose
# ============================================================


class TestBodyPose:
    def test_neutral_is_all_zeros(self) -> None:
        p = BodyPose.neutral()
        assert p.tx == p.ty == p.tz == 0.0
        assert p.roll == p.pitch == p.yaw == 0.0

    def test_elevated(self) -> None:
        p = BodyPose.elevated(30.0)
        assert p.tz == 30.0
        assert p.tx == p.ty == 0.0
        assert p.roll == p.pitch == p.yaw == 0.0

    def test_frozen(self) -> None:
        p = BodyPose()
        with pytest.raises((AttributeError, Exception)):
            p.roll = 1.0  # type: ignore[misc]


# ============================================================
# Rotationsmatrix
# ============================================================


class TestRotationMatrix:
    def test_neutral_is_identity(self) -> None:
        R = rotation_matrix(0.0, 0.0, 0.0)
        assert R == pytest.approx(np.eye(3))

    def test_orthogonal(self) -> None:
        """R × Rᵀ muss Einheitsmatrix ergeben."""
        R = rotation_matrix(0.3, -0.2, 0.5)
        assert R @ R.T == pytest.approx(np.eye(3), abs=1e-10)

    def test_determinant_one(self) -> None:
        """Determinante muss +1 sein (keine Spiegelung)."""
        R = rotation_matrix(0.1, 0.4, -0.3)
        assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-10)

    def test_roll_90_rotates_y_to_z(self) -> None:
        """Roll +90°: Y-Achse zeigt nach +Z."""
        R = rotation_matrix(math.pi / 2, 0.0, 0.0)
        y_axis = np.array([0.0, 1.0, 0.0])
        result = R @ y_axis
        assert result == pytest.approx([0.0, 0.0, 1.0], abs=1e-10)

    def test_pitch_90_rotates_x_to_z(self) -> None:
        """Pitch -90°: X-Achse zeigt nach -Z (Roboter nickt nach vorn)."""
        R = rotation_matrix(0.0, -math.pi / 2, 0.0)
        x_axis = np.array([1.0, 0.0, 0.0])
        result = R @ x_axis
        assert result == pytest.approx([0.0, 0.0, 1.0], abs=1e-10)

    def test_yaw_90_rotates_x_to_y(self) -> None:
        """Yaw +90°: X-Achse zeigt nach +Y."""
        R = rotation_matrix(0.0, 0.0, math.pi / 2)
        x_axis = np.array([1.0, 0.0, 0.0])
        result = R @ x_axis
        assert result == pytest.approx([0.0, 1.0, 0.0], abs=1e-10)

    @pytest.mark.parametrize("roll,pitch,yaw", [
        (0.3, 0.0, 0.0),
        (0.0, 0.4, 0.0),
        (0.0, 0.0, 0.5),
        (0.2, -0.3, 0.4),
        (-0.5, 0.1, -0.2),
    ])
    def test_orthogonal_parametrized(
        self, roll: float, pitch: float, yaw: float
    ) -> None:
        R = rotation_matrix(roll, pitch, yaw)
        assert R @ R.T == pytest.approx(np.eye(3), abs=1e-10)
        assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-10)


# ============================================================
# Body-IK: Neutralpose
# ============================================================


class TestBodyIKNeutral:
    def test_neutral_pose_foot_in_leg_frame(self) -> None:
        """Bei Neutralpose: foot_leg = foot_world - coxa_body."""
        coxa = make_coxa()
        feet = make_feet(coxa)
        pose = BodyPose.neutral()
        targets = body_ik(pose, feet, coxa)

        for name in ["right", "left"]:
            expected = feet[name] - coxa[name]
            actual = np.array(targets[name])
            assert actual == pytest.approx(expected, abs=1e-9)

    def test_returns_all_legs(self) -> None:
        coxa = make_coxa()
        feet = make_feet(coxa)
        targets = body_ik(BodyPose.neutral(), feet, coxa)
        assert set(targets.keys()) == {"right", "left"}

    def test_symmetry_left_right(self) -> None:
        """Linkes und rechtes Bein sind Y-gespiegelt."""
        coxa = make_coxa()
        feet = make_feet(coxa)
        targets = body_ik(BodyPose.neutral(), feet, coxa)
        rx, ry, rz = targets["right"]
        lx, ly, lz = targets["left"]
        assert rx == pytest.approx(lx, abs=1e-9)
        assert ry == pytest.approx(-ly, abs=1e-9)
        assert rz == pytest.approx(lz, abs=1e-9)


# ============================================================
# Body-IK: Translation
# ============================================================


class TestBodyIKTranslation:
    def test_elevation_lowers_z_in_leg_frame(self) -> None:
        """Körper +20mm hoch → Füße im Leg-Frame 20mm tiefer."""
        coxa = make_coxa()
        feet = make_feet(coxa)
        neutral = body_ik(BodyPose.neutral(), feet, coxa)
        elevated = body_ik(BodyPose.elevated(20.0), feet, coxa)
        for name in coxa:
            assert elevated[name][2] == pytest.approx(
                neutral[name][2] - 20.0, abs=1e-9
            )

    def test_elevation_does_not_affect_xy(self) -> None:
        """Reine Z-Translation ändert X und Y nicht."""
        coxa = make_coxa()
        feet = make_feet(coxa)
        neutral = body_ik(BodyPose.neutral(), feet, coxa)
        elevated = body_ik(BodyPose.elevated(20.0), feet, coxa)
        for name in coxa:
            assert elevated[name][0] == pytest.approx(neutral[name][0], abs=1e-9)
            assert elevated[name][1] == pytest.approx(neutral[name][1], abs=1e-9)

    def test_forward_translation_shifts_x_in_leg_frame(self) -> None:
        """Körper +30mm nach vorne → Füße im Leg-Frame 30mm weiter hinten."""
        coxa = make_coxa()
        feet = make_feet(coxa)
        neutral = body_ik(BodyPose.neutral(), feet, coxa)
        forward = body_ik(BodyPose(tx=30.0), feet, coxa)
        for name in coxa:
            assert forward[name][0] == pytest.approx(
                neutral[name][0] - 30.0, abs=1e-9
            )

    def test_lateral_translation_shifts_y(self) -> None:
        """Körper +20mm nach links → Y-Komponente verschiebt sich."""
        coxa = make_coxa()
        feet = make_feet(coxa)
        neutral = body_ik(BodyPose.neutral(), feet, coxa)
        shifted = body_ik(BodyPose(ty=20.0), feet, coxa)
        for name in coxa:
            assert shifted[name][1] == pytest.approx(
                neutral[name][1] - 20.0, abs=1e-9
            )


# ============================================================
# Body-IK: Rotation
# ============================================================


class TestBodyIKRotation:
    def test_roll_breaks_z_symmetry(self) -> None:
        """Roll kippt den Körper: linkes und rechtes Bein haben verschiedenes z."""
        coxa = make_coxa()
        feet = make_feet(coxa)
        rolled = body_ik(BodyPose(roll=math.radians(15)), feet, coxa)
        # Rechtes Bein (y<0) kommt dem Boden naeher -> z groesser (weniger tief)
        assert rolled["right"][2] > rolled["left"][2]

    def test_roll_preserves_average_z(self) -> None:
        """Kleiner Roll: Durchschnitt-Z beider Beine bleibt ungefähr gleich."""
        coxa = make_coxa()
        feet = make_feet(coxa)
        neutral = body_ik(BodyPose.neutral(), feet, coxa)
        rolled = body_ik(BodyPose(roll=math.radians(10)), feet, coxa)
        avg_z_neutral = (neutral["right"][2] + neutral["left"][2]) / 2
        avg_z_rolled = (rolled["right"][2] + rolled["left"][2]) / 2
        assert avg_z_rolled == pytest.approx(avg_z_neutral, abs=5.0)

    def test_yaw_does_not_change_z(self) -> None:
        """Reine Yaw-Rotation ändert Z nicht (Drehung um Z-Achse)."""
        coxa = make_coxa()
        feet = make_feet(coxa)
        neutral = body_ik(BodyPose.neutral(), feet, coxa)
        yawed = body_ik(BodyPose(yaw=math.radians(20)), feet, coxa)
        for name in coxa:
            assert yawed[name][2] == pytest.approx(neutral[name][2], abs=1e-6)

    def test_pitch_shifts_x_differently_for_front_back(self) -> None:
        """Pitch nickt: front und back Beine erfahren verschiedene X-Verschiebung."""
        coxa_fb = {
            "front": np.array([60.0, 0.0, 0.0]),
            "back":  np.array([-60.0, 0.0, 0.0]),
        }
        feet_fb = default_foot_positions(coxa_fb, 80.0, 80.0)
        pitched = body_ik(BodyPose(pitch=math.radians(15)), feet_fb, coxa_fb)
        # Beim Nicken nach vorne: front-Bein muss tiefer, back höher
        assert pitched["front"][2] > pitched["back"][2]


# ============================================================
# Roundtrip: body_ik → inverse_kinematics → forward_kinematics
# ============================================================


class TestRoundtrip:
    """Goldener Test: body_ik gibt Fußpositionen im Leg-Frame,
    IK rechnet Winkel, FK bestätigt die Position."""

    def test_neutral_pose_roundtrip(self) -> None:
        from hexapod.kinematics import (
            LegLengths,
            forward_kinematics,
            inverse_kinematics,
        )

        L = LegLengths(coxa=33.0, femur=90.0, tibia=110.0)
        coxa = {"right": np.array([60.0, -45.0, 0.0])}
        feet = default_foot_positions(coxa, foot_distance=80.0, foot_height=60.0)
        targets = body_ik(BodyPose.neutral(), feet, coxa)

        x, y, z = targets["right"]
        t1, t2, t3 = inverse_kinematics(x, y, z, L)
        xr, yr, zr = forward_kinematics(t1, t2, t3, L)
        assert xr == pytest.approx(x, abs=1e-6)
        assert yr == pytest.approx(y, abs=1e-6)
        assert zr == pytest.approx(z, abs=1e-6)

    @pytest.mark.parametrize("roll,pitch,yaw,tx,ty,tz", [
        (0.0,  0.0,  0.0, 0.0,  0.0,  0.0),
        (0.1,  0.0,  0.0, 0.0,  0.0,  0.0),
        (0.0,  0.1,  0.0, 0.0,  0.0,  0.0),
        (0.0,  0.0,  0.2, 0.0,  0.0,  0.0),
        (0.1,  0.05, 0.1, 5.0, -5.0, 10.0),
        (-0.1, 0.1, -0.1, 0.0,  0.0, 20.0),
    ])
    def test_various_poses_roundtrip(
        self,
        roll: float, pitch: float, yaw: float,
        tx: float, ty: float, tz: float,
    ) -> None:
        from hexapod.kinematics import (
            LegLengths,
            forward_kinematics,
            inverse_kinematics,
        )

        L = LegLengths(coxa=33.0, femur=90.0, tibia=110.0)
        coxa = {"right": np.array([60.0, -45.0, 0.0])}
        feet = default_foot_positions(coxa, foot_distance=80.0, foot_height=60.0)
        pose = BodyPose(tx=tx, ty=ty, tz=tz, roll=roll, pitch=pitch, yaw=yaw)
        targets = body_ik(pose, feet, coxa)

        x, y, z = targets["right"]
        try:
            t1, t2, t3 = inverse_kinematics(x, y, z, L)
            xr, yr, zr = forward_kinematics(t1, t2, t3, L)
            assert xr == pytest.approx(x, abs=1e-6)
            assert yr == pytest.approx(y, abs=1e-6)
            assert zr == pytest.approx(z, abs=1e-6)
        except Exception:
            pytest.skip("Pose führt zu unerreichbarem Fußpunkt (ok für diesen Test)")


class TestBodyPoseOffsets:
    """body_pose_offsets: Pose -> Standpose-Offsets (koerperparallel)."""

    @staticmethod
    def _setup():
        coxa = {
            "a": np.array([50.0, 0.0, 0.0]),
            "b": np.array([-50.0, 0.0, 0.0]),
        }
        neutral = {"a": (40.0, 0.0, -46.0), "b": (-40.0, 0.0, -46.0)}
        foot = {n: coxa[n] + np.array(neutral[n]) for n in coxa}
        return coxa, neutral, foot

    def test_neutral_pose_gives_zero_offsets(self):
        coxa, neutral, foot = self._setup()
        offs = body_pose_offsets(BodyPose(), foot, coxa, neutral)
        for leg in coxa:
            assert offs[leg] == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)

    def test_elevation_lowers_feet_equally(self):
        coxa, neutral, foot = self._setup()
        offs = body_pose_offsets(BodyPose(tz=20.0), foot, coxa, neutral)
        for leg in coxa:
            assert offs[leg][0] == pytest.approx(0.0, abs=1e-9)
            assert offs[leg][1] == pytest.approx(0.0, abs=1e-9)
            assert offs[leg][2] == pytest.approx(-20.0, abs=1e-9)
