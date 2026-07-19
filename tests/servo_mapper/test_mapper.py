"""Tests für die Servo-Mapping-Funktionen."""

from __future__ import annotations

import math

import pytest

from hexapod.servo_mapper import (
    OutOfRangeError,
    ServoMapping,
    angle_to_us,
    us_to_angle,
)
from hexapod.servo_mapper.mapper import MAX_ANGLE_RAD


# ============================================================
# Konstruktor-Validierung
# ============================================================


class TestConstruction:
    def test_valid_default(self) -> None:
        m = ServoMapping(center_us=1500.0, range_us=800.0, direction=1,
                         min_us=600.0, max_us=2400.0)
        assert m.center_us == 1500.0

    def test_frozen(self) -> None:
        m = ServoMapping(center_us=1500.0, range_us=800.0, direction=1,
                         min_us=600.0, max_us=2400.0)
        with pytest.raises((AttributeError, Exception)):
            m.center_us = 1600.0  # type: ignore[misc]

    @pytest.mark.parametrize("bad_dir", [0, 2, -2, 5, 100])
    def test_rejects_invalid_direction(self, bad_dir: int) -> None:
        with pytest.raises(ValueError, match="direction"):
            ServoMapping(center_us=1500.0, range_us=800.0, direction=bad_dir,
                         min_us=600.0, max_us=2400.0)

    @pytest.mark.parametrize("bad_range", [0.0, -1.0, -100.0])
    def test_rejects_nonpositive_range(self, bad_range: float) -> None:
        with pytest.raises(ValueError, match="range_us"):
            ServoMapping(center_us=1500.0, range_us=bad_range, direction=1,
                         min_us=600.0, max_us=2400.0)

    def test_rejects_min_geq_max(self) -> None:
        with pytest.raises(ValueError, match="min_us"):
            ServoMapping(center_us=1500.0, range_us=800.0, direction=1,
                         min_us=2400.0, max_us=600.0)

    def test_rejects_center_outside_min_max(self) -> None:
        with pytest.raises(ValueError, match="center_us"):
            ServoMapping(center_us=3000.0, range_us=800.0, direction=1,
                         min_us=600.0, max_us=2400.0)


# ============================================================
# Bekannte Punkte
# ============================================================


class TestKnownValues:
    """Konkrete bekannte Werte für die typische Kalibrierung
    center=1500, range=800."""

    @pytest.fixture
    def m(self) -> ServoMapping:
        return ServoMapping(center_us=1500.0, range_us=800.0, direction=1,
                            min_us=600.0, max_us=2400.0)

    def test_zero_angle_is_center(self, m: ServoMapping) -> None:
        assert m.angle_to_us(0.0) == pytest.approx(1500.0)

    def test_max_angle_is_range_above_center(self, m: ServoMapping) -> None:
        """Bei +π/2 (= 90°) sind wir genau range_us über Center."""
        assert m.angle_to_us(MAX_ANGLE_RAD) == pytest.approx(2300.0)

    def test_min_angle_is_range_below_center(self, m: ServoMapping) -> None:
        """Bei -π/2 sind wir genau range_us unter Center."""
        assert m.angle_to_us(-MAX_ANGLE_RAD) == pytest.approx(700.0)

    def test_45deg_is_half_range_above_center(self, m: ServoMapping) -> None:
        assert m.angle_to_us(math.radians(45)) == pytest.approx(1900.0)

    def test_45deg_negative(self, m: ServoMapping) -> None:
        assert m.angle_to_us(math.radians(-45)) == pytest.approx(1100.0)


# ============================================================
# Direction = -1 (gespiegelter Einbau)
# ============================================================


class TestMirroredServo:
    @pytest.fixture
    def m(self) -> ServoMapping:
        return ServoMapping(center_us=1500.0, range_us=800.0, direction=-1,
                            min_us=600.0, max_us=2400.0)

    def test_zero_still_center(self, m: ServoMapping) -> None:
        assert m.angle_to_us(0.0) == pytest.approx(1500.0)

    def test_positive_angle_yields_lower_us(self, m: ServoMapping) -> None:
        """Bei direction=-1 ergibt +45° eine niedrigere Pulsweite."""
        assert m.angle_to_us(math.radians(45)) == pytest.approx(1100.0)

    def test_negative_angle_yields_higher_us(self, m: ServoMapping) -> None:
        assert m.angle_to_us(math.radians(-45)) == pytest.approx(1900.0)


# ============================================================
# Out-of-range Behavior
# ============================================================


class TestOutOfRange:
    @pytest.fixture
    def m(self) -> ServoMapping:
        return ServoMapping(center_us=1500.0, range_us=800.0, direction=1,
                            min_us=600.0, max_us=2400.0)

    def test_above_max_raises_by_default(self, m: ServoMapping) -> None:
        with pytest.raises(OutOfRangeError, match="außerhalb"):
            m.angle_to_us(math.radians(150))  # weit jenseits

    def test_below_min_raises_by_default(self, m: ServoMapping) -> None:
        with pytest.raises(OutOfRangeError, match="außerhalb"):
            m.angle_to_us(math.radians(-150))

    def test_above_max_clips_when_requested(self, m: ServoMapping) -> None:
        assert m.angle_to_us(math.radians(150), clip=True) == pytest.approx(2400.0)

    def test_below_min_clips_when_requested(self, m: ServoMapping) -> None:
        assert m.angle_to_us(math.radians(-150), clip=True) == pytest.approx(600.0)

    def test_just_inside_max_works(self, m: ServoMapping) -> None:
        # max_us=2400, center=1500, range=800 -> max ist bei
        # angle = (2400-1500) / 800 * π/2 = 1.125 * π/2 = ~1.767 rad
        max_reachable_rad = (2400.0 - 1500.0) / 800.0 * MAX_ANGLE_RAD
        # Knapp drinnen
        us = m.angle_to_us(max_reachable_rad - 1e-9)
        assert us == pytest.approx(2400.0, abs=1e-6)


# ============================================================
# Roundtrip: angle_to_us → us_to_angle = identity
# ============================================================


# Eine Auswahl realistischer Kalibrierungen für die Robustheit
_MAPPINGS = [
    ServoMapping(center_us=1500.0, range_us=800.0, direction=1,
                 min_us=600.0, max_us=2400.0),
    ServoMapping(center_us=1500.0, range_us=800.0, direction=-1,
                 min_us=600.0, max_us=2400.0),
    ServoMapping(center_us=1450.0, range_us=750.0, direction=1,
                 min_us=550.0, max_us=2400.0),
    ServoMapping(center_us=1550.0, range_us=850.0, direction=-1,
                 min_us=600.0, max_us=2500.0),
]

# Realistische Winkel innerhalb [-60°, +60°] — typischer Bein-Arbeitsbereich
_ANGLES_DEG = [-60, -45, -30, -15, -1, 0, 1, 15, 30, 45, 60]


class TestRoundtrip:
    @pytest.mark.parametrize("m", _MAPPINGS)
    @pytest.mark.parametrize("angle_deg", _ANGLES_DEG)
    def test_angle_us_angle(self, m: ServoMapping, angle_deg: float) -> None:
        a = math.radians(angle_deg)
        us = m.angle_to_us(a)
        a_back = m.us_to_angle(us)
        assert a_back == pytest.approx(a, abs=1e-12)

    @pytest.mark.parametrize("m", _MAPPINGS)
    @pytest.mark.parametrize("us", [800.0, 1100.0, 1500.0, 1700.0, 2200.0])
    def test_us_angle_us(self, m: ServoMapping, us: float) -> None:
        # Übergebe nur Werte im erreichbaren Bereich (zwischen min und max)
        if not (m.min_us <= us <= m.max_us):
            pytest.skip(f"us {us} außerhalb [{m.min_us}, {m.max_us}]")
        a = m.us_to_angle(us)
        us_back = m.angle_to_us(a)
        assert us_back == pytest.approx(us, abs=1e-9)


# ============================================================
# Linearität: doppelter Winkel -> doppelte Abweichung vom Center
# ============================================================


class TestLinearity:
    @pytest.fixture
    def m(self) -> ServoMapping:
        return ServoMapping(center_us=1500.0, range_us=800.0, direction=1,
                            min_us=600.0, max_us=2400.0)

    def test_doubling_angle_doubles_offset(self, m: ServoMapping) -> None:
        a = math.radians(20)
        offset_a = m.angle_to_us(a) - m.center_us
        offset_2a = m.angle_to_us(2 * a) - m.center_us
        assert offset_2a == pytest.approx(2 * offset_a)

    def test_sign_symmetry(self, m: ServoMapping) -> None:
        """+a und -a müssen symmetrisch um Center liegen."""
        a = math.radians(33)
        us_pos = m.angle_to_us(a)
        us_neg = m.angle_to_us(-a)
        assert (us_pos - m.center_us) == pytest.approx(m.center_us - us_neg)


# ============================================================
# Funktions-Wrapper (stateless)
# ============================================================


class TestFunctionalAPI:
    def test_angle_to_us(self) -> None:
        us = angle_to_us(math.radians(45), center_us=1500.0, range_us=800.0,
                         direction=1, min_us=600.0, max_us=2400.0)
        assert us == pytest.approx(1900.0)

    def test_us_to_angle(self) -> None:
        a = us_to_angle(1900.0, center_us=1500.0, range_us=800.0, direction=1)
        assert math.degrees(a) == pytest.approx(45.0)

    def test_mirror(self) -> None:
        us = angle_to_us(math.radians(45), center_us=1500.0, range_us=800.0,
                        direction=-1, min_us=600.0, max_us=2400.0)
        assert us == pytest.approx(1100.0)
