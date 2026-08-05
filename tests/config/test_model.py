"""Tests für das Pydantic-Datenmodell."""

from __future__ import annotations

import pytest
from pydantic import ValidationError

from hexapod.config import (
    BodyConfig,
    CameraAxis,
    CameraServoConfig,
    MaestroBus,
    Joint,
    LegConfig,
    LegGeometry,
    LegRow,
    LegServoConfig,
    LegSide,
    RobotConfig,
)


# ============================================================
# Einfache Feld-Validierungen
# ============================================================


class TestMaestroBus:
    def test_defaults(self) -> None:
        d = MaestroBus()
        assert d.type == "maestro"
        assert d.port == "/dev/maestro_cmd"
        assert d.num_channels == 24

    def test_rejects_unknown_type(self) -> None:
        with pytest.raises(ValidationError):
            MaestroBus(type="something_else")  # type: ignore[arg-type]

    def test_rejects_zero_channels(self) -> None:
        with pytest.raises(ValidationError):
            MaestroBus(num_channels=0)

    def test_rejects_negative_timeout(self) -> None:
        with pytest.raises(ValidationError):
            MaestroBus(timeout=-1.0)

    def test_rejects_extra_fields(self) -> None:
        with pytest.raises(ValidationError, match="extra"):
            MaestroBus(typo_field="oops")  # type: ignore[call-arg]


class TestLegGeometry:
    def test_valid(self) -> None:
        g = LegGeometry(coxa_length=33.0, femur_length=90.0, tibia_length=110.0)
        assert g.coxa_length == 33.0

    @pytest.mark.parametrize("field", ["coxa_length", "femur_length", "tibia_length"])
    def test_rejects_zero_or_negative(self, field: str) -> None:
        kwargs = {"coxa_length": 33.0, "femur_length": 90.0, "tibia_length": 110.0}
        kwargs[field] = 0.0
        with pytest.raises(ValidationError):
            LegGeometry(**kwargs)


class TestLegConfig:
    def test_valid(self) -> None:
        leg = LegConfig(
            name="front_right",
            side=LegSide.RIGHT,
            row=LegRow.FRONT,
            mount_x=60.0,
            mount_y=-45.0,
            mount_angle_deg=-45.0,
        )
        assert leg.name == "front_right"

    def test_rejects_empty_name(self) -> None:
        with pytest.raises(ValidationError):
            LegConfig(
                name="", side=LegSide.RIGHT, row=LegRow.FRONT,
                mount_x=0.0, mount_y=0.0, mount_angle_deg=0.0,
            )

    @pytest.mark.parametrize("bad_angle", [-181.0, 181.0, 360.0])
    def test_rejects_angle_out_of_range(self, bad_angle: float) -> None:
        with pytest.raises(ValidationError):
            LegConfig(
                name="leg", side=LegSide.RIGHT, row=LegRow.FRONT,
                mount_x=0.0, mount_y=0.0, mount_angle_deg=bad_angle,
            )


# ============================================================
# ServoConfig: zwei Varianten, Discriminated Union
# ============================================================


class TestServoConfig:
    def test_leg_servo_valid(self) -> None:
        s = LegServoConfig(
            leg="front_right", joint=Joint.COXA, channel=0,
            center_us=1500.0, direction=1,
            min_us=600.0, max_us=2400.0, range_us=800.0,
        )
        assert s.kind == "leg"

    def test_camera_servo_valid(self) -> None:
        s = CameraServoConfig(
            axis=CameraAxis.PAN, channel=18,
            center_us=1500.0, direction=1,
            min_us=800.0, max_us=2200.0, range_us=700.0,
        )
        assert s.kind == "camera"

    def test_rejects_min_geq_max(self) -> None:
        with pytest.raises(ValidationError, match="min_us"):
            LegServoConfig(
                leg="leg", joint=Joint.COXA, channel=0,
                center_us=1500.0,
                min_us=2400.0, max_us=600.0,
                range_us=800.0,
            )

    def test_rejects_center_outside_min_max(self) -> None:
        with pytest.raises(ValidationError, match="center_us"):
            LegServoConfig(
                leg="leg", joint=Joint.COXA, channel=0,
                center_us=3000.0,  # > max_us
                min_us=600.0, max_us=2400.0,
                range_us=800.0,
            )

    @pytest.mark.parametrize("bad_dir", [0, 2, -2, 5])
    def test_rejects_invalid_direction(self, bad_dir: int) -> None:
        with pytest.raises(ValidationError):
            LegServoConfig(
                leg="leg", joint=Joint.COXA, channel=0,
                center_us=1500.0, direction=bad_dir,  # type: ignore[arg-type]
                min_us=600.0, max_us=2400.0, range_us=800.0,
            )


# ============================================================
# BodyConfig: 6 Beine, eindeutige Namen
# ============================================================


class TestBodyConfig:
    def test_valid(self, minimal_body: BodyConfig) -> None:
        assert len(minimal_body.legs) == 6

    def test_rejects_less_than_six_legs(self, minimal_legs: list[LegConfig]) -> None:
        with pytest.raises(ValidationError, match="min_length|too_short"):
            BodyConfig(
                leg_geometry=LegGeometry(coxa_length=33.0, femur_length=90.0, tibia_length=110.0),
                legs=minimal_legs[:5],
            )

    def test_rejects_more_than_six_legs(self, minimal_legs: list[LegConfig]) -> None:
        extra = LegConfig(
            name="extra", side=LegSide.LEFT, row=LegRow.MID,
            mount_x=0.0, mount_y=0.0, mount_angle_deg=0.0,
        )
        with pytest.raises(ValidationError):
            BodyConfig(
                leg_geometry=LegGeometry(coxa_length=33.0, femur_length=90.0, tibia_length=110.0),
                legs=[*minimal_legs, extra],
            )

    def test_rejects_duplicate_leg_names(self) -> None:
        legs = [
            LegConfig(name="dup", side=LegSide.RIGHT, row=LegRow.FRONT,
                      mount_x=0.0, mount_y=0.0, mount_angle_deg=0.0),
            LegConfig(name="dup", side=LegSide.LEFT, row=LegRow.FRONT,
                      mount_x=0.0, mount_y=0.0, mount_angle_deg=0.0),
            LegConfig(name="a", side=LegSide.RIGHT, row=LegRow.MID,
                      mount_x=0.0, mount_y=0.0, mount_angle_deg=0.0),
            LegConfig(name="b", side=LegSide.LEFT, row=LegRow.MID,
                      mount_x=0.0, mount_y=0.0, mount_angle_deg=0.0),
            LegConfig(name="c", side=LegSide.RIGHT, row=LegRow.BACK,
                      mount_x=0.0, mount_y=0.0, mount_angle_deg=0.0),
            LegConfig(name="d", side=LegSide.LEFT, row=LegRow.BACK,
                      mount_x=0.0, mount_y=0.0, mount_angle_deg=0.0),
        ]
        with pytest.raises(ValidationError, match="eindeutig"):
            BodyConfig(
                leg_geometry=LegGeometry(coxa_length=33.0, femur_length=90.0, tibia_length=110.0),
                legs=legs,
            )


# ============================================================
# RobotConfig: Cross-Validierungen
# ============================================================


class TestRobotConfig:
    def test_valid(self, minimal_config: RobotConfig) -> None:
        assert len(minimal_config.servos) == 20

    def test_rejects_duplicate_channels(self, minimal_config: RobotConfig) -> None:
        data = minimal_config.model_dump()
        data["servos"][1]["channel"] = data["servos"][0]["channel"]
        with pytest.raises(ValidationError, match="eindeutig"):
            RobotConfig.model_validate(data)

    def test_rejects_servo_referencing_unknown_leg(self, minimal_config: RobotConfig) -> None:
        data = minimal_config.model_dump()
        data["servos"][0]["leg"] = "no_such_leg"
        with pytest.raises(ValidationError, match="unbekanntes Bein"):
            RobotConfig.model_validate(data)

    def test_rejects_incomplete_leg_coverage(self, minimal_config: RobotConfig) -> None:
        data = minimal_config.model_dump()
        data["servos"] = [s for s in data["servos"]
                          if not (s.get("kind") == "leg"
                                  and s["leg"] == "front_right"
                                  and s["joint"] == "tibia")]
        with pytest.raises(ValidationError, match="fehlen Servos"):
            RobotConfig.model_validate(data)

    def test_rejects_channel_beyond_bus_range(self, minimal_config: RobotConfig) -> None:
        data = minimal_config.model_dump()
        data["buses"]["main"]["num_channels"] = 12
        with pytest.raises(ValidationError, match="liegt außerhalb von Bus"):
            RobotConfig.model_validate(data)


# ============================================================
# Komfort-Methoden
# ============================================================


class TestLookupMethods:
    def test_get_leg(self, minimal_config: RobotConfig) -> None:
        leg = minimal_config.get_leg("front_right")
        assert leg.row == LegRow.FRONT
        assert leg.side == LegSide.RIGHT

    def test_get_leg_unknown(self, minimal_config: RobotConfig) -> None:
        with pytest.raises(KeyError):
            minimal_config.get_leg("nope")

    def test_get_leg_servo(self, minimal_config: RobotConfig) -> None:
        s = minimal_config.get_leg_servo("front_right", Joint.COXA)
        assert s.channel == 0

    def test_get_leg_servo_unknown(self, minimal_config: RobotConfig) -> None:
        with pytest.raises(KeyError):
            minimal_config.get_leg_servo("nope", Joint.COXA)

    def test_get_camera_servo(self, minimal_config: RobotConfig) -> None:
        pan = minimal_config.get_camera_servo(CameraAxis.PAN)
        tilt = minimal_config.get_camera_servo(CameraAxis.TILT)
        assert pan.channel != tilt.channel
        assert pan.axis == CameraAxis.PAN
