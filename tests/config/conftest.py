"""Gemeinsame Fixtures für Konfig-Tests."""

from __future__ import annotations

import pytest

from hexapod.config import (
    BodyConfig,
    CameraAxis,
    CameraServoConfig,
    DriverConfig,
    Joint,
    LegConfig,
    LegGeometry,
    LegRow,
    LegServoConfig,
    LegSide,
    RobotConfig,
)


def _leg(name: str, side: LegSide, row: LegRow, x: float, y: float, ang: float) -> LegConfig:
    return LegConfig(
        name=name, side=side, row=row,
        mount_x=x, mount_y=y, mount_angle_deg=ang,
    )


def _leg_servo(leg: str, joint: Joint, channel: int) -> LegServoConfig:
    return LegServoConfig(
        leg=leg, joint=joint, channel=channel,
        center_us=1500.0, direction=1,
        min_us=600.0, max_us=2400.0, range_us=800.0,
    )


@pytest.fixture
def minimal_legs() -> list[LegConfig]:
    """6 Beine in paarweiser Reihenfolge."""
    return [
        _leg("front_right", LegSide.RIGHT, LegRow.FRONT, 60.0, -45.0, -45.0),
        _leg("front_left",  LegSide.LEFT,  LegRow.FRONT, 60.0,  45.0,  45.0),
        _leg("mid_right",   LegSide.RIGHT, LegRow.MID,    0.0, -65.0, -90.0),
        _leg("mid_left",    LegSide.LEFT,  LegRow.MID,    0.0,  65.0,  90.0),
        _leg("back_right",  LegSide.RIGHT, LegRow.BACK, -60.0, -45.0, -135.0),
        _leg("back_left",   LegSide.LEFT,  LegRow.BACK, -60.0,  45.0,  135.0),
    ]


@pytest.fixture
def minimal_body(minimal_legs: list[LegConfig]) -> BodyConfig:
    return BodyConfig(
        leg_geometry=LegGeometry(coxa_length=33.0, femur_length=90.0, tibia_length=110.0),
        legs=minimal_legs,
    )


@pytest.fixture
def minimal_servos() -> list[LegServoConfig | CameraServoConfig]:
    """18 Bein-Servos + 2 Kamera-Servos, eindeutige Kanäle."""
    leg_names = [
        "front_right", "front_left",
        "mid_right",   "mid_left",
        "back_right",  "back_left",
    ]
    out: list[LegServoConfig | CameraServoConfig] = []
    channel = 0
    for name in leg_names:
        for joint in [Joint.COXA, Joint.FEMUR, Joint.TIBIA]:
            out.append(_leg_servo(name, joint, channel))
            channel += 1
    # Kamera
    out.append(CameraServoConfig(
        axis=CameraAxis.PAN, channel=channel,
        center_us=1500.0, direction=1, min_us=800.0, max_us=2200.0, range_us=700.0,
    ))
    channel += 1
    out.append(CameraServoConfig(
        axis=CameraAxis.TILT, channel=channel,
        center_us=1500.0, direction=1, min_us=1000.0, max_us=2000.0, range_us=500.0,
    ))
    return out


@pytest.fixture
def minimal_config(
    minimal_body: BodyConfig,
    minimal_servos: list[LegServoConfig | CameraServoConfig],
) -> RobotConfig:
    """Eine vollständig gültige RobotConfig zum Aufbauen von Tests."""
    return RobotConfig(
        name="Test Hexapod",
        driver=DriverConfig(type="simulator", port="/dev/null", num_channels=24),
        body=minimal_body,
        servos=minimal_servos,
    )
