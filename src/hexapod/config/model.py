"""Pydantic-Datenmodell für die Hexapod-Konfiguration."""

from __future__ import annotations

from enum import StrEnum
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator


class LegSide(StrEnum):
    LEFT = "left"
    RIGHT = "right"


class LegRow(StrEnum):
    FRONT = "front"
    MID = "mid"
    BACK = "back"


class Joint(StrEnum):
    COXA = "coxa"
    FEMUR = "femur"
    TIBIA = "tibia"


class CameraAxis(StrEnum):
    PAN = "pan"
    TILT = "tilt"


class StrictBase(BaseModel):
    model_config = ConfigDict(extra="forbid", validate_assignment=True)


# =====================================================================
# Globale Servo-Sicherheitsgrenzen — eine einzige Quelle der Wahrheit
# =====================================================================


class ServoLimits(StrictBase):
    """Globale Hardware-Sicherheitsgrenzen für alle Servos.

    Diese Werte werden NIEMALS überschritten, egal was pro Servo
    konfiguriert ist. Sie schützen vor versehentlicher Beschädigung
    durch Tippfehler in der YAML oder Bugs im Code.
    """

    absolute_min_us: float = Field(
        default=400.0,
        gt=0.0,
        description="Untere absolute Grenze [µs] — nie unterschritten.",
    )
    absolute_max_us: float = Field(
        default=2600.0,
        gt=0.0,
        description="Obere absolute Grenze [µs] — nie überschritten.",
    )

    @model_validator(mode="after")
    def _validate_ordering(self) -> ServoLimits:
        if self.absolute_min_us >= self.absolute_max_us:
            raise ValueError(
                f"absolute_min_us ({self.absolute_min_us}) muss < "
                f"absolute_max_us ({self.absolute_max_us}) sein."
            )
        return self


class DriverConfig(StrictBase):
    type: Literal["maestro", "simulator"] = Field(default="maestro")
    port: str = Field(default="/dev/maestro_cmd")
    num_channels: int = Field(default=24, gt=0, le=24)
    timeout: float = Field(default=1.0, gt=0.0)


class LegGeometry(StrictBase):
    coxa_length: float = Field(gt=0.0)
    femur_length: float = Field(gt=0.0)
    tibia_length: float = Field(gt=0.0)


class LegConfig(StrictBase):
    name: str = Field(min_length=1)
    side: LegSide
    row: LegRow
    mount_x: float
    mount_y: float
    mount_angle_deg: float = Field(ge=-180.0, le=180.0)
    z_trim: float = Field(
        default=0.0,
        ge=-30.0,
        le=30.0,
        description=(
            "Höhen-Korrektur pro Bein in mm. Wird auf das Standpose-Fuß-Z "
            "addiert, um ungleichen Bodenkontakt (Kalibrier-/Mechanik-Toleranz) "
            "auszugleichen. Positiv = Fuß tiefer (Bein drückt mehr nach unten)."
        ),
    )


class BodyConfig(StrictBase):
    leg_geometry: LegGeometry
    legs: list[LegConfig] = Field(min_length=6, max_length=6)

    @model_validator(mode="after")
    def _validate_leg_names_unique(self) -> BodyConfig:
        names = [leg.name for leg in self.legs]
        if len(set(names)) != len(names):
            duplicates = {n for n in names if names.count(n) > 1}
            raise ValueError(f"Bein-Namen müssen eindeutig sein. Doppelt: {duplicates}")
        return self


class _ServoCommon(StrictBase):
    channel: int = Field(ge=0, lt=24)
    center_us: float = Field(gt=0.0)
    direction: Literal[-1, 1] = Field(default=1)
    min_us: float = Field(gt=0.0)
    max_us: float = Field(gt=0.0)
    range_us: float = Field(gt=0.0)

    @model_validator(mode="after")
    def _validate_us_ordering(self) -> _ServoCommon:
        if self.min_us >= self.max_us:
            raise ValueError(f"min_us ({self.min_us}) muss < max_us ({self.max_us}) sein.")
        if not (self.min_us <= self.center_us <= self.max_us):
            raise ValueError(
                f"center_us ({self.center_us}) muss zwischen "
                f"min_us ({self.min_us}) und max_us ({self.max_us}) liegen."
            )
        return self


class LegServoConfig(_ServoCommon):
    kind: Literal["leg"] = "leg"
    leg: str = Field(min_length=1)
    joint: Joint


class CameraServoConfig(_ServoCommon):
    kind: Literal["camera"] = "camera"
    axis: CameraAxis


ServoConfig = Annotated[
    LegServoConfig | CameraServoConfig,
    Field(discriminator="kind"),
]


class RobotConfig(StrictBase):
    name: str = Field(default="Hexapod", min_length=1)
    description: str = ""
    servo_limits: ServoLimits = Field(default_factory=ServoLimits)
    driver: DriverConfig = Field(default_factory=DriverConfig)
    body: BodyConfig
    servos: list[ServoConfig] = Field(min_length=1)

    @model_validator(mode="after")
    def _validate_channel_uniqueness(self) -> RobotConfig:
        channels = [s.channel for s in self.servos]
        if len(set(channels)) != len(channels):
            duplicates = {c for c in channels if channels.count(c) > 1}
            raise ValueError(f"Servo-Kanäle müssen eindeutig sein. Doppelt: {duplicates}")
        return self

    @model_validator(mode="after")
    def _validate_leg_servo_references(self) -> RobotConfig:
        leg_names = {leg.name for leg in self.body.legs}
        for s in self.servos:
            if isinstance(s, LegServoConfig) and s.leg not in leg_names:
                raise ValueError(
                    f"Servo auf Kanal {s.channel} referenziert unbekanntes Bein "
                    f"{s.leg}. Bekannte Beine: {sorted(leg_names)}"
                )
        return self

    @model_validator(mode="after")
    def _validate_complete_leg_coverage(self) -> RobotConfig:
        for leg in self.body.legs:
            joints_for_leg = {
                s.joint for s in self.servos
                if isinstance(s, LegServoConfig) and s.leg == leg.name
            }
            missing = set(Joint) - joints_for_leg
            if missing:
                raise ValueError(
                    f"Bein {leg.name} fehlen Servos für Gelenk(e): "
                    f"{sorted(j.value for j in missing)}"
                )
        return self

    @model_validator(mode="after")
    def _validate_channels_within_driver(self) -> RobotConfig:
        for s in self.servos:
            if s.channel >= self.driver.num_channels:
                raise ValueError(
                    f"Servo-Kanal {s.channel} außerhalb der Driver-Kanäle "
                    f"[0..{self.driver.num_channels - 1}]."
                )
        return self

    @model_validator(mode="after")
    def _validate_servo_limits(self) -> RobotConfig:
        """Alle per-Servo min/max müssen innerhalb der globalen Grenzen liegen."""
        lim = self.servo_limits
        for s in self.servos:
            if s.min_us < lim.absolute_min_us:
                raise ValueError(
                    f"Servo Kanal {s.channel}: min_us ({s.min_us}) unterschreitet "
                    f"absolute_min_us ({lim.absolute_min_us})."
                )
            if s.max_us > lim.absolute_max_us:
                raise ValueError(
                    f"Servo Kanal {s.channel}: max_us ({s.max_us}) überschreitet "
                    f"absolute_max_us ({lim.absolute_max_us})."
                )
        return self

    def get_leg(self, name: str) -> LegConfig:
        for leg in self.body.legs:
            if leg.name == name:
                return leg
        raise KeyError(f"Bein {name} nicht in der Konfiguration.")

    def get_leg_servo(self, leg_name: str, joint: Joint) -> LegServoConfig:
        for s in self.servos:
            if isinstance(s, LegServoConfig) and s.leg == leg_name and s.joint == joint:
                return s
        raise KeyError(f"Kein Servo für Bein {leg_name} Gelenk {joint.value}.")

    def get_camera_servo(self, axis: CameraAxis) -> CameraServoConfig:
        for s in self.servos:
            if isinstance(s, CameraServoConfig) and s.axis == axis:
                return s
        raise KeyError(f"Kein Kamera-Servo für Achse {axis.value}.")
