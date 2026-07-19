"""Lineare Servo-Abbildung mit Kalibrierung und Hardware-Grenzen."""

from __future__ import annotations

import math
from dataclasses import dataclass

MAX_ANGLE_RAD = math.pi / 2

# Fallback-Grenzen falls kein RobotConfig verfügbar (z.B. in Tests)
_DEFAULT_MIN_US = 400.0
_DEFAULT_MAX_US = 2600.0


class OutOfRangeError(ValueError):
    """Die berechnete Pulsweite liegt außerhalb von [min_us, max_us]."""


@dataclass(frozen=True, slots=True)
class ServoMapping:
    """Kalibrierte Abbildung für einen Servo."""

    center_us: float
    range_us: float
    direction: int
    min_us: float
    max_us: float

    def __post_init__(self) -> None:
        if self.direction not in (-1, 1):
            raise ValueError(f"direction muss +1 oder -1 sein, war {self.direction}")
        if self.range_us <= 0:
            raise ValueError(f"range_us muss positiv sein, war {self.range_us}")
        if self.min_us >= self.max_us:
            raise ValueError(
                f"min_us ({self.min_us}) muss < max_us ({self.max_us}) sein"
            )
        if not (self.min_us <= self.center_us <= self.max_us):
            raise ValueError(
                f"center_us ({self.center_us}) muss in [{self.min_us}, {self.max_us}] liegen"
            )

    def angle_to_us(self, angle_rad: float, *, clip: bool = False) -> float:
        us = self.center_us + self.direction * angle_rad * (self.range_us / MAX_ANGLE_RAD)
        if us < self.min_us or us > self.max_us:
            if clip:
                return max(self.min_us, min(self.max_us, us))
            raise OutOfRangeError(
                f"angle={math.degrees(angle_rad):.2f}° -> {us:.1f} µs außerhalb "
                f"[{self.min_us}, {self.max_us}]"
            )
        return us

    def us_to_angle(self, us: float) -> float:
        return (us - self.center_us) * MAX_ANGLE_RAD / (self.direction * self.range_us)


def angle_to_us(
    angle_rad: float,
    center_us: float,
    range_us: float,
    direction: int = 1,
    *,
    min_us: float = _DEFAULT_MIN_US,
    max_us: float = _DEFAULT_MAX_US,
    clip: bool = False,
) -> float:
    """Stateless Variante von ServoMapping.angle_to_us."""
    return ServoMapping(
        center_us=center_us,
        range_us=range_us,
        direction=direction,
        min_us=min_us,
        max_us=max_us,
    ).angle_to_us(angle_rad, clip=clip)


def us_to_angle(
    us: float,
    center_us: float,
    range_us: float,
    direction: int = 1,
) -> float:
    """Stateless Variante von ServoMapping.us_to_angle."""
    return ServoMapping(
        center_us=center_us,
        range_us=range_us,
        direction=direction,
        min_us=min(us, _DEFAULT_MIN_US) - 1.0,
        max_us=max(us, _DEFAULT_MAX_US) + 1.0,
    ).us_to_angle(us)
