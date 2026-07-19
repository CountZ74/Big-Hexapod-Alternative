"""Mapping zwischen Gelenkwinkeln (Radiant) und Pulsweiten (Mikrosekunden).

Nutzt die per-Servo-Kalibrierung aus der Konfig: center_us, range_us,
direction, min_us, max_us. Vollständig stateless — pro Servo eine
ServoMapping-Instanz, die für sich abbildet.
"""

from .mapper import (
    OutOfRangeError,
    ServoMapping,
    angle_to_us,
    us_to_angle,
)

__all__ = [
    "OutOfRangeError",
    "ServoMapping",
    "angle_to_us",
    "us_to_angle",
]
