"""MPU6050 6-Achsen-IMU (I2C 0x68) -- Beschleunigung + Gyro fuer Nivellierung.

Liefert Roll/Pitch aus dem Beschleunigungsvektor (Schwerkraftrichtung).
Robust wie der ADS7830-Treiber: fehlender Bus/Lesefehler -> None statt Absturz.

Die Achsen-/Vorzeichenzuordnung haengt von der Einbaulage ab und wird am
Roboter kalibriert (invert_roll/invert_pitch/swap_axes).
"""
from __future__ import annotations

import logging
import math
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from smbus2 import SMBus

logger = logging.getLogger(__name__)

MPU_ADDR = 0x68
REG_PWR_MGMT_1 = 0x6B
REG_ACCEL_XOUT_H = 0x3B
REG_GYRO_XOUT_H = 0x43
ACCEL_SCALE = 16384.0   # LSB/g bei +/-2g
GYRO_SCALE = 131.0      # LSB/(deg/s) bei +/-250 deg/s


def accel_to_tilt(ax: float, ay: float, az: float) -> tuple[float, float]:
    """Roll/Pitch (Grad) aus dem Beschleunigungsvektor.

    roll  = Drehung um X (Seitneigung), pitch = Drehung um Y (Nicken).
    Bei flach liegendem Sensor (Schwerkraft in -Z bzw. +Z) -> (0, 0).
    """
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
    return roll, pitch


def _s16(hi: int, lo: int) -> int:
    v = (hi << 8) | lo
    return v - 65536 if v >= 32768 else v


class MPU6050:
    def __init__(self, bus: int = 1, address: int = MPU_ADDR,
                 invert_roll: bool = False, invert_pitch: bool = False,
                 swap_axes: bool = False) -> None:
        self._busno = bus
        self._addr = address
        self._bus: SMBus | None = None
        self._failed = False
        self.invert_roll = invert_roll
        self.invert_pitch = invert_pitch
        self.swap_axes = swap_axes

    def _ensure(self) -> SMBus | None:
        if self._bus is None and not self._failed:
            try:
                import smbus2
                self._bus = smbus2.SMBus(self._busno)
                self._bus.write_byte_data(self._addr, REG_PWR_MGMT_1, 0)  # aufwecken
                time.sleep(0.1)  # erster Sample nach Wake ist sonst 0
            except Exception as e:
                logger.warning("MPU6050: I2C nicht verfuegbar (%s)", e)
                self._failed = True
        return self._bus

    def read_accel(self) -> tuple[float, float, float] | None:
        bus = self._ensure()
        if bus is None:
            return None
        try:
            d = bus.read_i2c_block_data(self._addr, REG_ACCEL_XOUT_H, 6)
        except Exception as e:
            logger.debug("MPU6050 Lesefehler: %s", e)
            return None
        ax = _s16(d[0], d[1]) / ACCEL_SCALE
        ay = _s16(d[2], d[3]) / ACCEL_SCALE
        az = _s16(d[4], d[5]) / ACCEL_SCALE
        return ax, ay, az

    def tilt(self) -> tuple[float, float] | None:
        """Roll/Pitch in Grad, mit Einbaulage-Korrektur; None bei Fehler."""
        a = self.read_accel()
        if a is None:
            return None
        ax, ay, az = a
        if self.swap_axes:
            ax, ay = ay, ax
        roll, pitch = accel_to_tilt(ax, ay, az)
        if self.invert_roll:
            roll = -roll
        if self.invert_pitch:
            pitch = -pitch
        return roll, pitch
