"""High-Level Hexapod-Steuerung.

Fassade, die Config, Driver, Mapper und IK zusammenbringt.
"""

from .hexapod import Hexapod

__all__ = ["Hexapod"]
