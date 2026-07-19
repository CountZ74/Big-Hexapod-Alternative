"""Tests fuer den ADS7830-Batterie-Reader (Fake-Bus, keine Hardware)."""
from __future__ import annotations

import pytest

from hexapod.drivers.adc import ADS7830, classify


class FakeBus:
    def __init__(self, value: int, raise_os: bool = False) -> None:
        self.value = value
        self.raise_os = raise_os
        self.writes: list[tuple[int, int]] = []

    def write_byte(self, addr: int, cmd: int) -> None:
        if self.raise_os:
            raise OSError(121, "Remote I/O error")
        self.writes.append((addr, cmd))

    def read_byte(self, addr: int) -> int:
        if self.raise_os:
            raise OSError(121, "Remote I/O error")
        return self.value


def make(value: int, raise_os: bool = False) -> ADS7830:
    a = ADS7830()
    a._bus = FakeBus(value, raise_os)   # Bus injizieren
    return a


class TestCommandByte:
    def test_channel0(self) -> None:
        a = make(0)
        a.read_channel_voltage(0)
        assert a._bus.writes[-1][1] == 0x84

    def test_channel4(self) -> None:
        a = make(0)
        a.read_channel_voltage(4)
        # 0x84 | ((((4<<2)|(4>>1))&7)<<4) = 0x84 | (2<<4) = 0xA4
        assert a._bus.writes[-1][1] == 0xA4


class TestVoltage:
    def test_conversion(self) -> None:
        # 125/255*5*3 = 7.35
        assert make(125).read_channel_voltage(4) == pytest.approx(7.35, abs=0.01)

    def test_full_scale(self) -> None:
        assert make(255).read_channel_voltage(0) == pytest.approx(15.0, abs=0.01)

    def test_oserror_returns_none(self) -> None:
        assert make(0, raise_os=True).read_channel_voltage(0) is None


class TestClassify:
    @pytest.mark.parametrize("v,exp", [
        (8.4, "ok"), (7.4, "ok"), (6.7, "ok"),
        (6.5, "warn"), (6.3, "warn"),
        (6.1, "critical"), (4.0, "critical"),
        (2.18, "absent"), (0.0, "absent"), (None, "absent"),
    ])
    def test_levels(self, v, exp) -> None:
        assert classify(v) == exp


from hexapod.drivers.adc import BatteryMonitor


class TestBatteryMonitor:
    def test_confirmed_critical_after_n(self) -> None:
        m = BatteryMonitor(crit_confirm=3)
        # 2x kritisch -> noch nicht bestaetigt, 3. mal -> bestaetigt + fire einmal
        assert not m.update({"pi": 6.0})["pi"]["confirmed_critical"]
        assert not m.update({"pi": 6.0})["pi"]["confirmed_critical"]
        r = m.update({"pi": 6.0})["pi"]
        assert r["confirmed_critical"] and r["fire_action"]

    def test_fire_only_once(self) -> None:
        m = BatteryMonitor(crit_confirm=1)
        assert m.update({"pi": 5.0})["pi"]["fire_action"]
        assert not m.update({"pi": 5.0})["pi"]["fire_action"]  # nicht erneut

    def test_recovery_resets(self) -> None:
        m = BatteryMonitor(crit_confirm=1)
        assert m.update({"pi": 5.0})["pi"]["fire_action"]
        m.update({"pi": 7.4})           # erholt -> acted reset
        assert m.update({"pi": 5.0})["pi"]["fire_action"]  # feuert wieder

    def test_changed_flag(self) -> None:
        m = BatteryMonitor()
        assert m.update({"pi": 7.4})["pi"]["changed"]      # erstes Mal
        assert not m.update({"pi": 7.3})["pi"]["changed"]  # bleibt ok
        assert m.update({"pi": 6.5})["pi"]["changed"]      # ok -> warn

    def test_absent_not_critical(self) -> None:
        m = BatteryMonitor(crit_confirm=1)
        r = m.update({"servo": 2.5})["servo"]
        assert r["state"] == "absent" and not r["fire_action"]
