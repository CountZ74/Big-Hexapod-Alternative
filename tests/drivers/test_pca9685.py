"""Tests fuer Pca9685Driver mit einem Fake-I2C-Bus (keine Hardware noetig).

Geprueft werden vor allem: das Freenove-Kanalmapping (Anschluss 0..31 auf
zwei Chips), die Umrechnung us -> 12-Bit-Ticks und das Registerlayout der
LED-Schreibzugriffe.
"""

from __future__ import annotations

import pytest

from hexapod.drivers.pca9685 import (
    ADDR_HIGH,
    ADDR_LOW,
    FULL_OFF,
    LED0_ON_L,
    MODE1,
    Pca9685Driver,
    Pca9685Error,
)


class FakeBus:
    """Protokolliert alle I2C-Zugriffe."""

    def __init__(self, raise_os: bool = False) -> None:
        self.raise_os = raise_os
        self.byte_writes: list[tuple[int, int, int]] = []
        self.block_writes: list[tuple[int, int, list[int]]] = []
        self.closed = False

    def write_byte_data(self, addr: int, register: int, value: int) -> None:
        if self.raise_os:
            raise OSError(121, "Remote I/O error")
        self.byte_writes.append((addr, register, value))

    def read_byte_data(self, addr: int, register: int) -> int:
        if self.raise_os:
            raise OSError(121, "Remote I/O error")
        return 0x00

    def write_i2c_block_data(self, addr: int, register: int, data: list[int]) -> None:
        if self.raise_os:
            raise OSError(121, "Remote I/O error")
        self.block_writes.append((addr, register, list(data)))

    def close(self) -> None:
        self.closed = True


def make_driver(**kwargs: object) -> tuple[Pca9685Driver, FakeBus]:
    fake = FakeBus()
    driver = Pca9685Driver(i2c=fake, **kwargs)  # type: ignore[arg-type]
    fake.byte_writes.clear()  # Init-Schreibzugriffe (Frequenz) ausblenden
    return driver, fake


# ---------- Kanal-Mapping (Freenove-Platine) ----------


class TestChannelMapping:
    def test_channel_0_goes_to_low_chip(self) -> None:
        driver, _ = make_driver()
        assert driver._resolve(0) == (ADDR_LOW, 0)

    def test_channel_15_is_last_on_low_chip(self) -> None:
        driver, _ = make_driver()
        assert driver._resolve(15) == (ADDR_LOW, 15)

    def test_channel_16_wraps_to_high_chip(self) -> None:
        driver, _ = make_driver()
        assert driver._resolve(16) == (ADDR_HIGH, 0)

    def test_camera_pan_channel_29(self) -> None:
        """Anschluss 29 = Pan -> Chip 0x40, Kanal 13."""
        driver, _ = make_driver()
        assert driver._resolve(29) == (ADDR_HIGH, 13)

    def test_camera_tilt_channel_30(self) -> None:
        """Anschluss 30 = Tilt -> Chip 0x40, Kanal 14."""
        driver, _ = make_driver()
        assert driver._resolve(30) == (ADDR_HIGH, 14)

    def test_out_of_range_raises(self) -> None:
        driver, _ = make_driver()
        with pytest.raises(ValueError, match="ausserhalb"):
            driver._resolve(32)

    def test_negative_channel_raises(self) -> None:
        driver, _ = make_driver()
        with pytest.raises(ValueError, match="ausserhalb"):
            driver._resolve(-1)


# ---------- Umrechnung us -> Ticks ----------


class TestTickConversion:
    def test_center_pulse_at_50hz(self) -> None:
        """1500 us bei 50 Hz (20000 us Periode) = 1500/20000*4096 = 307."""
        driver, _ = make_driver()
        assert driver._us_to_ticks(1500.0) == 307

    def test_min_pulse(self) -> None:
        driver, _ = make_driver()
        assert driver._us_to_ticks(500.0) == 102

    def test_max_pulse(self) -> None:
        driver, _ = make_driver()
        assert driver._us_to_ticks(2500.0) == 512

    def test_zero_is_zero(self) -> None:
        driver, _ = make_driver()
        assert driver._us_to_ticks(0.0) == 0

    def test_clamped_to_12bit(self) -> None:
        driver, _ = make_driver()
        assert driver._us_to_ticks(999_999.0) == 4095

    def test_frequency_affects_ticks(self) -> None:
        """Bei 100 Hz ist dieselbe Pulsweite doppelt so viele Ticks."""
        driver, _ = make_driver(freq_hz=100.0)
        assert driver._us_to_ticks(1500.0) == 614


# ---------- Schreibzugriffe ----------


class TestSetPosition:
    def test_writes_correct_register_and_data(self) -> None:
        driver, fake = make_driver()
        driver.set_position(29, 1500.0)
        addr, register, data = fake.block_writes[-1]
        assert addr == ADDR_HIGH
        assert register == LED0_ON_L + 4 * 13
        # on = 0, off = 307 (0x0133)
        assert data == [0x00, 0x00, 0x33, 0x01]

    def test_position_is_remembered(self) -> None:
        driver, _ = make_driver()
        driver.set_position(30, 1600.0)
        assert driver.get_position(30) == 1600.0

    def test_unset_channel_reads_zero(self) -> None:
        driver, _ = make_driver()
        assert driver.get_position(5) == 0.0

    def test_clamps_to_max(self) -> None:
        driver, _ = make_driver(max_pulse_us=2000.0)
        driver.set_position(29, 2400.0)
        assert driver.get_position(29) == 2000.0

    def test_clamps_to_min(self) -> None:
        driver, _ = make_driver(min_pulse_us=800.0)
        driver.set_position(29, 500.0)
        assert driver.get_position(29) == 800.0

    def test_negative_raises(self) -> None:
        driver, _ = make_driver()
        with pytest.raises(ValueError, match=">= 0"):
            driver.set_position(29, -1.0)

    def test_zero_disables(self) -> None:
        driver, fake = make_driver()
        driver.set_position(29, 0.0)
        _addr, _register, data = fake.block_writes[-1]
        assert data[3] == FULL_OFF
        assert driver.get_position(29) == 0.0

    def test_set_positions_batch(self) -> None:
        driver, fake = make_driver()
        driver.set_positions({29: 1500.0, 30: 1400.0})
        assert len(fake.block_writes) == 2
        assert driver.get_position(29) == 1500.0
        assert driver.get_position(30) == 1400.0


class TestDisable:
    def test_sets_full_off_bit(self) -> None:
        driver, fake = make_driver()
        driver.disable(30)
        addr, register, data = fake.block_writes[-1]
        assert addr == ADDR_HIGH
        assert register == LED0_ON_L + 4 * 14
        assert data == [0x00, 0x00, 0x00, FULL_OFF]

    def test_disable_all_touches_every_channel(self) -> None:
        driver, fake = make_driver()
        driver.disable_all(32)
        assert len(fake.block_writes) == 32


# ---------- Initialisierung ----------


class TestInit:
    def test_sets_prescale_on_both_chips(self) -> None:
        fake = FakeBus()
        Pca9685Driver(i2c=fake)
        addrs = {addr for addr, _reg, _v in fake.byte_writes}
        assert addrs == {ADDR_LOW, ADDR_HIGH}

    def test_prescale_value_for_50hz(self) -> None:
        """round(25e6 / (4096 * 50) - 1) = 121."""
        fake = FakeBus()
        Pca9685Driver(i2c=fake)
        prescales = [v for _a, reg, v in fake.byte_writes if reg == 0xFE]
        assert prescales and all(p == 121 for p in prescales)

    def test_mode1_is_written(self) -> None:
        fake = FakeBus()
        Pca9685Driver(i2c=fake)
        assert any(reg == MODE1 for _a, reg, _v in fake.byte_writes)

    def test_single_chip_only_inits_low(self) -> None:
        fake = FakeBus()
        Pca9685Driver(i2c=fake, num_channels=16)
        addrs = {addr for addr, _reg, _v in fake.byte_writes}
        assert addrs == {ADDR_LOW}

    def test_invalid_num_channels(self) -> None:
        with pytest.raises(ValueError, match="num_channels"):
            Pca9685Driver(i2c=FakeBus(), num_channels=33)

    def test_invalid_freq(self) -> None:
        with pytest.raises(ValueError, match="freq_hz"):
            Pca9685Driver(i2c=FakeBus(), freq_hz=0.0)

    def test_invalid_pulse_range(self) -> None:
        with pytest.raises(ValueError, match="min_pulse_us"):
            Pca9685Driver(i2c=FakeBus(), min_pulse_us=2000.0, max_pulse_us=1000.0)


# ---------- Robustheit ----------


class TestRobustness:
    def test_io_error_does_not_raise(self) -> None:
        """Ein I2C-Fehler darf den Roboter nicht lahmlegen."""
        fake = FakeBus(raise_os=True)
        driver = Pca9685Driver(i2c=fake)
        driver.set_position(29, 1500.0)  # darf nicht werfen
        assert driver.get_position(29) == 1500.0

    def test_close_disables_and_closes_bus(self) -> None:
        driver, fake = make_driver()
        driver.close()
        assert fake.closed
        assert driver.is_closed

    def test_close_without_disable_keeps_pose(self) -> None:
        driver, fake = make_driver()
        driver.close(disable=False)
        assert not fake.block_writes
        assert fake.closed

    def test_close_is_idempotent(self) -> None:
        driver, _ = make_driver()
        driver.close()
        driver.close()

    def test_use_after_close_raises(self) -> None:
        driver, _ = make_driver()
        driver.close()
        with pytest.raises(Pca9685Error, match="geschlossen"):
            driver.set_position(29, 1500.0)

    def test_context_manager(self) -> None:
        fake = FakeBus()
        with Pca9685Driver(i2c=fake) as driver:
            driver.set_position(29, 1500.0)
        assert fake.closed

    def test_speed_and_acceleration_are_noops(self) -> None:
        """PCA9685 kennt keine Rampen — die Aufrufe duerfen nur nichts tun."""
        driver, fake = make_driver()
        driver.set_speed(29, 20)
        driver.set_acceleration(29, 5)
        assert not fake.block_writes
