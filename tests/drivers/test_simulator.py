"""Tests für SimulatorDriver."""

from __future__ import annotations

import pytest

from hexapod.drivers.simulator import (
    DISABLED,
    _DEFAULT_MAX_US as MAX_PULSE_US,
    _DEFAULT_MIN_US as MIN_PULSE_US,
    SimulatorDriver,
)


# ---------- Konstruktor ----------


class TestConstruction:
    def test_default_has_24_channels(self) -> None:
        driver = SimulatorDriver()
        assert driver.num_channels == 24

    def test_custom_channel_count(self) -> None:
        driver = SimulatorDriver(num_channels=6)
        assert driver.num_channels == 6

    def test_starts_with_all_channels_disabled(self) -> None:
        driver = SimulatorDriver(num_channels=4)
        assert driver.snapshot() == {0: DISABLED, 1: DISABLED, 2: DISABLED, 3: DISABLED}

    def test_starts_not_closed(self) -> None:
        driver = SimulatorDriver()
        assert not driver.is_closed

    @pytest.mark.parametrize("invalid", [0, -1, -100])
    def test_rejects_invalid_channel_count(self, invalid: int) -> None:
        with pytest.raises(ValueError, match="muss positiv"):
            SimulatorDriver(num_channels=invalid)


# ---------- Setzen und Lesen ----------


class TestSetAndGet:
    def test_set_then_get(self) -> None:
        driver = SimulatorDriver(num_channels=4)
        driver.set_position(2, 1500.0)
        assert driver.get_position(2) == 1500.0

    def test_other_channels_unaffected(self) -> None:
        driver = SimulatorDriver(num_channels=4)
        driver.set_position(2, 1500.0)
        assert driver.get_position(0) == DISABLED
        assert driver.get_position(1) == DISABLED
        assert driver.get_position(3) == DISABLED

    def test_overwrite_existing_position(self) -> None:
        driver = SimulatorDriver(num_channels=4)
        driver.set_position(0, 1500.0)
        driver.set_position(0, 1800.0)
        assert driver.get_position(0) == 1800.0

    def test_can_set_to_disabled(self) -> None:
        driver = SimulatorDriver(num_channels=4)
        driver.set_position(0, 1500.0)
        driver.set_position(0, DISABLED)
        assert driver.get_position(0) == DISABLED

    def test_min_pulse_accepted(self) -> None:
        driver = SimulatorDriver(num_channels=1)
        driver.set_position(0, MIN_PULSE_US)
        assert driver.get_position(0) == MIN_PULSE_US

    def test_max_pulse_accepted(self) -> None:
        driver = SimulatorDriver(num_channels=1)
        driver.set_position(0, MAX_PULSE_US)
        assert driver.get_position(0) == MAX_PULSE_US


# ---------- Validierung ----------


class TestValidation:
    @pytest.mark.parametrize("bad_channel", [-1, 4, 100])
    def test_rejects_out_of_range_channel(self, bad_channel: int) -> None:
        driver = SimulatorDriver(num_channels=4)
        with pytest.raises(ValueError, match="außerhalb"):
            driver.set_position(bad_channel, 1500.0)

    def test_rejects_negative_pulse(self) -> None:
        driver = SimulatorDriver(num_channels=4)
        with pytest.raises(ValueError, match="< 0"):
            driver.set_position(0, -100.0)

    @pytest.mark.parametrize("bad_pulse", [100.0, 399.9, 2600.1, 3000.0])
    def test_rejects_implausible_pulse(self, bad_pulse: float) -> None:
        driver = SimulatorDriver(num_channels=4)
        with pytest.raises(ValueError, match="außerhalb plausibler"):
            driver.set_position(0, bad_pulse)


# ---------- Batch-Setzen ----------


class TestBatchSet:
    def test_sets_multiple_at_once(self) -> None:
        driver = SimulatorDriver(num_channels=4)
        driver.set_positions({0: 1500.0, 1: 1700.0, 2: 1800.0})
        assert driver.get_position(0) == 1500.0
        assert driver.get_position(1) == 1700.0
        assert driver.get_position(2) == 1800.0
        assert driver.get_position(3) == DISABLED

    def test_atomic_on_validation_error(self) -> None:
        """Wenn ein Wert ungültig ist, darf KEINER committed werden."""
        driver = SimulatorDriver(num_channels=4)
        driver.set_position(0, 1500.0)
        with pytest.raises(ValueError):
            driver.set_positions({1: 1700.0, 2: 99.0})  # 99 ist invalid
        # Kanal 0 noch der alte Wert, 1 und 2 nicht angerührt
        assert driver.get_position(0) == 1500.0
        assert driver.get_position(1) == DISABLED
        assert driver.get_position(2) == DISABLED


# ---------- Disable ----------


class TestDisable:
    def test_disable_sets_position_to_zero(self) -> None:
        driver = SimulatorDriver(num_channels=4)
        driver.set_position(0, 1500.0)
        driver.disable(0)
        assert driver.get_position(0) == DISABLED

    def test_disable_other_channels_unaffected(self) -> None:
        driver = SimulatorDriver(num_channels=4)
        driver.set_position(0, 1500.0)
        driver.set_position(1, 1700.0)
        driver.disable(0)
        assert driver.get_position(1) == 1700.0


# ---------- Context Manager + close ----------


class TestLifecycle:
    def test_context_manager_closes(self) -> None:
        with SimulatorDriver(num_channels=4) as driver:
            assert not driver.is_closed
        assert driver.is_closed

    def test_close_disables_all(self) -> None:
        driver = SimulatorDriver(num_channels=4)
        driver.set_position(0, 1500.0)
        driver.set_position(1, 1700.0)
        driver.close()
        assert driver.snapshot() == {0: DISABLED, 1: DISABLED, 2: DISABLED, 3: DISABLED}

    def test_close_idempotent(self) -> None:
        driver = SimulatorDriver(num_channels=4)
        driver.close()
        driver.close()  # darf nicht crashen
        assert driver.is_closed

    def test_operations_after_close_raise(self) -> None:
        driver = SimulatorDriver(num_channels=4)
        driver.close()
        with pytest.raises(RuntimeError, match="bereits geschlossen"):
            driver.set_position(0, 1500.0)
        with pytest.raises(RuntimeError, match="bereits geschlossen"):
            driver.get_position(0)
        with pytest.raises(RuntimeError, match="bereits geschlossen"):
            driver.disable(0)


# ---------- Snapshot ----------


class TestSnapshot:
    def test_snapshot_returns_copy(self) -> None:
        """Wer den Snapshot ändert, darf den Driver-Zustand nicht ändern."""
        driver = SimulatorDriver(num_channels=4)
        driver.set_position(0, 1500.0)
        snap = driver.snapshot()
        snap[0] = 9999.0  # Mutation am Snapshot
        assert driver.get_position(0) == 1500.0
