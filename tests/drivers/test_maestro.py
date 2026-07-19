"""Tests für MaestroDriver mit einem FakeSerial.

Diese Tests prüfen das exakte Byte-Protokoll, ohne echte Hardware.
Wenn der Maestro mal das Protokoll ändert (unwahrscheinlich) oder wir
einen Bug einführen, fangen die Tests das hier ab.
"""

from __future__ import annotations

import pytest

from hexapod.drivers.maestro import (
    CMD_GET_ERRORS,
    CMD_GET_POSITION,
    CMD_SET_ACCELERATION,
    CMD_SET_MULTIPLE,
    CMD_SET_SPEED,
    CMD_SET_TARGET,
    MaestroDriver,
    MaestroError,
)

from .fake_serial import FakeSerial


def make_driver(num_channels: int = 24) -> tuple[MaestroDriver, FakeSerial]:
    """Helper: erzeugt MaestroDriver + FakeSerial.

    initial_speed=None und initial_acceleration=None damit der Init gar
    keine Speed/Acceleration-Bytes schreibt und die Tests sauber bleiben.
    (0 wuerde den Maestro explizit auf "unbegrenzt" setzen und Bytes
    schreiben; None ueberspringt das.)
    """
    fake = FakeSerial()
    driver = MaestroDriver(
        port="/dev/fake",
        num_channels=num_channels,
        ser=fake,
        initial_speed=None,
        initial_acceleration=None,
    )
    return driver, fake


# ---------- Encoding-Mathematik ----------


class TestEncoding:
    def test_encode_zero(self) -> None:
        assert MaestroDriver._encode_14bit(0) == (0, 0)

    def test_encode_known_value(self) -> None:
        # 6000 = 1500 us * 4 qus/us
        # 6000 = 0b0001_0111_0111_0000
        # low (7 bit)  = 0b1110000 = 0x70 = 112
        # high (7 bit) = 0b0101110 = 0x2E = 46
        assert MaestroDriver._encode_14bit(6000) == (0x70, 0x2E)

    def test_encode_max_14bit(self) -> None:
        assert MaestroDriver._encode_14bit((1 << 14) - 1) == (0x7F, 0x7F)

    def test_encode_rejects_negative(self) -> None:
        with pytest.raises(ValueError, match="14-Bit"):
            MaestroDriver._encode_14bit(-1)

    def test_encode_rejects_too_large(self) -> None:
        with pytest.raises(ValueError, match="14-Bit"):
            MaestroDriver._encode_14bit(1 << 14)

    def test_decode_response_is_8bit_little_endian(self) -> None:
        # 6000 = 0x1770
        # low byte = 0x70, high byte = 0x17 (NICHT 0x2E!)
        assert MaestroDriver._decode_response_16bit(0x70, 0x17) == 6000

    def test_decode_response_zero(self) -> None:
        assert MaestroDriver._decode_response_16bit(0, 0) == 0


# ---------- Set Position: Bytes auf der Leitung ----------


class TestSetPosition:
    def test_writes_correct_bytes_for_1500us(self) -> None:
        driver, fake = make_driver()
        driver.set_position(0, 1500.0)
        # Erwartet: 0x84 0x00 0x70 0x2E
        assert bytes(fake.written) == bytes([CMD_SET_TARGET, 0, 0x70, 0x2E])

    def test_writes_correct_bytes_for_max_pulse(self) -> None:
        driver, fake = make_driver()
        driver.set_position(3, 2500.0)
        # 2500 * 4 = 10000 = 0x2710
        # 10000 = 0b10_0111_0001_0000
        # low (7 bit)  = 0b0010000 = 0x10
        # high (7 bit) = 0b1001110 = 0x4E
        assert bytes(fake.written) == bytes([CMD_SET_TARGET, 3, 0x10, 0x4E])

    def test_flush_is_called(self) -> None:
        driver, fake = make_driver()
        driver.set_position(0, 1500.0)
        assert fake.flush_count >= 1

    def test_rejects_out_of_range_channel(self) -> None:
        driver, _ = make_driver(num_channels=4)
        with pytest.raises(ValueError, match="außerhalb"):
            driver.set_position(5, 1500.0)

    def test_rejects_implausible_pulse(self) -> None:
        driver, _ = make_driver()
        with pytest.raises(ValueError, match="außerhalb plausibler"):
            driver.set_position(0, 100.0)


# ---------- Set Multiple ----------


class TestSetPositions:
    def test_uses_set_multiple_for_consecutive_channels(self) -> None:
        driver, fake = make_driver()
        driver.set_positions({0: 1500.0, 1: 1700.0, 2: 1800.0})
        # Erwartet: 0x9F 3 0 <low0><high0> <low1><high1> <low2><high2>
        # 1500 us -> 6000 qus -> (0x70, 0x2E)
        # 1700 us -> 6800 qus = 0x1A90 -> low=0x10, high=0x35
        # 1800 us -> 7200 qus = 0x1C20 -> low=0x20, high=0x38
        expected = bytes([
            CMD_SET_MULTIPLE, 3, 0,
            0x70, 0x2E,
            0x10, 0x35,
            0x20, 0x38,
        ])
        assert bytes(fake.written) == expected

    def test_splits_into_blocks_for_non_consecutive(self) -> None:
        driver, fake = make_driver()
        driver.set_positions({0: 1500.0, 1: 1500.0, 5: 1500.0})
        # Erwartet: zwei Blöcke
        # Block 1: 0x9F 2 0 (0x70, 0x2E) (0x70, 0x2E)
        # Block 2: 0x9F 1 5 (0x70, 0x2E)
        expected = bytes([
            CMD_SET_MULTIPLE, 2, 0, 0x70, 0x2E, 0x70, 0x2E,
            CMD_SET_MULTIPLE, 1, 5, 0x70, 0x2E,
        ])
        assert bytes(fake.written) == expected

    def test_empty_dict_does_nothing(self) -> None:
        driver, fake = make_driver()
        driver.set_positions({})
        assert bytes(fake.written) == b""

    def test_atomic_on_validation_error(self) -> None:
        driver, fake = make_driver()
        with pytest.raises(ValueError):
            driver.set_positions({0: 1500.0, 1: 100.0})  # 100 invalid
        # NICHTS darf geschrieben worden sein
        assert bytes(fake.written) == b""


# ---------- Get Position ----------


class TestGetPosition:
    def test_writes_correct_command(self) -> None:
        driver, fake = make_driver()
        fake.queue_response(0x70, 0x17)  # = 6000 qus = 1500 us
        driver.get_position(5)
        assert bytes(fake.written) == bytes([CMD_GET_POSITION, 5])

    def test_decodes_response_correctly(self) -> None:
        driver, fake = make_driver()
        fake.queue_response(0x70, 0x17)
        assert driver.get_position(0) == 1500.0

    def test_raises_on_short_response(self) -> None:
        driver, fake = make_driver()
        fake.queue_response(0x70)  # nur 1 Byte
        with pytest.raises(MaestroError, match="2 Bytes"):
            driver.get_position(0)


# ---------- Get Errors ----------


class TestGetErrors:
    def test_writes_correct_command(self) -> None:
        driver, fake = make_driver()
        fake.queue_response(0x00, 0x00)
        driver.get_errors()
        assert bytes(fake.written) == bytes([CMD_GET_ERRORS])

    def test_decodes_no_errors(self) -> None:
        driver, fake = make_driver()
        fake.queue_response(0x00, 0x00)
        assert driver.get_errors() == 0

    def test_decodes_error_flags(self) -> None:
        driver, fake = make_driver()
        # Bit 5 gesetzt = "Serial Protocol Error" (Beispiel)
        fake.queue_response(0x20, 0x00)
        assert driver.get_errors() == 0x0020


# ---------- Disable ----------


class TestDisable:
    def test_writes_set_target_zero(self) -> None:
        driver, fake = make_driver()
        driver.disable(7)
        # 0x84 7 0 0 — Position 0 deaktiviert
        assert bytes(fake.written) == bytes([CMD_SET_TARGET, 7, 0, 0])


# ---------- Lifecycle ----------


class TestLifecycle:
    def test_close_disables_all_channels(self) -> None:
        driver, fake = make_driver(num_channels=3)
        driver.close()
        # 3x Set-Target ch=0/1/2 mit Position 0
        expected = bytes([
            CMD_SET_TARGET, 0, 0, 0,
            CMD_SET_TARGET, 1, 0, 0,
            CMD_SET_TARGET, 2, 0, 0,
        ])
        assert bytes(fake.written) == expected
        assert driver.is_closed
        assert fake.closed

    def test_close_idempotent(self) -> None:
        driver, _ = make_driver()
        driver.close()
        driver.close()  # darf nicht crashen

    def test_operations_after_close_raise(self) -> None:
        driver, _ = make_driver()
        driver.close()
        with pytest.raises(RuntimeError, match="bereits geschlossen"):
            driver.set_position(0, 1500.0)

    def test_context_manager(self) -> None:
        fake = FakeSerial()
        with MaestroDriver(port="/dev/fake", num_channels=2, ser=fake) as driver:
            driver.set_position(0, 1500.0)
        assert driver.is_closed


# ---------- Init: Speed/Acceleration-Semantik ----------


class TestInitSpeedAccel:
    """Default 0 = unbegrenzt und wird AKTIV gesendet; None = nichts senden."""

    def test_none_sends_nothing(self) -> None:
        # None: Init fasst Speed/Accel nicht an (sauberes Byte-Protokoll).
        fake = FakeSerial()
        MaestroDriver(
            port="/dev/fake", num_channels=24, ser=fake,
            initial_speed=None, initial_acceleration=None,
        )
        assert bytes(fake.written) == b""

    def test_zero_is_sent_to_clear_stored_limit(self) -> None:
        # 0 = unbegrenzt, MUSS aber gesendet werden, damit eine evtl. im
        # Maestro aus einer frueheren Sitzung gespeicherte Begrenzung weg ist.
        fake = FakeSerial()
        n = 6
        MaestroDriver(
            port="/dev/fake", num_channels=n, ser=fake,
            initial_speed=0, initial_acceleration=0,
        )
        # Pro Kanal je ein Speed- und ein Acceleration-Befehl (je 4 Bytes).
        assert len(fake.written) == n * 4 * 2
        assert CMD_SET_SPEED in fake.written
        assert CMD_SET_ACCELERATION in fake.written
        # Erster Befehl: Speed ch0 = 0 -> 0x87 0x00 0x00 0x00
        assert bytes(fake.written[:4]) == bytes([CMD_SET_SPEED, 0, 0, 0])

    def test_default_is_unlimited_and_sent(self) -> None:
        # Default-Konstruktor (ohne explizite Werte) sendet 0/0 = unbegrenzt.
        fake = FakeSerial()
        MaestroDriver(port="/dev/fake", num_channels=4, ser=fake)
        assert len(fake.written) == 4 * 4 * 2
        assert bytes(fake.written[:4]) == bytes([CMD_SET_SPEED, 0, 0, 0])
