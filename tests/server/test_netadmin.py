"""Tests fuer die nmcli-Terse-Parser (reine Funktionen, keine Hardware)."""
from __future__ import annotations

from hexapod.server.netadmin import split_terse, parse_scan


class TestSplitTerse:
    def test_plain(self) -> None:
        assert split_terse("wlan0:wifi:connected:dd-wrt") == \
            ["wlan0", "wifi", "connected", "dd-wrt"]

    def test_escaped_colon(self) -> None:
        # SSID mit ':' wird von nmcli als '\:' escaped
        assert split_terse(r"yes:my\:ssid:80:WPA2") == \
            ["yes", "my:ssid", "80", "WPA2"]

    def test_trailing_empty(self) -> None:
        assert split_terse("wlan1:wifi:disconnected:") == \
            ["wlan1", "wifi", "disconnected", ""]


class TestParseScan:
    RAW = "\n".join([
        "no:dd-wrt:100:WPA2",
        "no:dd-wrt:84:WPA2",
        "yes:dd-wrt:82:WPA2",
        "no::77:WPA2",             # versteckte SSID -> raus
        "no:MeinRouter:67:WPA2",
        "no:Offen:50:",            # offen -> security 'open'
    ])

    def test_dedup_keeps_strongest(self) -> None:
        nets = parse_scan(self.RAW)
        by = {n["ssid"]: n for n in nets}
        assert by["dd-wrt"]["signal"] == 100

    def test_active_flag_survives_dedup(self) -> None:
        # dd-wrt ist in einer (schwaecheren) Zeile aktiv -> Flag muss bleiben
        by = {n["ssid"]: n for n in parse_scan(self.RAW)}
        assert by["dd-wrt"]["active"] is True

    def test_hidden_skipped(self) -> None:
        assert all(n["ssid"] for n in parse_scan(self.RAW))

    def test_open_security(self) -> None:
        by = {n["ssid"]: n for n in parse_scan(self.RAW)}
        assert by["Offen"]["security"] == "open"

    def test_sorted_by_signal(self) -> None:
        sigs = [n["signal"] for n in parse_scan(self.RAW)]
        assert sigs == sorted(sigs, reverse=True)
