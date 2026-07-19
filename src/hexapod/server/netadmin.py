"""WLAN-Verwaltung ueber NetworkManager (nmcli).

Read-only Status/Scan brauchen keine Sonderrechte; connect/hotspot/disconnect
laufen ueber eine polkit-Regel, die dem Dienst-User NM-Steuerung erlaubt.

Alle nmcli-Aufrufe nutzen argv-Listen (kein shell=True) -- SSID/Passwort sind
damit injektionssicher. Der iface-Name wird zusaetzlich gegen die real
vorhandenen WLAN-Geraete geprueft.
"""
from __future__ import annotations

import subprocess
from typing import Any

TIMEOUT = 25.0


class NetError(RuntimeError):
    pass


def _run(args: list[str], timeout: float = TIMEOUT) -> str:
    try:
        p = subprocess.run(["nmcli", *args], capture_output=True, text=True,
                           timeout=timeout)
    except subprocess.TimeoutExpired as e:
        raise NetError(f"nmcli Timeout: {' '.join(args)}") from e
    if p.returncode != 0:
        raise NetError((p.stderr or p.stdout or "nmcli Fehler").strip())
    return p.stdout


def split_terse(line: str) -> list[str]:
    """Zerlegt eine nmcli -t Zeile an unescapten ':' (Werte escapen ':' als '\\:')."""
    out, cur, i = [], [], 0
    while i < len(line):
        c = line[i]
        if c == "\\" and i + 1 < len(line):
            cur.append(line[i + 1])
            i += 2
            continue
        if c == ":":
            out.append("".join(cur))
            cur = []
            i += 1
            continue
        cur.append(c)
        i += 1
    out.append("".join(cur))
    return out


def wifi_ifaces() -> list[str]:
    """Nur echte WLAN-Geraete (kein p2p-dev)."""
    out = []
    for line in _run(["-t", "-f", "DEVICE,TYPE", "device"]).splitlines():
        f = split_terse(line)
        if len(f) >= 2 and f[1] == "wifi" and not f[0].startswith("p2p"):
            out.append(f[0])
    return out


def _conn_mode(conn: str) -> str:
    if not conn:
        return ""
    try:
        out = _run(["-t", "-f", "802-11-wireless.mode", "connection", "show", conn])
    except NetError:
        return ""
    for line in out.splitlines():
        f = split_terse(line)
        if len(f) >= 2:
            return f[1]  # 'infrastructure' | 'ap'
    return ""


def device_detail(iface: str) -> dict[str, Any]:
    out = _run(["-t", "-f", "GENERAL.CONNECTION,GENERAL.STATE,IP4.ADDRESS",
                "device", "show", iface])
    conn, state, ip = "", "", ""
    for line in out.splitlines():
        f = split_terse(line)
        if len(f) < 2:
            continue
        key, val = f[0], f[1]
        if key == "GENERAL.CONNECTION":
            conn = val
        elif key == "GENERAL.STATE":
            state = val  # z.B. '100 (connected)'
        elif key.startswith("IP4.ADDRESS"):
            ip = val
    mode_raw = _conn_mode(conn) if conn else ""
    mode = "ap" if mode_raw == "ap" else ("client" if conn else "off")
    connected = state.startswith("100")
    return {"iface": iface, "connection": conn, "state": state,
            "ip": ip, "mode": mode, "connected": connected}


def status() -> list[dict[str, Any]]:
    return [device_detail(i) for i in wifi_ifaces()]


def parse_scan(raw: str) -> list[dict[str, Any]]:
    """nmcli -t -f ACTIVE,SSID,SIGNAL,SECURITY ... -> deduplizierte Netzliste."""
    best: dict[str, dict[str, Any]] = {}
    for line in raw.splitlines():
        f = split_terse(line)
        if len(f) < 4:
            continue
        active, ssid, signal, sec = f[0], f[1], f[2], f[3]
        if not ssid:
            continue  # versteckte SSID ueberspringen
        try:
            sig = int(signal)
        except ValueError:
            sig = 0
        cur = best.get(ssid)
        if cur is None or sig > cur["signal"]:
            best[ssid] = {"ssid": ssid, "signal": sig,
                          "security": sec or "open", "active": active == "yes"}
        elif active == "yes":
            best[ssid]["active"] = True
    return sorted(best.values(), key=lambda n: -n["signal"])


def scan(iface: str, rescan: bool = True) -> list[dict[str, Any]]:
    _check_iface(iface)
    raw = _run(["-t", "-f", "ACTIVE,SSID,SIGNAL,SECURITY", "device", "wifi",
                "list", "ifname", iface, "--rescan", "yes" if rescan else "no"])
    return parse_scan(raw)


def _check_iface(iface: str) -> None:
    if iface not in wifi_ifaces():
        raise NetError(f"Unbekanntes WLAN-Geraet: {iface!r}")


def connect(iface: str, ssid: str, password: str = "") -> str:
    _check_iface(iface)
    if not ssid:
        raise NetError("SSID fehlt")
    args = ["device", "wifi", "connect", ssid, "ifname", iface]
    if password:
        args += ["password", password]
    return _run(args, timeout=40.0)


def hotspot(iface: str, ssid: str, password: str) -> str:
    _check_iface(iface)
    if not ssid:
        raise NetError("SSID fehlt")
    if len(password) < 8:
        raise NetError("Hotspot-Passwort braucht mindestens 8 Zeichen")
    return _run(["device", "wifi", "hotspot", "ifname", iface,
                 "ssid", ssid, "password", password], timeout=40.0)


def disconnect(iface: str) -> str:
    _check_iface(iface)
    return _run(["device", "disconnect", iface], timeout=30.0)
