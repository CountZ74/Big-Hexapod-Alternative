# Hexapod

Eine saubere Steuerungssoftware für den Freenove Big Hexapod, neu geschrieben mit:
- Pololu Mini Maestro 24-Channel als Servo-Controller (USB)
- Sauberer Trennung zwischen Hardware-Treiber, Kinematik, Gangart, Verhalten
- Konfiguration über YAML (`config/robot.yaml`)

## Entwicklung

```bash
uv sync --all-groups   # Dependencies installieren
uv run pytest          # Tests laufen lassen
uv run mypy src        # Typen prüfen
uv run ruff check src  # Linting
```

## Projekt-Setup auf einem neuen Rechner

Das zentrale Repo liegt auf dem hauseigenen Gitea-Server (LXC 116 auf proxmox1):

```bash
git clone https://github.com/CountZ74/Big-Hexapod-Alternative.git
cd hexapod
uv sync --all-groups
uv run pytest
```

Login/Push gegen Gitea mit dem Benutzer `sebi`. Von ausserhalb des Heimnetzes
ueber Tailscale/VPN.

## Webserver lokal (Simulator, ohne Hardware)

```bash
HEXAPOD_DRIVER=simulator uv run python -m hexapod.server
# -> http://localhost:8000
```

## Betrieb auf dem Roboter (Raspberry Pi)

Der Webserver laeuft auf dem Pi als systemd-Dienst `hexapod-web` (Autostart,
echter Maestro-Treiber, Port 8000):

```bash
sudo systemctl status hexapod-web     # Status
sudo systemctl restart hexapod-web    # nach Code-Aenderungen (git pull) neu laden
sudo journalctl -u hexapod-web -f     # Logs live
```

Der Roboter-Zustand wird ueber Neustarts hinweg in `/tmp/hexapod_robot_state`
gehalten; nach einem echten Reboot/Stromausfall startet er sauber in `off`.
