# Big Hexapod Alternative

Eine alternative, komplett neu geschriebene Steuerungssoftware für den
**Freenove Big Hexapod Robot Kit** (Raspberry Pi) — in Python, mit sauberer
Schichtenarchitektur, vollständiger Testabdeckung im Simulator und einer
Web-UI zur Steuerung.

*An alternative, from-scratch control stack for the Freenove Big Hexapod.
Docs and code comments are in German.*

> **Wichtig / Hardware-Voraussetzung:** Diese Software steuert die Servos über
> einen **Pololu Mini Maestro 24-Channel** (USB) statt über das originale
> Freenove-Servoboard. Ein Treiber für den originalen **PCA9685**-Controller
> existiert **noch nicht** — das Treiber-Interface (`drivers/base.py`) ist
> dafür vorbereitet, Beiträge sind willkommen. Ohne Maestro läuft die Software
> nur im Simulator.

## Features

- **Kinematik:** Inverse Kinematik pro Bein und für die Körperpose
  (Translation/Rotation bei stehenden Füßen)
- **Gangarten:** Tripod-Gait, kontinuierlicher kommandierbarer Gait
  (Joystick), sanftes Aufstehen/Hinlegen, Kletter-Sequenz für Stufen
- **Gesten:** Winken, Mantis-Pose, Bein heben u. a.
- **Sensorik:** MPU6050 (Selbstnivellierung im Stand), HC-SR04-Sonar
  (Hindernis-Stopp + Sweep-Scan), Akkuüberwachung, Kamera
- **Web-UI:** Dashboard mit Joystick, Pose-Slidern, Kamerabild und
  Netzwerk-Verwaltung (WLAN/Hotspot); zusätzlich Gamepad-Seite
- **Extras:** Android-App (`hexapod_android/`), CV-Follow-Client mit
  YOLO11 (`hexapod_vision/`), ESP32-CRSF-Funkbrücke (`tools/`)

## Architektur

```
Server/Web-UI / Android / CLI / Vision   <- was der Roboter "tut"
Gait Engine (tripod, gestures, climb, ...) <- Beine koordinieren
Trajectory + Executor                     <- Bahnen erzeugen, zeitgesteuert senden
Kinematik (leg_ik, body_ik)               <- Fusspunkt/Pose -> Gelenkwinkel
Driver (maestro | simulator)              <- Hardware austauschbar
```

Alle Bewegungslogik spricht nur das abstrakte Treiber-Interface; Positionen
werden projektweit in Mikrosekunden Pulsweite ausgedrückt.

## Schnellstart (ohne Hardware)

Benötigt Python 3.13 und [uv](https://docs.astral.sh/uv/):

```bash
git clone https://github.com/CountZ74/Big-Hexapod-Alternative.git
cd Big-Hexapod-Alternative
uv sync --all-groups
uv run pytest                  # komplette Testsuite laeuft im Simulator
HEXAPOD_DRIVER=simulator uv run python -m hexapod.server
# -> http://localhost:8000
```

## Betrieb auf dem Roboter

Voraussetzungen: Raspberry Pi im Freenove-Chassis, Mini Maestro per USB,
18x MG996R verkabelt, udev-Regeln aus `deploy/` installiert
(stabile Symlinks `/dev/maestro_cmd` / `/dev/maestro_ttl`).

Kanalzuordnung und Geometrie werden in `config/robot.yaml` konfiguriert;
Kalibrierung über die CLI: `uv run hexapod calibrate`.

Als Dauerbetrieb empfiehlt sich ein systemd-Dienst, der
`python -m hexapod.server` startet (Port 8000). Der Roboter-Zustand wird über
Server-Neustarts hinweg in `/tmp/hexapod_robot_state` gehalten; nach einem
Reboot startet er sauber in `off`.

**Sicherheitshinweis:** Nach dem Einschalten kennt der Controller die
mechanische Lage der Beine nicht. Die Software fährt deshalb beim Aufstehen
erst mit begrenzter Servo-Geschwindigkeit an (`prime()` -> `power_up`).
Neue Bewegungen immer zuerst im Simulator testen.

## Entwicklung

```bash
uv run pytest          # Tests (ohne Hardware)
uv run mypy src        # strict, Projekt ist typenrein
uv run ruff check src  # Linting
```

Commit-Konvention: deutsche Conventional Commits (`feat(gait): ...`).

## Status & Roadmap

- [x] Maestro-Treiber, IK, Tripod-Gait, Web-UI, Gesten, Klettern
- [ ] PCA9685-Treiber fuer das originale Freenove-Board
- [ ] Weitere Gangarten (Wave, Ripple)

## Lizenz & Hinweis

MIT-Lizenz (siehe `LICENSE`). Dieses Projekt steht in keiner Verbindung zu
Freenove oder Pololu; alle Produktnamen sind Eigentum ihrer Inhaber.
