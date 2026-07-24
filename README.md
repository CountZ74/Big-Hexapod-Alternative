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
- **Vier Gangarten:** Tripod (3+3, schnell), Tetrapod (2+2+2),
  Ripple (kontralateral) und Wave (metachronal, maximal stabil) —
  über eine gemeinsame Phasen-Mechanik definiert und zur Laufzeit umschaltbar
- **Bewegungen:** kontinuierlicher, kommandierbarer Gait (Joystick), sanftes
  Aufstehen/Hinlegen, Kletter-Sequenz für Stufen, Gesten (Winken, Mantis-Pose,
  Bein heben u. a.)
- **Sensorik:** MPU6050 (Selbstnivellierung im Stand), Akkuüberwachung,
  Kamera; HC-SR04-Sonar für Hindernis-Stopp und Sweep-Scan ist integriert,
  aber noch in Erprobung
- **Web-UI:** Dashboard mit Joystick, Pose-Slidern, Kamerabild und
  Netzwerk-Verwaltung (WLAN/Hotspot); zusätzlich Gamepad-Seite
- **CLI:** `uv run hexapod ...` mit u. a. `calibrate`, `walk`, `move`, `pose`,
  `trim`, `status`

## Fernsteuerung mit RC-Sender (ESP32-Modul)

`tools/esp32_crsf_wifi/` enthält die Firmware für ein **CRSF→WiFi-Modul** auf
Basis des Seeed XIAO ESP32-C6: Es steckt im externen Modulschacht einer
RC-Fernsteuerung (z. B. RadioMaster TX16S mit EdgeTX), liest die CRSF-Kanäle
direkt aus dem Schacht und sendet `walk`/`halt`/`pose`/`set_gait` per WLAN an
den Hexapod-Webserver — inklusive Failsafe. Nur drei Drähte, Versorgung aus
dem Modulschacht. Details, Verkabelung und Pin-Warnungen im dortigen README.

Alternativ liegen in `tools/` Python-Referenzimplementierungen
(`hexapod_crsf.py`, `hexapod_controller.py`) für den PC.

## Computer Vision & Follow-Modus

`hexapod_vision/` ist ein GPU-Client, der auf einem separaten Rechner läuft
(der Pi streamt nur sein Kamerabild): **YOLO11** für Objekt-/Personenerkennung,
**ByteTrack/BoT-SORT** fürs Tracking und **Depth Anything V2** für monokulare
Tiefenschätzung. Der `follow_client.py` schließt den Regelkreis: Person im Bild
verfolgen und dem Roboter Bewegungskommandos schicken. Setup-Anleitung
(PyTorch/CUDA via uv) im dortigen README.

## Android-App

`hexapod_android/` enthält eine native Android-App (Kotlin) als Alternative
zur Web-UI.

## Architektur

```
Server/Web-UI / Android / CLI / Vision / RC  <- was der Roboter "tut"
Gait Engine (gaits, gestures, climb, ...)    <- Beine koordinieren
Trajectory + Executor                        <- Bahnen erzeugen, zeitgesteuert senden
Kinematik (leg_ik, body_ik)                  <- Fusspunkt/Pose -> Gelenkwinkel
Driver (maestro | simulator)                 <- Hardware austauschbar
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

## Kalibrierung & Startposition

Die **initiale Kalibrierung** erfolgt in einer definierten mechanischen
Referenzlage: Der Roboter liegt auf der Chassis-Unterseite, alle sechs Beine
**flach radial nach außen gestreckt**. In dieser Position wird jedem
Maestro-Kanal per `uv run hexapod calibrate` der zugehörige Pulsweiten-Wert
zugeordnet (Zuordnung Kanal ↔ Gelenk siehe `servo_mapper/`).

Aus dieser Lage heraus ist auch der **Kaltstart** definiert: Femur hoch,
Tibia runter, Füße über die Standpunkte, dann langsames Heben des Körpers
(`power_up`) — alles mit begrenzter Servo-Geschwindigkeit.

Die **Feineinstellung** einzelner Gelenke erfolgt später im Betrieb über
`uv run hexapod trim` bzw. `stance-trim`. Ein 3D-druckbares **Kalibrierungsrig**
für reproduzierbare Referenzwinkel liegt unter [`tools/CalibrationJig.stl`](tools/CalibrationJig.stl).

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

- [x] Maestro-Treiber, IK, vier Gangarten (Tripod/Tetrapod/Ripple/Wave)
- [x] Web-UI, Gesten, Klettern, Selbstnivellierung
- [x] RC-Fernsteuerung via ESP32-CRSF-Modul, Android-App
- [x] Vision-Pipeline (YOLO11 + Tracking + Tiefe) mit Follow-Modus
- [ ] Sonar-Hindernisvermeidung fertigstellen (Hardware-Verifikation)
- [ ] Fußsensoren (Bodenkontakt-Erkennung)
- [x] Kalibrierungsrig-STL veröffentlichen
- [ ] PCA9685-Treiber fuer das originale Freenove-Board

## Lizenz & Hinweis

MIT-Lizenz (siehe `LICENSE`). Dieses Projekt steht in keiner Verbindung zu
Freenove oder Pololu; alle Produktnamen sind Eigentum ihrer Inhaber.
