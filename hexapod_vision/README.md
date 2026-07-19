# Hexapod Vision — Setup auf CachyOS (RTX 4070 Super)

Computer-Vision-Umgebung für den Hexapod: **Objekterkennung (YOLO11)**,
**Tracking** (ByteTrack/BoT-SORT) und **monokulare Tiefe/Hindernisse**
(Depth Anything V2). Läuft auf dem Hauptrechner unter CachyOS, GPU-beschleunigt
über die RTX 4070 Super (12 GB).

Die Schwerlast-Modelle laufen bewusst auf dem 4070-Rechner, nicht auf dem Pi —
der Roboter streamt sein Kamerabild her und bekommt Erkennungen/Tiefe zurück.

## Was installiert wird

| Komponente | Zweck |
|---|---|
| NVIDIA-Treiber (System) | GPU-Zugriff; **kein** CUDA-Toolkit nötig — PyTorch bringt die CUDA-Runtime mit |
| `uv` | Paket-/Env-Manager (gleiche Toolchain wie das `hexapod`-Repo) |
| PyTorch + torchvision (cu124) | Inferenz-Backend auf der GPU |
| Ultralytics (YOLO11) | Objekt-/Personenerkennung **und** Tracking |
| Depth Anything V2 (via transformers) | monokulare Tiefenschätzung für Hindernisse |
| OpenCV | Bild-/Video-I/O |

## Voraussetzung: NVIDIA-Treiber

CachyOS ist Arch-basiert. Für die RTX 4070 Super (Ada) ist der **nvidia-open**-Treiber empfohlen:

```bash
sudo pacman -S --needed nvidia-open-dkms nvidia-utils
# danach einmal neu starten
reboot
```

Prüfen:

```bash
nvidia-smi   # muss Karte + Treiberversion zeigen
```

> Falls du einen Custom-Kernel von CachyOS nutzt: `nvidia-open-dkms` baut das Modul
> passend zum Kernel. Bei reinem Stock-Kernel geht alternativ `nvidia-open`.

## Installation

```bash
cd hexapod_vision
chmod +x setup_vision_cachyos.sh
./setup_vision_cachyos.sh
```

Das Skript ist **idempotent** — mehrfaches Ausführen ist unschädlich. Es legt eine
lokale `.venv/` an, installiert alles hinein und lädt die YOLO11-Gewichte vor.

## Funktionstest

CachyOS nutzt standardmäßig **fish**:

```fish
source .venv/bin/activate.fish
python vision_smoketest.py
```

Unter bash/zsh stattdessen `source .venv/bin/activate`. Shell-unabhängig geht
auch ohne Aktivieren: `.venv/bin/python vision_smoketest.py`.

Erwartete Ausgabe: drei grüne Stufen — **CUDA**, **YOLO** (Detektion + Tracking),
**Depth**. Der Test lädt beim ersten Lauf ein Beispielbild (`bus.jpg`) und die
Depth-Gewichte (~100 MB) automatisch herunter.

Eigenes Bild testen:

```bash
python vision_smoketest.py --image /pfad/zum/bild.jpg
# Tiefe überspringen (nur Erkennung):
python vision_smoketest.py --skip-depth
```

## Integration ins `hexapod`-Repo

Das Verzeichnis ist so gebaut, dass es als Unterordner ins zentrale Gitea-Repo
(`https://github.com/CountZ74/Big-Hexapod-Alternative.git`) passt:

```bash
git clone https://github.com/CountZ74/Big-Hexapod-Alternative.git
cp -r hexapod_vision hexapod/
cd hexapod && git add hexapod_vision && git commit -m "feat(vision): CV-Setup fuer CachyOS (YOLO11 + Depth Anything)"
git push
```

`.venv/`, `models/`, `*.pt` und `sample.jpg` gehören **nicht** ins Repo — die
beiliegende `.gitignore` schließt sie aus.

## Nächste Schritte (Vorschläge)

1. **Kamera-Bridge:** Der Pi-Webserver (`src/hexapod/server/camera.py`) streamt
   bereits MJPEG. Ein kleiner Client auf dem 4070-Rechner zieht den Stream,
   lässt YOLO+Depth laufen und schickt Ziele (Bounding-Box-Zentrum, Distanz)
   zurück an die Motion-Control.
2. **Personen-Following:** YOLO-Track-ID + horizontaler Offset → Drehbefehl;
   Depth am Box-Zentrum → Vor/Zurück.
3. **Hindernisvermeidung:** Tiefenkarte im unteren Bildbereich schwellwerten →
   Stop/Ausweichen in die Gangart einspeisen.

Sag Bescheid, welchen dieser Schritte ich als nächstes ausbauen soll — den
Kamera-Client kann ich passend zur bestehenden `motion_control.py` schreiben.
