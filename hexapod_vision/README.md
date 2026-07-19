# Hexapod Vision — GPU-Client (YOLO11, Tracking, Tiefe)

Computer-Vision-Umgebung für den Hexapod: **Objekterkennung (YOLO11)**,
**Tracking** (ByteTrack/BoT-SORT) und **monokulare Tiefe/Hindernisse**
(Depth Anything V2). Läuft auf einem separaten Linux-Rechner mit NVIDIA-GPU
(entwickelt und getestet unter CachyOS/Arch mit einer RTX 4070 Super, 12 GB —
zu anderen Distributionen siehe unten).

Die Schwerlast-Modelle laufen bewusst auf dem GPU-Rechner, nicht auf dem Pi —
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

Einzige distro-spezifische Voraussetzung ist ein funktionierender
NVIDIA-Treiber. Unter Arch/CachyOS (für Ada-Karten wie die 4070 Super ist
**nvidia-open** empfohlen):

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

## Installation (CachyOS/Arch)

Das Setup-Skript heißt bewusst `setup_vision_cachyos.sh`, weil es nur auf
CachyOS/Arch läuft (es installiert Systempakete per `pacman`):

```bash
cd hexapod_vision
chmod +x setup_vision_cachyos.sh
./setup_vision_cachyos.sh
```

Das Skript ist **idempotent** — mehrfaches Ausführen ist unschädlich. Es legt eine
lokale `.venv/` an, installiert alles hinein und lädt die YOLO11-Gewichte vor.

## Andere Distributionen

Nur die Systempakete und der Treiber unterscheiden sich — alles Weitere
(uv, venv, PyTorch mit mitgelieferter CUDA-Runtime) ist distro-unabhängig.
Manuell statt Skript:

```bash
# 1. Systempakete (Beispiele)
sudo apt install git build-essential ffmpeg      # Ubuntu/Debian
sudo dnf install git gcc gcc-c++ make ffmpeg     # Fedora (Treiber: RPM Fusion, akmod-nvidia)

# 2. uv installieren: https://docs.astral.sh/uv/
# 3. venv + Pakete (identisch zu den Schritten im Skript):
uv venv --python 3.13 .venv
uv pip install --index-url https://download.pytorch.org/whl/cu124 torch torchvision
uv pip install "ultralytics>=8.3.0" "lap>=0.5.12" "transformers>=4.44.0" \\
    "opencv-python>=4.10.0" "pillow>=10.0.0" "numpy<2.2"
```

Hinweise: AMD-GPUs bräuchten die ROCm-Wheels von PyTorch (nicht getestet);
ohne GPU läuft alles auf CPU, Depth Anything wird dann allerdings langsam.

## Funktionstest

```bash
source .venv/bin/activate          # bash/zsh
source .venv/bin/activate.fish     # fish (CachyOS-Standard)
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
