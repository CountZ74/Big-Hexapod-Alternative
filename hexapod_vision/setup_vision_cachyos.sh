#!/usr/bin/env bash
#
# setup_vision_cachyos.sh
# -----------------------
# Richtet die Computer-Vision-Umgebung fuer den Hexapod auf CachyOS (Arch-based)
# mit NVIDIA RTX 4070 Super ein.
#
# Stack:
#   - NVIDIA-Treiber-Check (kein CUDA-Toolkit noetig: PyTorch bringt CUDA-Runtime mit)
#   - uv (passt zur uv-Struktur des hexapod-Repos)
#   - PyTorch + torchvision (CUDA 12.4 wheels)
#   - Ultralytics  -> YOLO11 Objekterkennung + integriertes Tracking (ByteTrack/BoT-SORT)
#   - Depth Anything V2 (via transformers) -> monokulare Tiefe / Hindernisse
#   - OpenCV
#
# Das Skript ist idempotent: mehrfaches Ausfuehren ist unschaedlich.
#
# Aufruf:
#   chmod +x setup_vision_cachyos.sh
#   ./setup_vision_cachyos.sh
#
set -euo pipefail

# ---------------------------------------------------------------------------
# Hilfsfunktionen
# ---------------------------------------------------------------------------
BOLD="$(tput bold 2>/dev/null || true)"
RESET="$(tput sgr0 2>/dev/null || true)"
log()  { echo "${BOLD}==>${RESET} $*"; }
warn() { echo "${BOLD}!! ${RESET} $*" >&2; }
die()  { echo "${BOLD}xx ${RESET} $*" >&2; exit 1; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ---------------------------------------------------------------------------
# 1. Sanity-Checks
# ---------------------------------------------------------------------------
[[ "$(uname -s)" == "Linux" ]] || die "Dieses Skript ist fuer Linux/CachyOS gedacht."
command -v pacman >/dev/null 2>&1 || warn "pacman nicht gefunden -- ist das wirklich CachyOS/Arch? Fahre trotzdem fort."

# ---------------------------------------------------------------------------
# 2. NVIDIA-Treiber pruefen
# ---------------------------------------------------------------------------
log "Pruefe NVIDIA-Treiber ..."
if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader || true
else
    warn "nvidia-smi nicht verfuegbar oder kein funktionierender Treiber."
    warn "Installiere den proprietaeren NVIDIA-Treiber (Ada/RTX 4070 Super -> nvidia-open empfohlen):"
    warn "    sudo pacman -S --needed nvidia-open-dkms nvidia-utils"
    warn "  (oder, je nach Kernel: nvidia-dkms). Danach EINMAL neu starten und dieses Skript erneut ausfuehren."
    read -r -p "Trotzdem ohne GPU-Treiber fortfahren (CPU-Fallback)? [y/N] " ans
    [[ "${ans,,}" == "y" ]] || die "Abgebrochen. Bitte erst den Treiber einrichten."
fi

# ---------------------------------------------------------------------------
# 3. Systempakete (minimal: git, uv-Voraussetzungen, OpenCV-Laufzeit-Libs)
# ---------------------------------------------------------------------------
log "Stelle Basis-Systempakete sicher ..."
if command -v pacman >/dev/null 2>&1; then
    sudo pacman -S --needed --noconfirm git base-devel ffmpeg \
        || warn "pacman-Installation teilweise fehlgeschlagen -- pruefe manuell."
fi

# ---------------------------------------------------------------------------
# 4. uv installieren (falls nicht vorhanden)
# ---------------------------------------------------------------------------
if ! command -v uv >/dev/null 2>&1; then
    log "Installiere uv ..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    # uv landet in ~/.local/bin
    export PATH="$HOME/.local/bin:$PATH"
fi
command -v uv >/dev/null 2>&1 || die "uv konnte nicht installiert werden."
log "uv: $(uv --version)"

# ---------------------------------------------------------------------------
# 5. Virtuelle Umgebung + Python
# ---------------------------------------------------------------------------
PYTHON_VERSION="3.12"
log "Erstelle venv (.venv) mit Python ${PYTHON_VERSION} ..."
uv venv --python "${PYTHON_VERSION}" .venv
# shellcheck disable=SC1091
source .venv/bin/activate

# ---------------------------------------------------------------------------
# 6. PyTorch mit CUDA 12.4
# ---------------------------------------------------------------------------
log "Installiere PyTorch (CUDA 12.4) ..."
uv pip install --index-url https://download.pytorch.org/whl/cu124 \
    torch torchvision

# ---------------------------------------------------------------------------
# 7. Vision-Pakete
# ---------------------------------------------------------------------------
log "Installiere Ultralytics, transformers, OpenCV ..."
uv pip install \
    "ultralytics>=8.3.0" \
    "lap>=0.5.12" \
    "transformers>=4.44.0" \
    "opencv-python>=4.10.0" \
    "pillow>=10.0.0" \
    "numpy<2.2"
# 'lap' wird von Ultralytics fuer das Tracking (ByteTrack/BoT-SORT) benoetigt.
# Vorab via uv installieren, da das uv-venv kein pip fuer Laufzeit-AutoUpdate hat.

# ---------------------------------------------------------------------------
# 8. Modellgewichte vorab laden (YOLO11)
# ---------------------------------------------------------------------------
log "Lade YOLO11-Gewichte vor (yolo11n + yolo11s) ..."
mkdir -p models
python - <<'PY'
from ultralytics import YOLO
for name in ("yolo11n.pt", "yolo11s.pt"):
    try:
        YOLO(name)
        print(f"  geladen: {name}")
    except Exception as e:  # noqa: BLE001
        print(f"  WARN konnte {name} nicht laden: {e}")
PY

log "Hinweis: Depth-Anything-V2-Gewichte werden beim ersten Smoke-Test automatisch"
log "von HuggingFace geladen (depth-anything/Depth-Anything-V2-Small-hf, ~100 MB)."

# ---------------------------------------------------------------------------
# 9. Fertig
# ---------------------------------------------------------------------------
log "Installation abgeschlossen."
echo
echo "Naechste Schritte:"
echo "  # fish (CachyOS-Default):"
echo "  source ${SCRIPT_DIR}/.venv/bin/activate.fish"
echo "  # bash/zsh:"
echo "  source ${SCRIPT_DIR}/.venv/bin/activate"
echo "  # danach:"
echo "  python ${SCRIPT_DIR}/vision_smoketest.py"
echo "  # oder ohne Aktivieren (shell-egal):"
echo "  ${SCRIPT_DIR}/.venv/bin/python ${SCRIPT_DIR}/vision_smoketest.py"
echo
