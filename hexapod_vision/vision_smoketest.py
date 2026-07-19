#!/usr/bin/env python3
"""
vision_smoketest.py
-------------------
Prueft die Computer-Vision-Umgebung des Hexapods auf dem CachyOS-Rechner
(RTX 4070 Super). Drei Stufen:

  1. CUDA/PyTorch  -- ist die GPU sichtbar und nutzbar?
  2. YOLO11        -- Objekterkennung + Tracking (ByteTrack) auf einem Testbild.
  3. Depth Anything V2 -- monokulare Tiefenschaetzung auf demselben Bild.

Aufruf (nach Aktivierung der venv):
    python vision_smoketest.py
    python vision_smoketest.py --image /pfad/zum/bild.jpg

Exit-Code 0 = alles ok, sonst Anzahl fehlgeschlagener Stufen.
"""
from __future__ import annotations

import argparse
import sys
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
SAMPLE = HERE / "sample.jpg"
SAMPLE_URL = "https://ultralytics.com/images/bus.jpg"

PASS = "[ OK ]"
FAIL = "[FAIL]"


def ensure_sample(image: Path) -> Path:
    """Stellt sicher, dass ein Testbild vorhanden ist (laedt es sonst herunter)."""
    if image.exists():
        return image
    print(f"  Lade Testbild von {SAMPLE_URL} ...")
    urllib.request.urlretrieve(SAMPLE_URL, image)  # noqa: S310
    return image


def check_cuda() -> bool:
    print("\n=== 1. CUDA / PyTorch ===")
    try:
        import torch
    except Exception as e:  # noqa: BLE001
        print(f"{FAIL} torch-Import fehlgeschlagen: {e}")
        return False

    print(f"  torch {torch.__version__}")
    if not torch.cuda.is_available():
        print(f"{FAIL} CUDA NICHT verfuegbar -- Treiber/Wheels pruefen (laeuft sonst nur auf CPU).")
        return False

    dev = torch.cuda.get_device_name(0)
    cap = torch.cuda.get_device_capability(0)
    total_gb = torch.cuda.get_device_properties(0).total_memory / 1e9
    print(f"  GPU: {dev}  (compute {cap[0]}.{cap[1]}, {total_gb:.1f} GB)")

    # Kleiner Matmul-Lasttest auf der GPU
    x = torch.randn(4096, 4096, device="cuda")
    torch.cuda.synchronize()
    t0 = time.time()
    _ = x @ x
    torch.cuda.synchronize()
    print(f"  GPU-Matmul 4096x4096 ok ({(time.time() - t0) * 1e3:.1f} ms)")
    print(f"{PASS} CUDA einsatzbereit.")
    return True


def check_yolo(image: Path) -> bool:
    print("\n=== 2. YOLO11 Objekterkennung + Tracking ===")
    try:
        from ultralytics import YOLO
    except Exception as e:  # noqa: BLE001
        print(f"{FAIL} ultralytics-Import fehlgeschlagen: {e}")
        return False

    try:
        model = YOLO("yolo11n.pt")
        # Detektion
        res = model.predict(str(image), device=0, verbose=False)[0]
        names = res.names
        dets = [(names[int(b.cls)], float(b.conf)) for b in res.boxes]
        print(f"  Detektion: {len(dets)} Objekte")
        for label, conf in dets[:8]:
            print(f"    - {label}: {conf:.2f}")

        # Tracking (ByteTrack) auf demselben Bild als Stand-in fuer einen Frame
        tr = model.track(str(image), device=0, persist=True,
                         tracker="bytetrack.yaml", verbose=False)[0]
        n_ids = 0 if tr.boxes is None or tr.boxes.id is None else len(tr.boxes.id)
        print(f"  Tracking: {n_ids} Track-IDs vergeben")
        print(f"{PASS} YOLO11 Detektion + Tracking ok.")
        return True
    except Exception as e:  # noqa: BLE001
        print(f"{FAIL} YOLO-Inferenz fehlgeschlagen: {e}")
        return False


def check_depth(image: Path) -> bool:
    print("\n=== 3. Depth Anything V2 (monokulare Tiefe) ===")
    try:
        import numpy as np
        import torch
        from PIL import Image
        from transformers import pipeline
    except Exception as e:  # noqa: BLE001
        print(f"{FAIL} Import fehlgeschlagen: {e}")
        return False

    try:
        device = 0 if torch.cuda.is_available() else -1
        pipe = pipeline(
            task="depth-estimation",
            model="depth-anything/Depth-Anything-V2-Small-hf",
            device=device,
        )
        img = Image.open(image).convert("RGB")
        out = pipe(img)
        depth = np.asarray(out["depth"], dtype="float32")
        print(f"  Tiefenkarte: shape={depth.shape}, "
              f"min={depth.min():.1f}, max={depth.max():.1f}")
        print(f"{PASS} Depth Anything V2 ok.")
        return True
    except Exception as e:  # noqa: BLE001
        print(f"{FAIL} Depth-Inferenz fehlgeschlagen: {e}")
        return False


def main() -> int:
    ap = argparse.ArgumentParser(description="Hexapod Vision Smoke-Test")
    ap.add_argument("--image", type=Path, default=SAMPLE,
                    help="Pfad zu einem Testbild (Default: laedt bus.jpg)")
    ap.add_argument("--skip-depth", action="store_true",
                    help="Tiefenschaetzung ueberspringen")
    args = ap.parse_args()

    image = ensure_sample(args.image)

    results = {
        "CUDA": check_cuda(),
        "YOLO": check_yolo(image),
    }
    if not args.skip_depth:
        results["Depth"] = check_depth(image)

    print("\n=== Zusammenfassung ===")
    failed = 0
    for name, ok in results.items():
        print(f"  {PASS if ok else FAIL} {name}")
        failed += 0 if ok else 1

    if failed == 0:
        print("\nAlles bereit fuer den Hexapod. ")
    else:
        print(f"\n{failed} Stufe(n) fehlgeschlagen -- siehe Meldungen oben.")
    return failed


if __name__ == "__main__":
    sys.exit(main())
