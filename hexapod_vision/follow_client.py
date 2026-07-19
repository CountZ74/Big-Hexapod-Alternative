#!/usr/bin/env python3
"""Vision-Follow-Client fuer den Hexapod (laeuft auf dem 4070-Rechner).

Zwei entkoppelte Ebenen:
  * Kamera (Pan/Tilt) -- schnell, haelt die Zielperson zentriert (action="camera")
  * Koerper (Laufen)  -- traege, Abstand + Nachdrehen ab Pan-Schwelle

Zusatzfunktionen:
  * ID-Lock: verfolgt EINE feste ByteTrack-ID statt "groesste pro Frame"
    (--lock-on largest|center|off). Re-Acquire erst nach --lost-grace Frames.
  * Feed-Forward: dreht der Koerper, wird der Kamera-Pan vorausschauend
    mitgezogen, statt die Koerperdrehung visuell hinterherzujagen (--ff-gain).

Sicherheits-Defaults: --dry-run (sendet nichts), --camera-only (Koerper nie).
"""
from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass

import numpy as np

TARGET_CLASS = "person"
MIN_CONF = 0.45

# Kamera
CAM_DEADZONE = 0.05
PAN_GAIN_DEG = 3.5
TILT_GAIN_DEG = 3.0
PAN_LIMIT = 70.0
TILT_LIMIT = 45.0
CAM_SEND_MIN_DELTA = 1.0
MAX_CAM_STEP = 2.5  # max. Pan/Tilt-Aenderung pro Frame (Grad)

# Koerper
SIZE_TARGET = 0.45
SIZE_DEADZONE = 0.07
MAX_VX = 30.0
MAX_OMEGA_DEG = 12.0
BODY_SIGN = 1   # Vorzeichen Koerperdrehung relativ zum Pan (falsche Richtung -> umdrehen)
PAN_BODY_THRESH = 20.0
OBSTACLE_FRACTION = 0.18
WALK_HEIGHT = 30.0
WALK_STEPS = 16
WALK_RATE_HZ = 45.0


@dataclass
class CameraCommand:
    pan_deg: float
    tilt_deg: float

    def to_api(self) -> dict:
        return {"action": "camera",
                "pan_deg": round(self.pan_deg, 1),
                "tilt_deg": round(self.tilt_deg, 1)}


@dataclass
class WalkCommand:
    walk: bool
    vx: float = 0.0
    omega_deg: float = 0.0
    reason: str = ""

    def to_api(self) -> dict:
        if not self.walk:
            return {"action": "halt"}
        return {"action": "walk", "vx": round(self.vx, 1), "vy": 0.0,
                "omega_deg": round(self.omega_deg, 1), "height": WALK_HEIGHT,
                "steps": WALK_STEPS, "rate_hz": WALK_RATE_HZ}


# --------------------------------------------------------------------------
# Bildquellen
# --------------------------------------------------------------------------
class FrameSource:
    def read(self): ...
    def close(self) -> None: ...


class FileSource(FrameSource):
    def __init__(self, path: str, loop: bool, fps: float = 10.0) -> None:
        import cv2
        self._img = cv2.imread(path)
        if self._img is None:
            raise FileNotFoundError(f"Bild nicht lesbar: {path}")
        self._loop = loop
        self._delay = 1.0 / max(1.0, fps)
        self._served = False

    def read(self):
        if self._served and not self._loop:
            return None
        if self._served:
            time.sleep(self._delay)
        self._served = True
        return self._img.copy()

    def close(self) -> None:
        pass


class WebcamSource(FrameSource):
    def __init__(self, index: int) -> None:
        import cv2
        self._cap = cv2.VideoCapture(index)
        if not self._cap.isOpened():
            raise RuntimeError(f"Webcam {index} nicht verfuegbar")

    def read(self):
        ok, frame = self._cap.read()
        return frame if ok else None

    def close(self) -> None:
        self._cap.release()


class MjpegSource(FrameSource):
    def __init__(self, url: str, timeout: float = 10.0) -> None:
        import requests
        self._resp = requests.get(url, stream=True, timeout=timeout)
        self._resp.raise_for_status()
        self._it = self._resp.iter_content(chunk_size=8192)
        self._buf = bytearray()

    def read(self):
        import cv2
        while True:
            soi = self._buf.find(b"\xff\xd8")
            eoi = self._buf.find(b"\xff\xd9", soi + 2) if soi >= 0 else -1
            if soi >= 0 and eoi >= 0:
                jpg = bytes(self._buf[soi:eoi + 2])
                del self._buf[:eoi + 2]
                arr = np.frombuffer(jpg, dtype=np.uint8)
                return cv2.imdecode(arr, cv2.IMREAD_COLOR)
            try:
                chunk = next(self._it)
            except StopIteration:
                return None
            self._buf.extend(chunk)

    def close(self) -> None:
        self._resp.close()


def make_source(args) -> FrameSource:
    if args.source == "file":
        return FileSource(args.input, loop=not args.once)
    if args.source == "webcam":
        return WebcamSource(int(args.input))
    if args.source == "mjpeg":
        return MjpegSource(args.input)
    raise ValueError(f"Unbekannte Quelle: {args.source}")


class VisionEngine:
    def __init__(self, weights: str, use_depth: bool, device: int) -> None:
        from ultralytics import YOLO
        self._model = YOLO(weights)
        self._device = device
        self._depth = None
        if use_depth:
            import torch
            from transformers import pipeline
            self._depth = pipeline(
                task="depth-estimation",
                model="depth-anything/Depth-Anything-V2-Small-hf",
                device=device if torch.cuda.is_available() else -1,
            )

    def detect_persons(self, frame):
        """Liefert ALLE Personen als Liste (cx, cy, box_h, track_id, conf)."""
        res = self._model.track(
            frame, device=self._device, persist=True,
            tracker="bytetrack.yaml", verbose=False,
        )[0]
        out = []
        if res.boxes is None or len(res.boxes) == 0:
            return out
        names = res.names
        h_img = frame.shape[0]
        w_img = frame.shape[1]
        for b in res.boxes:
            if names[int(b.cls)] != TARGET_CLASS or float(b.conf) < MIN_CONF:
                continue
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
            cx = ((x1 + x2) / 2.0) / w_img * 2.0 - 1.0
            cy = ((y1 + y2) / 2.0) / h_img * 2.0 - 1.0
            box_h = (y2 - y1) / h_img
            tid = int(b.id) if b.id is not None else -1
            out.append((cx, cy, box_h, tid, float(b.conf), (x2 - x1) / w_img))
        return out

    def obstacle_close(self, frame) -> bool:
        if self._depth is None:
            return False
        from PIL import Image
        import cv2
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        depth = np.asarray(self._depth(Image.fromarray(rgb))["depth"], dtype="float32")
        h, w = depth.shape
        roi = depth[int(h * 0.6):, int(w * 0.3):int(w * 0.7)]
        thr = depth.max() * 0.75
        return float((roi > thr).mean()) > OBSTACLE_FRACTION


class TargetSelector:
    """Sperrt sich auf EINE Track-ID. Re-Acquire erst nach lost_grace Frames.

    policy: 'largest' -> groesste Person beim Anvisieren,
            'center'  -> mittigste Person beim Anvisieren,
            'off'     -> kein Lock, jede Frame die groesste (altes Verhalten).
    """
    def __init__(self, policy: str, lost_grace: int) -> None:
        self.policy = policy
        self.lost_grace = lost_grace
        self.locked_id: int | None = None
        self.lost = 0

    def _acquire(self, persons):
        if not persons:
            return None
        if self.policy == "center":
            return min(persons, key=lambda p: abs(p[0]))
        return max(persons, key=lambda p: p[2])  # largest

    def select(self, persons):
        if self.policy == "off":
            return self._acquire(persons)
        if self.locked_id is not None:
            for p in persons:
                if p[3] == self.locked_id:
                    self.lost = 0
                    return p
            # gesperrte ID gerade nicht sichtbar
            self.lost += 1
            if self.lost <= self.lost_grace:
                return None  # kurz warten, nicht sofort umspringen
            self.locked_id = None  # endgueltig verloren -> neu anvisieren
        chosen = self._acquire(persons)
        if chosen is not None and chosen[3] >= 0:
            self.locked_id = chosen[3]
            self.lost = 0
        return chosen


class CameraTracker:
    def __init__(self, pan_sign: int, tilt_sign: int) -> None:
        self.pan = 0.0
        self.tilt = 0.0
        self._ps = pan_sign
        self._ts = tilt_sign

    def feedforward(self, delta_pan: float) -> None:
        """Zieht den Pan vorausschauend mit (Koerperdrehungs-Kompensation)."""
        self.pan = float(np.clip(self.pan + delta_pan, -PAN_LIMIT, PAN_LIMIT))

    def update(self, target) -> CameraCommand:
        if target is not None:
            cx, cy = target[0], target[1]
            if abs(cx) > CAM_DEADZONE:
                step = float(np.clip(self._ps * PAN_GAIN_DEG * cx, -MAX_CAM_STEP, MAX_CAM_STEP))
                self.pan = float(np.clip(self.pan + step, -PAN_LIMIT, PAN_LIMIT))
            if abs(cy) > CAM_DEADZONE:
                step = float(np.clip(self._ts * TILT_GAIN_DEG * cy, -MAX_CAM_STEP, MAX_CAM_STEP))
                self.tilt = float(np.clip(self.tilt + step, -TILT_LIMIT, TILT_LIMIT))
        return CameraCommand(self.pan, self.tilt)


def compute_body(target, obstacle: bool, pan_deg: float) -> WalkCommand:
    if obstacle:
        return WalkCommand(False, reason="Hindernis nah -> Stopp")
    if target is None:
        return WalkCommand(False, reason="kein Ziel")
    cx, cy, box_h, tid, conf = target[:5]
    vx = 0.0
    err = SIZE_TARGET - (target[5] if len(target) > 5 else box_h)
    if abs(err) > SIZE_DEADZONE:
        vx = float(np.clip(err / SIZE_TARGET, -1.0, 1.0)) * MAX_VX
    omega = 0.0
    if abs(pan_deg) > PAN_BODY_THRESH:
        over = (abs(pan_deg) - PAN_BODY_THRESH) / (PAN_LIMIT - PAN_BODY_THRESH)
        omega = BODY_SIGN * float(np.sign(pan_deg)) * float(np.clip(over, 0.0, 1.0)) * MAX_OMEGA_DEG
    walk = not (vx == 0.0 and omega == 0.0)
    reason = f"id={tid} conf={conf:.2f} cx={cx:+.2f} h={box_h:.2f} w={(target[5] if len(target)>5 else box_h):.2f} pan={pan_deg:+.0f}"
    return WalkCommand(walk, vx=vx, omega_deg=omega, reason=reason)


def send_command(pi_url: str, payload: dict, timeout: float = 2.0) -> str:
    import requests
    r = requests.post(f"{pi_url}/command", json=payload, timeout=timeout)
    r.raise_for_status()
    return r.text


def main() -> int:
    global SIZE_TARGET, SIZE_DEADZONE, BODY_SIGN, PAN_BODY_THRESH
    ap = argparse.ArgumentParser(description="Hexapod Vision Follow-Client")
    ap.add_argument("--source", choices=("file", "webcam", "mjpeg"), required=True)
    ap.add_argument("--input", required=True, help="Pfad / Index / URL je nach Quelle")
    ap.add_argument("--pi", default="http://hexapod.local:8000", help="Basis-URL des Pi")
    ap.add_argument("--weights", default="yolo11n.pt")
    ap.add_argument("--device", type=int, default=0, help="CUDA (0) oder -1 CPU")
    ap.add_argument("--no-depth", action="store_true", help="Hindernispruefung aus")
    ap.add_argument("--camera-only", action="store_true",
                    help="nur Kamera-Servos, Koerper NIE (sicheres Testen)")
    ap.add_argument("--no-camera", action="store_true", help="Kamerafuehrung aus")
    ap.add_argument("--pan-sign", type=int, choices=(1, -1), default=-1)
    ap.add_argument("--tilt-sign", type=int, choices=(1, -1), default=-1)
    ap.add_argument("--lock-on", choices=("largest", "center", "off"), default="largest",
                    help="Ziel-Lock-Strategie: groesste/mittigste Person, oder off")
    ap.add_argument("--lost-grace", type=int, default=15,
                    help="Frames, die die gesperrte ID fehlen darf, bevor neu anvisiert wird")
    ap.add_argument("--ff-gain", type=float, default=1.0,
                    help="Feed-Forward-Staerke der Koerperdrehung auf den Pan (0=aus, neg=Vorzeichen drehen)")
    ap.add_argument("--size-target", type=float, default=SIZE_TARGET,
                    help="Soll-Boxhoehe (Anteil Bildhoehe); hoeher=naeher dranbleiben")
    ap.add_argument("--size-deadzone", type=float, default=SIZE_DEADZONE,
                    help="Toleranzband um die Sollgroesse")
    ap.add_argument("--pan-body-thresh", type=float, default=PAN_BODY_THRESH,
                    help="ab welchem Pan-Winkel der Koerper nachdreht (Grad); niedriger = frueher")
    ap.add_argument("--body-sign", type=int, choices=(1, -1), default=BODY_SIGN,
                    help="Vorzeichen Koerperdrehung (falsche Richtung -> umdrehen)")
    ap.add_argument("--once", action="store_true", help="nur ein Frame (file)")
    ap.add_argument("--max-frames", type=int, default=0, help="nach N Frames stoppen")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--dry-run", action="store_true", default=True, help="(Default) nur ausgeben")
    g.add_argument("--send", dest="dry_run", action="store_false", help="wirklich senden")
    args = ap.parse_args()
    SIZE_TARGET = args.size_target
    SIZE_DEADZONE = args.size_deadzone
    BODY_SIGN = args.body_sign
    PAN_BODY_THRESH = args.pan_body_thresh

    mode = ("KAMERA-ONLY" if args.camera_only else
            "OHNE KAMERA" if args.no_camera else "KAMERA+KOERPER")
    print(f"Quelle={args.source}:{args.input}  Pi={args.pi}  "
          f"{'DRY-RUN' if args.dry_run else 'SENDEN'}  [{mode}]  "
          f"lock={args.lock_on} ff={args.ff_gain}")

    engine = VisionEngine(args.weights, use_depth=not args.no_depth, device=args.device)
    source = make_source(args)
    cam = CameraTracker(args.pan_sign, args.tilt_sign)
    selector = TargetSelector(args.lock_on, args.lost_grace)

    use_camera = not args.no_camera
    use_body = not args.camera_only
    halfcycle = WALK_STEPS / WALK_RATE_HZ
    last_cam = None
    last_body = None
    last_t = time.time()
    n = 0
    try:
        while True:
            frame = source.read()
            if frame is None:
                break
            persons = engine.detect_persons(frame)
            target = selector.select(persons)
            obstacle = engine.obstacle_close(frame) if use_body else False

            now = time.time()
            dt = now - last_t
            last_t = now

            line = f"[{n:04d}] n={len(persons)}"
            if selector.locked_id is not None:
                line += f" lock={selector.locked_id}"

            if use_body:
                bcmd = compute_body(target, obstacle, cam.pan)
                bpay = bcmd.to_api()
                if bpay["action"] == "walk" and bpay["omega_deg"] != 0.0 and args.ff_gain:
                    body_rot = bpay["omega_deg"] / halfcycle * dt
                    cam.feedforward(-args.ff_gain * body_rot)
                line += f" | {bpay['action']:5s}"
                if bpay["action"] == "walk":
                    line += f" vx={bpay['vx']:+5.1f} omega={bpay['omega_deg']:+5.1f}"
                line += f"  {bcmd.reason}"
                if not args.dry_run and bpay != last_body:
                    try:
                        send_command(args.pi, bpay)
                        last_body = bpay
                    except Exception as e:
                        print(f"   ! body senden fehlgeschlagen: {e}", file=sys.stderr)

            if use_camera:
                ccmd = cam.update(target)
                cpay = ccmd.to_api()
                line += f" | cam(pan={cpay['pan_deg']:+5.1f} tilt={cpay['tilt_deg']:+5.1f})"
                if not args.dry_run and (
                    last_cam is None
                    or abs(cpay["pan_deg"] - last_cam["pan_deg"]) >= CAM_SEND_MIN_DELTA
                    or abs(cpay["tilt_deg"] - last_cam["tilt_deg"]) >= CAM_SEND_MIN_DELTA
                ):
                    try:
                        send_command(args.pi, cpay)
                        last_cam = cpay
                    except Exception as e:
                        print(f"   ! camera senden fehlgeschlagen: {e}", file=sys.stderr)
            elif not use_body:
                line += "  (idle)"

            print(line, flush=True)
            n += 1
            if args.once or (args.max_frames and n >= args.max_frames):
                break
    except KeyboardInterrupt:
        print("\nAbbruch durch Benutzer.")
    finally:
        source.close()
        if not args.dry_run and use_body:
            try:
                send_command(args.pi, {"action": "halt"})
                print("Koerper angehalten (halt gesendet).")
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
