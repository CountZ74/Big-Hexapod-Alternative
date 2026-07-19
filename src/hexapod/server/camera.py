"""Kamera-Streaming ueber rpicam-vid (MJPEG).

Startet rpicam-vid als Subprozess (MJPEG = aneinandergereihte JPEGs auf
stdout) und liefert die Frames als multipart/x-mixed-replace fuer ein
<img>-Tag im Browser. Unabhaengig vom Roboter-Worker -- die Kamera (CSI)
und die Servos (Maestro) sind getrennte Hardware.
"""
from __future__ import annotations

import logging
import subprocess
import threading
from collections.abc import Iterator

logger = logging.getLogger(__name__)

# Erlaubte Aufloesungen (Breite, Hoehe) -- vom Endpoint validiert.
PRESETS: set[tuple[int, int]] = {(640, 480), (800, 600), (1296, 972), (1920, 1080)}
DEFAULT: tuple[int, int] = (640, 480)

_lock = threading.Lock()
_current: subprocess.Popen[bytes] | None = None


def _stop_current() -> None:
    global _current
    if _current is not None and _current.poll() is None:
        _current.terminate()
        try:
            _current.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            _current.kill()
    _current = None


def mjpeg_stream(width: int, height: int, fps: int, quality: int) -> Iterator[bytes]:
    """Liefert MJPEG-Frames als multipart-Bytes. Ein Stream gleichzeitig."""
    global _current
    if (width, height) not in PRESETS:
        width, height = DEFAULT
    fps = max(5, min(30, int(fps)))
    quality = max(20, min(95, int(quality)))
    cmd = [
        "rpicam-vid", "-t", "0", "--codec", "mjpeg", "--nopreview",
        "--width", str(width), "--height", str(height),
        "--framerate", str(fps), "--quality", str(quality),
        "--flush", "-o", "-",
    ]
    with _lock:
        _stop_current()
        proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0
        )
        _current = proc
    logger.info("Kamera-Stream: %dx%d @%dfps q%d", width, height, fps, quality)
    buf = bytearray()
    try:
        assert proc.stdout is not None
        while True:
            chunk = proc.stdout.read(8192)
            if not chunk:
                break
            buf.extend(chunk)
            while True:
                soi = buf.find(b"\xff\xd8")
                if soi < 0:
                    del buf[:-1]
                    break
                eoi = buf.find(b"\xff\xd9", soi + 2)
                if eoi < 0:
                    if soi > 0:
                        del buf[:soi]
                    break
                frame = bytes(buf[soi:eoi + 2])
                del buf[:eoi + 2]
                yield (
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n"
                )
    finally:
        with _lock:
            if _current is proc:
                _stop_current()
        logger.info("Kamera-Stream beendet")
