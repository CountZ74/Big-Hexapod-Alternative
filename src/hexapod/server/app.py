"""FastAPI-Server fuer die Hexapod-Telemetrie (read-only, Iteration 1).

Endpunkte:
    GET  /status            -> grober Daemon-Status
    GET  /telemetry         -> letzter Telemetrie-Snapshot
    WS   /ws/telemetry      -> Live-Stream der Snapshots
    GET  /healthz           -> einfacher Health-Check

Der Server kommandiert KEINE Bewegung. Er liest nur den Snapshot des
Roboter-Worker-Threads.
"""

from __future__ import annotations

import asyncio
import json
import os
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, StreamingResponse

from . import camera, netadmin
from .models import (
    CommandAck,
    CommandRequest,
    LedRequest,
    NetConnectRequest,
    NetHotspotRequest,
    NetIfaceRequest,
    ServerStatus,
    TelemetrySnapshot,
)
from .worker import RobotWorker

CONFIG_PATH = os.environ.get("HEXAPOD_CONFIG", "config/robot.yaml")
POLL_HZ = float(os.environ.get("HEXAPOD_POLL_HZ", "5.0"))
LED_FILE = os.environ.get("HEXAPOD_LED_FILE", "/tmp/hexapod_led.json")

worker: RobotWorker | None = None

# Gueltige Kommando-Actions: Vereinigung aller je Zustand erlaubten Actions des
# Workers -- so kann der Endpoint nicht mehr aus der Worker-Logik herauslaufen.
VALID_ACTIONS = frozenset().union(*RobotWorker._ALLOWED.values())


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    global worker
    worker = RobotWorker(CONFIG_PATH, poll_hz=POLL_HZ)
    worker.start()
    try:
        yield
    finally:
        worker.stop()


app = FastAPI(title="Hexapod Telemetry", version="0.1.0", lifespan=lifespan)

_STATIC_DIR = Path(__file__).parent / "static"


@app.get("/", response_class=HTMLResponse)
def index() -> str:
    """Liefert das read-only Telemetrie-Dashboard aus."""
    return (_STATIC_DIR / "index.html").read_text(encoding="utf-8")


@app.get("/gamepad", response_class=HTMLResponse)
def gamepad() -> str:
    return (_STATIC_DIR / "gamepad.html").read_text(encoding="utf-8")


@app.get("/healthz")
def healthz() -> dict[str, bool]:
    return {"ok": worker is not None and worker.running}


@app.get("/camera/stream")
def camera_stream(
    w: int = 640, h: int = 480, fps: int = 20, q: int = 80
) -> StreamingResponse:
    """MJPEG-Livestream der Kamera (einstellbare Aufloesung via Query-Param)."""
    return StreamingResponse(
        camera.mjpeg_stream(w, h, fps, q),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.post("/led")
def led(req: LedRequest) -> dict[str, Any]:
    """Schreibt die LED-Override-Datei (vom root-LED-Dienst gelesen).

    Kein Roboter-Zugriff -- der separate root-Dienst rendert daraus.
    """
    data: dict[str, Any] = {"mode": req.mode}
    if req.mode == "manual":
        data.update(r=req.r, g=req.g, b=req.b)
    with open(LED_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f)
    return {"ok": True, "mode": req.mode}


@app.get("/network", response_class=HTMLResponse)
def network_page() -> str:
    """Netzwerk-Verwaltungsseite (WLAN/Hotspot pro Adapter)."""
    return (_STATIC_DIR / "network.html").read_text(encoding="utf-8")


@app.get("/net/status")
def net_status() -> list[dict[str, Any]]:
    return netadmin.status()


@app.get("/net/scan")
def net_scan(iface: str) -> list[dict[str, Any]]:
    from fastapi import HTTPException
    try:
        return netadmin.scan(iface)
    except netadmin.NetError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e


@app.post("/net/connect")
def net_connect(req: NetConnectRequest) -> dict[str, bool]:
    from fastapi import HTTPException
    try:
        netadmin.connect(req.iface, req.ssid, req.password)
        return {"ok": True}
    except netadmin.NetError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e


@app.post("/net/hotspot")
def net_hotspot(req: NetHotspotRequest) -> dict[str, bool]:
    from fastapi import HTTPException
    try:
        netadmin.hotspot(req.iface, req.ssid, req.password)
        return {"ok": True}
    except netadmin.NetError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e


@app.post("/net/disconnect")
def net_disconnect(req: NetIfaceRequest) -> dict[str, bool]:
    from fastapi import HTTPException
    try:
        netadmin.disconnect(req.iface)
        return {"ok": True}
    except netadmin.NetError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e


@app.get("/status", response_model=ServerStatus)
def status() -> ServerStatus:
    assert worker is not None
    return ServerStatus(**worker.status())


@app.get("/telemetry", response_model=TelemetrySnapshot | None)
def telemetry() -> TelemetrySnapshot | None:
    assert worker is not None
    return worker.snapshot()


@app.post("/command", response_model=CommandAck)
def command(req: CommandRequest) -> CommandAck:
    from fastapi import HTTPException
    assert worker is not None
    if req.action not in VALID_ACTIONS:
        raise HTTPException(status_code=422, detail=f"Unbekannte action: {req.action!r}")
    try:
        cmd_id, queued = worker.enqueue(req.model_dump())
    except ValueError as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return CommandAck(accepted=True, id=cmd_id, queued=queued)


@app.websocket("/ws/telemetry")
async def ws_telemetry(ws: WebSocket) -> None:
    await ws.accept()
    assert worker is not None
    interval = 1.0 / POLL_HZ
    last_ts: float | None = None
    try:
        while True:
            snap = worker.snapshot()
            if snap is not None and snap.timestamp != last_ts:
                last_ts = snap.timestamp
                await ws.send_json(snap.model_dump())
            await asyncio.sleep(interval)
    except WebSocketDisconnect:
        return
