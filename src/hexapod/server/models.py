"""Pydantic-Modelle fuer die Telemetrie-API (read-only)."""

from __future__ import annotations

from typing import Any

from pydantic import BaseModel


class LegTelemetry(BaseModel):
    """Zustand eines Beins, aus der Hardware zurueckgelesen."""

    name: str
    angles_deg: list[float] | None  # [theta1, theta2, theta3] oder None bei Fehler
    foot_leg_frame_mm: list[float] | None  # [x, y, z] im Leg-Frame oder None
    servo_us: list[float | None]  # rohe Pulsweiten [coxa, femur, tibia]


class TelemetrySnapshot(BaseModel):
    """Vollstaendiger Telemetrie-Snapshot zu einem Zeitpunkt."""

    timestamp: float
    ok: bool  # True, wenn alle Beine sauber gelesen werden konnten
    legs: list[LegTelemetry]


class ServerStatus(BaseModel):
    """Statischer/grober Status des Daemons."""

    robot_name: str
    driver_type: str
    num_legs: int
    worker_running: bool
    poll_rate_hz: float
    last_update: float | None  # timestamp des letzten Snapshots
    queued_commands: int = 0
    last_command: dict[str, Any] | None = None
    robot_state: str = "off"  # off | standing | walking | lying
    gait: str = "tripod"  # aktive Gangart
    battery: dict[str, float | None] | None = None       # {"pi": U, "servo": U}
    battery_state: dict[str, str] | None = None          # {"pi": "ok", ...}
    obstacle: dict[str, Any] | None = None                         # Sonar: guard/distance/blocked/free_dir/scanning


class CommandRequest(BaseModel):
    """Diskreter Bewegungsbefehl an den Worker (Iteration 2).

    action="stance": in die Standpose.
    action="pose": Koerper-Pose; Translation in mm, Rotation in Grad.
    """

    action: str  # "stance" | "pose" | "walk" | "halt"
    tx: float = 0.0
    ty: float = 0.0
    tz: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    # Kamera-Parameter (Grad)
    pan_deg: float = 0.0
    tilt_deg: float = 0.0
    # Gangart-Auswahl (action="set_gait")
    gait: str = "tripod"
    # Geste (action="gesture"): wave | lift_leg | mantis
    gesture: str = ""
    # Nivellierung (action="level")
    on: bool = True
    # walk-Parameter
    vx: float = 0.0
    vy: float = 0.0
    omega_deg: float = 0.0
    height: float = 30.0
    steps: int = 30
    rate_hz: float = 40.0


class CommandAck(BaseModel):
    """Quittung: Befehl angenommen und in die Queue gelegt."""

    accepted: bool
    id: int
    queued: int  # Anzahl noch wartender Befehle


class LedRequest(BaseModel):
    """Manuelle LED-Ring-Steuerung. mode: auto|off|manual|rainbow."""

    mode: str = "auto"
    r: int = 0
    g: int = 0
    b: int = 0


class NetConnectRequest(BaseModel):
    """WLAN-Client-Verbindung auf einem Interface herstellen."""

    iface: str
    ssid: str
    password: str = ""


class NetHotspotRequest(BaseModel):
    """Eigenen Access Point (Hotspot) auf einem Interface starten."""

    iface: str
    ssid: str = "Hexapod"
    password: str


class NetIfaceRequest(BaseModel):
    """Nur ein Interface-Name (z. B. fuer disconnect)."""

    iface: str
