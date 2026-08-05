"""Integrationstest: MotionController laeuft jede Gangart auf dem Simulator
stabil (kein Abbruch in den Idle-Modus, Fuss-Offsets bleiben begrenzt)."""
from __future__ import annotations

import pytest

from hexapod.config.loader import load_robot_config
from hexapod.gait.gaits import GAITS, LEGS_ALL
from hexapod.robot import Hexapod
from hexapod.server.motion_control import MotionController

CONFIG_PATH = "config/robot.yaml"


@pytest.fixture
def sim_robot() -> Hexapod:
    config = load_robot_config(CONFIG_PATH)
    data = config.model_dump()
    data["buses"] = {
        n: {"type": "simulator", "num_channels": b["num_channels"]}
        for n, b in data["buses"].items()
    }
    return Hexapod(config.model_validate(data))


@pytest.mark.parametrize("name", sorted(GAITS))
def test_gait_walks_stable(sim_robot, monkeypatch, name) -> None:
    monkeypatch.setattr("time.sleep", lambda *_a, **_k: None)
    robot = sim_robot
    robot.stance()

    mc = MotionController(robot)
    mc.set_gait(name)
    assert mc.gait_name == name
    mc.set_walk(vx=20.0, vy=0.0, omega_deg=0.0, height=25.0, steps=12, rate_hz=200.0)

    n = GAITS[name].n_phases
    # Drei volle Zyklen abfahren.
    for _ in range(n * 3):
        mc.step_once()
        # Nach jeder Phase: nicht in den Sicherheits-Idle gefallen.
        assert mc.is_walking, f"{name}: Schritt brach in Idle ab"
        # Fuss-Offsets bleiben begrenzt (kein Weglaufen).
        for leg in LEGS_ALL:
            dx, dy, dz = robot.current_offset(leg)
            assert abs(dx) < 60.0 and abs(dy) < 60.0 and abs(dz) < 60.0, (
                f"{name}/{leg}: Offset zu gross ({dx:.1f},{dy:.1f},{dz:.1f})"
            )


@pytest.mark.parametrize("name", sorted(GAITS))
def test_gait_advances_body(sim_robot, monkeypatch, name) -> None:
    # Bei Vorwaerts-Befehl muessen die Standbeine den Koerper netto vorschieben:
    # ueber einen vollen Zyklus zeigt die mittlere Fussbewegung nach hinten (-x).
    monkeypatch.setattr("time.sleep", lambda *_a, **_k: None)
    robot = sim_robot
    robot.stance()
    mc = MotionController(robot)
    mc.set_gait(name)
    mc.set_walk(vx=20.0, vy=0.0, omega_deg=0.0, height=25.0, steps=12, rate_hz=200.0)

    xs_start = {leg: robot.current_offset(leg)[0] for leg in LEGS_ALL}
    # Eine einzelne Standphase eines Beins schiebt nach -x (Fuss relativ Koerper).
    # Wir pruefen: nach der ersten Phase hat mindestens ein Standbein -x bewegt.
    mc.step_once()
    moved_back = [
        leg for leg in LEGS_ALL
        if robot.current_offset(leg)[0] < xs_start[leg] - 0.5
    ]
    assert moved_back, f"{name}: kein Standbein schob nach hinten"


@pytest.mark.parametrize("name", sorted(GAITS))
def test_forward_command_drives_body_forward(sim_robot, monkeypatch, name) -> None:
    """Regression: vx>0 muss den Koerper NETTO vorwaerts (+x) treiben.

    Fuss-Bodenvektor = -(Koerperbewegung); im Stand bewegt sich ein Standbein
    also nach -x, was den Koerper nach +x schiebt. Wir summieren die
    Standphasen-Bewegungen aller belasteten Beine ueber einen vollen Zyklus
    auf (body_dx = -sum(delta foot_x | Bein am Boden)) und verlangen body_dx>0.
    Faengt die fruehere Vorzeichen-Inversion von tetrapod/wave/ripple ab.
    """
    monkeypatch.setattr("time.sleep", lambda *_a, **_k: None)
    robot = sim_robot
    robot.stance()
    mc = MotionController(robot)
    mc.set_gait(name)
    mc.set_walk(vx=20.0, vy=0.0, omega_deg=0.0, height=25.0, steps=20, rate_hz=400.0)

    prev = {leg: robot.current_offset(leg) for leg in LEGS_ALL}
    body_dx = 0.0
    orig = robot.set_all_foot_offsets

    def hook(frame, clip=True):
        nonlocal body_dx
        orig(frame, clip=clip)
        for leg in LEGS_ALL:
            ox, oy, oz = robot.current_offset(leg)
            if oz < 1.0:  # Bein am Boden -> traegt zur Koerperbewegung bei
                body_dx += -(ox - prev[leg][0])
            prev[leg] = (ox, oy, oz)

    robot.set_all_foot_offsets = hook
    for _ in range(GAITS[name].n_phases * 2):
        mc.step_once()

    assert body_dx > 0, f"{name}: vx>0 trieb Koerper nach {body_dx:+.1f}mm (nicht vorwaerts)"
