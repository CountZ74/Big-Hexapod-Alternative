"""CLI-Befehle für Bewegungen: power_up, stand_up, lie_down, walk, stance."""

from __future__ import annotations

import time
from collections.abc import Callable
from pathlib import Path

import typer

from hexapod.robot.hexapod import Hexapod


def _open_robot(config: Path) -> Hexapod:
    """Öffnet den Roboter aus der Config (echter Maestro)."""
    return Hexapod.from_config(config)


def _run_holding(config: Path, fn: Callable[[Hexapod], None]) -> None:
    """Führt fn(robot) aus und hält danach die Pose (Servos bleiben aktiv)."""
    robot = _open_robot(config)
    try:
        fn(robot)
    finally:
        # disable=False: Servos behalten Signal, Roboter sackt NICHT zusammen.
        robot.close(disable=False)


def _run_releasing(config: Path, fn: Callable[[Hexapod], None]) -> None:
    """Führt fn(robot) aus und deaktiviert danach die Servos."""
    robot = _open_robot(config)
    try:
        fn(robot)
    finally:
        robot.close(disable=True)


def cmd_power_up(config: Path, confirm: bool) -> None:
    """Kaltstart: aus der Kalibrierposition aufstehen."""
    from hexapod.gait.posture import CALIB_X, power_up

    typer.echo("KALTSTART (power_up)")
    typer.echo("Voraussetzung: Beine flach in Kalibrierposition (radial gestreckt).")
    if confirm:
        typer.confirm("Beine gerichtet und bereit?", abort=True)

    def _seq(robot: Hexapod) -> None:
        legs = robot._leg_names
        # P1: ungebremster erster Maestro-Zug auf die Kalibrierposition.
        typer.echo("P1: Kalibrierposition kommandieren...")
        robot.set_all_foot_positions(
            {leg: (CALIB_X, 0.0, 0.0) for leg in legs}, clip=True
        )
        time.sleep(2)
        # Freie Speed (Executor begrenzt selbst), dann Aufstehsequenz.
        robot.set_speed_all(0)
        robot.set_acceleration_all(0)
        time.sleep(0.3)
        typer.echo("Aufstehen...")
        power_up(robot)
        typer.echo("Steht in Standpose.")

    _run_holding(config, _seq)


def cmd_stand_up(config: Path) -> None:
    """Körper aus abgesetzter Lage in die Standpose heben."""
    from hexapod.gait.posture import stand_up

    def _seq(robot: Hexapod) -> None:
        typer.echo("Körper anheben (stand_up)...")
        stand_up(robot)
        typer.echo("Steht.")

    _run_holding(config, _seq)


def cmd_lie_down(config: Path) -> None:
    """Körper in die abgesetzte Lage senken."""
    from hexapod.gait.posture import lie_down

    def _seq(robot: Hexapod) -> None:
        typer.echo("Körper absenken (lie_down)...")
        lie_down(robot)
        typer.echo("Abgesetzt.")

    _run_releasing(config, _seq)


def cmd_stance(config: Path) -> None:
    """Alle Beine in die Standpose (Offsets = 0)."""
    from hexapod.gait.posture import settle_to_stance

    def _seq(robot: Hexapod) -> None:
        typer.echo("Standpose (Beine einzeln umsetzen, kein Schleifen)...")
        settle_to_stance(robot)
        typer.echo("Standpose erreicht.")

    _run_holding(config, _seq)


def cmd_walk(
    config: Path,
    cycles: int,
    stride: float,
    height: float,
    rate_hz: float,
    direction: float,
    touch_percent: float | None = None,
) -> None:
    """Tripod-Gait laufen."""
    from hexapod.gait.tripod import walk

    def _seq(robot: Hexapod) -> None:
        typer.echo(f"Tripod-Gait: {cycles} Zyklen, stride={stride}mm, dir={direction}")
        if touch_percent is not None:
            typer.echo(f"Aufsetz-Erkennung aktiv bei {touch_percent:.1f} % Federweg")
        robot.stance(clip=True)
        time.sleep(1.0)
        treffer = walk(
            robot,
            cycles=cycles,
            stride=stride,
            height=height,
            rate_hz=rate_hz,
            direction=direction,
            touch_level=None if touch_percent is None else touch_percent / 100.0,
        )
        if treffer:
            typer.echo("\nFrüher aufgesetzt (Höhe über Standpose):")
            for leg, dz in sorted(treffer.items(), key=lambda kv: -kv[1]):
                typer.echo(f"  {leg:<13}{dz:+6.1f} mm")
        elif touch_percent is not None:
            typer.echo("\nKein Bein hat früher Boden gefunden.")
        typer.echo("Fertig.")

    _run_holding(config, _seq)


def cmd_move(
    config: Path,
    vx: float,
    vy: float,
    omega: float,
    cycles: int,
    height: float,
    rate_hz: float,
) -> None:
    """Verallgemeinerter Tripod-Gait aus einem Bewegungsbefehl (vx, vy, omega)."""
    from hexapod.gait.command_tripod import walk_command

    def _seq(robot: Hexapod) -> None:
        typer.echo(
            f"Move: vx={vx}mm vy={vy}mm omega={omega}rad, {cycles} Zyklen"
        )
        robot.stance(clip=True)
        time.sleep(1.0)
        used = walk_command(
            robot, vx, vy, omega,
            cycles=cycles, height=height, rate_hz=rate_hz,
        )
        if (used[0], used[1], used[2]) != (vx, vy, omega):
            typer.echo(
                f"(begrenzt auf vx={used[0]:.1f} vy={used[1]:.1f} "
                f"omega={used[2]:.3f})"
            )
        typer.echo("Fertig.")

    _run_holding(config, _seq)


def cmd_pose(
    config: Path,
    tx: float,
    ty: float,
    tz: float,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    steps: int,
    rate_hz: float,
) -> None:
    """Körperpose über stehenden Füßen einnehmen (Translation + Rotation)."""
    import math

    from hexapod.gait.posture import move_to_body_pose
    from hexapod.kinematics.body_ik import BodyPose

    def _seq(robot: Hexapod) -> None:
        pose = BodyPose(
            tx=tx, ty=ty, tz=tz,
            roll=math.radians(roll_deg),
            pitch=math.radians(pitch_deg),
            yaw=math.radians(yaw_deg),
        )
        typer.echo(
            f"Pose: tx={tx} ty={ty} tz={tz} mm | "
            f"roll={roll_deg} pitch={pitch_deg} yaw={yaw_deg}\u00b0"
        )
        move_to_body_pose(robot, pose, steps=steps, rate_hz=rate_hz)
        typer.echo("Pose erreicht.")

    _run_holding(config, _seq)
