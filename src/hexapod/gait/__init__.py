"""Gangarten und Fuß-Trajektorien für den Hexapod."""

from hexapod.gait.trajectory import (
    Vec3,
    lerp,
    linear_path,
    stance_path,
    swing_path,
)

__all__ = [
    "Vec3",
    "lerp",
    "linear_path",
    "stance_path",
    "swing_path",
]

from hexapod.gait.posture import (
    lie_down,
    move_to_body_pose,
    power_up,
    settle_to_stance,
    stand_up,
)

__all__ += ["lie_down", "move_to_body_pose", "power_up", "settle_to_stance", "stand_up"]

from hexapod.gait.command_tripod import command_half_cycle_paths, walk_command  # noqa: E402

__all__ += ["command_half_cycle_paths", "walk_command"]
