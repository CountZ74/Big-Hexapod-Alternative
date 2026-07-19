"""Body-IK: Körper-Pose → Fußpositionen im Leg-Frame.

Gegeben:
    - Eine Körper-Pose (Translation + Rotation im Welt-Frame)
    - Die gewünschten Fußpositionen im Welt-Frame (wo die Füße stehen sollen)
    - Die Coxa-Mount-Positionen der Beine im Body-Frame

Gesucht:
    - Fußpositionen im Leg-Frame jedes Beins (Eingabe für die Leg-IK)

Mathematik:
    foot_leg = Rᵀ × (foot_world - coxa_world)

    wobei coxa_world = R × coxa_body + body_pos

    Konvention: R = Rz(yaw) × Ry(pitch) × Rx(roll)
    (erst roll, dann pitch, dann yaw — extrinsische Rotationen)

Einheiten: mm für Positionen, Radiant für Winkel.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

# Typ-Alias für lesbarere Signaturen
Vec3 = npt.NDArray[np.float64]   # shape (3,)
Mat3 = npt.NDArray[np.float64]   # shape (3, 3)


# ---------------------------------------------------------------------
# Körper-Pose
# ---------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class BodyPose:
    """Pose des Körpermittelpunkts im Welt-Frame.

    Translation: wie weit der Körper vom Neutral-Ursprung verschoben ist.
    Rotation: wie der Körper geneigt/gedreht ist.

    Alle Winkel in Radiant, alle Abstände in mm.
    """

    # Translation
    tx: float = 0.0   # vor/zurück
    ty: float = 0.0   # links/rechts
    tz: float = 0.0   # hoch/runter

    # Rotation (Euler-Winkel, extrinsisch XYZ)
    roll:  float = 0.0   # Kippen links/rechts (um X-Achse)
    pitch: float = 0.0   # Nicken vor/zurück (um Y-Achse)
    yaw:   float = 0.0   # Drehen (um Z-Achse)

    @classmethod
    def neutral(cls) -> BodyPose:
        """Neutralpose: kein Versatz, keine Neigung."""
        return cls()

    @classmethod
    def elevated(cls, height_mm: float) -> BodyPose:
        """Körper um `height_mm` angehoben (positives tz)."""
        return cls(tz=height_mm)


# ---------------------------------------------------------------------
# Rotationsmatrix
# ---------------------------------------------------------------------


def rotation_matrix(roll: float, pitch: float, yaw: float) -> Mat3:
    """Berechnet die 3×3-Rotationsmatrix R = Rz(yaw) × Ry(pitch) × Rx(roll).

    Konvention: extrinsische Rotationen — erst roll um die feste X-Achse,
    dann pitch um die feste Y-Achse, dann yaw um die feste Z-Achse.

    Eigenschaften:
    - R ist orthogonal: R × Rᵀ = I
    - R⁻¹ = Rᵀ  (Umkehrung ist kostenlos: einfach transponieren)
    - det(R) = 1  (keine Spiegelung, nur reine Rotation)
    """
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)

    # Rx (Roll um X)
    Rx: Mat3 = np.array([
        [1,  0,   0],
        [0,  cr, -sr],
        [0,  sr,  cr],
    ], dtype=np.float64)

    # Ry (Pitch um Y)
    Ry: Mat3 = np.array([
        [ cp, 0, sp],
        [  0, 1,  0],
        [-sp, 0, cp],
    ], dtype=np.float64)

    # Rz (Yaw um Z)
    Rz: Mat3 = np.array([
        [cy, -sy, 0],
        [sy,  cy, 0],
        [ 0,   0, 1],
    ], dtype=np.float64)

    return Rz @ Ry @ Rx


# ---------------------------------------------------------------------
# Body-IK: Kernfunktion
# ---------------------------------------------------------------------


def body_ik(
    pose: BodyPose,
    foot_positions_world: Mapping[str, Vec3 | tuple[float, float, float]],
    coxa_positions_body: Mapping[str, Vec3],
) -> dict[str, tuple[float, float, float]]:
    """Berechnet Fußpositionen im Leg-Frame für alle Beine.

    Args:
        pose: Aktuelle Körper-Pose (Translation + Rotation).
        foot_positions_world: Gewünschte Fußpositionen im Welt-Frame,
            pro Bein. Bleiben beim Stehen konstant.
        coxa_positions_body: Position der Coxa-Drehachsen im Body-Frame,
            pro Bein. Kommen aus der Konfig (mount_x, mount_y, 0).

    Returns:
        Dict von leg_name → (x, y, z) körperparallel & coxa-relativ
        (Body-Frame-Achsen, Ursprung Coxa) — bereit für
        set_all_foot_positions_world(), das die Mount-Drehung
        in den Leg-Frame übernimmt.

    Mathematik (pro Bein):
        body_pos    = [tx, ty, tz]
        R           = rotation_matrix(roll, pitch, yaw)
        coxa_world  = R × coxa_body + body_pos
        foot_leg    = Rᵀ × (foot_world - coxa_world)
    """
    body_pos: Vec3 = np.array([pose.tx, pose.ty, pose.tz], dtype=np.float64)
    R = rotation_matrix(pose.roll, pose.pitch, pose.yaw)
    Rt = R.T  # Inverse = Transponierte (da R orthogonal)

    result: dict[str, tuple[float, float, float]] = {}

    for leg_name, foot_world in foot_positions_world.items():
        coxa_body = coxa_positions_body[leg_name]

        # Wo sitzt die Coxa-Achse im Welt-Frame?
        # (Körper-Rotation dreht die Coxa-Position mit)
        coxa_world = R @ coxa_body + body_pos

        # Fußpunkt relativ zur Coxa, im Body-Frame ausgedrückt
        # = Rᵀ × (foot_world - coxa_world)
        fw = np.asarray(foot_world, dtype=np.float64)
        foot_leg_vec: Vec3 = Rt @ (fw - coxa_world)

        result[leg_name] = (
            float(foot_leg_vec[0]),
            float(foot_leg_vec[1]),
            float(foot_leg_vec[2]),
        )

    return result


# ---------------------------------------------------------------------
# Hilfsfunktion: Standard-Fußpositionen aus der Konfig aufbauen
# ---------------------------------------------------------------------


def default_foot_positions(
    coxa_positions_body: dict[str, Vec3],
    foot_distance: float,
    foot_height: float,
) -> dict[str, Vec3]:
    """Erzeugt die Neutral-Fußpositionen im Welt-Frame.

    Jeder Fuß steht `foot_distance` mm vom Coxa-Drehpunkt entfernt
    (in lokaler Bein-Richtung) und `foot_height` mm unter der Coxa-Achse.

    Args:
        coxa_positions_body: Coxa-Positionen aus der Konfig.
            Keys: Bein-Namen, Values: [x, y, 0] im Body-Frame.
        foot_distance: Horizontale Distanz vom Coxa-Punkt zum Fuß [mm].
        foot_height: Wie tief der Fuß unter der Coxa liegt [mm] (positiv!).

    Returns:
        Dict von leg_name → foot_position im Welt-Frame.
    """
    result: dict[str, Vec3] = {}
    for leg_name, coxa_body in coxa_positions_body.items():
        # In Neutral-Pose (kein Roll/Pitch/Yaw, kein tz):
        # coxa_world = coxa_body (keine Transformation nötig)
        # Fuß liegt `foot_distance` weiter außen in XY-Richtung
        # und `foot_height` tiefer in Z.
        cx, cy = float(coxa_body[0]), float(coxa_body[1])
        length = math.hypot(cx, cy)

        if length < 1e-9:
            # Coxa direkt im Ursprung (sollte nicht vorkommen)
            foot_x, foot_y = float(cx), float(cy)
        else:
            # Normalisiere die XY-Richtung und skaliere auf foot_distance
            foot_x = cx + cx / length * foot_distance
            foot_y = cy + cy / length * foot_distance

        result[leg_name] = np.array([foot_x, foot_y, -foot_height], dtype=np.float64)

    return result


# ---------------------------------------------------------------------
# Körperpose als Standpose-Offsets (für den Trajektorien-Executor)
# ---------------------------------------------------------------------


def body_pose_offsets(
    pose: BodyPose,
    foot_positions_world: Mapping[str, Vec3 | tuple[float, float, float]],
    coxa_positions_body: Mapping[str, Vec3],
    neutral_world: Mapping[str, tuple[float, float, float]],
) -> dict[str, tuple[float, float, float]]:
    """Körperpose als (dx, dy, dz)-Offsets relativ zur Standpose.

    Wandelt eine BodyPose in körperparallele, coxa-relative Offsets pro Bein
    um — genau das Format, das set_all_foot_offsets() und der Trajektorien-
    Executor erwarten. Damit lässt sich eine Pose mit derselben adaptiven
    Glättung anfahren wie ein Gait.

        offset = body_ik(pose) − neutral_world
        (beide körperparallel, Ursprung = Coxa)

    Bei pose = neutral ist der Offset für jedes Bein exakt (0, 0, 0).
    """
    leg_targets = body_ik(pose, foot_positions_world, coxa_positions_body)
    result: dict[str, tuple[float, float, float]] = {}
    for leg_name, (fx, fy, fz) in leg_targets.items():
        nx, ny, nz = neutral_world[leg_name]
        result[leg_name] = (fx - nx, fy - ny, fz - nz)
    return result
