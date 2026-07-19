"""Kontinuierliches Phasen-Modell fuer ueberlappende Gangarten (Ripple).

Anders als das partitionierende Gait-Modell in :mod:`gaits` (jedes Bein
schwingt genau einmal, die Phasen sind disjunkt) beschreibt eine
kontinuierliche Gangart jedes Bein durch einen Phasen-Offset ``phi`` in
``[0, 1)`` und einen gemeinsamen Duty-Factor ``beta`` (Anteil des Zyklus am
Boden). Schwung- und Standfenster verschiedener Beine duerfen sich
UEBERLAPPEN -- genau das macht den echten Ripple-Gang aus: bei ``beta = 2/3``
sind staendig ZWEI Beine gleichzeitig in der Luft (vier am Boden), waehrend
Wave (``beta = 5/6``) nur eins hebt.

Der Fuss-Offset eines Beins relativ zur Standpose ist eine reine Funktion
seiner lokalen Phase ``u = (t - phi) mod 1``:

    Stand   (u in [0, beta)):  Fuss am Boden, schiebt linear von -s/2 nach +s/2.
    Schwung (u in [beta, 1)):  Fuss bogenfoermig zurueck von +s/2 nach -s/2,
                               Hub als Sinus-Halbwelle, Horizontale per
                               smoothstep (senkrechtes Ab-/Aufsetzen).

``s`` ist der volle Boden-Bewegungsvektor ``(dx, dy)`` des Beins pro Zyklus.
Der Offset pendelt symmetrisch um die Standpose (``+/- s/2``) und kehrt ueber
den vollen Zyklus exakt zur Standpose zurueck. Der Uebergang Stand->Schwung
und der Zyklus-Umschlag (u: 1 -> 0) sind in Position UND Z stetig.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from hexapod.gait.trajectory import smoothstep

Vec2 = tuple[float, float]
Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class ContinuousGait:
    """Eine ueberlappende Gangart: Phasen-Offset je Bein + Duty-Factor.

    phase_offsets[leg] = Startzeitpunkt des Standfensters in [0, 1).
    duty = Anteil des Zyklus, den ein Bein am Boden steht (in (0, 1)).
    """

    name: str
    phase_offsets: dict[str, float]
    duty: float
    description: str = ""

    def __post_init__(self) -> None:
        if not 0.0 < self.duty < 1.0:
            raise ValueError(f"duty muss in (0,1) liegen, war {self.duty}")
        for leg, phi in self.phase_offsets.items():
            if not 0.0 <= phi < 1.0:
                raise ValueError(
                    f"phase_offset {leg!r}={phi} ausserhalb [0,1)"
                )

    def legs_in_swing(self, t: float) -> tuple[str, ...]:
        """Beine, die zum Zyklus-Zeitpunkt t gerade schwingen."""
        return tuple(
            leg
            for leg, phi in self.phase_offsets.items()
            if ((t - phi) % 1.0) >= self.duty
        )


# Kontralaterale Hebereihenfolge, gleichmaessig 1/6 versetzt. Duty 2/3 ->
# Schwungfenster 1/3 = 2 x 1/6 -> staendig genau zwei Beine in der Luft.
_RIPPLE_ORDER: tuple[str, ...] = (
    "front_right", "back_left", "mid_right",
    "front_left", "back_right", "mid_left",
)
RIPPLE = ContinuousGait(
    "ripple",
    {leg: i / 6.0 for i, leg in enumerate(_RIPPLE_ORDER)},
    duty=2.0 / 3.0,
    description="2 Beine ueberlappend in der Luft (Duty 2/3), echter Ripple",
)

CONTINUOUS_GAITS: dict[str, ContinuousGait] = {RIPPLE.name: RIPPLE}


def foot_offset(stride: Vec2, height: float, u: float, duty: float) -> Vec3:
    """Fuss-Offset (dx, dy, dz) bei lokaler Phase ``u`` in [0, 1).

    Stand (u < duty): linear von +s/2 nach -s/2, z = 0.
    Schwung (u >= duty): Horizontale smoothstep -s/2 -> +s/2,
        z = height * sin(pi * w) mit w = (u - duty) / (1 - duty).
    """
    sx, sy = stride
    if u < duty:
        f = u / duty if duty > 0 else 0.0
        frac = -0.5 + f              # -s/2 -> +s/2: Fuss bewegt sich um +stride
        return (sx * frac, sy * frac, 0.0)
    w = (u - duty) / (1.0 - duty)    # 0..1 ueber das Schwungfenster
    frac = 0.5 - smoothstep(w)       # +s/2 -> -s/2 (Rueckhol-Schwung, weich)
    z = height * math.sin(math.pi * w)
    return (sx * frac, sy * frac, z)


def cycle_targets(
    gait: ContinuousGait,
    strides: dict[str, Vec2],
    height: float,
    t: float,
) -> dict[str, Vec3]:
    """Ziel-Offsets aller Beine zum globalen Zyklus-Zeitpunkt ``t`` in [0, 1)."""
    out: dict[str, Vec3] = {}
    for leg, stride in strides.items():
        u = (t - gait.phase_offsets[leg]) % 1.0
        out[leg] = foot_offset(stride, height, u, gait.duty)
    return out
