"""Gangart-Definitionen und verallgemeinerte Phasen-Mechanik.

Eine Gangart ist eine geordnete Liste von Phasen. In jeder Phase schwingt
eine Teilmenge der Beine durch die Luft nach vorne, der Rest steht am Boden
und schiebt den Koerper. Jedes Bein schwingt GENAU EINMAL pro vollem Zyklus
-- die Phasen partitionieren also die sechs Beine.

    tripod:   2 Phasen x 3 Beine (3 am Boden)  -> schnellste, klassisch
    tetrapod: 3 Phasen x 2 Beine (4 am Boden)  -> mittlere Geschwindigkeit
    ripple:   6 Phasen x 1 Bein  (5 am Boden)  -> langsam, kontralateral rippelnd
    wave:     6 Phasen x 1 Bein  (5 am Boden)  -> am stabilsten, metachronal

Mehr Phasen = mehr Zeit pro vollem Zyklus = langsamer, aber stabiler.

Geometrie (verallgemeinert, akkumulierend):

Pro Phase schiebt jedes Standbein den Koerper um ``stride/N`` vor (Fuss
bewegt sich um ``-stride/N`` relativ zum Koerper). Ein Bein steht ``N-1``
Phasen (driftet nach hinten) und holt in seiner einen Schwungphase die
gesamte Strecke ``(N-1)*stride/N`` wieder auf. Der Fuss-Offset pendelt damit
symmetrisch um die Standpose im Bereich ``+/-(N-1)/(2N)*stride`` und kehrt
ueber den vollen Zyklus exakt zur Standpose zurueck. Der Tripod (N=2) wird
weiterhin vom bestehenden zentrierten Halbzyklus gefahren; diese Mechanik
dient den mehrphasigen Gangarten.
"""
from __future__ import annotations

from dataclasses import dataclass

Vec2 = tuple[float, float]
Vec3 = tuple[float, float, float]

# Kanonische Beinnamen (muessen mit der Roboter-Konfiguration uebereinstimmen).
LEGS_ALL: tuple[str, ...] = (
    "front_left", "front_right",
    "mid_left", "mid_right",
    "back_left", "back_right",
)


@dataclass(frozen=True)
class Gait:
    """Eine Gangart als geordnete Phasen-Sequenz.

    phases[i] = Tupel der Beine, die in Phase i schwingen. Die Vereinigung
    aller Phasen ergibt jedes Bein genau einmal.
    """

    name: str
    phases: tuple[tuple[str, ...], ...]
    description: str = ""

    def __post_init__(self) -> None:
        seen = [leg for ph in self.phases for leg in ph]
        if sorted(seen) != sorted(LEGS_ALL):
            raise ValueError(
                f"Gait {self.name!r}: Phasen muessen alle sechs Beine genau "
                f"einmal enthalten, war {seen}"
            )

    @property
    def n_phases(self) -> int:
        return len(self.phases)

    @property
    def swing_phase(self) -> dict[str, int]:
        """Bein-Name -> Index der Phase, in der es schwingt."""
        return {leg: i for i, ph in enumerate(self.phases) for leg in ph}


TRIPOD = Gait(
    "tripod",
    (("front_right", "mid_left", "back_right"),
     ("front_left", "mid_right", "back_left")),
    "3+3 Beine, schnell",
)
TETRAPOD = Gait(
    "tetrapod",
    (("front_left", "back_right"),
     ("mid_left", "mid_right"),
     ("front_right", "back_left")),
    "2+2+2 Beine, mittel",
)
RIPPLE = Gait(
    "ripple",
    (("front_right",), ("back_left",), ("mid_right",),
     ("front_left",), ("back_right",), ("mid_left",)),
    "1 Bein, kontralateral rippelnd",
)
WAVE = Gait(
    "wave",
    (("back_right",), ("mid_right",), ("front_right",),
     ("back_left",), ("mid_left",), ("front_left",)),
    "1 Bein, metachronal, sehr stabil",
)

GAITS: dict[str, Gait] = {g.name: g for g in (TRIPOD, TETRAPOD, RIPPLE, WAVE)}


def get_gait(name: str) -> Gait:
    """Gangart nach Name; wirft ValueError bei unbekanntem Namen."""
    try:
        return GAITS[name]
    except KeyError:
        raise ValueError(
            f"Unbekannte Gangart {name!r}. Verfuegbar: {sorted(GAITS)}"
        ) from None


def phase_amplitude(n_phases: int, phase_index: int, swing_phase: int) -> float:
    """Offset-Faktor eines Beins am ENDE einer Phase, in Einheiten von stride.

    Das Bein steht ``j = (phase_index - swing_phase) mod N`` Phasen seit dem
    Ende seiner Schwungphase (j=0 = es hat gerade geschwungen, ganz vorne).
    Der Faktor laeuft linear von ``-(N-1)/(2N)`` (hinten, gerade gelandet) in
    Schritten von ``+1/N`` bis ``+(N-1)/(2N)`` (vorne) und ist um 0 zentriert.
    """
    n = n_phases
    j = (phase_index - swing_phase) % n
    # Gerade geschwungenes Bein (j=0) startet HINTEN (-(n-1)/2n) und laeuft in
    # +stride-Richtung nach vorne -- gleiche Konvention wie der Tripod, sodass
    # ein Vorwaerts-Befehl den Koerper auch vorwaerts treibt.
    return (j - (n - 1) / 2.0) / n


def phase_end_offsets(
    gait: Gait, phase_index: int, strides: dict[str, Vec2]
) -> dict[str, Vec3]:
    """Ziel-Fussoffsets (un-posed) am Ende der Phase ``phase_index``.

    Args:
        gait: die aktive Gangart.
        phase_index: aktuelle Phase (0..N-1).
        strides: Bein-Name -> (dx, dy) voller Boden-Bewegungsvektor pro Zyklus.

    Returns:
        Bein-Name -> (x, y, 0.0) Ziel-Offset relativ zur Standpose.
    """
    sp = gait.swing_phase
    out: dict[str, Vec3] = {}
    for leg, (sx, sy) in strides.items():
        a = phase_amplitude(gait.n_phases, phase_index, sp[leg])
        out[leg] = (sx * a, sy * a, 0.0)
    return out
