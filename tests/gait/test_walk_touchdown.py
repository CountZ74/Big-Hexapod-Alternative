"""Aufsetz-Erkennung im Tripod-Gang.

Anders als beim Absetzen einzelner Beine schwingen hier drei gleichzeitig.
Findet eines frueher Boden, muss genau dieses halten -- die anderen beiden
duerfen sich davon nicht stoeren lassen, sonst bricht der Gang zusammen.
"""

from __future__ import annotations

import pytest

from hexapod.config.loader import load_robot_config
from hexapod.drivers.foot_sensor import FootSensorReading
from hexapod.gait.tripod import GROUP_A, GROUP_B, walk
from hexapod.robot import Hexapod

CONFIG_PATH = "config/robot.yaml"


@pytest.fixture
def sim_hexapod() -> Hexapod:
    config = load_robot_config(CONFIG_PATH)
    data = config.model_dump()
    data["buses"] = {
        n: {"type": "simulator", "num_channels": b["num_channels"]}
        for n, b in data["buses"].items()
    }
    robot = Hexapod(config.model_validate(data))
    robot.stance()
    return robot


# Im Stand ist die Feder schon ein Stueck eingedrueckt -- rund 20 % von
# 5,5 mm. Der Fuss beruehrt den Boden also, waehrend das Bein noch auf
# Standpose-Hoehe steht.
STAND_EINFEDERUNG_MM = 1.1
FEDERWEG_MM = 5.5


def _boden(robot: Hexapod, hoehen: dict[str, float],
           nachsinken_mm: float = 0.0) -> None:
    """Simulierter Boden mit Feder.

    hoehen: Bein -> Hoehe des Untergrunds als Offset zur Standpose. 0.0 ist
        ebener Boden, positive Werte sind Hindernisse.
    nachsinken_mm: Wie weit der Fuss beim Anheben zusaetzlich in Kontakt
        bleibt. Bildet ab, dass der Koerper absackt, wenn ein Bein seine Last
        abgibt -- die uebrigen fuenf federn dabei ein. Genau daran ist die
        erste Fassung gescheitert: sie hat die Beine beim ABHEBEN
        eingefroren, weil sie dort noch belastet waren.
    """
    array = robot.foot_sensors
    assert array is not None

    def read(leg: str, *, samples: int | None = None) -> FootSensorReading:
        z = robot.current_offset(leg)[2]
        kontakt_bis = (hoehen.get(leg, 0.0) + STAND_EINFEDERUNG_MM
                       + nachsinken_mm)
        stauchung = kontakt_bis - z
        level = max(0.0, min(1.0, stauchung / FEDERWEG_MM))
        return FootSensorReading(leg=leg, channel=0, raw=0.0, level=level)

    array.read = read  # type: ignore[method-assign]


def test_ohne_erkennung_unveraendert(sim_hexapod: Hexapod) -> None:
    treffer = walk(sim_hexapod, cycles=1, rate_hz=500.0, steps=8)
    assert treffer == {}


def test_ebener_boden_loest_nicht_aus(sim_hexapod: Hexapod) -> None:
    """Beim normalen Aufsetzen meldet jedes Bein Kontakt -- das ist kein Grund
    anzuhalten, sonst sackt der Roboter nie in die Standpose."""
    _boden(sim_hexapod, dict.fromkeys(sim_hexapod.leg_names, 0.0))
    treffer = walk(sim_hexapod, cycles=1, rate_hz=500.0, steps=8, touch_level=0.05)
    assert treffer == {}


def test_hindernis_haelt_nur_das_betroffene_bein(sim_hexapod: Hexapod) -> None:
    hoehen = dict.fromkeys(sim_hexapod.leg_names, 0.0)
    hoehen["mid_left"] = 20.0          # mid_left gehoert zu GROUP_A
    _boden(sim_hexapod, hoehen)

    treffer = walk(sim_hexapod, cycles=1, rate_hz=500.0, steps=10, touch_level=0.05)

    assert "mid_left" in treffer
    assert treffer["mid_left"] > 12.0
    # Die beiden anderen der Gruppe sind unbehelligt geblieben
    for leg in GROUP_A:
        if leg != "mid_left":
            assert leg not in treffer


def test_gemessene_hoehe_passt_zum_hindernis(sim_hexapod: Hexapod) -> None:
    """Die zurueckgegebene Hoehe ist die Gelaendehoehe an dieser Stelle."""
    hoehen = dict.fromkeys(sim_hexapod.leg_names, 0.0)
    hoehen["front_left"] = 20.0        # front_left gehoert zu GROUP_B
    _boden(sim_hexapod, hoehen)

    treffer = walk(sim_hexapod, cycles=1, rate_hz=500.0, steps=12, touch_level=0.05)

    assert "front_left" in treffer
    # Etwas mehr als die Klotzhoehe: der Sensor meldet Kontakt schon,
    # waehrend die Feder noch nachgibt.
    assert 18.0 < treffer["front_left"] < 25.0, treffer["front_left"]


def test_beide_gruppen_werden_geprueft(sim_hexapod: Hexapod) -> None:
    hoehen = dict.fromkeys(sim_hexapod.leg_names, 0.0)
    hoehen[GROUP_A[0]] = 20.0
    hoehen[GROUP_B[0]] = 20.0
    _boden(sim_hexapod, hoehen)

    treffer = walk(sim_hexapod, cycles=1, rate_hz=500.0, steps=10, touch_level=0.05)

    assert GROUP_A[0] in treffer and GROUP_B[0] in treffer


def test_gang_laeuft_nach_dem_hindernis_weiter(sim_hexapod: Hexapod) -> None:
    """Ein eingefrorenes Bein darf den Zyklus nicht abbrechen."""
    hoehen = dict.fromkeys(sim_hexapod.leg_names, 0.0)
    hoehen["mid_left"] = 20.0
    _boden(sim_hexapod, hoehen)

    walk(sim_hexapod, cycles=2, rate_hz=500.0, steps=8, touch_level=0.05)

    # Alle uebrigen Beine stehen am Ende wieder sauber in der Standpose
    for leg in sim_hexapod.leg_names:
        if leg == "mid_left":
            continue
        assert abs(sim_hexapod.current_offset(leg)[2]) < 0.5, leg


def test_kein_einfrieren_beim_abheben(sim_hexapod: Hexapod) -> None:
    """Der Fehler, den der Roboter gezeigt hat: die Fuesse gingen nicht hoch.

    Beim Anheben verlaesst der Fuss den Boden nicht sofort -- die Feder
    entspannt sich erst, und der Koerper sackt nach, weil die uebrigen Beine
    die Last uebernehmen. Das Bein ist also noch belastet, waehrend sein
    Offset die Mindesthoehe laengst ueberschritten hat. Wer dort auf Kontakt
    prueft, friert das Bein direkt beim Abheben ein.
    """
    _boden(sim_hexapod, dict.fromkeys(sim_hexapod.leg_names, 0.0),
           nachsinken_mm=8.0)

    hoch: dict[str, float] = dict.fromkeys(sim_hexapod.leg_names, 0.0)
    original = sim_hexapod.set_all_foot_offsets

    def mitschreiben(offsets, **kwargs):  # type: ignore[no-untyped-def]
        for leg, (_, _, z) in offsets.items():
            hoch[leg] = max(hoch[leg], z)
        return original(offsets, **kwargs)

    sim_hexapod.set_all_foot_offsets = mitschreiben  # type: ignore[method-assign]
    gelaende = walk(sim_hexapod, cycles=1, rate_hz=500.0, steps=12,
                    touch_level=0.05, height=30.0)

    for leg, z in hoch.items():
        assert z > 25.0, f"{leg} kam nur auf {z:.1f} mm statt 30 mm"
    assert gelaende == {}, "ebener Boden darf kein Gelaende melden"


def test_gleichmaessiges_absacken_ist_kein_gelaende(sim_hexapod: Hexapod) -> None:
    """Absolut gemessen setzen alle Beine zu hoch auf -- relativ nicht.

    Genau dafuer ist der Bezug auf den Gruppenmedian da: ein tiefer sitzender
    Koerper verschiebt alle drei Beine der Gruppe gleich und faellt heraus.
    """
    _boden(sim_hexapod, dict.fromkeys(sim_hexapod.leg_names, 0.0),
           nachsinken_mm=6.0)
    assert walk(sim_hexapod, cycles=1, rate_hz=500.0, steps=12,
                touch_level=0.05, height=30.0) == {}


def test_hindernis_hebt_sich_von_der_gruppe_ab(sim_hexapod: Hexapod) -> None:
    """Ein Klotz unter einem Fuss -- gemeldet wird der Abstand zur Gruppe."""
    hoehen = dict.fromkeys(sim_hexapod.leg_names, 0.0)
    hoehen["front_left"] = 10.0
    _boden(sim_hexapod, hoehen, nachsinken_mm=6.0)

    gelaende = walk(sim_hexapod, cycles=1, rate_hz=500.0, steps=40,
                    touch_level=0.05, height=30.0)

    assert set(gelaende) == {"front_left"}
    # Gemeldet wird mehr als die 10 mm Klotzdicke: der Fuss meldet Kontakt
    # schon, waehrend die Feder noch nachgibt. Die Feder ist ein mechanischer
    # Tiefpass -- sie gleicht kleine Unebenheiten aus, und genau deshalb
    # setzt die Erkennung frueher an, als der Klotz hoch ist.
    assert gelaende["front_left"] > 9.0, gelaende["front_left"]


def test_normales_bein_wird_nicht_als_loch_gemeldet(sim_hexapod: Hexapod) -> None:
    """Am Roboter gemessen: back_right +6.2 mm, mid_right -6.2 mm, ohne Hindernis.

    Die -6.2 entstanden, weil zwei Beine der Gruppe auf dem systematischen
    Kontaktniveau angehalten hatten und das dritte regulaer durchgefedert
    war. Gegen diesen Median sah das normale Bein aus wie ein Loch.
    """
    hoehen = dict.fromkeys(sim_hexapod.leg_names, 0.0)
    for leg in ("front_left", "back_left"):
        hoehen[leg] = 14.0
    _boden(sim_hexapod, hoehen)

    gelaende = walk(sim_hexapod, cycles=1, rate_hz=500.0, steps=40,
                    touch_level=0.05, height=30.0, touch_margin_mm=10.0)

    assert "mid_right" not in gelaende, gelaende
    # Dass die beiden Stufenbeine hier NICHT gemeldet werden, ist die Kehrseite
    # des Gruppenbezugs: stehen zwei von drei Beinen auf derselben Stufe, ist
    # sie der Median und damit definitionsgemaess "der Boden". Ein relatives
    # Mass kann das nicht aufloesen -- dafuer braucht es den MPU6050.


def test_ebener_boden_meldet_nichts_mit_der_gang_grenze(sim_hexapod: Hexapod) -> None:
    """Mit der gemessenen Grenze loest das Absacken beim Schwung nicht aus."""
    _boden(sim_hexapod, dict.fromkeys(sim_hexapod.leg_names, 0.0),
           nachsinken_mm=6.2)
    assert walk(sim_hexapod, cycles=2, rate_hz=500.0, steps=40,
                touch_level=0.05, height=30.0) == {}
