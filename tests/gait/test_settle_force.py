"""settle_to_stance ueberspringt Beine, die schon in der Standpose stehen.

Fuer den Lastabgleich ist genau das falsch: dort geht es nicht darum, eine
Position zu erreichen, sondern die Reibung zu loesen, damit sich die Last neu
verteilen kann. Ausserdem liegen die Korrekturen des Auto-Trims fast immer
unter der 0,5-mm-Toleranz -- ohne force kaeme keine davon je am Roboter an,
waehrend z_trim munter weitergerechnet wird.
"""

from __future__ import annotations

import pytest

from hexapod.config.loader import load_robot_config
from hexapod.gait.posture import settle_to_stance
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
    return Hexapod(config.model_validate(data))


def _zaehle_saetze(robot: Hexapod, monkeypatch: pytest.MonkeyPatch) -> list[int]:
    """Zaehlt, wie viele Positionspakete waehrend eines Aufrufs rausgehen."""
    zaehler = [0]
    for driver in robot.drivers.values():
        orig = driver.set_positions

        def spy(positions, *a, _orig=orig, **k):  # type: ignore[no-untyped-def]
            zaehler[0] += 1
            return _orig(positions, *a, **k)

        monkeypatch.setattr(driver, "set_positions", spy)
    return zaehler


def test_ohne_force_wird_in_der_standpose_nichts_gesendet(
    sim_hexapod: Hexapod, monkeypatch: pytest.MonkeyPatch
) -> None:
    sim_hexapod.stance()
    zaehler = _zaehle_saetze(sim_hexapod, monkeypatch)
    settle_to_stance(sim_hexapod, rate_hz=500.0, pause=0.0)
    assert zaehler[0] == 0


def test_mit_force_werden_alle_beine_angehoben(
    sim_hexapod: Hexapod, monkeypatch: pytest.MonkeyPatch
) -> None:
    sim_hexapod.stance()
    zaehler = _zaehle_saetze(sim_hexapod, monkeypatch)
    settle_to_stance(sim_hexapod, force=True, rate_hz=500.0, pause=0.0)
    # Sechs Beine, jedes mit Hub- und Absetzbahn -> viele Pakete.
    assert zaehler[0] > 6


def test_force_endet_wieder_in_der_standpose(
    sim_hexapod: Hexapod,
) -> None:
    sim_hexapod.stance()
    settle_to_stance(sim_hexapod, force=True, rate_hz=500.0, pause=0.0)
    for leg in sim_hexapod.leg_names:
        cx, cy, cz = sim_hexapod.current_offset(leg)
        assert abs(cx) < 0.01 and abs(cy) < 0.01 and abs(cz) < 0.01, leg


def test_kleine_korrektur_erreicht_die_hardware(
    sim_hexapod: Hexapod, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Der eigentliche Bug: 0,2 mm liegen unter der Ueberspring-Toleranz."""
    sim_hexapod.stance()
    # set_z_trim ist absolut -- fuer eine kleine AENDERUNG auf den
    # bestehenden Wert aufaddieren, sonst springt das Bein um den ganzen
    # bisherigen Trim.
    sim_hexapod.set_z_trim("front_left", sim_hexapod.get_z_trim("front_left") + 0.2)

    zaehler = _zaehle_saetze(sim_hexapod, monkeypatch)
    settle_to_stance(sim_hexapod, rate_hz=500.0, pause=0.0)
    assert zaehler[0] == 0, "ohne force wird die Korrektur verschluckt"

    zaehler[0] = 0
    settle_to_stance(sim_hexapod, force=True, rate_hz=500.0, pause=0.0)
    assert zaehler[0] > 0
