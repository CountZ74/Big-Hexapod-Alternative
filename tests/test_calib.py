"""Tests für die Lastauswertung — die Kernidee ohne jede Hardware."""

from __future__ import annotations

import pytest

from hexapod.calib import (
    estimate_travel_mm,
    fit_load_plane,
    z_trim_corrections,
)

# Sechs Fußpositionen wie am echten Roboter (x vorwaerts, y links), in mm.
FUESSE = {
    "front_right": (75.0, -85.0),
    "front_left": (75.0, 85.0),
    "mid_right": (0.0, -100.0),
    "mid_left": (0.0, 100.0),
    "back_right": (-75.0, -85.0),
    "back_left": (-75.0, 85.0),
}


def _levels(z0: float = 0.2, a: float = 0.0, b: float = 0.0,
            fehler: dict[str, float] | None = None) -> dict[str, float]:
    """Federwege aus Ebene + Bein-Einzelfehlern zusammensetzen."""
    fehler = fehler or {}
    return {
        leg: z0 + a * x + b * y + fehler.get(leg, 0.0)
        for leg, (x, y) in FUESSE.items()
    }


# ---------------------------------------------------------------------
# Trennung von Ebene und Einzelfehler
# ---------------------------------------------------------------------


def test_perfekte_beine_ergeben_null_residuen() -> None:
    r = fit_load_plane(_levels(), FUESSE)
    for leg, res in r.residuals.items():
        assert res == pytest.approx(0.0, abs=1e-9), leg


def test_schwerpunkt_vorne_faellt_heraus() -> None:
    """Mehr Last vorne ist kein Beinfehler, sondern Schwerpunkt."""
    r = fit_load_plane(_levels(a=0.001), FUESSE)   # +0.1 Federweg je 100 mm
    for leg, res in r.residuals.items():
        assert res == pytest.approx(0.0, abs=1e-9), leg
    assert r.plane[1] == pytest.approx(0.001)


def test_bodenneigung_faellt_ebenfalls_heraus() -> None:
    """Eine schiefe Flaeche erzeugt nur eine Ebene — kein Residuum."""
    r = fit_load_plane(_levels(a=0.0008, b=-0.0005), FUESSE)
    assert max(abs(v) for v in r.residuals.values()) < 1e-9


def test_einzelnes_zu_langes_bein_wird_isoliert() -> None:
    r = fit_load_plane(_levels(a=0.001, fehler={"front_left": 0.10}), FUESSE)
    assert r.worst_leg == "front_left"
    assert r.residuals["front_left"] > 0.0
    # Der Ausreisser hat das groesste Residuum -- aber nicht das einzige:
    # der Fit kippt ihm hinterher und erzeugt bei den Nachbarbeinen
    # Gegenresiduen. Deshalb korrigiert die Schleife iterativ.
    assert all(
        abs(v) < r.residuals["front_left"]
        for leg, v in r.residuals.items() if leg != "front_left"
    )


def test_zu_kurzes_bein_bekommt_negatives_residuum() -> None:
    r = fit_load_plane(_levels(fehler={"back_right": -0.08}), FUESSE)
    assert r.residuals["back_right"] < 0.0
    assert r.worst_leg == "back_right"


def test_residuum_unterschaetzt_den_fehler_systematisch() -> None:
    """Der Ebenen-Fit kippt dem Ausreisser ein Stueck hinterher.

    Bei sechs Punkten und drei Parametern liegt die mittlere Hebelwirkung
    bei 3/6 = 0.5 -- ein Einzelfehler taucht also nur etwa zur Haelfte im
    Residuum auf. Das ist der Grund, warum die Trimm-Schleife iteriert,
    statt in einem Schritt fertig zu sein: jede Runde nimmt einen Teil weg,
    der Rest wird beim naechsten Mal sichtbar.
    """
    fehler = 0.10
    r = fit_load_plane(_levels(fehler={"front_left": fehler}), FUESSE)
    anteil = r.residuals["front_left"] / fehler
    assert 0.35 < anteil < 0.75, anteil


def test_zu_wenige_beine() -> None:
    with pytest.raises(ValueError, match="Mindestens"):
        fit_load_plane({"a": 0.1, "b": 0.2, "c": 0.3},
                       {"a": (0.0, 0.0), "b": (1.0, 0.0), "c": (0.0, 1.0)})


# ---------------------------------------------------------------------
# Korrekturen
# ---------------------------------------------------------------------


def test_korrektur_wirkt_dem_residuum_entgegen() -> None:
    korr = z_trim_corrections({"a": 0.10, "b": -0.10, "c": 0.0, "d": 0.0},
                              travel_mm=5.0, damping=1.0)
    assert korr["a"] < 0    # traegt zu viel -> Fuss hoeher
    assert korr["b"] > 0    # traegt zu wenig -> Fuss tiefer


def test_korrektur_ist_mittelwertfrei() -> None:
    """Sonst wandert der ganze Koerper hoch oder runter."""
    korr = z_trim_corrections({"a": 0.30, "b": 0.20, "c": 0.10, "d": 0.0},
                              travel_mm=5.0)
    assert sum(korr.values()) == pytest.approx(0.0, abs=1e-9)


def test_daempfung_skaliert_die_korrektur() -> None:
    voll = z_trim_corrections({"a": 0.1, "b": -0.1, "c": 0.0, "d": 0.0},
                              travel_mm=5.0, damping=1.0)
    halb = z_trim_corrections({"a": 0.1, "b": -0.1, "c": 0.0, "d": 0.0},
                              travel_mm=5.0, damping=0.5)
    assert halb["a"] == pytest.approx(voll["a"] * 0.5)


def test_unsinnige_parameter() -> None:
    with pytest.raises(ValueError, match="travel_mm"):
        z_trim_corrections({"a": 0.0, "b": 0.0}, travel_mm=0.0)
    with pytest.raises(ValueError, match="damping"):
        z_trim_corrections({"a": 0.0, "b": 0.0}, travel_mm=5.0, damping=1.5)


# ---------------------------------------------------------------------
# Selbstvermessung des Federwegs
# ---------------------------------------------------------------------


def test_vollweg_wird_zurueckgerechnet() -> None:
    """1 mm z_trim erzeugt 0.2 Federweg -> Vollweg 5 mm."""
    got = estimate_travel_mm(
        {"a": 1.0, "b": -1.0, "c": 0.5},
        {"a": 0.20, "b": 0.20, "c": 0.20},
        {"a": 0.40, "b": 0.00, "c": 0.30},
    )
    assert got == pytest.approx(5.0)


def test_zu_kleine_schritte_liefern_keine_schaetzung() -> None:
    assert estimate_travel_mm(
        {"a": 0.001, "b": -0.001, "c": 0.0},
        {"a": 0.2, "b": 0.2, "c": 0.2},
        {"a": 0.2, "b": 0.2, "c": 0.2},
    ) is None


def test_verrauschte_reaktion_sprengt_die_schaetzung_nicht() -> None:
    """Der Fall, der einmal 15 mm statt 5,5 mm geliefert hat.

    Ein Bein reagiert kaum -- als Median von Quotienten waere das ein
    riesiger Ausreisser. Die Regression gewichtet es klein.
    """
    got = estimate_travel_mm(
        {"a": 1.0, "b": -1.0, "c": 0.5, "d": -0.5},
        dict.fromkeys("abcd", 0.20),
        {"a": 0.38, "b": 0.02, "c": 0.29, "d": 0.199},   # d fast keine Reaktion
    )
    assert got is not None
    assert 4.5 < got < 6.5, got


def test_gegenlaeufige_reaktion_wird_verworfen() -> None:
    assert estimate_travel_mm(
        {"a": 1.0, "b": -1.0, "c": 0.5},
        dict.fromkeys("abc", 0.20),
        {"a": 0.10, "b": 0.30, "c": 0.15},   # falsche Richtung
    ) is None


def test_schleife_konvergiert() -> None:
    """Mehrere Runden aus Messen und Korrigieren raeumen den Fehler ab.

    Formal: das Residuum ist (I - H)·fehler mit der Hut-Matrix H des Fits.
    (I - H) ist eine Projektion, ihre Eigenwerte sind 0 und 1. Der Anteil
    des Fehlers, der in der Ebenen-Ebene liegt, bleibt deshalb unangetastet
    -- er ist von Schwerpunkt und Bodenneigung nicht unterscheidbar. Alles
    Uebrige faellt mit (1 - damping) je Runde.
    """
    travel = 5.0
    fehler = {"front_left": 0.10, "back_right": -0.06}
    z_trim = dict.fromkeys(FUESSE, 0.0)

    verlauf = []
    for _ in range(6):
        # z_trim wirkt wie ein zusaetzlicher Beinlaengenfehler
        eff = {leg: fehler.get(leg, 0.0) + z_trim[leg] / travel for leg in FUESSE}
        r = fit_load_plane(_levels(a=0.001, fehler=eff), FUESSE)
        verlauf.append(r.spread)
        for leg, d in z_trim_corrections(r.residuals, travel_mm=travel).items():
            z_trim[leg] += d

    assert verlauf[-1] < verlauf[0] / 20, verlauf


# ---------------------------------------------------------------------
# Neigungsanteil (das, was die Lastsensoren nicht sehen koennen)
# ---------------------------------------------------------------------


def test_neigung_hebt_die_hohe_seite_an() -> None:
    """Positiver Roll = Koerper links hoch -> linke Fuesse kuerzer."""
    from hexapod.calib import tilt_corrections
    korr = tilt_corrections(0.02, 0.0, FUESSE)
    assert korr["front_left"] < 0 and korr["mid_left"] < 0
    assert korr["front_right"] > 0 and korr["mid_right"] > 0


def test_nicken_wirkt_auf_die_laengsachse() -> None:
    """Positiver Pitch = Koerper vorne tief -> vordere Fuesse laenger."""
    from hexapod.calib import tilt_corrections
    korr = tilt_corrections(0.0, 0.02, FUESSE)
    assert korr["front_left"] > 0 and korr["front_right"] > 0
    assert korr["back_left"] < 0 and korr["back_right"] < 0


def test_neigungskorrektur_ist_mittelwertfrei() -> None:
    from hexapod.calib import tilt_corrections
    korr = tilt_corrections(0.01, -0.02, FUESSE)
    assert sum(korr.values()) == pytest.approx(0.0, abs=1e-9)


def test_lastresiduen_sehen_reine_neigung_nicht() -> None:
    """Der Kern der Arbeitsteilung: eine Ebene erzeugt keine Residuen.

    Genau deshalb braucht es die IMU -- und genau deshalb darf man sie
    NICHT alleine benutzen, denn ein schiefer Boden saehe identisch aus.
    """
    from hexapod.calib import tilt_corrections
    neigung = tilt_corrections(0.02, 0.01, FUESSE)   # als Beinfehler gedacht
    r = fit_load_plane(_levels(fehler={k: v / 5.0 for k, v in neigung.items()}), FUESSE)
    assert max(abs(v) for v in r.residuals.values()) < 1e-9
