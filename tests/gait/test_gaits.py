"""Tests fuer die Gangart-Definitionen und die Phasen-Mechanik."""
from __future__ import annotations

import pytest

from hexapod.gait.gaits import (
    GAITS,
    LEGS_ALL,
    get_gait,
    phase_amplitude,
    phase_end_offsets,
)


class TestGaitDefinitions:
    @pytest.mark.parametrize("name", sorted(GAITS))
    def test_each_leg_swings_exactly_once(self, name) -> None:
        gait = GAITS[name]
        seen = [leg for ph in gait.phases for leg in ph]
        assert sorted(seen) == sorted(LEGS_ALL)
        assert len(seen) == len(set(seen)) == 6

    def test_phase_counts(self) -> None:
        assert GAITS["tripod"].n_phases == 2
        assert GAITS["tetrapod"].n_phases == 3
        assert GAITS["ripple"].n_phases == 6
        assert GAITS["wave"].n_phases == 6

    def test_swing_phase_mapping(self) -> None:
        wave = GAITS["wave"]
        # Jede Phase genau ein Bein -> swing_phase ist eine Bijektion 0..5.
        assert sorted(wave.swing_phase.values()) == [0, 1, 2, 3, 4, 5]

    def test_get_gait_unknown_raises(self) -> None:
        with pytest.raises(ValueError, match="Unbekannte Gangart"):
            get_gait("gallop")

    def test_invalid_gait_rejected(self) -> None:
        from hexapod.gait.gaits import Gait
        with pytest.raises(ValueError, match="genau"):
            Gait("bad", (("front_left",),))  # nicht alle Beine


class TestPhaseMechanics:
    @pytest.mark.parametrize("name", sorted(GAITS))
    def test_amplitude_centered_and_bounded(self, name) -> None:
        gait = GAITS[name]
        n = gait.n_phases
        amps = []
        for sp in gait.swing_phase.values():
            for k in range(n):
                amps.append(phase_amplitude(n, k, sp))
        bound = (n - 1) / (2.0 * n)
        assert max(amps) == pytest.approx(bound)
        assert min(amps) == pytest.approx(-bound)
        # Symmetrisch um 0 (Mittelwert ueber alle Beine/Phasen = 0).
        assert sum(amps) == pytest.approx(0.0, abs=1e-9)

    @pytest.mark.parametrize("name", sorted(GAITS))
    def test_foot_returns_to_neutral_over_cycle(self, name) -> None:
        # Summe der Stand-Rueckschuebe + Schwung-Vorlauf eines Beins ueber
        # einen vollen Zyklus = 0 (kein Drift).
        gait = GAITS[name]
        n = gait.n_phases
        strides = {leg: (10.0, 4.0) for leg in LEGS_ALL}
        # Offsets ueber alle Phasen fuer ein Bein einsammeln; Differenzen
        # (= Bewegung pro Phase) muessen sich zu 0 aufaddieren.
        leg = LEGS_ALL[0]
        offs = [phase_end_offsets(gait, k, strides)[leg] for k in range(n)]
        # Ringschluss: letzter -> erster
        total = [0.0, 0.0]
        prev = offs[-1]
        for cur in offs:
            total[0] += cur[0] - prev[0]
            total[1] += cur[1] - prev[1]
            prev = cur
        assert total[0] == pytest.approx(0.0, abs=1e-9)
        assert total[1] == pytest.approx(0.0, abs=1e-9)

    def test_swing_leg_lands_farthest_in_travel_dir(self) -> None:
        # Das Bein, das in dieser Phase schwingt, landet am Ende seines Schwungs
        # am weitesten in Fahrtrichtung. Fahrtrichtung = -stride (der Fuss schiebt
        # im Stand um +stride zurueck). Bei positivem stride (10,0) ist das die
        # kleinste x-Koordinate.
        gait = GAITS["wave"]
        strides = {leg: (10.0, 0.0) for leg in LEGS_ALL}
        for k in range(gait.n_phases):
            offs = phase_end_offsets(gait, k, strides)
            swing_leg = gait.phases[k][0]
            fwd = {leg: x for leg, (x, _y, _z) in offs.items()}
            assert fwd[swing_leg] == min(fwd.values())

    def test_zero_stride_zero_offset(self) -> None:
        gait = GAITS["tetrapod"]
        offs = phase_end_offsets(gait, 0, {leg: (0.0, 0.0) for leg in LEGS_ALL})
        assert all(o == (0.0, 0.0, 0.0) for o in offs.values())
