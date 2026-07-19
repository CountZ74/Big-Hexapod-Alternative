"""Tests fuer die Sonar-Hindernislogik (rein, keine Hardware)."""
from hexapod.server.sonar_thread import decide_free_dir, SonarThread


class TestDecideFreeDir:
    def test_right_blocked_left_open(self):
        prof = {-40.0: 1.5, -20.0: 1.2, 0.0: 0.2, 20.0: 0.2, 40.0: 0.25}
        assert decide_free_dir(prof) == "left"

    def test_left_blocked_right_open(self):
        prof = {-40.0: 0.2, -20.0: 0.25, 0.0: 0.2, 20.0: 1.4, 40.0: 1.6}
        assert decide_free_dir(prof) == "right"

    def test_both_blocked(self):
        prof = {-40.0: 0.3, -20.0: 0.2, 0.0: 0.2, 20.0: 0.25, 40.0: 0.3}
        assert decide_free_dir(prof) == "none"

    def test_both_open_picks_more_clearance(self):
        prof = {-40.0: 0.8, -20.0: 0.9, 0.0: 0.2, 20.0: 1.8, 40.0: 2.0}
        assert decide_free_dir(prof) == "right"

    def test_none_distance_counts_as_open(self):
        # kein Echo = frei; links None -> links frei
        prof = {-40.0: None, -20.0: None, 0.0: 0.2, 20.0: 0.2, 40.0: 0.25}
        assert decide_free_dir(prof) == "left"


class TestSonarDebounce:
    def _thr(self):
        # Sonar=None-Dummy vermeiden: echten SonarThread ohne Start, nur _update testen
        import hexapod.server.sonar_thread as m

        class _FakeSonar:
            def distance(self, samples=3):
                return None
            def close(self):
                pass
        return SonarThread(sonar=_FakeSonar(), threshold_m=0.25)

    def test_block_needs_confirm(self):
        t = self._thr()
        t.set_enabled(True)
        t._update(0.2, 0.25)
        assert t.blocked is False   # erst 1x
        t._update(0.2, 0.25)
        assert t.blocked is True    # 2x -> blockiert

    def test_clear_needs_confirm(self):
        t = self._thr()
        t.set_enabled(True)
        t._update(0.2, 0.25); t._update(0.2, 0.25)
        assert t.blocked is True
        t._update(1.0, 0.25); t._update(1.0, 0.25)
        assert t.blocked is True    # noch nicht 3x frei
        t._update(1.0, 0.25)
        assert t.blocked is False

    def test_disabled_never_blocked(self):
        t = self._thr()
        t.set_enabled(True); t._update(0.2, 0.25); t._update(0.2, 0.25)
        t.set_enabled(False)
        assert t.blocked is False
