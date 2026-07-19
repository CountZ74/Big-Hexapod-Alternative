"""stand_up darf im Stehen NICHT in die Liegepose springen.

Regression: stand_up() faehrt bedingungslos aus der abgesetzten Lage hoch
(Fuesse am Boden). Wird die Action im Stehen ausgeloest (z.B. 2-Pos-Schalter
am Controller), sackte der Koerper erst ab und stand dann wieder auf. Der
Worker muss den Zustand pruefen: nur aus 'lying' die Aufsteh-Sequenz fahren,
im Stehen nur sauber in die Standpose (move_to_stance).
"""
from __future__ import annotations

import threading

import hexapod.server.worker as worker_mod
from hexapod.server.worker import RobotWorker


class FakeMotion:
    def halt(self) -> None:
        pass


def make_worker(state: str, tmp_path) -> RobotWorker:
    # Ohne __init__ (keine Hardware/ADC): nur die fuer _execute noetigen Felder.
    w = RobotWorker.__new__(RobotWorker)
    w._lock = threading.Lock()
    w._motion = FakeMotion()
    w._robot_state = state
    w._state_file = str(tmp_path / "state")
    w._last_command = None
    return w


def _patch(monkeypatch):
    calls: list[str] = []
    monkeypatch.setattr(worker_mod, "stand_up", lambda r, **k: calls.append("stand_up"))
    monkeypatch.setattr(worker_mod, "move_to_stance", lambda r, **k: calls.append("stance"))
    return calls


def test_stand_up_while_standing_does_not_drop(monkeypatch, tmp_path) -> None:
    calls = _patch(monkeypatch)
    w = make_worker("standing", tmp_path)
    w._execute(object(), {"action": "stand_up"})
    assert calls == ["stance"]          # KEINE Ground-first-Aufstehsequenz
    assert w._robot_state == "standing"


def test_stand_up_while_lying_stands_up(monkeypatch, tmp_path) -> None:
    calls = _patch(monkeypatch)
    w = make_worker("lying", tmp_path)
    w._execute(object(), {"action": "stand_up"})
    assert calls == ["stand_up"]        # echte Aufsteh-Sequenz
    assert w._robot_state == "standing"
