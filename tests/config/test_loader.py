"""Tests für load_robot_config und dump_robot_config."""

from __future__ import annotations

from pathlib import Path

import pytest
from pydantic import ValidationError

from hexapod.config import RobotConfig
from hexapod.config.loader import dump_robot_config, load_robot_config


# Pfad zur echten Beispiel-Konfig im Repo
EXAMPLE_CONFIG = Path(__file__).resolve().parents[2] / "config" / "robot.yaml"


class TestLoadRealConfig:
    """Stellt sicher, dass die mitgelieferte robot.yaml gültig ist."""

    def test_real_config_loads(self) -> None:
        assert EXAMPLE_CONFIG.exists(), f"Erwartet Beispiel-Konfig bei {EXAMPLE_CONFIG}"
        config = load_robot_config(EXAMPLE_CONFIG)
        assert isinstance(config, RobotConfig)

    def test_real_config_has_six_legs(self) -> None:
        config = load_robot_config(EXAMPLE_CONFIG)
        assert len(config.body.legs) == 6

    def test_real_config_has_twenty_servos(self) -> None:
        config = load_robot_config(EXAMPLE_CONFIG)
        assert len(config.servos) == 20


class TestLoaderErrors:
    def test_raises_on_missing_file(self, tmp_path: Path) -> None:
        with pytest.raises(FileNotFoundError):
            load_robot_config(tmp_path / "nope.yaml")

    def test_raises_on_invalid_yaml(self, tmp_path: Path) -> None:
        bad = tmp_path / "bad.yaml"
        bad.write_text(":\n  -invalid\n: garbage:", encoding="utf-8")
        with pytest.raises(Exception):
            load_robot_config(bad)

    def test_raises_on_non_mapping_top_level(self, tmp_path: Path) -> None:
        bad = tmp_path / "list.yaml"
        bad.write_text("- just\n- a\n- list\n", encoding="utf-8")
        with pytest.raises(ValueError, match="Mapping"):
            load_robot_config(bad)

    def test_raises_on_invalid_schema(self, tmp_path: Path) -> None:
        bad = tmp_path / "schema.yaml"
        bad.write_text("name: only_name_no_body\n", encoding="utf-8")
        with pytest.raises(ValidationError):
            load_robot_config(bad)


class TestRoundtrip:
    """Konfig laden → schreiben → wieder laden sollte gleich sein."""

    def test_roundtrip(self, tmp_path: Path, minimal_config: RobotConfig) -> None:
        out = tmp_path / "out.yaml"
        dump_robot_config(minimal_config, out)
        reloaded = load_robot_config(out)
        assert reloaded.model_dump() == minimal_config.model_dump()
