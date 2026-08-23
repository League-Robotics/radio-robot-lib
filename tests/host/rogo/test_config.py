"""tests/host/rogo/test_config.py -- `rogo.config`: load/persist the
minimal robot config subset (`geometry.trackwidth`,
`calibration.rotational_slip`/`geometry.rotational_slip`, identity)
against fixture JSON built fresh per test with `tmp_path` -- never the
real `config/robots/*.json` files (sprint.md's own testing note), and
never anything module-scoped that could leak state between tests.
"""

from __future__ import annotations

import json
from pathlib import Path

from rogo import config


def _write_json(path: Path, data: dict) -> Path:
    path.write_text(json.dumps(data))
    return path


# ---------------------------------------------------------------------------
# load_robot_config() -- direct file load, tolerant of whatever is present.
# ---------------------------------------------------------------------------

def test_load_robot_config_missing_file_returns_none(tmp_path):
    assert config.load_robot_config(tmp_path / "nope.json") is None


def test_load_robot_config_malformed_json_returns_none(tmp_path):
    bad = tmp_path / "bad.json"
    bad.write_text("{not valid json")
    assert config.load_robot_config(bad) is None


def test_load_robot_config_non_object_json_returns_none(tmp_path):
    not_an_object = tmp_path / "list.json"
    not_an_object.write_text("[1, 2, 3]")
    assert config.load_robot_config(not_an_object) is None


def test_load_robot_config_reads_full_fields(tmp_path):
    path = _write_json(tmp_path / "tovez.json", {
        "identity": {"robot_name": "tovez", "uid": "tovez", "common_name": "classroom-bot"},
        "geometry": {"trackwidth": 128, "rotational_slip": 1.02},
    })
    cfg = config.load_robot_config(path)
    assert cfg is not None
    assert cfg.name == "tovez"
    assert cfg.uid == "tovez"
    assert cfg.common_name == "classroom-bot"
    assert cfg.trackwidth_mm == 128.0
    assert cfg.rotational_slip == 1.02
    assert cfg.path == path


def test_load_robot_config_tolerates_a_completely_empty_document(tmp_path):
    path = _write_json(tmp_path / "empty.json", {})
    cfg = config.load_robot_config(path)
    assert cfg is not None
    assert cfg.name is None
    assert cfg.uid is None
    assert cfg.common_name is None
    assert cfg.trackwidth_mm is None
    assert cfg.rotational_slip is None


def test_load_robot_config_tolerates_missing_identity_group(tmp_path):
    path = _write_json(tmp_path / "no_identity.json", {"geometry": {"trackwidth": 150}})
    cfg = config.load_robot_config(path)
    assert cfg.name is None
    assert cfg.trackwidth_mm == 150.0


def test_load_robot_config_falls_back_to_geometry_rotational_slip(tmp_path):
    # Today's actual staged shape (config/robots/*.json): rotational_slip
    # lives under geometry, not a top-level calibration group at all.
    path = _write_json(tmp_path / "geo_slip.json", {"geometry": {"rotational_slip": 0.97}})
    cfg = config.load_robot_config(path)
    assert cfg.rotational_slip == 0.97


def test_load_robot_config_prefers_calibration_group_when_present(tmp_path):
    path = _write_json(tmp_path / "both.json", {
        "calibration": {"rotational_slip": 1.10},
        "geometry": {"rotational_slip": 0.97},
    })
    cfg = config.load_robot_config(path)
    assert cfg.rotational_slip == 1.10


def test_load_robot_config_distance_scale_defaults_to_none(tmp_path):
    # No staged config/robots/*.json file carries distance_scale under
    # either shape today (ticket 005's own field, not an elite import) --
    # absence must mean "uncalibrated", not a crash.
    path = _write_json(tmp_path / "no_scale.json", {"geometry": {"trackwidth": 128}})
    cfg = config.load_robot_config(path)
    assert cfg.distance_scale is None


def test_load_robot_config_falls_back_to_geometry_distance_scale(tmp_path):
    path = _write_json(tmp_path / "geo_scale.json", {"geometry": {"distance_scale": 0.98}})
    cfg = config.load_robot_config(path)
    assert cfg.distance_scale == 0.98


def test_load_robot_config_prefers_calibration_group_distance_scale_when_present(tmp_path):
    path = _write_json(tmp_path / "both_scale.json", {
        "calibration": {"distance_scale": 1.05},
        "geometry": {"distance_scale": 0.98},
    })
    cfg = config.load_robot_config(path)
    assert cfg.distance_scale == 1.05


def test_load_robot_config_ignores_non_numeric_trackwidth_instead_of_crashing(tmp_path):
    path = _write_json(tmp_path / "weird.json", {"geometry": {"trackwidth": "not-a-number"}})
    cfg = config.load_robot_config(path)
    assert cfg.trackwidth_mm is None


# ---------------------------------------------------------------------------
# load_active_robot() -- active_robot.json pointer resolution.
# ---------------------------------------------------------------------------

def test_load_active_robot_returns_none_when_active_robot_json_missing(tmp_path):
    assert config.load_active_robot(tmp_path) is None


def test_load_active_robot_returns_none_when_pointed_file_missing(tmp_path):
    _write_json(tmp_path / "active_robot.json", {"path": "data/robots/ghost.json"})
    assert config.load_active_robot(tmp_path) is None


def test_load_active_robot_resolves_pointer_by_basename_against_config_dir(tmp_path):
    # Mirrors this repo's real active_robot.json: its "path" field is
    # copied verbatim from elite's own layout ("data/robots/<name>.json"),
    # a directory that does not exist here at all -- only the basename
    # is meaningful, resolved against THIS directory instead.
    _write_json(tmp_path / "active_robot.json", {"path": "data/robots/tovez.json"})
    _write_json(tmp_path / "tovez.json", {
        "identity": {"robot_name": "tovez"},
        "geometry": {"trackwidth": 128, "rotational_slip": 1.0},
    })
    cfg = config.load_active_robot(tmp_path)
    assert cfg is not None
    assert cfg.name == "tovez"
    assert cfg.trackwidth_mm == 128.0
    assert cfg.path == tmp_path / "tovez.json"


def test_load_active_robot_handles_an_inline_full_config(tmp_path):
    _write_json(tmp_path / "active_robot.json", {
        "identity": {"robot_name": "inline-bot"},
        "geometry": {"trackwidth": 99},
    })
    cfg = config.load_active_robot(tmp_path)
    assert cfg is not None
    assert cfg.name == "inline-bot"
    assert cfg.trackwidth_mm == 99.0


def test_load_active_robot_returns_none_for_malformed_pointer_json(tmp_path):
    (tmp_path / "active_robot.json").write_text("{not json")
    assert config.load_active_robot(tmp_path) is None


def test_load_active_robot_returns_none_when_path_field_absent(tmp_path):
    _write_json(tmp_path / "active_robot.json", {"note": "no path or identity key here"})
    assert config.load_active_robot(tmp_path) is None


def test_default_config_dir_points_at_config_robots():
    assert config.default_config_dir().parts[-2:] == ("config", "robots")


# ---------------------------------------------------------------------------
# save_robot_config() -- writeback preserves every other field.
# ---------------------------------------------------------------------------

def test_save_robot_config_round_trips_rotational_slip(tmp_path):
    path = _write_json(tmp_path / "robot.json", {
        "identity": {"robot_name": "gopiv"},
        "geometry": {"trackwidth": 128, "rotational_slip": 1.0},
        "motors": {"vel_kp": 0.0016},
    })
    cfg = config.load_robot_config(path)
    updated = config.RobotConfig(
        name=cfg.name, uid=cfg.uid, common_name=cfg.common_name,
        trackwidth_mm=cfg.trackwidth_mm, rotational_slip=1.05,
        distance_scale=cfg.distance_scale, path=cfg.path,
    )
    config.save_robot_config(updated)

    on_disk = json.loads(path.read_text())
    assert on_disk["geometry"]["rotational_slip"] == 1.05
    assert on_disk["geometry"]["trackwidth"] == 128  # untouched
    assert on_disk["identity"] == {"robot_name": "gopiv"}  # untouched
    assert on_disk["motors"] == {"vel_kp": 0.0016}  # untouched

    reloaded = config.load_robot_config(path)
    assert reloaded.rotational_slip == 1.05


def test_save_robot_config_with_none_slip_leaves_existing_value_untouched(tmp_path):
    path = _write_json(tmp_path / "robot.json", {"geometry": {"rotational_slip": 0.9}})
    cfg = config.load_robot_config(path)
    unchanged = config.RobotConfig(
        name=cfg.name, uid=cfg.uid, common_name=cfg.common_name,
        trackwidth_mm=cfg.trackwidth_mm, rotational_slip=None,
        distance_scale=cfg.distance_scale, path=cfg.path,
    )
    config.save_robot_config(unchanged)
    on_disk = json.loads(path.read_text())
    assert on_disk["geometry"]["rotational_slip"] == 0.9


def test_save_robot_config_creates_geometry_group_if_absent(tmp_path):
    path = _write_json(tmp_path / "robot.json", {"identity": {"robot_name": "bare"}})
    cfg = config.RobotConfig(
        name="bare", uid=None, common_name=None, trackwidth_mm=None,
        rotational_slip=1.2, distance_scale=None, path=path,
    )
    config.save_robot_config(cfg)
    on_disk = json.loads(path.read_text())
    assert on_disk["geometry"]["rotational_slip"] == 1.2
    assert on_disk["identity"] == {"robot_name": "bare"}


def test_save_robot_config_round_trips_distance_scale_independently_of_slip(tmp_path):
    # rotational_slip and distance_scale are written independently --
    # a caller updating one (e.g. `rogo calibrate distance`) must not
    # disturb the other (e.g. an existing rotational_slip from a prior
    # `rogo calibrate turns` run).
    path = _write_json(tmp_path / "robot.json", {
        "geometry": {"trackwidth": 128, "rotational_slip": 0.93},
    })
    cfg = config.load_robot_config(path)
    updated = config.RobotConfig(
        name=cfg.name, uid=cfg.uid, common_name=cfg.common_name,
        trackwidth_mm=cfg.trackwidth_mm, rotational_slip=cfg.rotational_slip,
        distance_scale=1.02, path=cfg.path,
    )
    config.save_robot_config(updated)

    on_disk = json.loads(path.read_text())
    assert on_disk["geometry"]["distance_scale"] == 1.02
    assert on_disk["geometry"]["rotational_slip"] == 0.93  # untouched
    assert on_disk["geometry"]["trackwidth"] == 128  # untouched

    reloaded = config.load_robot_config(path)
    assert reloaded.distance_scale == 1.02
    assert reloaded.rotational_slip == 0.93
