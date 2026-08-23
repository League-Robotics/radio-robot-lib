"""config.py -- load/persist the minimal robot config subset the ported
Rogo commands actually consume: `geometry.trackwidth`,
`calibration.rotational_slip`, and identity (sprint.md's Design
Rationale Decision 2). Reads `config/robots/active_robot.json` and the
file it points to, from `config/robots/*.json`.

**Tolerant by construction, per sprint.md's Migration Concerns**:
`config/robots/*.json` were copied verbatim from `radio-robot-elite`
and have never been validated against a schema in this repo (see
`config/MANIFEST.md`). Two consequences this module builds around
rather than treats as bugs:

1. Fields absent from a given robot's JSON (or the whole file/pointer
   missing or unreadable) must produce `None`/sane defaults, never a
   crash -- there is no schema here to enforce a shape, unlike elite's
   own generated 10-group pydantic model this repo deliberately does
   not port (Decision 2's own "no current caller" reasoning).
2. `rotational_slip` is documented as living under a top-level
   `calibration` group, but every file actually staged in
   `config/robots/` carries it under `geometry` instead (there is no
   `calibration` group in any of them at all). This module checks
   `calibration.rotational_slip` first (honoring the documented shape,
   should a future file use it) and falls back to
   `geometry.rotational_slip` (today's actual shape) -- exactly the
   "tolerate whichever subset of fields is actually present" instruction
   this ticket was given, not a schema this module gets to assume.
3. `active_robot.json`'s own `path` field is copied verbatim from
   elite too, and points at elite's own layout (`data/robots/<name>.json`),
   not this repo's (`config/robots/<name>.json`). Only the file's
   basename is meaningful here; it is resolved against this repo's own
   `config/robots/` directory, not treated as a path to open directly.
"""

from __future__ import annotations

import dataclasses
import json
from pathlib import Path


@dataclasses.dataclass(frozen=True)
class RobotConfig:
    """The minimal subset of a robot's JSON config the ported commands
    need. Any field may be `None` -- see module docstring. `path` is
    the file this was loaded from, kept so `save_robot_config()` can
    write back to the same place without the caller re-resolving it."""

    name: str | None
    uid: str | None
    common_name: str | None
    trackwidth_mm: float | None
    rotational_slip: float | None
    path: Path


def _repo_root() -> Path:
    # src/host/rogo/config.py -> rogo -> host -> src -> repo root.
    return Path(__file__).resolve().parents[3]


def default_config_dir() -> Path:
    """`config/robots/` at the repo root -- the default search
    directory for `load_active_robot()`."""
    return _repo_root() / "config" / "robots"


def _read_json(path: Path) -> object | None:
    """Return the parsed JSON at `path`, or `None` for any reason it
    could not be read/parsed -- missing file, permission error,
    malformed JSON. Callers treat `None` as "nothing usable here", not
    an exception to propagate (module docstring's "never a crash")."""
    try:
        return json.loads(path.read_text())
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return None


def _as_float_or_none(value: object) -> float | None:
    if value is None:
        return None
    try:
        return float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None


def _as_str_or_none(value: object) -> str | None:
    return value if isinstance(value, str) else None


def _dict_or_empty(data: dict, key: str) -> dict:
    value = data.get(key)
    return value if isinstance(value, dict) else {}


def _parse_robot_config(data: dict, path: Path) -> RobotConfig:
    identity = _dict_or_empty(data, "identity")
    geometry = _dict_or_empty(data, "geometry")
    calibration = _dict_or_empty(data, "calibration")

    rotational_slip = calibration.get("rotational_slip")
    if rotational_slip is None:
        rotational_slip = geometry.get("rotational_slip")  # today's actual shape

    return RobotConfig(
        name=_as_str_or_none(identity.get("robot_name")),
        uid=_as_str_or_none(identity.get("uid")),
        common_name=_as_str_or_none(identity.get("common_name")),
        trackwidth_mm=_as_float_or_none(geometry.get("trackwidth")),
        rotational_slip=_as_float_or_none(rotational_slip),
        path=path,
    )


def load_robot_config(path: Path | str) -> RobotConfig | None:
    """Load one robot's config file directly. Returns `None` (never
    raises) if `path` is missing, unreadable, malformed JSON, or not a
    JSON object at all."""
    path = Path(path)
    data = _read_json(path)
    if not isinstance(data, dict):
        return None
    return _parse_robot_config(data, path)


def load_active_robot(config_dir: Path | str | None = None) -> RobotConfig | None:
    """Read `active_robot.json` in `config_dir` (default
    `default_config_dir()`) and load the robot config it points to.

    Tolerates every shape `active_robot.json` might legitimately take
    (per elite's own documented resolution order, still honored here
    for robustness even though today's staged file uses only the first):
    a `{"path": "..."}` pointer (this repo's actual file), or a full
    config inline (has its own `identity` key). Returns `None` -- never
    raises -- if the directory, the pointer file, or the pointed-to file
    is missing, unreadable, or malformed.
    """
    directory = Path(config_dir) if config_dir is not None else default_config_dir()
    pointer = _read_json(directory / "active_robot.json")
    if not isinstance(pointer, dict):
        return None

    if "identity" in pointer:
        # active_robot.json is itself a full config, not just a pointer.
        return _parse_robot_config(pointer, directory / "active_robot.json")

    path_field = pointer.get("path")
    if not isinstance(path_field, str) or not path_field:
        return None
    # The stored path is copied verbatim from elite's own layout
    # (data/robots/<name>.json); only the basename is meaningful here --
    # resolve it against THIS repo's config/robots/ directory instead.
    robot_path = directory / Path(path_field).name
    return load_robot_config(robot_path)


def save_robot_config(config: RobotConfig) -> None:
    """Persist `config`'s `rotational_slip` back into the JSON file at
    `config.path`, round-tripping every other field in that file
    verbatim (no field this module doesn't understand is ever dropped).
    Used by the calibration flow (ticket 005) to write back an updated
    slip value; a `None` `rotational_slip` leaves the file's existing
    value untouched rather than clearing it.
    """
    data = _read_json(config.path)
    if not isinstance(data, dict):
        data = {}
    geometry = data.get("geometry")
    if not isinstance(geometry, dict):
        geometry = {}
        data["geometry"] = geometry
    if config.rotational_slip is not None:
        geometry["rotational_slip"] = config.rotational_slip
    config.path.write_text(json.dumps(data, indent=2) + "\n")
