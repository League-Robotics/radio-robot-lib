# Configuration Collection Manifest

Snapshot of robot, camera, and playfield configuration gathered from multiple
projects on **2026-08-22**. This directory serves as a backup and shared source
of configuration for multiple robotics projects. The `dev/`, `prod/`, `local/`,
`sops.yaml`, and `dotconfig.yaml` entries are dotconfig-managed environment
config (see `AGENTS.md`) and are unrelated to this collection.

## robots/ — from radio-robot-elite

Source: `/Volumes/Proj/proj/RobotProjects/radio-robot-elite`

| File | Original location |
|------|-------------------|
| `gopiv.json`, `togov.json`, `tovez.json`, `tovez_nocal.json`, `vevov.json` | `data/robots/` — per-robot configuration (with calibration) |
| `active_robot.json` | `data/robots/` — pointer to the currently active robot |
| `robot_config.schema.json` | `data/robots/` — JSON Schema for robot configs |
| `devices.json` | `config/` — USB device registry (UID → board name, role, port, serial) |

Not copied: `data/calibration/` rotation-sweep CSVs and notebook (raw
measurement data, not configuration).

## cameras/

### cameras/aprilcam-old/ — from the AprilTags repo

Source: `/Volumes/Proj/proj/RobotProjects/AprilTags/config/aprilcam-old`

- `cameras/` — verbatim copy: `registry.json` plus per-camera directories
  (`info.json`, `config.json`, `calibration.json`, `paths.json` where present).
- `hosts.json` — camera host registry.

Not copied: `aprilcamd.log` (log file, not configuration).

### cameras/aprilcam-live/ — live XDG state (current aprilcam config)

The current aprilcam installation splits state across two XDG roots; both are
preserved verbatim so a restore can put files back exactly where they came from:

- `xdg-config/` ← `~/.config/aprilcam/`
  - `config.json` — top-level application config
  - `cameras/<name>/config.json` — per-camera settings (incl. dated backup
    variants for `arducam-ov9782-usb-camera`)
- `xdg-data/` ← `~/.local/share/aprilcam/`
  - `cameras/registry.json` — camera registry
  - `cameras/<name>/calibration.json`, `info.json`, `paths.json`
  - `hosts.json` — camera host registry

## playfields/

### playfields/aprilcam-old/ — from the AprilTags repo

Source: `/Volumes/Proj/proj/RobotProjects/AprilTags/config/aprilcam-old`

- `main-playfield.json` (from `playfields/`)
- `playfield_layout.svg` — playfield layout drawing
- `tags.json` — AprilTag assignments

### playfields/aprilcam-live/ — live XDG state

- `main-playfield.json`, `secondary-playfield.json` ← `~/.config/aprilcam/playfields/`
- `tags.json` ← `~/.local/share/aprilcam/`

## Restore notes

- **aprilcam live state**: copy `cameras/aprilcam-live/xdg-config/*` back to
  `~/.config/aprilcam/` and `cameras/aprilcam-live/xdg-data/*` plus
  `playfields/aprilcam-live/tags.json` back to `~/.local/share/aprilcam/`;
  live playfield JSONs go to `~/.config/aprilcam/playfields/`.
- **radio-robot-elite robots**: files belong in `data/robots/` except
  `devices.json`, which belongs in `config/`.
- No secrets were copied; env secrets stay in each project's own
  dotconfig/SOPS-managed `config/` tree.

## mat/ — playfield mats (native to this repo)

`mat/mattools.py` — grid/px/playfield conversions, feature lookup, tag-free
relocation (see `mat/bb-obstacle/README.md`). Per-mat directories hold
`mat.png` (deskewed, 8 px/cm), `mat.json`, `features.json`, `frames/`.
Currently `mat/bb-obstacle` (Botball obstacle mat, built from camera 3).
