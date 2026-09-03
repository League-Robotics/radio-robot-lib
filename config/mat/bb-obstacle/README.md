# bb-obstacle — Botball obstacle mat (KIPR)

## Files
- `mat.png` — the mat, deskewed and trimmed to the paper edge (black border kept),
  at **8 px per playfield cm**, mat-aligned. Made from camera 3 on 2026-09-03.
- `mat.json` — `grid` (label convention), `tags` (the AprilTag layout used for the
  initial fit), `image` (pixel↔grid scalars), `pose` (where the mat is on the playfield).
- `features.json` — 12 numbered circles (centre + common radius) and 3 squares
  (colour, 4 corner points, which side is the dashed opening). Each feature is given in
  template pixels, grid `uv`, and mat-cm.
- `cameras/<slug>.json` — the AprilTag-based fit that seeded everything (provenance).
- `frames/` — source frames and check overlays.
- `build_features.py` — regenerates `features.json` from `mat.png`.

## Coordinates
- **Grid** `u` = number axis (−8..37), `v` = letter axis (A=0..V=21). A label names a
  square's centre; boundaries are at half-integers. One grid step ≈ 2.57 cm (u) /
  2.60 cm (v) in playfield units — not exactly an inch.
- **Template pixel** (x,y) in `mat.png`: `u = u_at_px0 − x/(px_per_cm·cm_per_u)`,
  `v = v_at_px0 − y/(px_per_cm·cm_per_v)`. x runs toward playfield east, y toward south.
- **Playfield cm** = `R(theta) · (x/px_per_cm, −y/px_per_cm) + (x_cm, y_cm)` using `pose`.

## Use (`../mattools.py`, run with the aprilcam venv python)
```
mattools.py bb-obstacle locate circle 5        # -> playfield cm of circle 5
mattools.py bb-obstacle locate square green    # -> corners, centre, opening in cm
mattools.py bb-obstacle locate N15             # -> playfield cm of grid square N15
mattools.py bb-obstacle relocate --write       # mat moved? re-find it, update pose
mattools.py bb-obstacle show                   # overlay features on a live frame
```

## Relocation (no tags needed)
`relocate` warps a fresh camera-3 frame to a top-down playfield view at the same
8 px/cm, matches SIFT features against `mat.png`, fits a rigid transform with RANSAC
(scale is fixed by the camera calibration), then optionally refines with ECC when that
is clearly better. Validated 2026-09-03: synthetic moves up to 180° and 40 cm recover
the pose within 0.1 cm / 0.03°, with a robot on the field (≈50–120 inliers).
Fails loudly below 40 inliers — re-check lighting or occlusion.
