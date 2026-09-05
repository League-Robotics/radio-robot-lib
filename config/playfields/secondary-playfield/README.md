# secondary-playfield — white welding table with the black cross (camera 2, `hd-usb-camera`)

## Files
- `playfield.png` — the whole 110 x 70 cm field, top-down at **8 px per cm** (880 x 560),
  x right = east, y down = south, field centre at pixel (440, 280). Made 2026-09-04 from the
  raw frame below: undistorted with the lens model in `playfield.json`, then a homography from
  the **four corner ArUco markers** at (±55, ±35) cm.
- `playfield.json` — size, image scalars, the lens model, the tag-free deskew anchors
  (`cross_reference_cm`), and the aprilcam marker survey this field was calibrated with.
- `features.json` — the cross (centre, arm angles, 2.0 cm bar width, the four T-cap midpoints)
  and every coloured patch (colour, centre, size), in template pixels and playfield cm.
- `frames/hd-usb-camera-2026-09-04.png` — the raw 1920 x 1080 source frame (field clear).
- `frames/features-overlay.png` — the features drawn on the template.
- `frames/deskew-from-cross.png` — the raw frame deskewed **without any tags** (see below).
- `frames/hd-usb-camera-2026-09-04-robot.png`, `frames/deskew-from-cross-robot.png` — the same with a
  robot parked on the cross (fallback path).
- `fieldtools.py` — the tooling; run it with the aprilcam pipx venv python.

## Coordinates
Playfield cm = the aprilcam world frame for `secondary-playfield`: origin at the field centre,
x east (camera image right), y north (camera image top).
`x_cm = (px - 440) / 8`, `y_cm = (280 - py) / 8`.

## Tag-free deskew (`fieldtools.py deskew`)
The cross is enough to recover the camera-to-field homography without markers:
1. find the largest dark blob containing the frame centre (the cross), fit both arm centrelines,
   intersect them (centre), walk each arm outward until the perpendicular dark run widens to the
   T-cap (four cap midpoints) — five well-separated points;
2. solve a homography against `cross_reference_cm`;
3. refine with the eight coloured patches: segment saturated blobs, project them through the
   cross homography, match to `features.json` by colour and proximity, re-solve with 13 points.

Validated 2026-09-04 against the ArUco survey the markers were never given to the fit:

| marker | corner | error |
|---|---|---|
| 0, 1, 2, 3 | the four corners | 0.02–0.05 cm |
| 5, 6, 7 | edge middles | 0.9–1.1 cm |
| 4 | north middle | 2.7 cm |

The corner numbers are partly circular (the template was built from those corners on the same
frame); the edge-middle numbers are real and are the markers' own placement error, not the
method's — see "the lens" below.

**When the cross is partly hidden** (a robot parked on it), `deskew` falls back to the last good
homography stored in `playfield.json` (`last_homography`) to seed the patch matching, and solves
from the patches with RANSAC (1.5 cm). Validated on `frames/hd-usb-camera-2026-09-04-robot.png`
(robot on the top arm, only the centre visible): 8 inliers, mean marker error **2.5 cm**. Good
enough to find the field, not for fine positioning — keep the cross clear when you need the 0.1 cm
result, or add the corner tags back in for that frame.

## The lens (why there is almost no barrel distortion to correct)
`fieldtools.py analyze` fits a single radial term by the plumb-line method: it takes every long
straight edge in the raw frame and finds the k1 that makes them straightest. Result: the lines are
already straight to **0.64 px rms** across >1000 px; the best correction (k1 = −0.12 at f = 1853)
improves that to 0.61 px. This lens is effectively rectilinear.

That contradicts the lens model aprilcam fitted from the nine tag points (k1 = −0.30), which
actually *bends* the drawn lines (0.74 px). What that fit absorbed was the edge-middle markers
sitting 1–3 cm outside the corner-to-corner lines, i.e. placement error. The drawn lines are the
better ground truth on this field.

## Use
```
fieldtools.py features                                   # rebuild features.json + overlay
fieldtools.py analyze  --frame frames/<raw>.png          # line straightness / lens check
fieldtools.py deskew   --frame frames/<raw>.png          # tag-free top-down view (+ tag check)
fieldtools.py locate cross | cap-top | red | orange ...  # feature -> playfield cm
```
Without `--frame`, a live frame is fetched from the aprilcam daemon (`aprilcam camera image`).
