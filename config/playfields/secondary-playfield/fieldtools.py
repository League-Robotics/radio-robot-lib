#!/usr/bin/env python
"""Playfield tools for a field with a drawn cross: tag-free deskew, lens check, features.

    fieldtools.py features                     -> rebuild features.json from playfield.png
    fieldtools.py analyze [--frame F]          -> line straightness + plumb-line distortion estimate
    fieldtools.py deskew [--frame F] [--out P] -> top-down 8 px/cm view from the CROSS ALONE (no tags)
    fieldtools.py locate <colour|cross|cap-n>  -> playfield cm of a feature

Run with the aprilcam pipx venv python (numpy + opencv-contrib). Without --frame,
a live frame is fetched from the aprilcam daemon for the camera in playfield.json.
"""
import argparse, json, os, subprocess, sys, tempfile
import numpy as np, cv2

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = json.load(open(os.path.join(HERE, "playfield.json")))
PPC = CFG["image"]["px_per_cm"]; W_CM, H_CM = CFG["size_cm"]
TW, TH = int(round(W_CM * PPC)), int(round(H_CM * PPC))

def cm_from_px(x, y): return (float((x - TW / 2) / PPC), float((TH / 2 - y) / PPC))
def px_from_cm(x, y): return (TW / 2 + x * PPC, TH / 2 - y * PPC)

def live_frame():
    with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as f: path = f.name
    subprocess.run(["aprilcam", "camera", "image", CFG["source"]["camera"], "-o", path], check=True, capture_output=True)
    return cv2.imread(path)

# ---------------------------------------------------------------- cross finding (raw or template)
def find_cross(img):
    """Return (centre, caps) in image pixels: the two arm centrelines' intersection and the
    four cap midpoints (top, bottom, left, right), found from the largest dark component."""
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ink = (g < 80).astype(np.uint8) * 255
    ink = cv2.morphologyEx(ink, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(ink)
    # the cross is the dark component whose bounding box contains the frame centre (the floor
    # around the table is a bigger dark blob in a raw frame, so "largest" is not enough)
    ch, cw = ink.shape[0] / 2, ink.shape[1] / 2
    cands = [j for j in range(1, n) if st[j, cv2.CC_STAT_AREA] > 1500
             and st[j, cv2.CC_STAT_LEFT] <= cw <= st[j, cv2.CC_STAT_LEFT] + st[j, cv2.CC_STAT_WIDTH]
             and st[j, cv2.CC_STAT_TOP] <= ch <= st[j, cv2.CC_STAT_TOP] + st[j, cv2.CC_STAT_HEIGHT]
             and st[j, cv2.CC_STAT_WIDTH] < 0.9 * ink.shape[1]]
    if not cands: raise SystemExit("no cross-like dark component contains the frame centre")
    i = max(cands, key=lambda j: st[j, cv2.CC_STAT_AREA]); cross = (lab == i).astype(np.uint8)
    x0, y0, w, h = st[i, cv2.CC_STAT_LEFT], st[i, cv2.CC_STAT_TOP], st[i, cv2.CC_STAT_WIDTH], st[i, cv2.CC_STAT_HEIGHT]
    # arm centrelines: per-row centroid of the vertical arm inside a band around the column of max mass, and vice versa
    colmass = cross.sum(0); rowmass = cross.sum(1)
    cx0 = int(np.argmax(colmass)); cy0 = int(np.argmax(rowmass)); band = max(int(0.06 * max(w, h)), 15)
    pts_v = [(np.nonzero(cross[y, cx0 - band:cx0 + band])[0].mean() + cx0 - band, y) for y in range(y0, y0 + h)
             if 0 < len(np.nonzero(cross[y, cx0 - band:cx0 + band])[0]) < 2 * band * 0.6]
    pts_h = [(x, np.nonzero(cross[cy0 - band:cy0 + band, x])[0].mean() + cy0 - band) for x in range(x0, x0 + w)
             if 0 < len(np.nonzero(cross[cy0 - band:cy0 + band, x])[0]) < 2 * band * 0.6]
    lv = cv2.fitLine(np.float32(pts_v), cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
    lh = cv2.fitLine(np.float32(pts_h), cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
    def inter(a, b):
        (vx1, vy1, x1, y1), (vx2, vy2, x2, y2) = a, b
        t = np.linalg.solve(np.array([[vx1, -vx2], [vy1, -vy2]]), np.array([x2 - x1, y2 - y1])); return np.array([x1 + vx1 * t[0], y1 + vy1 * t[0]])
    centre = inter(lv, lh)
    # caps: walk each arm outward from the centre along the fitted arm direction; the cap (the
    # T-bar) is where the run of dark pixels perpendicular to the arm jumps to >2.5x the bar width.
    def cap(line, want):
        d = np.array([line[0], line[1]], float); d /= np.linalg.norm(d)
        if want == "top" and d[1] > 0: d = -d
        if want == "bottom" and d[1] < 0: d = -d
        if want == "left" and d[0] > 0: d = -d
        if want == "right" and d[0] < 0: d = -d
        nrm = np.array([-d[1], d[0]]); reach = int(0.25 * max(w, h)); widths = []
        for s_ in range(0, int(1.2 * max(w, h))):
            p = centre + d * s_; xi, yi = int(round(p[0])), int(round(p[1]))
            if not (0 <= xi < cross.shape[1] and 0 <= yi < cross.shape[0]): break
            run = 0
            for k in range(-reach, reach + 1):
                q = p + nrm * k; qx, qy = int(round(q[0])), int(round(q[1]))
                if 0 <= qx < cross.shape[1] and 0 <= qy < cross.shape[0] and cross[qy, qx]: run += 1
            widths.append(run)
            if run == 0 and s_ > 10: break
        widths = np.array(widths)
        if len(widths) < 20: return None
        bar = np.median(widths[len(widths) // 4:len(widths) // 2])  # plain-bar width, away from the crossing
        wide = np.nonzero(widths > 2.5 * bar)[0]; wide = wide[wide > 3 * bar]  # skip the crossing with the other arm
        if len(wide) == 0: return None
        return centre + d * float(np.mean([wide.min(), wide.max()]))
    caps = {k: cap(lv if k in ("top", "bottom") else lh, k) for k in ("top", "bottom", "left", "right")}
    return centre, caps, (lv, lh, pts_v, pts_h)

def undistort_pts(pts, f, k1, shape):
    h, w = shape[:2]; K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]], float); D = np.array([k1, 0, 0, 0, 0], float)
    return cv2.undistortPoints(np.float64(pts).reshape(-1, 1, 2), K, D, P=K).reshape(-1, 2)

# ---------------------------------------------------------------- commands
def cmd_features(_):
    t = cv2.imread(os.path.join(HERE, "playfield.png"))
    centre, caps, (lv, lh, pv, ph) = find_cross(t)
    ang_v = float(np.degrees(np.arctan2(lv[0], lv[1]))); ang_h = float(np.degrees(np.arctan2(lh[1], lh[0])))
    hsv = cv2.cvtColor(t, cv2.COLOR_BGR2HSV); Hh, Ss, Vv = cv2.split(hsv)
    sat = ((Ss > 45) & (Vv > 15) & (Vv < 235)).astype(np.uint8) * 255; sat = cv2.morphologyEx(sat, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(sat); patches = []
    for i in range(1, n):
        if st[i, cv2.CC_STAT_AREA] < 400 or st[i, cv2.CC_STAT_AREA] > 8000: continue
        hue = float(np.median(Hh[lab == i])); vv = float(np.median(Vv[lab == i]))
        name = ("red" if hue < 8 or hue > 165 else "orange" if hue < 22 else "yellow" if hue < 38 else "green" if hue < 85 else "cyan" if hue < 100 else "blue" if hue < 135 else "purple")
        if name == "blue" and vv < 110: name = "navy"
        patches.append({"colour": name, "hue": round(hue), "value": round(vv), "centre_px": [round(float(cen[i][0]), 1), round(float(cen[i][1]), 1)],
                        "centre_cm": [round(v, 2) for v in cm_from_px(*cen[i])], "size_cm": [round(st[i, cv2.CC_STAT_WIDTH] / PPC, 1), round(st[i, cv2.CC_STAT_HEIGHT] / PPC, 1)]})
    feat = {"playfield": CFG["name"], "coordinates": {"px": "template pixel in playfield.png", "cm": "playfield cm, origin field centre, x east, y north"},
            "cross": {"centre_px": [round(float(v), 1) for v in centre], "centre_cm": [round(float(v), 2) for v in cm_from_px(*centre)],
                      "vertical_arm_angle_deg": round(ang_v, 2), "horizontal_arm_angle_deg": round(ang_h, 2), "bar_width_cm": 2.0,
                      "caps": {k: {"px": [round(float(q), 1) for q in v], "cm": [round(float(q), 2) for q in cm_from_px(*v)]} for k, v in caps.items() if v is not None}},
            "patches": sorted(patches, key=lambda p: (-p["centre_cm"][1], p["centre_cm"][0]))}
    json.dump(feat, open(os.path.join(HERE, "features.json"), "w"), indent=1)
    ov = t.copy(); cv2.circle(ov, tuple(int(v) for v in centre), 6, (0, 0, 255), 2)
    for k, v in caps.items():
        if v is not None: cv2.circle(ov, tuple(int(q) for q in v), 6, (255, 0, 255), 2); cv2.putText(ov, k, (int(v[0]) + 8, int(v[1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
    for p in patches: cv2.putText(ov, p["colour"], (int(p["centre_px"][0]) - 20, int(p["centre_px"][1])), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    cv2.imwrite(os.path.join(HERE, "frames", "features-overlay.png"), ov)
    print(json.dumps({k: feat["cross"][k] for k in ("centre_cm", "vertical_arm_angle_deg", "horizontal_arm_angle_deg")}), f"caps {len(feat['cross']['caps'])}, patches {len(patches)}")

def cmd_analyze(a):
    img = cv2.imread(a.frame) if a.frame else live_frame(); h, w = img.shape[:2]; g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    segs = cv2.createLineSegmentDetector().detect(g)[0].reshape(-1, 4)
    segs = segs[np.hypot(segs[:, 2] - segs[:, 0], segs[:, 3] - segs[:, 1]) > 0.13 * w]; edges = cv2.Canny(g, 60, 160)
    groups = []
    for s in segs:
        pts = []
        for t in np.linspace(0, 1, 60):
            x, y = s[0] + t * (s[2] - s[0]), s[1] + t * (s[3] - s[1]); xi, yi = int(round(x)), int(round(y)); win = edges[max(yi - 3, 0):yi + 4, max(xi - 3, 0):xi + 4]
            if win.any():
                ys, xs = np.nonzero(win); k = np.argmin((xs - 3) ** 2 + (ys - 3) ** 2); pts.append((xi - 3 + xs[k], yi - 3 + ys[k]))
        if len(pts) >= 30: groups.append(np.float64(pts))
    def straight(f, k1):
        res = []
        for p in groups:
            u = undistort_pts(p, f, k1, img.shape); c = u.mean(0); n = np.linalg.svd(u - c)[2][-1]; res.extend(((u - c) @ n).tolist())
        return float(np.sqrt(np.mean(np.square(res))))
    f = float(CFG["lens"]["f_px"]); ks = np.arange(-0.6, 0.31, 0.01); rs = [straight(f, float(k)) for k in ks]; i = int(np.argmin(rs))
    print(f"{len(groups)} long straight segments; straightness rms uncorrected {straight(1e6, 0):.3f} px; best k1 at f={f:.0f}: {ks[i]:+.2f} -> {rs[i]:.3f} px")
    print("verdict:", "lens is effectively rectilinear (no correction needed)" if straight(1e6, 0) - rs[i] < 0.1 else f"apply k1={ks[i]:+.2f}")

def cross_homography(img):
    """Homography raw-pixel -> playfield cm from the cross alone (5 points), after the field's lens model."""
    centre, caps, _ = find_cross(img); lens = CFG["lens"]
    ref = CFG["cross_reference_cm"]; src, dst = [centre], [ref["centre"]]
    for k in ("top", "bottom", "left", "right"):
        if caps.get(k) is not None: src.append(caps[k]); dst.append(ref["caps"][k])
    u = undistort_pts(src, lens["f_px"], lens["k1"], img.shape)
    if len(src) >= 4:
        H, _ = cv2.findHomography(np.float32(u), np.float32(dst), 0)
    elif CFG.get("last_homography"):
        # cross partly hidden (a robot on it): start from the last good homography so the
        # coloured patches below can still be matched, then re-solve from whatever is visible
        H = np.array(CFG["last_homography"], float); print(f"only {len(src)} cross points visible -- starting from last_homography")
    else:
        raise SystemExit(f"only {len(src)} cross points found and no last_homography in playfield.json -- cannot deskew")
    # stage 2: the coloured patches (features.json) widen the anchor set to the field's edges.
    # Find saturated blobs in the raw frame, project them through the cross-only H, match each to
    # the nearest surveyed patch of the same colour family within 6 cm, and re-solve with everything.
    fp = os.path.join(HERE, "features.json")
    if os.path.exists(fp):
        feat = json.load(open(fp)); hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV); Hh, Ss, Vv = cv2.split(hsv)
        sat = ((Ss > 60) & (Vv > 35)).astype(np.uint8) * 255; sat = cv2.morphologyEx(sat, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        n, lab, st, cen = cv2.connectedComponentsWithStats(sat); extra_src, extra_dst = [], []
        for i in range(1, n):
            if st[i, cv2.CC_STAT_AREA] < 300: continue
            hue = float(np.median(Hh[lab == i]))
            fam = ("red" if hue < 8 or hue > 165 else "orange" if hue < 22 else "yellow" if hue < 38 else "green" if hue < 85 else "cyan" if hue < 100 else "blue" if hue < 135 else "purple")
            c = undistort_pts([cen[i]], lens["f_px"], lens["k1"], img.shape); w_ = cv2.perspectiveTransform(np.float32(c).reshape(-1, 1, 2), H).reshape(2)
            same = [q for q in feat["patches"] if (q["colour"] == fam or {q["colour"], fam} <= {"blue", "navy"})]
            if not same: continue
            best = min(same, key=lambda q: np.hypot(*(w_ - q["centre_cm"])))
            if np.hypot(*(w_ - best["centre_cm"])) < 6.0: extra_src.append(c[0]); extra_dst.append(best["centre_cm"])
        if len(extra_src) >= 3 and len(src) + len(extra_src) >= 4:
            allu = np.vstack([u, np.float64(extra_src)]) if len(src) else np.float64(extra_src)
            alld = np.vstack([np.float64(dst), np.float64(extra_dst)]) if len(src) else np.float64(extra_dst)
            # RANSAC (1.5 cm): a robot's own coloured parts produce blobs that match the wrong patch
            H2, mask = cv2.findHomography(np.float32(allu), np.float32(alld), cv2.RANSAC, 1.5)
            inl = int(mask.sum()) if mask is not None else 0
            if H2 is not None and inl >= 6:
                print(f"refined with {inl} inliers of {len(allu)} anchors"); _remember(H2); return H2, inl
            if len(src) >= 4: _remember(H); return H, len(src)
            raise SystemExit(f"cross hidden and only {inl} consistent patch matches -- cannot deskew")
    if len(src) < 4: raise SystemExit("cross hidden and too few patches matched -- cannot deskew")
    _remember(H); return H, len(src)

def _remember(H):
    """Persist the last good raw-px -> cm homography in playfield.json (fallback seed)."""
    try:
        path = os.path.join(HERE, "playfield.json"); cfg = json.load(open(path)); cfg["last_homography"] = [[float(v) for v in row] for row in H]
        json.dump(cfg, open(path, "w"), indent=1)
    except OSError: pass

def cmd_deskew(a):
    img = cv2.imread(a.frame) if a.frame else live_frame(); H, npts = cross_homography(img)
    lens = CFG["lens"]; h, w = img.shape[:2]; K = np.array([[lens["f_px"], 0, w / 2], [0, lens["f_px"], h / 2], [0, 0, 1]], float)
    und = cv2.undistort(img, K, np.array([lens["k1"], 0, 0, 0, 0], float), None, K)
    T = np.array([[PPC, 0, TW / 2], [0, -PPC, TH / 2], [0, 0, 1]], float)  # cm -> template px
    out = cv2.warpPerspective(und, T @ H, (TW, TH)); path = a.out or os.path.join(HERE, "frames", "deskew-from-cross.png"); cv2.imwrite(path, out)
    print(f"deskewed from {npts} cross points -> {path}")
    # validation against any visible ArUco markers with known positions
    try:
        from aprilcam.vision.detector import TagDetector
        known = {m["id"]: (m["x"], m["y"]) for m in CFG["aprilcam_definition"]["aruco_tags"]}
        u = {}
        for d in TagDetector().detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)):
            if "ARUCO" in str(d.family).upper() and d.number in known: u[d.number] = (d.center.x, d.center.y)
        if u:
            pts = undistort_pts(list(u.values()), lens["f_px"], lens["k1"], img.shape); p = cv2.perspectiveTransform(np.float32(pts).reshape(-1, 1, 2), H).reshape(-1, 2)
            errs = [np.hypot(*(p[i] - known[k])) for i, k in enumerate(u)]
            print("check vs ArUco survey: " + ", ".join(f"id {k} err {e:.2f} cm" for k, e in zip(u, errs)) + f"; mean {np.mean(errs):.2f} cm")
    except Exception as exc: print("(tag check skipped:", exc, ")")

def cmd_locate(a):
    feat = json.load(open(os.path.join(HERE, "features.json"))); q = a.what.lower()
    if q == "cross": print("cross centre cm:", feat["cross"]["centre_cm"]); return
    if q.startswith("cap"): k = q.split("-")[-1]; print(f"cap {k} cm:", feat["cross"]["caps"][k]["cm"]); return
    hits = [p for p in feat["patches"] if p["colour"] == q]
    for p in hits: print(f"{p['colour']} patch centre cm {p['centre_cm']} size cm {p['size_cm']}")
    if not hits: print("no such feature; colours:", sorted({p['colour'] for p in feat['patches']}))

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("features").set_defaults(fn=cmd_features)
    p = sub.add_parser("analyze"); p.add_argument("--frame"); p.set_defaults(fn=cmd_analyze)
    p = sub.add_parser("deskew"); p.add_argument("--frame"); p.add_argument("--out"); p.set_defaults(fn=cmd_deskew)
    p = sub.add_parser("locate"); p.add_argument("what"); p.set_defaults(fn=cmd_locate)
    a = ap.parse_args(); a.fn(a)
