#!/usr/bin/env python
"""Mat tools: grid/px/playfield conversions, feature lookup, and tag-free relocation.

    mattools.py <mat> locate <C9 | circle 5 | square green>     -> playfield cm
    mattools.py <mat> px2grid X Y                                 -> template pixel -> grid label
    mattools.py <mat> relocate [--camera SLUG] [--frame FILE] [--write]
    mattools.py <mat> show [--frame FILE]                          -> overlay of features on a fresh frame

<mat> is a directory name under config/mat/. Needs numpy + opencv-contrib (the aprilcam pipx venv works).
"""
import argparse, json, math, os, subprocess, sys, tempfile, time
import numpy as np, cv2

HERE = os.path.dirname(os.path.abspath(__file__))
APRILCAM_STATE = os.path.expanduser("~/.local/share/aprilcam")
APRILCAM_CONF = os.path.expanduser("~/.config/aprilcam")

class Mat:
    def __init__(self, name):
        self.dir = os.path.join(HERE, name); self.name = name
        self.cfg = json.load(open(os.path.join(self.dir, "mat.json")))
        fp = os.path.join(self.dir, "features.json")
        self.features = json.load(open(fp)) if os.path.exists(fp) else None
        self.img = self.cfg["image"]; self.ppc = self.img["px_per_cm"]; self.grid = self.img["grid"]
        self.letters = self.cfg["grid"]["letter_axis"]["letters"]
        self.set_pose(self.cfg["pose"])
    def set_pose(self, pose):
        self.pose = pose; t = math.radians(pose["theta_deg"])
        self.R = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]]); self.t = np.array([pose["x_cm"], pose["y_cm"]])
    # --- template pixel <-> grid (u,v) ---
    def px_to_uv(self, x, y):
        g = self.grid; return (g["u_at_px0"] - x / (self.ppc * g["cm_per_u"]), g["v_at_px0"] - y / (self.ppc * g["cm_per_v"]))
    def uv_to_px(self, u, v):
        g = self.grid; return ((g["u_at_px0"] - u) * self.ppc * g["cm_per_u"], (g["v_at_px0"] - v) * self.ppc * g["cm_per_v"])
    def label(self, u, v):
        li = int(round(v)); return f"{self.letters[li] if 0 <= li < len(self.letters) else '?'}{int(round(u))}"
    def parse_label(self, s):
        s = s.strip().upper(); return (float(s[1:]), float(self.letters.index(s[0])))
    # --- template pixel <-> playfield cm (via pose) ---
    def px_to_playfield(self, x, y):
        return self.R @ np.array([x / self.ppc, -y / self.ppc]) + self.t
    def playfield_to_px(self, X, Y):
        m = self.R.T @ (np.array([X, Y]) - self.t); return (m[0] * self.ppc, -m[1] * self.ppc)
    def uv_to_playfield(self, u, v): return self.px_to_playfield(*self.uv_to_px(u, v))
    def playfield_to_uv(self, X, Y): return self.px_to_uv(*self.playfield_to_px(X, Y))
    # --- features ---
    def circle(self, n):
        return next(c for c in self.features["circles"] if c["n"] == int(n))
    def square(self, color):
        return next(s for s in self.features["squares"] if s["color"] == color)
    def locate(self, what):
        """'C9' | 'circle 5' | 'square green' -> dict with playfield cm."""
        w = what.split()
        if w[0] == "circle":
            c = self.circle(w[1]); P = self.px_to_playfield(*c["px"])
            return {"what": what, "label": c["label"], "playfield_cm": P.round(2).tolist(), "radius_cm": self.features["circle_radius_cm"]}
        if w[0] == "square":
            s = self.square(w[1]); cs = [self.px_to_playfield(*p).round(2).tolist() for p in s["corners_px"]]
            ctr = np.mean(cs, axis=0).round(2).tolist(); o = s["opening"]
            return {"what": what, "center_cm": ctr, "corners_cm": cs, "opening_cm": [cs[o[0]], cs[o[1]]], "side_cm": s["side_cm"]}
        u, v = self.parse_label(w[0]); return {"what": what, "uv": [u, v], "playfield_cm": self.uv_to_playfield(u, v).round(2).tolist()}

# ---------------- camera / relocation ----------------
def camera_homography(slug):
    return np.array(json.load(open(f"{APRILCAM_STATE}/cameras/{slug}/calibration.json"))["homography"])
def playfield_size(name="main-playfield"):
    p = json.load(open(f"{APRILCAM_CONF}/playfields/{name}.json"))["playfield"]; return p["width_cm"], p["height_cm"]
def capture(slug, out):
    subprocess.run(["aprilcam", "camera", "image", slug, "--encoding", "png", "-o", out], check=True, capture_output=True)
def topdown(frame, slug, ppc, playfield="main-playfield"):
    """Warp a camera frame to a north-up, east-right playfield image at ppc px/cm."""
    Wc, Hc = playfield_size(playfield); Sc = np.array([[ppc, 0, Wc / 2 * ppc], [0, -ppc, Hc / 2 * ppc], [0, 0, 1]])
    return cv2.warpPerspective(frame, Sc @ camera_homography(slug), (int(Wc * ppc), int(Hc * ppc)), flags=cv2.INTER_CUBIC), Sc

def relocate(mat, frame, slug, verbose=True, use_ecc=True):
    """Find the mat template in a fresh frame. Returns (pose dict, report dict, top-down image)."""
    top, Sc = topdown(frame, slug, mat.ppc)
    tmpl = cv2.imread(os.path.join(mat.dir, mat.img["file"]))
    g1 = cv2.cvtColor(tmpl, cv2.COLOR_BGR2GRAY); g2 = cv2.cvtColor(top, cv2.COLOR_BGR2GRAY)
    sift = cv2.SIFT_create(nfeatures=6000); k1, d1 = sift.detectAndCompute(g1, None); k2, d2 = sift.detectAndCompute(g2, None)
    good = [a for a, b in cv2.BFMatcher(cv2.NORM_L2).knnMatch(d1, d2, k=2) if a.distance < 0.75 * b.distance]
    p1 = np.float32([k1[a.queryIdx].pt for a in good]); p2 = np.float32([k2[a.trainIdx].pt for a in good])
    A, inl = cv2.estimateAffinePartial2D(p1, p2, method=cv2.RANSAC, ransacReprojThreshold=3.0, maxIters=5000, confidence=0.999)
    if A is None or inl.sum() < 40: raise RuntimeError(f"relocation failed: {0 if A is None else int(inl.sum())} inliers")
    sc = math.hypot(A[0, 0], A[1, 0]); E = A.copy(); E[:2, :2] /= sc            # force rigid (scale is fixed by calibration)
    inl = inl.ravel().astype(bool); rms = float(np.sqrt(np.mean(np.sum((cv2.transform(p1[inl].reshape(-1,1,2), E).reshape(-1,2) - p2[inl])**2, axis=1))))
    # guarded ECC refinement
    def cc_of(Ew):
        w = cv2.warpAffine(g2, Ew, (g1.shape[1], g1.shape[0]), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
        return float(cv2.computeECC(g1, w))
    cc0 = cc_of(E); E_final = E; cc1 = None; used_ecc = False
    try:
        if not use_ecc: raise cv2.error("ecc disabled")
        cc1, Er = cv2.findTransformECC(g1, g2, E.astype(np.float32), cv2.MOTION_EUCLIDEAN, (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1e-5), None, 5)
        dtrans = np.linalg.norm(Er[:, 2] - E[:, 2]); dang = abs(math.degrees(math.atan2(Er[1, 0], Er[0, 0]) - math.atan2(E[1, 0], E[0, 0])))
        # accept only a small, clearly-better refinement; SIFT+RANSAC is already sub-pixel
        if cc1 > cc0 and cc1 > 0.9 and dtrans < 6 and dang < 0.3: E_final = Er.astype(np.float64); used_ecc = True
    except cv2.error: pass
    # pose from E_final (template px -> topdown px)
    a = math.atan2(E_final[1, 0], E_final[0, 0]); theta = -a
    o = np.linalg.inv(Sc) @ np.array([E_final[0, 2], E_final[1, 2], 1.0])
    pose = {"x_cm": round(float(o[0]), 3), "y_cm": round(float(o[1]), 3), "theta_deg": round(math.degrees(theta), 4),
            "formula": mat.pose["formula"], "method": "sift+ecc relocation" if used_ecc else "sift relocation", "camera": slug,
            "when": time.strftime("%Y-%m-%dT%H:%M:%S")}
    rep = {"matches": len(good), "inliers": int(inl.sum()), "sift_scale": round(sc, 4), "inlier_rms_px": round(rms, 2), "ecc_before": round(cc0, 3), "ecc_after": None if cc1 is None else round(float(cc1), 3), "used_ecc": used_ecc}
    if verbose: print(json.dumps(rep))
    return pose, rep, top, E_final

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("mat"); sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("locate"); s.add_argument("what", nargs="+")
    s = sub.add_parser("px2grid"); s.add_argument("x", type=float); s.add_argument("y", type=float)
    for c in ("relocate", "show"):
        s = sub.add_parser(c); s.add_argument("--camera", default="arducam-ov9782-usb-camera"); s.add_argument("--frame"); s.add_argument("--write", action="store_true"); s.add_argument("--out")
    a = ap.parse_args(); mat = Mat(a.mat)
    if a.cmd == "locate": print(json.dumps(mat.locate(" ".join(a.what)))); return
    if a.cmd == "px2grid": u, v = mat.px_to_uv(a.x, a.y); print(mat.label(u, v), round(u, 2), round(v, 2)); return
    if a.frame: frame = cv2.imread(a.frame)
    else:
        tmp = os.path.join(tempfile.gettempdir(), f"mat-{a.camera}.png"); capture(a.camera, tmp); frame = cv2.imread(tmp)
    if a.cmd == "relocate":
        pose, rep, top, E = relocate(mat, frame, a.camera)
        old = mat.pose; d = math.hypot(pose["x_cm"] - old["x_cm"], pose["y_cm"] - old["y_cm"])
        print(json.dumps({"old": {k: old[k] for k in ("x_cm", "y_cm", "theta_deg")}, "new": {k: pose[k] for k in ("x_cm", "y_cm", "theta_deg")}, "moved_cm": round(d, 2), "rotated_deg": round(pose["theta_deg"] - old["theta_deg"], 3)}))
        if a.write:
            mat.cfg["pose"] = pose; json.dump(mat.cfg, open(os.path.join(mat.dir, "mat.json"), "w"), indent=1); print("pose written to mat.json")
        mat.set_pose(pose)
    # overlay of features on the top-down view
    top, Sc = topdown(frame, a.camera, mat.ppc); ov = top.copy()
    def P(X, Y): p = Sc @ np.array([X, Y, 1.0]); return (int(round(p[0])), int(round(p[1])))
    if mat.features:
        for c in mat.features["circles"]:
            X, Y = mat.px_to_playfield(*c["px"]); cv2.circle(ov, P(X, Y), int(mat.features["circle_radius_cm"] * mat.ppc), (0, 0, 255), 2); cv2.putText(ov, str(c["n"]), P(X, Y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        for s in mat.features["squares"]:
            cs = [P(*mat.px_to_playfield(*p)) for p in s["corners_px"]]; cv2.polylines(ov, [np.array(cs)], True, (255, 0, 255), 2)
            o = s["opening"]; cv2.line(ov, cs[o[0]], cs[o[1]], (0, 255, 255), 3)
    w, h = mat.img["width"], mat.img["height"]
    cv2.polylines(ov, [np.array([P(*mat.px_to_playfield(x, y)) for x, y in [(0, 0), (w, 0), (w, h), (0, h)]])], True, (0, 255, 0), 2)
    out = a.out or os.path.join(tempfile.gettempdir(), f"mat-{a.mat}-overlay.png"); cv2.imwrite(out, ov); print("overlay:", out)

if __name__ == "__main__": main()
