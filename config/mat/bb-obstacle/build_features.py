"""Extract circles + squares from mat.png (run once; provenance for features.json)."""
import cv2, numpy as np, json, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__)))); from mattools import Mat
mat = Mat(os.path.basename(os.path.dirname(os.path.abspath(__file__)))); ppc = mat.ppc
crop = cv2.imread(os.path.join(mat.dir, "mat.png")); hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV); Hh, Ss, Vv = cv2.split(hsv)
g = cv2.medianBlur(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), 3)
circ = cv2.HoughCircles(g, cv2.HOUGH_GRADIENT, dp=1, minDist=40, param1=120, param2=22, minRadius=18, maxRadius=34)[0]
# number assignment: nearest to hand-verified approximate positions (grid uv)
approx = {1: (8.7, 1.7), 2: (6.2, 10.5), 3: (5.1, 16.7), 4: (11.8, 10.5), 5: (17.2, 5.0), 6: (17.3, 10.5), 7: (17.3, 15.9), 8: (20.5, 20.5), 9: (28.3, 10.6), 10: (32.9, 3.0), 11: (36.8, 10.5), 12: (32.9, 18.0)}
circles = []
for x, y, r in circ.astype(float):
    u, v = map(float, mat.px_to_uv(x, y)); n = min(approx, key=lambda k: np.hypot(u - approx[k][0], v - approx[k][1]))
    if np.hypot(u - approx[n][0], v - approx[n][1]) >= 0.6: print("skip false circle at", round(u,2), round(v,2)); continue
    circles.append({"n": int(n), "label": mat.label(u, v), "px": [float(x), float(y)], "uv": [round(u, 3), round(v, 3)], "mat_cm": [round(x / ppc, 2), round(-y / ppc, 2)], "r_px": float(r)})
assert len(circles) == 12 and len({c["n"] for c in circles}) == 12
r = float(np.mean([c["r_px"] for c in circles]))
masks = {"blue": ((Hh > 100) & (Hh < 135) & (Ss > 70) & (Vv < 150)), "green": ((Hh > 35) & (Hh < 70) & (Ss > 50) & (Vv < 195)), "orange": ((Hh > 5) & (Hh < 36) & (Ss > 28) & (Vv < 196))}
def inter(l1, l2):
    (vx1, vy1, x1, y1), (vx2, vy2, x2, y2) = l1, l2
    t = np.linalg.solve(np.array([[vx1, -vx2], [vy1, -vy2]], float), np.array([x2 - x1, y2 - y1], float)); return np.array([x1 + vx1 * t[0], y1 + vy1 * t[0]])
squares = []
for name, m in masks.items():
    m = m.astype(np.uint8) * 255; m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m); m = np.isin(lab, [i for i in range(1, n) if st[i, cv2.CC_STAT_AREA] > 150]).astype(np.uint8) * 255
    ys, xs = np.nonzero(m); pts = np.c_[xs, ys].astype(np.float32); box = cv2.boxPoints(cv2.minAreaRect(pts))
    sides = []
    for i in range(4):
        a, b = box[i], box[(i + 1) % 4]; d = (b - a) / np.linalg.norm(b - a); nrm = np.array([-d[1], d[0]]); sides.append(pts[np.abs((pts - a) @ nrm) < 10])
    op = int(np.argmin([len(s) for s in sides])); order = [(op + 1) % 4, (op + 2) % 4, (op + 3) % 4]
    lines = [cv2.fitLine(sides[i], cv2.DIST_HUBER, 0, 0.01, 0.01).flatten() for i in order]
    P1 = inter(lines[0], lines[1]); P2 = inter(lines[1], lines[2]); L = np.linalg.norm(P2 - P1)
    def far(line, side, P): d = np.array(line[:2]); return P + L * d * np.sign(np.dot(d, side.mean(axis=0) - P))
    corners = np.array([far(lines[0], sides[order[0]], P1), P1, P2, far(lines[2], sides[order[2]], P2)])
    squares.append({"color": name, "side_cm": round(L / ppc, 2), "corners_px": corners.round(2).tolist(), "corners_uv": [[round(q, 3) for q in mat.px_to_uv(*c)] for c in corners],
                    "corners_mat_cm": [[round(c[0] / ppc, 2), round(-c[1] / ppc, 2)] for c in corners], "opening": [3, 0], "opening_note": "corners[3]->corners[0] is the dashed (open) side; corners are the centre lines of the drawn border"})
    print(name, "side cm", round(L / ppc, 2), "side counts", [len(s) for s in sides], "opening", op)
feat = {"mat": mat.name, "coordinates": {"px": "template pixel in mat.png", "uv": "grid (number, letter index A=0)", "mat_cm": "cm from template pixel (0,0), x right, y up"},
        "circle_radius_cm": round(r / ppc, 2), "circles": sorted(circles, key=lambda c: c["n"]), "squares": squares}
json.dump(feat, open(os.path.join(mat.dir, "features.json"), "w"), indent=1)
ov = crop.copy()
for c in circles: cv2.circle(ov, tuple(int(q) for q in c["px"]), int(r), (0, 0, 255), 1); cv2.putText(ov, f"{c['n']} {c['label']}", (int(c["px"][0]) - 14, int(c["px"][1]) + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1)
for s in squares:
    cs = np.array(s["corners_px"]).astype(int); cv2.polylines(ov, [cs], True, (0, 0, 0), 1); cv2.line(ov, tuple(cs[3]), tuple(cs[0]), (0, 0, 255), 2)
    for i, p in enumerate(cs): cv2.putText(ov, str(i), tuple(p + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)
cv2.imwrite(os.path.join(mat.dir, "frames", "features-overlay.png"), ov)
for c in feat["circles"]: print(c["n"], c["label"], c["uv"])
