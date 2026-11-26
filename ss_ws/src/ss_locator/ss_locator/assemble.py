"""金具単位の検出（根元・リング中心・リング面の両端の 4 点）→ 盤の配置（床上の中心・正面の向き・幅・奥行）。

先行試作 lifting-lug-pose `9265d2d` の `src/assemble_lugs.py` から写した。変えたところ:
- import を `from common import ...` から `from .geometry import undistort_rays` に変え、
  定数 `LUG_INSET`・`RING_DZ` をこのファイルに置いた（`PANEL_H` は引数にしたので置かない）
- 天板の高さを定数 `PANEL_H`（試作は 2.35）から引数 `top_height_m` にした（`lift` と `Z_RING` を使う箇所）
- `orient_rows`（並んだ盤で幅の向きを決め直す）は写さない。幅は spec の「機体側に最も向いている辺」で決める
- `assemble` は `fuse` → `group` → `fit` まで（`orient_rows` を呼ばない）
- `group` は 3〜4 個を金具の位置だけで組む（輪の向きは使わない）

前提: カメラの内部・外部パラメータは既知、天板上面の高さは `top_height_m`、
リング面は盤の中心を通る向きに付いている（実物の取付規則、2026-11-03）。背中合わせの盤は正面側の 2 個だけ。

1. lift: 1 枚の検出を床に戻す。根元は天板上面の水平面との交点、リング面の向きは両端の 2 点を
   リング中心の高さの水平面に投影した向き（180° 周期）。画素 0.5 px のずれに対する感度から、位置と向きのばらつきを見積もる
2. fuse: 複数視点の同じ金具をまとめる（根元が近く、違う画像のもの）。向きはばらつきの逆数で重み付け
3. group: 金具を盤ごとに分ける。同じ盤の金具は、リング面の線がどれも盤の中心を通り、中心から等距離にある。
   2 個の組から中心の候補を作り、それを支持する金具の多い候補から順に採る
4. fit: 金具 4 個（3 個）は長方形の四隅として中心・向き・幅・奥行を出す。正面と背面はリングが対称なので決まらない。
   金具 2 個は正面の 2 個（背中合わせの盤）とみなし、中心はリング面の線から、正面は中心と反対側とする
"""
import math

import numpy as np

from .geometry import undistort_rays

RING_DZ = 0.0365                 # 天板上面からリング中心までの高さ（アイボルト M20 の実測、大きさ ±5% で 0.035〜0.038）
LUG_INSET = 0.0675               # 天板の角から金具の軸までの距離（0.05〜0.085 の中央）
MOUNT_SIGMA = 3.0                # 取付のずれなど、画素の誤差によらない向きのばらつき [deg]


def _plane_xy(px, cam, z):
    K, dist, T = cam
    C, d = undistort_rays(px, K, dist, T)
    t = (z - C[2]) / np.where(np.abs(d[:, 2]) < 1e-9, -1e-9, d[:, 2])
    P = C + t[:, None] * d
    P[t <= 0] = np.nan
    return P[:, :2]


def _axis_deg(lr, cam, z_ring):
    X = _plane_xy(lr, cam, z_ring)
    if not np.all(np.isfinite(X)):
        return None
    d = X[1] - X[0]
    return math.degrees(math.atan2(d[1], d[0])) % 180.0


def adiff(a, b):
    """180° 周期の角度差（-90〜90）。"""
    return (a - b + 90.0) % 180.0 - 90.0


def lift(kp, conf, cam, view, top_height_m):
    """1 枚の検出 kp (N,4,≥2) → 金具の観測 [{view, p (2,), axis, s_axis, s_pos, conf, dist}]。"""
    K, dist, T = cam
    C = -T[:3, :3].T @ T[:3, 3]
    z_ring = top_height_m + RING_DZ
    out = []
    for k, c in zip(np.asarray(kp, float), conf):
        b = _plane_xy(k[:1, :2], cam, top_height_m)[0]
        az = _axis_deg(k[2:4, :2], cam, z_ring)
        if not np.all(np.isfinite(b)) or az is None:
            continue
        # 画素を 0.5 px ずらしたときの変化 → ばらつきの見積もり
        dp = [np.linalg.norm(_plane_xy(k[:1, :2] + e, cam, top_height_m)[0] - b) for e in ([[0.5, 0]], [[0, 0.5]])]
        g = []
        for i in range(2):
            for j in range(2):
                q = k[2:4, :2].copy()
                q[i, j] += 0.5
                a2 = _axis_deg(q, cam, z_ring)
                g.append(0.0 if a2 is None else abs(adiff(a2, az)))
        s_axis = math.hypot(math.sqrt(sum(x * x for x in g)), MOUNT_SIGMA)
        out.append({"view": view, "p": b, "axis": az, "s_axis": min(s_axis, 60.0),
                    "s_pos": max(max(dp), 0.003), "conf": float(c),
                    "dist": float(np.linalg.norm(np.r_[b, top_height_m] - C))})
    return out


def _axial_mean(angles, w):
    a = np.radians(np.asarray(angles) * 2)
    return math.degrees(math.atan2(np.sum(w * np.sin(a)), np.sum(w * np.cos(a)))) / 2 % 180.0


def fuse(obs, gate=0.06, dup=0.03):
    """複数視点の観測 → 金具 [{p, axis, s_axis, s_pos, conf, n_views, obs}]。
    確信度の高い順に、根元が gate [m] 以内の違う画像の観測をまとめる。同じ画像で dup 以内のものは重複として捨てる。"""
    lugs = []
    for o in sorted(obs, key=lambda o: -o["conf"]):
        best = None
        for L in lugs:
            d = np.linalg.norm(L["p"] - o["p"])
            if d < min(gate, max(dup, 3 * math.hypot(L["s_pos"], o["s_pos"]))) and (best is None or d < best[0]):
                best = (d, L)
        if best is not None and any(m["view"] == o["view"] for m in best[1]["obs"]):
            if best[0] < dup:
                continue                 # 同じ画像の重複
            best = None
        if best is None:
            L = {"obs": [o]}
            lugs.append(L)
        else:
            L = best[1]
            L["obs"].append(o)
        _update(L)
    return lugs


def _update(L):
    ob = L["obs"]
    wp = np.array([1 / o["s_pos"] ** 2 for o in ob])
    wa = np.array([1 / o["s_axis"] ** 2 for o in ob])
    L["p"] = np.average([o["p"] for o in ob], axis=0, weights=wp)
    L["s_pos"] = float(1 / math.sqrt(wp.sum()))
    L["axis"] = _axial_mean([o["axis"] for o in ob], wa)
    L["s_axis"] = float(1 / math.sqrt(wa.sum()))
    L["conf"] = max(o["conf"] for o in ob)
    L["n_views"] = len({o["view"] for o in ob})


def _dir(a):
    return np.array([math.cos(math.radians(a)), math.sin(math.radians(a))])


def _line_dist(L, c):
    v = c - L["p"]
    u = _dir(L["axis"])
    return abs(v[0] * u[1] - v[1] * u[0])


def _pair_center(A, B):
    """金具 2 個 → 盤の中心の候補。長方形の四隅は中心から等距離なので、中心は 2 個の垂直二等分線の上にある。
    その線の上で、2 本のリング面の線からの距離（ばらつきで重み付け）が最小になる点。対角の 2 個なら中点になる。"""
    mid = (A["p"] + B["p"]) / 2
    e = B["p"] - A["p"]
    e /= np.linalg.norm(e)
    n = np.array([-e[1], e[0]])
    num = den = 0.0
    for L in (A, B):
        u = _dir(L["axis"])
        a = (mid - L["p"]) @ np.array([u[1], -u[0]])      # 中点での線からの符号付き距離
        b = n @ np.array([u[1], -u[0]])
        w = 1 / math.radians(L["s_axis"]) ** 2
        num += w * a * b
        den += w * b * b
    if den < 1e-9:
        return None
    return mid - num / den * n


def _between(lugs, idx, i, j):
    """金具 i と j を結ぶ線分の途中に、ほかの金具があるか（隣の盤どうしの組を除くため）。"""
    a, b = lugs[i]["p"], lugs[j]["p"]
    d = b - a
    L2 = d @ d
    for k in idx:
        if k in (i, j):
            continue
        t = (lugs[k]["p"] - a) @ d / L2
        if 0.1 < t < 0.9 and np.linalg.norm(a + t * d - lugs[k]["p"]) < 0.08:
            return True
    return False


def group(lugs):
    """Position-only rectangle groups with three or four corner lugs."""
    from itertools import combinations

    side_min = 0.6 - 2 * LUG_INSET - 0.05
    side_max = 1.2 - 2 * LUG_INSET + 0.05
    diagonal_max = math.sqrt(2.0) * side_max
    points = np.asarray([lug["p"] for lug in lugs], dtype=float)
    n_lugs = len(points)
    if n_lugs < 3:
        return []

    # A rectangle's longest pair is a diagonal. Spatial adjacency avoids testing
    # all 4-combinations when a scan contains many unrelated lugs.
    neighbors = []
    for i, p in enumerate(points):
        neighbors.append([j for j in range(n_lugs) if j != i and
                          np.linalg.norm(points[j] - p) <= diagonal_max + 1e-9])

    candidates = []
    seen = set()
    all_indices = range(n_lugs)

    def no_interior(P, members):
        origin = P[0]
        e1, e2 = P[1] - origin, P[3] - origin
        l1, l2 = float(e1 @ e1), float(e2 @ e2)
        if l1 < 1e-12 or l2 < 1e-12:
            return False
        member_points = points[list(members)]
        for k in all_indices:
            if k in members or np.min(np.linalg.norm(member_points - points[k], axis=1)) < 0.08:
                continue
            q = points[k] - origin
            a, b = float(q @ e1 / l1), float(q @ e2 / l2)
            if -0.05 < a < 1.05 and -0.05 < b < 1.05:
                return False
        return True

    def ordered_quad(P):
        center = P.mean(axis=0)
        order = np.argsort(np.arctan2(P[:, 1] - center[1], P[:, 0] - center[0]))
        return P[order]

    # Four-corner rectangles: only local 4-cliques are considered.
    for i in range(n_lugs):
        near = [j for j in neighbors[i] if j > i]
        for rest in combinations(near, 3):
            members = (i,) + rest
            if any(not np.any(np.isin(neighbors[a], [b])) for a, b in combinations(members, 2)):
                continue
            P = ordered_quad(points[list(members)])
            d02, d13 = P[2] - P[0], P[3] - P[1]
            center_error = float(np.linalg.norm((P[0] + P[2] - P[1] - P[3]) / 2))
            diagonal_error = abs(float(np.linalg.norm(d02)) - float(np.linalg.norm(d13)))
            sides = [float(np.linalg.norm(P[(k + 1) % 4] - P[k])) for k in range(4)]
            if (center_error > 0.05 or diagonal_error > 0.05 or
                    not all(side_min <= side <= side_max for side in sides)):
                continue
            if not no_interior(P, members):
                continue
            key = tuple(sorted(members))
            if key not in seen:
                seen.add(key)
                candidates.append(((0, center_error + diagonal_error), key))

    # Three-corner rectangles: each member can be the right-angle corner.
    for members in combinations(range(n_lugs), 3):
        if any(np.linalg.norm(points[a] - points[b]) > diagonal_max + 1e-9
               for a, b in combinations(members, 2)):
            continue
        for corner in members:
            others = [k for k in members if k != corner]
            u, v = points[others[0]] - points[corner], points[others[1]] - points[corner]
            lu, lv = float(np.linalg.norm(u)), float(np.linalg.norm(v))
            if not (side_min <= lu <= side_max and side_min <= lv <= side_max):
                continue
            cosine = abs(float(u @ v)) / (lu * lv)
            if cosine > 0.08:
                continue
            P = np.array([points[corner], points[others[0]],
                          points[others[0]] + v, points[others[1]]])
            if no_interior(P, members):
                candidates.append(((1, cosine), tuple(sorted(members))))

    # Stable ordering resolves equal geometric scores independently of ring axes.
    candidates.sort(key=lambda item: (item[0], item[1]))
    groups = []
    assigned = set()
    for _score, members in candidates:
        if assigned.isdisjoint(members):
            groups.append(list(members))
            assigned.update(members)
    return groups


def _rect(P):
    """長方形の四隅（3〜4 点）→ 中心、x 軸の向き（0〜90°）、その向きの辺の長さ、直交する辺の長さ。"""
    if len(P) == 3:
        # 直角の角になる点を探し、残り 2 点の中点を中心にする
        best = None
        for k in range(3):
            a, b = [P[m] - P[k] for m in range(3) if m != k]
            cos = abs(a @ b) / (np.linalg.norm(a) * np.linalg.norm(b))
            if best is None or cos < best[0]:
                best = (cos, k)
        k = best[1]
        o = [P[m] for m in range(3) if m != k]
        P = np.array([P[k], o[0], o[1], o[0] + o[1] - P[k]])
    c = P.mean(axis=0)
    ang = np.arctan2(*(P - c)[:, ::-1].T)
    P = P[np.argsort(ang)]
    E = np.roll(P, -1, axis=0) - P                       # 周に沿った 4 辺
    th = np.degrees(np.arctan2(E[:, 1], E[:, 0]))
    t = math.degrees(math.atan2(np.sum(np.sin(np.radians(4 * th))), np.sum(np.cos(np.radians(4 * th))))) / 4 % 90.0
    ln = np.linalg.norm(E, axis=1)
    along = np.abs(adiff(th, t)) < 45
    if along.all() or not along.any():                     # 崩れた四角形（3 点がほぼ一直線など）
        return c, t, float(ln.mean()), float(ln.mean())
    return c, t, float(ln[along].mean()), float(ln[~along].mean())


def fit(lugs, m):
    """金具の組 → 盤 {center, front_yaw, width, depth, n_lugs, front_known, x_known}。
    front_yaw は中心から正面へ向かう向き [deg]。"""
    P = np.array([lugs[i]["p"] for i in m])
    if len(m) >= 3:
        c, t, a, b = _rect(P)
        # x 軸（幅の方向）は、とりあえず長い辺とする（幅の向きは panels.py で決め直す）
        if a >= b:
            x, wl, dl = t, a, b
        else:
            x, wl, dl = t + 90.0, b, a
        return {"center": c, "front_yaw": (x - 90.0) % 360.0, "width": wl + 2 * LUG_INSET,
                "depth": dl + 2 * LUG_INSET, "n_lugs": len(m), "front_known": False, "x_known": False, "lugs": m}
    A, B = lugs[m[0]], lugs[m[1]]
    mid = (A["p"] + B["p"]) / 2
    e = B["p"] - A["p"]
    L = float(np.linalg.norm(e))
    e /= L
    n = np.array([-e[1], e[0]])
    h = float((_pair_center(A, B) - mid) @ n)
    if abs(h) < 0.1:                     # 対角の 2 個: 向きは決まらない。中心だけ使う
        return {"center": mid, "front_yaw": math.degrees(math.atan2(e[1], e[0])) % 360.0, "width": L / math.sqrt(2),
                "depth": L / math.sqrt(2), "n_lugs": 2, "front_known": False, "x_known": False, "diagonal": True,
                "lugs": m}
    center = mid + h * n
    front = -np.sign(h) * n
    return {"center": center, "front_yaw": math.degrees(math.atan2(front[1], front[0])) % 360.0,
            "width": L + 2 * LUG_INSET, "depth": 2 * (abs(h) + LUG_INSET), "n_lugs": 2,
            "front_known": True, "x_known": True, "lugs": m}


def assemble(obs):
    """観測（lift の出力を複数枚ぶん）→ 盤のリスト、金具のリスト（`orient_rows` は通さない）。"""
    lugs = fuse(obs)
    panels = [fit(lugs, m) for m in group(lugs)]
    return panels, lugs
