"""組み立ての結果（試作の `fit`）を、依頼する側に返す盤（`PanelTop`）へ直す。

角度は rad（`yaw_std_deg` だけ °）。ROS 2 を import しない純関数だけを置く。
基準の辺は「機体側に最も向いている辺」で、その長さを `width_m` にする。
"""
import math

import numpy as np

from .assemble import LUG_INSET, _between, _line_dist, _pair_center
from .scan_plan import normalize

TAU = 2.0 * math.pi


def _in_range(azimuth, azimuth_from_rad, azimuth_span_rad):
    """方位角が、開始から反時計回りに幅の分の範囲に入るか。幅 2π 以上は全周。"""
    if azimuth_span_rad >= TAU - 1e-12:
        return True
    return 0.0 <= (azimuth - azimuth_from_rad) % TAU <= azimuth_span_rad


def to_panel_top(panel, lugs, top_height_m):
    """`assemble.fit` の出力 1 面 → `PanelTop` と同じキーの dict。"""
    center = np.asarray(panel["center"], float)
    cx, cy = float(center[0]), float(center[1])

    front = math.radians(float(panel["front_yaw"]))
    to_origin = math.atan2(-cy, -cx)
    normals = [front, front + math.pi, front + math.pi / 2.0, front - math.pi / 2.0]
    base = min(range(4), key=lambda i: abs(normalize(normals[i] - to_origin)))
    yaw_rad = normalize(normals[base])

    # 基準の辺の外向き法線が front_yaw / front_yaw+180 なら幅、±90 なら奥行
    if base < 2:
        width_m = float(panel["width"])
        depth_m = float(panel["depth"])
    else:
        width_m = float(panel["depth"])
        depth_m = float(panel["width"])

    members = [lugs[i] for i in panel["lugs"]]
    n = len(members)
    if n:
        s_pos = [float(L["s_pos"]) for L in members]
        line_dist = float(np.mean([_line_dist(L, center) for L in members]))
        var = sum(s * s for s in s_pos) / (n * n) + line_dist * line_dist
        if panel.get("front_known", False) and not panel.get("diagonal", False):
            # 金具 2 個で正面が分かる盤は、中心をリング面の向きから決める。その分のばらつきを足す
            # （-core.md §4。足さないと小さすぎた: source/std-check.md）
            r_mean = float(np.mean([np.linalg.norm(center - np.asarray(L["p"], float)) for L in members]))
            s_axis_mean = float(np.mean([float(L["s_axis"]) for L in members]))
            a = r_mean * math.radians(s_axis_mean)
            var += a * a
        pos_std_m = math.sqrt(var)
        mean_s_pos = sum(s_pos) / n
    else:
        mean_s_pos = 0.0
        pos_std_m = 0.0

    if panel.get("diagonal", False):
        # 対角の 2 個は向きが決まらない（2026-11-05 ユーザー決定）
        yaw_std_deg = 90.0
    else:
        yaw_std_deg = math.degrees(math.atan(math.sqrt(2.0) * mean_s_pos / width_m))

    front_known = bool(panel.get("front_known", False))
    return {
        "x_m": cx,
        "y_m": cy,
        "top_height_m": float(top_height_m),
        "height_measured": False,
        "depth_measured": True,  # 3〜4 個の盤は両方の列を測った
        "yaw_rad": yaw_rad,
        "width_m": width_m,
        "depth_m": depth_m,
        "pos_std_m": pos_std_m,
        "yaw_std_deg": yaw_std_deg,
        "lugs": int(panel["n_lugs"]),
        "front_known": front_known,
        "front_yaw_rad": normalize(front) if front_known else 0.0,
    }


# ------------------------------------------------------------------ かけら（-core.md §4.1）
# かけら: 向きの軸 u（金具の列に沿う）と法線 n、測った列の n 方向の区間
# [m_lo, m_hi]、u 方向の金具の範囲 [u_lo, u_hi]、すべての列（測った列と
# 測っていない列）の (s, 測ったか) の並び rows_all。金具 2 個の組は
# 測った列が 1 本だけなので m_lo == m_hi。まとめる条件の D などは
# 測った列どうしで測る（測っていない列は最終の矩形を広げるときだけ使う）。


def _rect_fragment(panel, lugs):
    """金具 3〜4 個の盤 → かけら（両方の列を測った）。"""
    members = [lugs[i] for i in panel["lugs"]]
    center = np.asarray(panel["center"], float)
    front = math.radians(float(panel["front_yaw"]))
    f_axis = np.array([math.cos(front), math.sin(front)])
    w_axis = np.array([-f_axis[1], f_axis[0]])
    # 奥行きの軸は、機体の原点から中心へ向かう向きに最も近い方（手前と奥の列を分ける軸）
    norm = float(np.linalg.norm(center))
    if norm > 1e-9:
        radial = center / norm
        n = f_axis if abs(float(f_axis @ radial)) >= abs(float(w_axis @ radial)) else w_axis
    else:
        n = f_axis
    u = np.array([-n[1], n[0]])                        # 金具の列の向き
    s = [float(np.asarray(L["p"], float) @ n) for L in members]
    t = [float(np.asarray(L["p"], float) @ u) for L in members]
    s_pos = [float(L["s_pos"]) for L in members]
    count = len(members)
    line_dist = float(np.mean([_line_dist(L, center) for L in members]))
    pos_var = sum(x * x for x in s_pos) / (count * count) + line_dist * line_dist
    return {
        "u": u, "n": n,
        "m_lo": min(s), "m_hi": max(s),
        "u_lo": min(t), "u_hi": max(t),
        "rows_all": [(min(s), True), (max(s), True)],
        "lugs": list(panel["lugs"]),
        "conf_sum": sum(float(L["conf"]) for L in members),
        "pos_vars": [pos_var],
        "s_pos": s_pos,
        "kind": "rect",
        "spread_vec": None,
    }


def _pair_fragment(i, j, lugs, depth_default_m):
    """金具 2 個の組 → かけら（測った列 1 本。反対側は既定の奥行き分だけ置く）。"""
    a, b = lugs[i], lugs[j]
    pa = np.asarray(a["p"], float)
    pb = np.asarray(b["p"], float)
    mid = (pa + pb) / 2.0
    e = pb - pa
    d = float(np.linalg.norm(e))
    e = e / d
    c = np.asarray(_pair_center(a, b), float)
    spread = c - mid
    spread_vec = spread / float(np.linalg.norm(spread))  # 広がる側
    s_meas = float(mid @ spread_vec)
    s_other = s_meas + (float(depth_default_m) - 2.0 * LUG_INSET)
    t = [float(pa @ e), float(pb @ e)]
    s_pos = [float(a["s_pos"]), float(b["s_pos"])]
    return {
        "u": e, "n": spread_vec,
        "m_lo": s_meas, "m_hi": s_meas,
        "u_lo": min(t), "u_hi": max(t),
        "rows_all": [(s_meas, True), (s_other, False)],
        "lugs": [i, j],
        "conf_sum": float(a["conf"]) + float(b["conf"]),
        "pos_vars": [sum(x * x for x in s_pos) / 4.0],
        "s_pos": s_pos,
        "kind": "pair",
        "spread_vec": spread_vec,
    }


def _seam_neighbours(lugs, gap_min_m, gap_max_m):
    neighbours = [set() for _ in lugs]
    for i in range(len(lugs)):
        pi = np.asarray(lugs[i]["p"], float)
        for k in range(len(lugs)):
            if k == i:
                continue
            d = float(np.linalg.norm(np.asarray(lugs[k]["p"], float) - pi))
            if float(gap_min_m) <= d <= float(gap_max_m):
                neighbours[i].add(k)
    return neighbours


def _segments_cross(p1, p2, p3, p4):
    """線分 p1-p2 と p3-p4 が内部で交わるか。"""
    def orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    d1, d2 = orient(p3, p4, p1), orient(p3, p4, p2)
    d3, d4 = orient(p1, p2, p3), orient(p1, p2, p4)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)) and d1 != 0 and d2 != 0 and d3 != 0 and d4 != 0


def _seam_trap(lugs, i, j, neighbours, gap_min_m, gap_max_m):
    """継ぎ目をまたぐ斜めの組を捨てる（-core.md §4.1「継ぎ目のわな」①②）。"""
    pa = np.asarray(lugs[i]["p"], float)
    pb = np.asarray(lugs[j]["p"], float)
    dij = float(np.linalg.norm(pb - pa))
    for k in neighbours[i]:
        if float(np.linalg.norm(np.asarray(lugs[k]["p"], float) - pb)) < dij:
            return True  # ②
    for k in neighbours[j]:
        if float(np.linalg.norm(np.asarray(lugs[k]["p"], float) - pa)) < dij:
            return True  # ②
    for k in neighbours[i]:
        pk = np.asarray(lugs[k]["p"], float)
        for l in neighbours[j]:
            if k == l:
                continue
            pl = np.asarray(lugs[l]["p"], float)
            # ① 対角線の長さがそろうときだけ当てる（継ぎ目の 2 本の対角線はほぼ同じ長さ）
            if abs(float(np.linalg.norm(pl - pk)) - dij) >= float(gap_min_m):
                continue
            # ① k と l が線 i–j から離れて反対側にあるときだけ当てる
            # （ほぼ一直線の同じ列の金具が誤差で「交わる」にならないように）
            edge = pb - pa
            length = float(np.linalg.norm(edge))
            cross_k = float(edge[0] * (pk[1] - pa[1]) - edge[1] * (pk[0] - pa[0])) / length
            cross_l = float(edge[0] * (pl[1] - pa[1]) - edge[1] * (pl[0] - pa[0])) / length
            if abs(cross_k) < float(gap_min_m) / 2.0 or abs(cross_l) < float(gap_min_m) / 2.0:
                continue
            if (cross_k > 0) == (cross_l > 0):
                continue
            if _segments_cross(pa, pb, pk, pl):
                return True  # ①
    return False


def _make_fragments(raw_panels, lugs, spacing_min_m, spacing_max_m,
                    depth_default_m, gap_min_m, gap_max_m):
    """§4.1 の 1〜3: かけらの並びを作る。"""
    fragments = []
    used = set()
    for panel in raw_panels:
        if int(panel["n_lugs"]) >= 3:
            fragments.append(_rect_fragment(panel, lugs))
            used.update(panel["lugs"])
    pool = [i for i in range(len(lugs)) if i not in used]
    neighbours = _seam_neighbours(lugs, gap_min_m, gap_max_m)
    candidates = []
    for ai in range(len(pool)):
        for bi in range(ai + 1, len(pool)):
            i, j = pool[ai], pool[bi]
            a, b = lugs[i], lugs[j]
            pa = np.asarray(a["p"], float)
            pb = np.asarray(b["p"], float)
            d = float(np.linalg.norm(pb - pa))
            if not float(spacing_min_m) <= d <= float(spacing_max_m):
                continue
            if _between(lugs, range(len(lugs)), i, j):
                continue
            c = _pair_center(a, b)
            if c is None or float(np.linalg.norm(np.asarray(c, float) - (pa + pb) / 2.0)) < 1e-9:
                continue  # 広がる側を決められない
            if _seam_trap(lugs, i, j, neighbours, gap_min_m, gap_max_m):
                continue
            candidates.append((float(a["conf"]) + float(b["conf"]), i, j))
    candidates.sort(key=lambda item: -item[0])
    taken = set()
    for _, i, j in candidates:
        if i in taken or j in taken:
            continue
        taken.add(i)
        taken.add(j)
        fragments.append(_pair_fragment(i, j, lugs, depth_default_m))
    return fragments


def _merge_pair(p, q, angle_tol_deg, overlap_ratio,
                gap_min_m, gap_max_m, depth_min_m, depth_max_m, merge_tol_m):
    """かけら 2 つがまとめる条件をすべて満たせば、まとめたかけらを返す。"""
    up = np.asarray(p["u"], float)
    uq = np.asarray(q["u"], float)
    dot = float(up @ uq)
    angle = math.degrees(math.acos(max(-1.0, min(1.0, abs(dot)))))
    if angle > float(angle_tol_deg):
        return None  # (i)
    # 軸の向きを P に揃える（u と n は向きのない軸なので、それぞれで符号を合わせる）
    qq = dict(q)
    qq["u"] = uq if dot >= 0 else -uq
    if dot < 0:
        qq["u_lo"], qq["u_hi"] = -q["u_hi"], -q["u_lo"]
    pn = np.asarray(p["n"], float)
    qn = np.asarray(q["n"], float)
    qq["n"] = qn if float(pn @ qn) >= 0 else -qn
    if float(pn @ qn) < 0:
        qq["m_lo"], qq["m_hi"] = -q["m_hi"], -q["m_lo"]
        qq["rows_all"] = [(-s, m) for s, m in q["rows_all"]]
    if qq["m_lo"] >= p["m_hi"]:
        lower, upper = p, qq
    elif p["m_lo"] >= qq["m_hi"]:
        lower, upper = qq, p
    else:
        return None  # (ii) n に沿って重なる
    # (iii) 向かい合う列は測った列どうし（測った区間の端は測った列）
    if not (any(m for s, m in lower["rows_all"] if s == lower["m_hi"]) and
            any(m for s, m in upper["rows_all"] if s == upper["m_lo"])):
        return None
    overlap = min(p["u_hi"], qq["u_hi"]) - max(p["u_lo"], qq["u_lo"])
    shorter = min(p["u_hi"] - p["u_lo"], qq["u_hi"] - qq["u_lo"])
    if shorter <= 0 or overlap / shorter < float(overlap_ratio):
        return None  # (iv)
    d = float(upper["m_lo"] - lower["m_hi"])
    # k: 向かい合う列から見て相手の側へ広がっているかけらの数
    pn = np.asarray(p["n"], float)
    k = 0
    for frag, sign in ((lower, 1.0), (upper, -1.0)):
        if frag["kind"] == "pair" and float(np.asarray(frag["spread_vec"], float) @ (sign * pn)) > 0:
            k += 1
    L = LUG_INSET
    if k == 0:
        ranges = [(float(gap_min_m), float(gap_max_m))]
    elif k == 1:
        ranges = [(float(depth_min_m) - 2 * L + float(gap_min_m),
                   float(depth_max_m) - 2 * L + float(gap_max_m))]
    else:
        ranges = [(float(depth_min_m) - 2 * L - float(merge_tol_m),
                   float(depth_max_m) - 2 * L + float(merge_tol_m)),
                  (2 * float(depth_min_m) - 2 * L + float(gap_min_m),
                   2 * float(depth_max_m) - 2 * L + float(gap_max_m))]
    if not any(lo <= d <= hi for lo, hi in ranges):
        return None  # (v)
    return {
        "u": up, "n": pn,
        "m_lo": min(lower["m_lo"], upper["m_lo"]),
        "m_hi": max(lower["m_hi"], upper["m_hi"]),
        "u_lo": min(p["u_lo"], qq["u_lo"]),
        "u_hi": max(p["u_hi"], qq["u_hi"]),
        "rows_all": list(p["rows_all"]) + list(qq["rows_all"]),
        "lugs": list(p["lugs"]) + list(qq["lugs"]),
        "conf_sum": p["conf_sum"] + qq["conf_sum"],
        "pos_vars": list(p["pos_vars"]) + list(qq["pos_vars"]),
        "s_pos": list(p["s_pos"]) + list(qq["s_pos"]),
        "kind": "merged",
        "spread_vec": None,
    }


def _merge_fragments(fragments, angle_tol_deg, overlap_ratio,
                     gap_min_m, gap_max_m, depth_min_m, depth_max_m, merge_tol_m):
    """§4.1 の 4: かけらをまとめる。相手は高々 1 つ（conf の和の大きい順）。"""
    pairs = []
    for a in range(len(fragments)):
        for b in range(a + 1, len(fragments)):
            merged = _merge_pair(fragments[a], fragments[b], angle_tol_deg,
                                 overlap_ratio, gap_min_m, gap_max_m,
                                 depth_min_m, depth_max_m, merge_tol_m)
            if merged is not None:
                pairs.append((fragments[a]["conf_sum"] + fragments[b]["conf_sum"], a, b, merged))
    pairs.sort(key=lambda item: -item[0])
    taken = set()
    merged_by = {}
    for _, a, b, merged in pairs:
        if a in taken or b in taken:
            continue
        taken.add(a)
        taken.add(b)
        merged_by[a] = merged
    out = []
    for idx, frag in enumerate(fragments):
        if idx in merged_by:
            out.append(merged_by[idx])
        elif idx not in taken:
            out.append(frag)
    return out


def _fragment_top(frag, top_height_m, depth_std_m):
    """かけら（まとめた盤も 1 面の盤も）→ `PanelTop` と同じキーの dict。"""
    u = np.asarray(frag["u"], float)
    n = np.asarray(frag["n"], float)
    rows = frag["rows_all"]
    lo_all = min(s for s, _ in rows)
    hi_all = max(s for s, _ in rows)
    measured_lo = any(m for s, m in rows if abs(s - lo_all) < 1e-9)
    measured_hi = any(m for s, m in rows if abs(s - hi_all) < 1e-9)
    len_u = (frag["u_hi"] - frag["u_lo"]) + 2.0 * LUG_INSET
    len_n = (hi_all - lo_all) + 2.0 * LUG_INSET
    center = u * ((frag["u_lo"] + frag["u_hi"]) / 2.0) + n * ((lo_all + hi_all) / 2.0)
    cx, cy = float(center[0]), float(center[1])
    to_origin = math.atan2(-cy, -cx)
    candidates = [(-n, len_u), (n, len_u), (-u, len_n), (u, len_n)]
    normal, width_m = min(candidates, key=lambda nd: abs(normalize(math.atan2(float(nd[0][1]), float(nd[0][0])) - to_origin)))
    depth_m = len_n if width_m == len_u else len_u
    # width_m が len_u と len_n で等しい場合の取り違えを避けるため、辺の長さで再計算する
    if abs(len_u - len_n) < 1e-12:
        depth_m = len_u
    yaw_rad = normalize(math.atan2(float(normal[1]), float(normal[0])))
    unmeasured = (0 if measured_lo else 1) + (0 if measured_hi else 1)
    pos_var = sum(frag["pos_vars"]) / len(frag["pos_vars"]) + unmeasured * float(depth_std_m) ** 2
    mean_s_pos = sum(frag["s_pos"]) / len(frag["s_pos"]) if frag["s_pos"] else 0.0
    return {
        "x_m": cx,
        "y_m": cy,
        "top_height_m": float(top_height_m),
        "height_measured": False,
        "depth_measured": bool(measured_lo and measured_hi),
        "yaw_rad": yaw_rad,
        "width_m": float(width_m),
        "depth_m": float(depth_m),
        "pos_std_m": math.sqrt(pos_var),
        "yaw_std_deg": math.degrees(math.atan(math.sqrt(2.0) * mean_s_pos / width_m)),
        "lugs": int(len(frag["lugs"])),
        "front_known": False,
        "front_yaw_rad": 0.0,
    }


def make_panel_tops(raw_panels, lugs, top_height_m,
                    spacing_min_m, spacing_max_m, depth_default_m, depth_std_m,
                    depth_min_m, depth_max_m, gap_min_m, gap_max_m,
                    merge_tol_m, angle_tol_deg, overlap_ratio):
    """`assemble` の結果から返す盤の並び（-core.md §4.1 の 1〜4）。"""
    fragments = _make_fragments(raw_panels, lugs, spacing_min_m, spacing_max_m,
                                depth_default_m, gap_min_m, gap_max_m)
    fragments = _merge_fragments(fragments, angle_tol_deg, overlap_ratio,
                                 gap_min_m, gap_max_m, depth_min_m, depth_max_m,
                                 merge_tol_m)
    return [_fragment_top(f, top_height_m, depth_std_m) for f in fragments]


def filter_panels(tops, azimuth_from_rad, azimuth_span_rad, max_distance_m):
    """中心の方位角が範囲に入り、原点からの水平距離が `max_distance_m` 以下の盤だけを返す。"""
    out = []
    for top in tops:
        x, y = float(top["x_m"]), float(top["y_m"])
        if not _in_range(math.atan2(y, x), azimuth_from_rad, azimuth_span_rad):
            continue
        if math.hypot(x, y) > max_distance_m:
            continue
        out.append(top)
    return out


def count_lugs_in_range(lugs, azimuth_from_rad, azimuth_span_rad, max_distance_m):
    """`fuse` 後の金具のうち、範囲と距離に入るものの数。"""
    n = 0
    for lug in lugs:
        p = np.asarray(lug["p"], float)
        x, y = float(p[0]), float(p[1])
        if not _in_range(math.atan2(y, x), azimuth_from_rad, azimuth_span_rad):
            continue
        if math.hypot(x, y) > max_distance_m:
            continue
        n += 1
    return n


def finish_reason(n_lugs_in_range, n_panels):
    """終わり方の `reason`。盤が見つかれば `completed`、無ければ金具の有無で分ける。"""
    if n_panels >= 1:
        return "completed"
    if n_lugs_in_range == 0:
        return "no_lugs"
    return "lugs_ungrouped"
