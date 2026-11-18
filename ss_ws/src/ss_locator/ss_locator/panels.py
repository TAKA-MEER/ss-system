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
        "yaw_rad": yaw_rad,
        "width_m": width_m,
        "depth_m": depth_m,
        "pos_std_m": pos_std_m,
        "yaw_std_deg": yaw_std_deg,
        "lugs": int(panel["n_lugs"]),
        "front_known": front_known,
        "front_yaw_rad": normalize(front) if front_known else 0.0,
    }


def _two_lug_top(lug_a, lug_b, top_height_m, depth_default_m, depth_std_m):
    """金具 2 個の組 → `PanelTop` と同じキーの dict（-core.md §4.1 の 4）。"""
    pa = np.asarray(lug_a["p"], float)
    pb = np.asarray(lug_b["p"], float)
    mid = (pa + pb) / 2.0
    e = pb - pa
    d = float(np.linalg.norm(e))
    e = e / d
    n = np.array([-e[1], e[0]])
    if float(mid @ n) <= 0.0:
        n = -n  # 機体の原点から遠ざかる向き
    center = mid + n * (float(depth_default_m) / 2.0 - LUG_INSET)
    width_m = d + 2.0 * LUG_INSET
    s_pos = [float(lug_a["s_pos"]), float(lug_b["s_pos"])]
    pos_std_m = math.sqrt(sum(s * s for s in s_pos) / 4.0 + float(depth_std_m) ** 2)
    mean_s_pos = sum(s_pos) / 2.0
    return {
        "x_m": float(center[0]),
        "y_m": float(center[1]),
        "top_height_m": float(top_height_m),
        "height_measured": False,
        "yaw_rad": normalize(math.atan2(-float(n[1]), -float(n[0]))),
        "width_m": float(width_m),
        "depth_m": float(depth_default_m),
        "pos_std_m": float(pos_std_m),
        "yaw_std_deg": math.degrees(math.atan(math.sqrt(2.0) * mean_s_pos / width_m)),
        "lugs": 2,
        "front_known": False,
        "front_yaw_rad": 0.0,
    }


def two_lug_panels(raw_panels, lugs, top_height_m, spacing_min_m, spacing_max_m,
                   depth_default_m, depth_std_m):
    """`assemble` の盤のうち金具 2 個の盤（対角を含む）を捨て、その金具を候補に戻し、
    間隔と向こう側の判定で組み直した `PanelTop` と同じキーの dict の並びを返す
    （-core.md §4.1 の 1〜4）。金具 3〜4 個の盤は呼び出し側が `to_panel_top` で返す。"""
    used = {i for panel in raw_panels if int(panel["n_lugs"]) >= 3 for i in panel["lugs"]}
    pool = [i for i in range(len(lugs)) if i not in used]
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
            if c is None:
                continue
            mid = (pa + pb) / 2.0
            e = (pb - pa) / d
            n = np.array([-e[1], e[0]])
            if float(mid @ n) <= 0.0:
                n = -n
            if float((np.asarray(c, float) - mid) @ n) <= 0.0:
                continue
            candidates.append((float(a["conf"]) + float(b["conf"]), i, j))
    candidates.sort(key=lambda item: -item[0])
    taken = set()
    tops = []
    for _, i, j in candidates:
        if i in taken or j in taken:
            continue
        taken.add(i)
        taken.add(j)
        tops.append(_two_lug_top(lugs[i], lugs[j], top_height_m,
                                 depth_default_m, depth_std_m))
    return tops


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
