"""組み立ての結果（試作の `fit`）を、依頼する側に返す盤（`PanelTop`）へ直す。

角度は rad（`yaw_std_deg` だけ °）。ROS 2 を import しない純関数だけを置く。
基準の辺は「機体側に最も向いている辺」で、その長さを `width_m` にする。
"""
import math

import numpy as np

from .assemble import _line_dist
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
        pos_std_m = math.sqrt(sum(s * s for s in s_pos) / (n * n) + line_dist * line_dist)
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
