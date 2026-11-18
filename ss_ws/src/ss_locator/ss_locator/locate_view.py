"""依頼を手で試す道具の純関数（引数 → 依頼の値・結果 → 表・結果 → 図・記録）。

ROS 2 を読み込まない。結果・状態・後片付けの完了は dict で受ける
（`locate_cli` が口のメッセージを dict に直して渡す）。
文字の図は OpenCV で描くので、図の中の文字は数字と英字だけ。

詳細設計: `docs/plan/detailed/DetailedDesign-tester.md`、`DetailedDesign-names.md` §8。
"""
from __future__ import annotations

import math

import cv2
import numpy as np

# この道具が知っている口の版（-tester.md §3 の 3）
INTERFACE_VERSION = "3"

DEFAULT_MAX_DISTANCE_M = 5.0
IMAGE_SIZE = 800
YAW_UNKNOWN_DEG = 90.0

# 終わり方（reason）の説明。-locate.md §2.4 の 11 個
REASON_DESCRIPTIONS = {
    "completed": "最後まで見終えた",
    "no_lugs": "範囲に吊り金具が見えない",
    "lugs_ungrouped": "吊り金具は見えたが盤に組めない",
    "no_image": "画像が取れない",
    "lift_refused": "昇降機が動かない・拒否された",
    "camera_too_low": "必要な高さまで上がれない（天井が近い・上端）",
    "ceiling_unknown": "天井までの距離が分からず上がれない（カメラモジュールが繋がっていない・値が古い）",
    "camera_pose_unknown": "カメラの向きが分からない（ヨーの原点を合わせていない・カメラモジュールが再起動した）",
    "canceled": "取り消された",
    "timeout": "時間切れ",
    "human_override": "人の操作で中断した",
}

TABLE_HEADER = (
    " #    x[m]   y[m]  方位[°] 距離[m] 向き[°]  幅[m] 奥行[m] 位置σ[m] 向きσ[°] 金具 正面"
)


# ---------------------------------------------------------------- 引数 → 依頼

def goal_from_args(from_deg, span_deg, max_distance_m, time_limit_s):
    """引数（度）から依頼の値（rad）を作る。範囲の外は ValueError。"""
    from_deg = float(from_deg)
    span_deg = float(span_deg)
    max_distance_m = float(max_distance_m)
    time_limit_s = float(time_limit_s)
    if not (0.0 < span_deg <= 360.0):
        raise ValueError(f"方向の範囲の幅は 0 より大きく 360 以下にしてください: {span_deg}")
    if not (max_distance_m >= 0.0):
        raise ValueError(f"最大の距離は 0 以上にしてください: {max_distance_m}")
    if not (time_limit_s > 0.0):
        raise ValueError(f"制限時間は 0 より大きくしてください: {time_limit_s}")
    if not math.isfinite(from_deg):
        raise ValueError(f"開始の角度が正しくありません: {from_deg}")
    start_deg = ((from_deg + 180.0) % 360.0) - 180.0
    span_rad = 2.0 * math.pi if span_deg == 360.0 else math.radians(span_deg)
    return {
        "azimuth_from_rad": math.radians(start_deg),
        "azimuth_span_rad": span_rad,
        "max_distance_m": max_distance_m,
        "time_limit_s": time_limit_s,
    }


def cleanup_wait_limit_s(result, margin_s):
    """後片付けの完了を待つ上限。結果の `cleanup_limit_s` ＋ 余裕（固定の秒数にしない）。"""
    return float(result.get("cleanup_limit_s", 0.0)) + float(margin_s)


# ---------------------------------------------------------------- 表

def _deg(rad):
    return math.degrees(float(rad))


def format_status(status):
    reject = status.get("last_reject_reason", "") or "-"
    return (
        f"状態: interface_version={status.get('interface_version', '')}  "
        f"phase={status.get('phase', '')}  last_reject_reason={reject}"
    )


def format_reason(reason):
    """`reason` の名前と説明。知らない名前は名前だけ。"""
    description = REASON_DESCRIPTIONS.get(reason)
    if description is None:
        return reason
    return f"{reason}（{description}）"


def format_seen_ranges(seen_from_rad, seen_span_rad):
    if not seen_from_rad:
        return "なし"
    parts = []
    for start, span in zip(seen_from_rad, seen_span_rad):
        parts.append(f"{_deg(start):+.1f}°〜{_deg(float(start) + float(span)):+.1f}°")
    return ", ".join(parts)


def _panel_row(index, panel):
    x = float(panel["x_m"])
    y = float(panel["y_m"])
    bearing = math.degrees(math.atan2(y, x))
    distance = math.hypot(x, y)
    yaw_std = float(panel["yaw_std_deg"])
    yaw_std_text = "不明" if yaw_std >= YAW_UNKNOWN_DEG else f"{yaw_std:.1f}"
    front = f"{_deg(panel['front_yaw_rad']):.1f}" if panel.get("front_known") else "-"
    depth_text = f"{float(panel['depth_m']):6.2f}"
    if not panel.get("depth_measured", True):
        depth_text += "*"
    return (
        f"{index:2d} {x:7.3f} {y:7.3f} {bearing:7.1f} {distance:7.2f} "
        f"{_deg(panel['yaw_rad']):7.1f} {float(panel['width_m']):6.2f} "
        f"{depth_text} {float(panel['pos_std_m']):8.2f} "
        f"{yaw_std_text:>7} {int(panel['lugs']):4d} {front:>5}"
    )


def format_result(result):
    """結果の dict から、見出し・表（盤 0 面なら理由の説明）・後片付けの上限を作る。"""
    reason = result.get("reason", "")
    panels = result.get("panels", [])
    seen = format_seen_ranges(result.get("seen_from_rad", []), result.get("seen_span_rad", []))
    lines = [f"結果: {reason}   盤 {len(panels)} 面   見た範囲: {seen}"]
    if panels:
        lines.append(TABLE_HEADER)
        for i, panel in enumerate(panels, start=1):
            lines.append(_panel_row(i, panel))
        if any(not p.get("depth_measured", True) for p in panels):
            lines.append("* 奥行きは測っていない（既定値）")
    else:
        lines.append(f"盤は見つかりませんでした: {format_reason(reason)}")
    lines.append(f"後片付けの上限: {float(result.get('cleanup_limit_s', 0.0)):.1f} s")
    return "\n".join(lines)


def format_cleanup(cleanup):
    restored = "戻した" if cleanup.get("lift_restored") else "戻していない"
    return (
        f"後片付けの完了: reason={cleanup.get('reason', '')}  "
        f"lift_restored={bool(cleanup.get('lift_restored'))}（昇降機を元の高さに{restored}）"
    )


# ---------------------------------------------------------------- 図

BG = (255, 255, 255)
GRID = (210, 210, 210)
ROBOT = (60, 60, 60)
REQUEST = (200, 120, 0)
SEEN = (255, 220, 170)
PANEL = (0, 0, 200)
FRONT = (0, 140, 0)
TEXT = (0, 0, 0)


def _radius_m(goal):
    max_distance = float((goal or {}).get("max_distance_m", 0.0))
    return max_distance if max_distance > 0.0 else DEFAULT_MAX_DISTANCE_M


def _make_projection(goal):
    scale = (IMAGE_SIZE / 2.0) / (_radius_m(goal) + 0.5)
    center = IMAGE_SIZE / 2.0

    def to_px(x, y):
        # 機体の前（+x）が上、左（+y）が左
        return (int(round(center - y * scale)), int(round(center - x * scale)))

    return to_px, scale


def _sector_points(to_px, start, span, radius):
    steps = max(2, int(math.degrees(span) / 2.0) + 1)
    points = [to_px(0.0, 0.0)]
    for i in range(steps + 1):
        a = start + span * i / steps
        points.append(to_px(radius * math.cos(a), radius * math.sin(a)))
    return points


def panel_corners(panel):
    """矩形の 4 隅 [(x, y)...]。先頭 2 つが基準の辺（+n 側）。"""
    yaw = float(panel["yaw_rad"])
    n = (math.cos(yaw), math.sin(yaw))
    t = (-math.sin(yaw), math.cos(yaw))
    cx, cy = float(panel["x_m"]), float(panel["y_m"])
    half_d = float(panel["depth_m"]) / 2.0
    half_w = float(panel["width_m"]) / 2.0
    corners = []
    for sn, st in ((+1, +1), (+1, -1), (-1, -1), (-1, +1)):
        corners.append(
            (
                cx + sn * half_d * n[0] + st * half_w * t[0],
                cy + sn * half_d * n[1] + st * half_w * t[1],
            )
        )
    return corners


def draw_top_view(result, goal=None):
    """上から見た図（800×800 の BGR）。機体が中央、前が上、左が左。"""
    image = np.full((IMAGE_SIZE, IMAGE_SIZE, 3), BG, dtype=np.uint8)
    to_px, scale = _make_projection(goal)
    origin = to_px(0.0, 0.0)
    radius = _radius_m(goal)

    # 1 m ごとの同心円
    ring = 1
    while ring <= radius + 0.5:
        cv2.circle(image, origin, int(round(ring * scale)), GRID, 1)
        ring += 1

    # 見た範囲（薄く塗る）
    overlay = image.copy()
    for start, span in zip(result.get("seen_from_rad", []), result.get("seen_span_rad", [])):
        span = min(float(span), 2.0 * math.pi)
        points = np.array(_sector_points(to_px, float(start), span, radius), dtype=np.int32)
        cv2.fillPoly(overlay, [points], SEEN)
    image = cv2.addWeighted(overlay, 0.6, image, 0.4, 0)

    # 依頼された範囲（扇の輪郭）
    if goal is not None:
        span = min(float(goal["azimuth_span_rad"]), 2.0 * math.pi)
        if span >= 2.0 * math.pi - 1e-9:
            cv2.circle(image, origin, int(round(radius * scale)), REQUEST, 1)
        else:
            points = np.array(
                _sector_points(to_px, float(goal["azimuth_from_rad"]), span, radius),
                dtype=np.int32,
            )
            cv2.polylines(image, [points], True, REQUEST, 1)

    # 機体（前向きの三角）
    size = 0.25
    triangle = np.array(
        [to_px(size, 0.0), to_px(-size * 0.6, size * 0.6), to_px(-size * 0.6, -size * 0.6)],
        dtype=np.int32,
    )
    cv2.fillPoly(image, [triangle], ROBOT)

    # 盤
    for index, panel in enumerate(result.get("panels", []), start=1):
        corners = [to_px(x, y) for x, y in panel_corners(panel)]
        for k in range(4):
            a = corners[k]
            b = corners[(k + 1) % 4]
            thickness = 4 if k == 0 else 1  # corners[0]-[1] が基準の辺
            cv2.line(image, a, b, PANEL, thickness)
        center = to_px(float(panel["x_m"]), float(panel["y_m"]))
        if panel.get("front_known"):
            fy = float(panel["front_yaw_rad"])
            length = 0.4
            tip = to_px(
                float(panel["x_m"]) + length * math.cos(fy),
                float(panel["y_m"]) + length * math.sin(fy),
            )
            cv2.line(image, center, tip, FRONT, 2)
        cv2.putText(
            image, str(index), (center[0] - 4, center[1] + 5),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT, 1,
        )

    cv2.putText(
        image, f"{radius:.1f}m", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT, 1
    )
    return image


# ---------------------------------------------------------------- 記録

def result_to_dict(goal, result, cleanup=None, accepted_stamp=None):
    """`--json` に書く中身。盤の値は口の値のまま（rad・m）で、丸めない。"""
    goal_out = dict(goal)
    goal_out["azimuth_from_deg"] = math.degrees(goal["azimuth_from_rad"])
    goal_out["azimuth_span_deg"] = math.degrees(goal["azimuth_span_rad"])
    return {
        "goal": goal_out,
        "accepted_stamp": accepted_stamp,
        "result": {
            "reason": result.get("reason", ""),
            "panels": [dict(p) for p in result.get("panels", [])],
            "seen_from_rad": list(result.get("seen_from_rad", [])),
            "seen_span_rad": list(result.get("seen_span_rad", [])),
            "cleanup_limit_s": result.get("cleanup_limit_s", 0.0),
        },
        "cleanup_done": (
            None
            if cleanup is None
            else {
                "reason": cleanup.get("reason", ""),
                "lift_restored": bool(cleanup.get("lift_restored")),
            }
        ),
    }
