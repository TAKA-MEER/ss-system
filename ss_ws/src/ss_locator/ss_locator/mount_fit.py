"""カメラの取り付けの角度のずれ（δθ・δψ・φ）を当てはめる純関数。

床からの高さと水平の位置を巻き尺で測った金具（2 個以上）を、ヨー・ピッチを変えて
何枚か撮った観測（ヘッダの向きと金具の画素）から、ずれを求める。
画素を `geometry.camera_T`（ずれを入れたもの）で天板の高さの面に下ろし、測った位置との
差を最小にする。どの測った金具に当たるかは、ずれの候補の格子で一番近い金具に割り当て、
当てはめたあとに割り当て直して確かめる。高さは既定では動かさない（ピッチのずれと
区別がつかない）。

詳細設計: `docs/plan/detailed/DetailedDesign-core.md` §2.1（`WP-CAL-02`、
外れた検出を捨てる `WP-CAL-03`）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares

from .geometry import camera_T, undistort_rays

# ずれの候補の格子（core §2.1: δψ ±10°・δθ ±6°、0.5° 刻み）
_GRID_YAW_RAD = math.radians(10.0)
_GRID_PITCH_RAD = math.radians(6.0)
_GRID_STEP_RAD = math.radians(0.5)


class FitError(ValueError):
    """当てはめられない（測った金具・観測が足りないなど）。"""


@dataclass
class FitResult:
    pitch_offset_rad: float       # δθ
    yaw_offset_rad: float         # δψ
    roll_rad: float               # φ
    height_m: float               # ピッチの回転軸の床からの高さ（fit_height のときだけ動く）
    residuals: list = field(default_factory=list)  # 観測ごとの {"mean_m", "max_m", "n"}（n は使った検出の数）
    mean_m: float = 0.0
    max_m: float = 0.0
    # WP-CAL-03: 捨てた検出。1 個ごとに {"observation", "pixel", "distance_m"}。
    # observation は元の観測の番号、pixel は [u, v]（画素）、
    # distance_m は捨てたときの、その検出が一番近い測った金具から遠い距離（m）。
    rejected: list = field(default_factory=list)


def _pixels_array(pixels):
    """画素の並び → (N,2)。検出 0 個は形を (0,2) にそろえる。

    `[]`（json の `pixels: []`）をそのまま `atleast_2d` すると (1,0) になり、
    検出 1 個と数えられて `undistort_rays` で落ちる。空観測は当てはめに
    使わず `n=0` と示すため、ここで空の (0,2) に直す。
    """
    arr = np.asarray(pixels, float)
    if arr.size == 0:
        return np.zeros((0, 2))
    return arr.reshape(-1, 2)


def ground_point(px, K, dist, T, plane_z_m):
    """画素 (N,2) → 高さ `plane_z_m` の面との交点 (N,2)。平行・後ろ向きは nan。"""
    px = _pixels_array(px)
    if len(px) == 0:
        return np.zeros((0, 2))
    C, d = undistort_rays(px, K, dist, T)
    dz = d[:, 2]
    t = np.where(np.abs(dz) < 1e-12, np.nan, (plane_z_m - C[2]) / dz)
    P = C + t[:, None] * d
    P[~(np.isfinite(t) & (t > 0))] = np.nan
    return P[:, :2]


def _ground_all(observations, params, K, dist, mount, top_height_m):
    """ずれの候補 `params` (δψ, δθ, φ, 高さ) で全観測の画素を面に下ろす。"""
    yaw_off, pitch_off, roll, height = params
    out = []
    for obs in observations:
        T = camera_T(
            height,
            float(obs["yaw_rad"]),
            float(obs["pitch_down_rad"]),
            mount["mount_x_m"],
            mount["mount_y_m"],
            mount["forward_offset_m"],
            optical_offset_m=mount["optical_offset_m"],
            pitch_offset_rad=pitch_off,
            yaw_offset_rad=yaw_off,
            roll_rad=roll,
        )
        out.append(ground_point(obs["pixels"], K, dist, T, top_height_m))
    return out


def _assign(points, lugs_xy):
    """各画素に一番近い測った金具の index を返す。"""
    if len(points) == 0:
        return np.zeros(0, dtype=int)
    d = np.linalg.norm(points[:, None, :] - lugs_xy[None, :, :], axis=2)
    return np.argmin(d, axis=1)


def _cost(points_list, lugs_xy, reject_m=0.3):
    # 格子で始める値を選ぶときの差は、金具 1 個につき `reject_m` で頭打ちにする
    # （WP-CAL-03）。遠い検出が格子の値を引っぱらないため。
    total = 0.0
    cap = reject_m ** 2 if reject_m is not None else None
    for points in points_list:
        if not np.all(np.isfinite(points)):
            return np.inf
        idx = _assign(points, lugs_xy)
        sq = np.sum((points - lugs_xy[idx]) ** 2, axis=1)
        if cap is not None:
            sq = np.minimum(sq, cap)
        total += float(np.sum(sq))
    return total


def _grid_search(observations, lugs_xy, K, dist, mount, top_height_m, height_m,
                 reject_m=0.3):
    yaws = np.arange(-_GRID_YAW_RAD, _GRID_YAW_RAD + 1e-9, _GRID_STEP_RAD)
    pitches = np.arange(-_GRID_PITCH_RAD, _GRID_PITCH_RAD + 1e-9, _GRID_STEP_RAD)
    best = (np.inf, 0.0, 0.0)
    for dyaw in yaws:
        for dpitch in pitches:
            cost = _cost(
                _ground_all(observations, (dyaw, dpitch, 0.0, height_m),
                            K, dist, mount, top_height_m),
                lugs_xy, reject_m,
            )
            if cost < best[0]:
                best = (cost, float(dyaw), float(dpitch))
    return best[1], best[2]


def _fit_once(observations, lugs_xy, K, dist, mount, top_height_m, height_m,
              fit_height, reject_m):
    """1 回の当てはめ（格子 + 最小二乗 + 割り当て直し）。末尾のずれと面の点を返す。"""
    yaw0, pitch0 = _grid_search(observations, lugs_xy, K, dist, mount,
                                top_height_m, height_m, reject_m)
    x0 = [yaw0, pitch0, 0.0] + ([height_m] if fit_height else [])
    lo = [-0.35, -0.35, -0.6] + ([height_m - 2.0] if fit_height else [])
    hi = [0.35, 0.35, 0.6] + ([height_m + 2.0] if fit_height else [])
    params = np.array(x0, float)
    assignment = None
    for _ in range(4):
        full = np.r_[params[:3], params[3] if fit_height else height_m]
        points_list = _ground_all([dict(o) for o in observations], full,
                                  K, dist, mount, top_height_m)
        new_assignment = [_assign(p, lugs_xy) for p in points_list]
        if assignment is not None and all(
                np.array_equal(a, b) for a, b in zip(assignment, new_assignment)):
            assignment = new_assignment
            break
        assignment = new_assignment

        def residuals(x):
            p = np.r_[x[:3], x[3] if fit_height else height_m]
            pts = _ground_all(observations, p, K, dist, mount, top_height_m)
            out = []
            for points, idx in zip(pts, assignment):
                out.append((points - lugs_xy[idx]).ravel())
            return np.concatenate(out) if out else np.zeros(0)

        # 残差は `reject_m` 付近で頭打ち（cauchy。遠い検出への勾配が弱まる）。
        # 遠い検出をそのまま入れると最初の当てはめが引っぱられ、近い検出まで
        # 遠く見えて誤って外すため（格子だけの頭打ちでは防げない。WP-CAL-03）。
        # cauchy でも huber でも 2026-11-18 の実機の観測では、近い検出が
        # 0.3 m を超えて誤って外れた。cauchy の弱まる勾配で防げる。
        # 近い検出だけでは従来の二乗と同じで、元の当てはめの精度は変わらない。
        solved = least_squares(residuals, params, bounds=(lo, hi),
                               loss="cauchy", f_scale=reject_m,
                               xtol=1e-14, ftol=1e-14, gtol=1e-14)
        params = solved.x
    height = float(params[3]) if fit_height else height_m
    full = np.array([params[0], params[1], params[2], height])
    points_list = _ground_all(observations, full, K, dist, mount, top_height_m)
    return full, points_list


def fit_mount_offsets(observations, lugs_xy, K, dist, *, height_m, top_height_m,
                      mount_x_m=0.0, mount_y_m=0.0, forward_offset_m=0.0,
                      optical_offset_m=0.0, fit_height=False, reject_m=0.3):
    """観測と測った金具の位置 → δθ・δψ・φ と差。

    `observations`: `{"yaw_rad", "pitch_down_rad", "pixels" (N,2)}` の並び。
    `lugs_xy`: 測った金具の水平の位置 (M,2)。
    `height_m`: ピッチの回転軸の床からの高さ。`fit_height` のときだけこれも当てはめる。
    `reject_m`（既定 0.3 m）: これより測った金具から遠い検出を外して当てはめ直す
    （WP-CAL-03）。格子の差は 1 個につき `reject_m` で頭打ちにする。外した検出は
    結果の `rejected` に載る。
    """
    lugs_xy = np.asarray(lugs_xy, float).reshape(-1, 2)
    if len(lugs_xy) < 2:
        raise FitError(
            "measured lugs too few: need at least 2, got %d" % len(lugs_xy))
    # 検出 0 個の観測は当てはめに使わない（`active` から外れる）。形は
    # `_pixels_array` で (0,2) にそろえ、出力の観測ごとの行で n=0 と示す。
    observations = [
        {"yaw_rad": float(o["yaw_rad"]),
         "pitch_down_rad": float(o["pitch_down_rad"]),
         "pixels": _pixels_array(o["pixels"])}
        for o in observations
    ]
    n_pixels = sum(len(o["pixels"]) for o in observations)
    if n_pixels < 2:
        raise FitError(
            "observations too few: need at least 2 pixels, got %d" % n_pixels)

    K = np.asarray(K, float).reshape(3, 3)
    dist = np.asarray(dist, float)
    mount = {"mount_x_m": float(mount_x_m), "mount_y_m": float(mount_y_m),
             "forward_offset_m": float(forward_offset_m),
             "optical_offset_m": float(optical_offset_m)}
    height_m = float(height_m)

    # 外す処理（WP-CAL-03）: 当てはめ → `reject_m` より遠い検出を外す →
    # 当てはめ直し。外すものが無くなるまで繰り返す。
    reject_m = float(reject_m)
    rejected = []
    # working[i] は元の観測 i の残っている画素の並び（順序は元の順）。
    working = [o["pixels"].copy() for o in observations]
    params = None
    points_list = None
    while True:
        active = [
            {"yaw_rad": o["yaw_rad"], "pitch_down_rad": o["pitch_down_rad"],
             "pixels": working[i]}
            for i, o in enumerate(observations) if len(working[i])
        ]
        if not active:
            raise FitError(
                "measured lugs too few after rejecting outliers: "
                "need at least 2 lugs with assigned detections")
        params, active_points = _fit_once(active, lugs_xy, K, dist, mount,
                                          top_height_m, height_m, fit_height,
                                          reject_m)
        # active の並び（元の観測の番号）を辿って、遠い検出を探す。
        active_indices = [i for i in range(len(observations)) if len(working[i])]
        to_remove = []  # (元の観測番号, その観測の中の位置, 距離)
        for pos, points in zip(active_indices, active_points):
            if len(points) == 0:
                continue
            idx = _assign(points, lugs_xy)
            d = np.linalg.norm(points - lugs_xy[idx], axis=1)
            for j, dist_m in enumerate(d):
                if not np.isfinite(dist_m) or float(dist_m) > reject_m:
                    to_remove.append((pos, j, float(dist_m) if np.isfinite(dist_m) else float("inf")))
        if not to_remove:
            points_list = active_points
            break
        removed_per_obs = {}
        for pos, j, dist_m in to_remove:
            pixel = working[pos][j]
            rejected.append({"observation": int(pos),
                             "pixel": [float(pixel[0]), float(pixel[1])],
                             "distance_m": dist_m})
            removed_per_obs.setdefault(pos, set()).add(j)
        for pos, removed_js in removed_per_obs.items():
            keep = [k for k in range(len(working[pos])) if k not in removed_js]
            working[pos] = working[pos][keep] if keep else np.zeros((0, 2))

    # 外したあと、測った金具 2 個以上に割り当たる検出が残らなければ当てはめない。
    used_lug_indices = set()
    for points in points_list:
        if len(points):
            used_lug_indices.update(int(v) for v in _assign(points, lugs_xy))
    if len(used_lug_indices) < 2:
        raise FitError(
            "measured lugs too few after rejecting outliers: need at least 2, "
            "got %d assigned" % len(used_lug_indices))

    per_obs = []
    all_d = []
    active_indices = [i for i in range(len(observations)) if len(working[i])]
    surviving = dict()
    for pos, points in zip(active_indices, points_list):
        if len(points) == 0:
            surviving[pos] = np.zeros(0)
            continue
        idx = _assign(points, lugs_xy)
        surviving[pos] = np.linalg.norm(points - lugs_xy[idx], axis=1)
    for i, o in enumerate(observations):
        d = surviving.get(i, np.zeros(0))
        all_d.append(d)
        per_obs.append({"mean_m": float(np.mean(d)) if len(d) else 0.0,
                        "max_m": float(np.max(d)) if len(d) else 0.0,
                        "n": int(len(d))})
    flat = np.concatenate(all_d) if all_d else np.zeros(0)
    return FitResult(
        pitch_offset_rad=float(params[1]),
        yaw_offset_rad=float(params[0]),
        roll_rad=float(params[2]),
        height_m=float(params[3]),
        residuals=per_obs,
        mean_m=float(np.mean(flat)) if len(flat) else 0.0,
        max_m=float(np.max(flat)) if len(flat) else 0.0,
        rejected=rejected,
    )
