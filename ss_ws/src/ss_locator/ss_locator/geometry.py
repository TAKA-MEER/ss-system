"""カメラの外部パラメータと、歪み補正つきの射影。

先行試作 lifting-lug-pose `9265d2d` の `src/common.py` から写した。変えたところ:
- import を numpy だけにした（試作は datetime・hashlib・json・sys・pathlib も使う）
- カメラの姿勢から外部パラメータを作る `camera_T` を足した（試作は外部パラメータを既知として受けていた）
- アノテーション読み込みやキーポイントの可視性など、合成データ専用の関数は写さない
"""
import numpy as np


def fold_radius(dist):
    """歪み多項式 r*(1+k1 r^2+k2 r^4) が単調増加でなくなる正規化半径（これより外は射影が折り返す）。"""
    k1, k2 = dist[0], dist[1]
    # d/dr [r f(r)] = 1 + 3 k1 t + 5 k2 t^2, t = r^2
    roots = np.roots([5 * k2, 3 * k1, 1.0]) if abs(k2) > 1e-12 else (
        np.array([-1.0 / (3 * k1)]) if abs(k1) > 1e-12 else np.array([]))
    roots = [r.real for r in np.atleast_1d(roots) if abs(r.imag) < 1e-9 and r.real > 0]
    return float(np.sqrt(min(roots))) if roots else np.inf


def project(Xw, K, dist, T, w=None, h=None):
    """ワールド点 (N,3) → 画素 (N,2), 有効フラグ (N,)。

    有効 = カメラ前方 かつ 歪みの折り返しより内側（w, h を渡すと画像内も条件にする）。
    折り返しより外の点は無歪みのピンホールで射影する（必ず画像外に出る）。
    """
    Xw = np.atleast_2d(np.asarray(Xw, float))
    Xc = Xw @ T[:3, :3].T + T[:3, 3]
    z = Xc[:, 2]
    zs = np.where(np.abs(z) < 1e-9, 1e-9, z)
    x, y = Xc[:, 0] / zs, Xc[:, 1] / zs
    r2 = x * x + y * y
    f = 1 + dist[0] * r2 + dist[1] * r2 * r2
    rf = fold_radius(dist)
    ok = (z > 0.05) & (np.sqrt(r2) < rf)
    f = np.where(ok, f, 1.0)
    u = K[0, 0] * x * f + K[0, 2]
    v = K[1, 1] * y * f + K[1, 2]
    if w is not None:
        ok &= (u >= 0) & (u < w) & (v >= 0) & (v < h)
    return np.stack([u, v], 1), ok


def undistort_rays(px, K, dist, T, iters=20):
    """画素 (N,2) → カメラ中心 C (3,) とワールド方向 (N,3)。"""
    px = np.atleast_2d(np.asarray(px, float))
    xd = (px[:, 0] - K[0, 2]) / K[0, 0]
    yd = (px[:, 1] - K[1, 2]) / K[1, 1]
    x, y = xd.copy(), yd.copy()
    for _ in range(iters):
        r2 = x * x + y * y
        f = 1 + dist[0] * r2 + dist[1] * r2 * r2
        x, y = xd / f, yd / f
    d_cam = np.stack([x, y, np.ones_like(x)], 1)
    R, t = T[:3, :3], T[:3, 3]
    C = -R.T @ t
    return C, d_cam @ R          # (R^T d)^T = d^T R


def camera_T(height_m, yaw_rad, pitch_down_rad, mount_x_m, mount_y_m, forward_offset_m):
    """昇降機の高さ・ヨー・ピッチと取付の位置 → 外部パラメータ `base_link` → OpenCV カメラ (4×4)。

    ヨーは機体の前が 0・左が正、ピッチは見下ろしが正。光学中心の床からの高さは `height_m`。
    取付の位置はヨーの回転軸の `base_link` での位置 (`mount_x_m`, `mount_y_m`) と、
    回転軸から光学中心までの前向きの距離 `forward_offset_m`。
    """
    cy, sy = np.cos(yaw_rad), np.sin(yaw_rad)
    cp, sp = np.cos(pitch_down_rad), np.sin(pitch_down_rad)
    f = np.array([cy * cp, sy * cp, -sp])          # 前（OpenCV の z）
    r = np.array([sy, -cy, 0.0])                   # 右（OpenCV の x）
    d = np.cross(f, r)                             # 下（OpenCV の y）
    R = np.column_stack([r, d, f])                 # カメラ → base_link
    C = np.array([mount_x_m + forward_offset_m * cy,
                  mount_y_m + forward_offset_m * sy,
                  height_m])
    T = np.eye(4)
    T[:3, :3] = R.T
    T[:3, 3] = -R.T @ C
    return T
