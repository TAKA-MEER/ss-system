import json
import math
from pathlib import Path

import numpy as np

from ss_locator import geometry as G

FIXTURE = Path(__file__).with_name("fixtures") / "proto_9265d2d.json"


def test_camera_T_axes():
    T = G.camera_T(3.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    # 外部パラメータの各行は、カメラの軸の base_link での向き
    np.testing.assert_allclose(T[2, :3], [1.0, 0.0, 0.0], atol=1e-12)   # z（前）= +x
    np.testing.assert_allclose(T[0, :3], [0.0, -1.0, 0.0], atol=1e-12)  # x（右）= -y
    np.testing.assert_allclose(T[1, :3], [0.0, 0.0, -1.0], atol=1e-12)  # y（下）= -z
    T90 = G.camera_T(3.0, math.pi / 2, 0.0, 0.0, 0.0, 0.0)
    np.testing.assert_allclose(T90[2, :3], [0.0, 1.0, 0.0], atol=1e-12)  # ヨー +90° で z = +y


def test_camera_T_roundtrip():
    fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))
    first = fixture["scenes"][0]["views"][0]
    K = np.array(first["K"], float)
    dist = np.array(first["dist"], float)
    w, h = 1280, 720
    mx, my, off = 0.1, -0.05, 0.2

    for yaw in np.linspace(-np.pi, np.pi, 9):
        for pitch_deg in (0, 10, 20, 30, 40):
            T = G.camera_T(3.0, yaw, math.radians(pitch_deg), mx, my, off)
            C = np.array([mx + off * math.cos(yaw), my + off * math.sin(yaw), 3.0])
            pts = []
            for r in (1, 2, 4, 6, 8, 10, 12):
                for lat in (-0.6, 0.0, 0.6):
                    pts.append([C[0] + r * math.cos(yaw) - lat * math.sin(yaw),
                                C[1] + r * math.sin(yaw) + lat * math.cos(yaw), 0.0])
            P = np.array(pts)
            uv, ok = G.project(P, K, dist, T, w, h)
            assert ok.sum() >= 1, f"ヨー {yaw:.3f}・見下ろし {pitch_deg}° で画像内の点が無い"

            Cc, d = G.undistort_rays(uv[ok], K, dist, T)
            t = (0.0 - Cc[2]) / d[:, 2]
            P2 = Cc + t[:, None] * d
            np.testing.assert_allclose(P2, P[ok], atol=1e-3, rtol=0,
                                       err_msg=f"ヨー {yaw:.3f}・見下ろし {pitch_deg}°")
