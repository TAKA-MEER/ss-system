"""確からしさの式（`pos_std_m`・`yaw_std_deg`）が、合成データの誤差に当てはまるかを確かめる。

試作 `lifting-lug-pose`（読むだけ）の `robot_v3_test` 全 100 シーン・4 視点を統合し、
`preds.json` の検出（確信度 0.25 以上）から `ss_locator.assemble` で盤を組み立て、
`ss_locator.panels.to_panel_top` で返す形に直して、真値と比べる。
**式が合わなくても式は変えない。結果だけを `docs/plan/source/std-check.md` に書く。**

使い方（コンテナの中）:
  python3 /ws/tools/check_std.py --proto /proto
"""
import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.optimize import linear_sum_assignment

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ss_ws" / "src" / "ss_locator"))

from ss_locator import assemble as A  # noqa: E402
from ss_locator import panels as P  # noqa: E402

CONF = 0.25
TOP_HEIGHT_M = 2.35
GATE_M = 0.4
TAU = 2.0 * math.pi
OUT = ROOT / "docs" / "plan" / "source" / "std-check.md"


def _load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _cam(ann):
    c = ann["camera"]
    return (np.array(c["K"], float),
            np.array(c["dist_coeffs"], float),
            np.array(c["T_cv_from_world"], float))


def _angle_diff(a, b, period):
    """周期 `period` の角度差（0 以上 period/2 以下）。"""
    d = (a - b) % period
    return min(d, period - d)


def _stats(values):
    a = np.asarray(values, float)
    if a.size == 0:
        return {"n": 0, "median": None, "p90": None, "le1": None, "le2": None}
    return {"n": int(a.size),
            "median": float(np.median(a)),
            "p90": float(np.percentile(a, 90.0)),
            "le1": float(np.mean(a <= 1.0)),
            "le2": float(np.mean(a <= 2.0))}


def _fmt_row(label, s):
    if s["n"] == 0:
        return f"| {label} | 0 | - | - | - | - |"
    return (f"| {label} | {s['n']} | {s['median']:.2f} | {s['p90']:.2f} | "
            f"{s['le1'] * 100:.1f}% | {s['le2'] * 100:.1f}% |")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--proto", required=True, help="試作 lifting-lug-pose のフォルダ")
    args = ap.parse_args()
    proto = Path(args.proto)

    preds = _load(proto / "results" / "lug_v3_on_v3_test" / "preds.json")

    scenes = defaultdict(list)
    for path in sorted((proto / "data" / "synthetic" / "robot_v3_test"
                        / "annotations").glob("*.json")):
        ann = _load(path)
        scenes[ann["scene"]].append(ann)

    pos_ratios, yaw_ratios, front_ratios = [], [], []
    n_gt = n_matched = n_diag = 0

    for sid in sorted(scenes):
        anns = scenes[sid]
        obs = []
        for ann in anns:
            entry = preds.get(ann["image"], {"kp": [], "conf": []})
            kp = np.asarray(entry["kp"], float).reshape(-1, 4, 3)
            conf = np.asarray(entry["conf"], float).reshape(-1)
            keep = conf >= CONF
            obs += A.lift(kp[keep], conf[keep], _cam(ann), ann["view"], TOP_HEIGHT_M)

        est_panels, lugs = A.assemble(obs)
        tops = [P.to_panel_top(p, lugs, TOP_HEIGHT_M) for p in est_panels]

        gts = [p["footprint_world"] for p in anns[0]["panels"]]
        n_gt += len(gts)
        if not tops or not gts:
            continue

        est_xy = np.array([[t["x_m"], t["y_m"]] for t in tops], float)
        gt_xy = np.array([g["center_xy"] for g in gts], float).reshape(-1, 2)
        cost = np.full((len(tops), len(gts)), 1e6)
        for i in range(len(tops)):
            for j in range(len(gts)):
                d = float(np.linalg.norm(est_xy[i] - gt_xy[j]))
                if d <= GATE_M:
                    cost[i, j] = d

        for i, j in zip(*linear_sum_assignment(cost)):
            if cost[i, j] >= 1e6:
                continue
            n_matched += 1
            top, gt = tops[i], gts[j]

            pos_err = float(np.linalg.norm(est_xy[i] - gt_xy[j]))
            if top["pos_std_m"] > 0.0:
                pos_ratios.append(pos_err / top["pos_std_m"])

            if est_panels[i].get("diagonal", False):
                n_diag += 1
                continue

            gt_yaw = math.radians(float(gt["front_yaw_deg"]))
            # 矩形の対称を考えて 90° 周期
            yaw_err = _angle_diff(top["yaw_rad"], gt_yaw, math.pi / 2.0)
            if top["yaw_std_deg"] > 0.0:
                yaw_ratios.append(math.degrees(yaw_err) / top["yaw_std_deg"])

            # 金具 2 個で正面が分かる盤は、正面の向きを 360° 周期でも見る
            if top["front_known"] and top["lugs"] == 2 and top["yaw_std_deg"] > 0.0:
                front_err = _angle_diff(top["front_yaw_rad"], gt_yaw, TAU)
                front_ratios.append(math.degrees(front_err) / top["yaw_std_deg"])

    pos = _stats(pos_ratios)
    yaw = _stats(yaw_ratios)
    front = _stats(front_ratios)

    lines = [
        "# 確からしさの当てはまり（WP-EST-02）",
        "",
        f"2026-11-05 に、試作 `robot_v3_test` の全 {len(scenes)} シーン（4 視点を統合）で確かめた。",
        "推定は `ss_locator.assemble`（試作 `9265d2d` の写し）と "
        "`ss_locator.panels.to_panel_top`。",
        "**式は変えていない。結果だけを書く。**",
        "",
        "```bash",
        "python3 tools/check_std.py --proto /proto",
        "```",
        "",
        "| 項目 | サンプル数 | 中央値 | 90% | 1 以下の割合 | 2 以下の割合 |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
        _fmt_row("位置の誤差 ÷ `pos_std_m`", pos),
        _fmt_row("向きの誤差 ÷ `yaw_std_deg`（90° 周期）", yaw),
        _fmt_row("向きの誤差 ÷ `yaw_std_deg`（360° 周期・金具 2 個で正面が分かる盤）", front),
        "",
        "| 項目 | 数 |",
        "| --- | ---: |",
        f"| 対応した盤 | {n_matched} |",
        f"| 真値の盤 | {n_gt} |",
        f"| 対角の盤（向きの集計から除いた） | {n_diag} |",
        "",
    ]
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(f"scenes: {len(scenes)}  matched: {n_matched} / gt: {n_gt}  diagonal: {n_diag}")
    for name, s in (("posErr/posStd", pos), ("yawErr/yawStd(90)", yaw),
                    ("frontErr/yawStd(360)", front)):
        if s["n"]:
            print(f"{name}: n={s['n']} median={s['median']:.3f} p90={s['p90']:.3f} "
                  f"<=1:{s['le1'] * 100:.1f}% <=2:{s['le2'] * 100:.1f}%")
        else:
            print(f"{name}: n=0")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
