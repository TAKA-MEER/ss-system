"""試作 lifting-lug-pose の環境で回し、回帰試験の材料を作る。

試作 `9265d2d` の Windows の Python（`<試作>/.venv/Scripts/python.exe`）で実行する。
試作の `src` を `sys.path` に足して、試作の `common.py`・`assemble_lugs.py` を import する。
`data/synthetic/robot_v3_test` の最初の 10 シーンについて、視点ごとの `K`・`dist`・`T` と検出
（`results/lug_v3_on_v3_test/preds.json` を確信度 0.25 で絞ったもの）、
4 視点を統合して試作の `fuse`→`group`→`fit` を回した出力（`orient_rows` なし）を JSON に書く。

使い方:
  <試作>/.venv/Scripts/python.exe tools/make_proto_fixture.py
  <試作>/.venv/Scripts/python.exe tools/make_proto_fixture.py --proto <試作>
"""
import argparse
import json
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
CONF = 0.25
N_SCENES = 10
TOP_HEIGHT_M = 2.35
SCENE_REL = Path("data/synthetic/robot_v3_test")
PREDS_REL = Path("results/lug_v3_on_v3_test/preds.json")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--proto", type=Path,
                    default=Path(os.environ.get(
                        "LIFTING_LUG_POSE", "C:/Users/yisah/Documents/code/MIRS-local/lifting-lug-pose")),
                    help="先行試作 lifting-lug-pose のディレクトリ（読むだけ）")
    ap.add_argument("--out", type=Path,
                    default=REPO / "ss_ws" / "src" / "ss_locator" / "tests" / "fixtures" / "proto_9265d2d.json")
    args = ap.parse_args()

    sys.path.insert(0, str(args.proto / "src"))
    import numpy as np
    from assemble_lugs import fit, fuse, group, lift
    from common import PANEL_H, cam_params, load_ann

    assert PANEL_H == TOP_HEIGHT_M, f"試作の PANEL_H {PANEL_H} が {TOP_HEIGHT_M} と違う"

    scenes = {}
    for a in sorted((args.proto / SCENE_REL / "annotations").glob("*.json")):
        ann = load_ann(a)
        scenes.setdefault(ann["scene"], []).append(ann)
    preds = json.loads((args.proto / PREDS_REL).read_text(encoding="utf-8"))

    out = {"source": "lifting-lug-pose 9265d2d", "conf": CONF, "top_height_m": TOP_HEIGHT_M,
           "scenes": []}
    for sid in sorted(scenes)[:N_SCENES]:
        views, obs = [], []
        for ann in sorted(scenes[sid], key=lambda a: a["view"]):
            K, dist, T = cam_params(ann)
            entry = preds[ann["image"]]
            conf = np.asarray(entry["conf"], float)
            kp = np.asarray(entry["kp"], float).reshape(-1, 4, 3)[conf >= CONF]
            conf = conf[conf >= CONF]
            views.append({"view": ann["view"], "image": ann["image"],
                          "K": K.tolist(), "dist": dist.tolist(), "T": T.tolist(),
                          "kp": kp.tolist(), "conf": conf.tolist()})
            obs += lift(kp, conf, (K, dist, T), ann["view"])
        lugs = fuse(obs)
        panels = [fit(lugs, m) for m in group(lugs)]
        out["scenes"].append({
            "scene": sid,
            "views": views,
            "panels": [{"center": p["center"].tolist(), "front_yaw": p["front_yaw"],
                        "width": p["width"], "depth": p["depth"], "n_lugs": p["n_lugs"],
                        "front_known": bool(p["front_known"]), "x_known": bool(p["x_known"]),
                        "diagonal": bool(p.get("diagonal", False))} for p in panels],
        })

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"{args.out}: {len(out['scenes'])} シーン, "
          f"{sum(len(s['panels']) for s in out['scenes'])} 盤")


if __name__ == "__main__":
    main()
