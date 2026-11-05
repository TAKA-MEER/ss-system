# CPU での検出時間（WP-DET-01）

2026-11-05 に、**このデスクトップ PC**（学習用。GPU は RTX 4060 Ti 8GB）の Docker（Ubuntu 22.04・GPU なし）で測った。
**本番のノートパソコン（Ubuntu 22.04・GPU なし）での値ではない。**

| 項目 | 値 |
| --- | --- |
| CPU | 13th Gen Intel(R) Core(TM) i7-13700F（24 コア） |
| PyTorch | 2.11.0+cpu |
| torch のスレッド数 | 8 |
| 画像 | `robot_v3_test` の 50 枚（名前順。最初の 1 枚は暖機として除く） |
| 1 枚あたりの時間の中央値 | 0.1825 s |
| 1 枚あたりの時間の 90% | 0.2053 s |
| imgsz・確信度 | 1280・0.25（`config/locator.yaml` の既定値） |

```bash
docker run --rm -v "$PWD":/ws -v <試作>:/proto:ro ss-system:dev \
  python3 /ws/tools/measure_cpu_time.py \
  --images /proto/data/synthetic/robot_v3_test/images --n 50
```

Detector は CPU（`device='cpu'`）で動かした。時間は 1 枚の `Detector.detect`（前処理・推論・後処理を含む）を計ったもの。
