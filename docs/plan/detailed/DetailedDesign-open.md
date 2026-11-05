# DetailedDesign-open — 未確定・申し送り・レビュー指摘

[DetailedDesign.md](DetailedDesign.md) の詳細。

---

## 1. 詳細設計側の未確定事項

| # | 問い | 状態 |
| --- | --- | --- |
| 1 | 盤の位置を探す口（[-locate.md](DetailedDesign-locate.md)）を fk-system が確かめる | **済み**（2026-11-05 fk-system が案のとおりと確かめた。fk は `phase` が `IDLE` でない間は機体も昇降機も動かさない。`cleanup_timeout_s` の 30 s より fk の待つ上限（60 s）が長いことを伝えた） |
| 2 | **金具が対角の 2 個だけの盤**（向き・幅・奥行が決まらない。試作は中心だけ出し、幅・奥行を仮に辺の長さ ÷ √2 にしている）を返すか（[-core.md](DetailedDesign-core.md) §4） | **済み**（2026-11-05 ユーザー決定: 案 B。返して `yaw_std_deg` を 90 にする。[-core.md](DetailedDesign-core.md) §4） |
| 3 | 後片付けで**カメラのヨー・ピッチを元に戻すか**（spec は昇降機を戻すことだけ決めている。[-core.md](DetailedDesign-core.md) §5） | **済み**（2026-11-05 ユーザー決定: ヨー・ピッチも戻す。`human_override` では戻さない。[-core.md](DetailedDesign-core.md) §5）。**元のヨーを知るにはヨーの向きが読める必要がある**（hve `H-A11`。それまでは偽のカメラの値） |
| 4 | **ヨーの届く範囲**（全周に向けられるか）と、届かない方向を頼まれたときの扱い（[-core.md](DetailedDesign-core.md) §3） | hve のカメラモジュールの口を待つ |
| 5 | 確からしさ（`pos_std_m`・`yaw_std_deg`）の式が推定の誤差に当てはまるか（[-core.md](DetailedDesign-core.md) §4） | **確かめた（2026-11-05。[source/std-check.md](../source/std-check.md)）: 位置は小さく見積もりすぎ**（誤差 ÷ `pos_std_m` の中央値 0.62・90% 5.46・1 以下 59%）。**向きは大きめ**（90° 周期で 90% 0.95・1 以下 91%）。金具 2 個の正面は 360° 周期で 90% 9.73（正面の取り違えが混じる）。**扱いは案 1**（2026-11-05 ユーザー決定: 今の式のまま進め、大きな誤差の出どころを `WP-EST-03` で調べてから式を直す）。**`WP-EST-03` の結果: 2σ 超えの 121 盤のうち 120 盤が金具 2 個の盤**（金具 2 個の 255 盤のうち 120。その誤差の中央値 0.07 m）。金具 3〜4 個はほぼ当てはまる。**式の直し方は未定（ユーザーが決める）** |
| 6 | カメラの取付の位置（`camera_mount_*`・`camera_forward_offset_m`・`camera_height_offset_m`）と `scan_pitch_down_deg` | 実機で測る・決める（[-names.md](DetailedDesign-names.md) §5） |

## 2. 仕様書からの申し送り

| 仕様書の ID | 詳細設計で要ること |
| --- | --- |
| `SS-A1`・`SS-A2` | 依頼する側との口は [-locate.md](DetailedDesign-locate.md)（案）、中の構成は [-core.md](DetailedDesign-core.md)（案）。ヨーの測り方（hve `H-A11`）が決まったら [-core.md](DetailedDesign-core.md) §2・§6 を直す |
| `SS-V3` | `WP-DET-01` で測り、結果を spec に戻す |

## 3. レビュー指摘管理表

| # | 回 | 指摘 | 対応 |
| --- | --- | --- | --- |
| — | — | — | — |
