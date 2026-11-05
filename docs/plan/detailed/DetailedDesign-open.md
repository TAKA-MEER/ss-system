# DetailedDesign-open — 未確定・申し送り・レビュー指摘

[DetailedDesign.md](DetailedDesign.md) の詳細。

---

## 1. 詳細設計側の未確定事項

| # | 問い | 状態 |
| --- | --- | --- |
| 1 | 盤の位置を探す口（[-locate.md](DetailedDesign-locate.md)）を fk-system が確かめる | **済み**（2026-11-05 fk-system が案のとおりと確かめた。fk は `phase` が `IDLE` でない間は機体も昇降機も動かさない。`cleanup_timeout_s` の 30 s より fk の待つ上限（60 s）が長いことを伝えた） |
| 2 | **金具が対角の 2 個だけの盤**（向き・幅・奥行が決まらない。試作は中心だけ出し、幅・奥行を仮に辺の長さ ÷ √2 にしている）を返すか（[-core.md](DetailedDesign-core.md) §4） | **未定（ユーザーが決める）**。案 A: 返さない（組めなかった扱い）。案 B: 中心だけ意味があるとして返し、`yaw_std_deg` を 90 にする |
| 3 | 後片付けで**カメラのヨー・ピッチを元に戻すか**（spec は昇降機を戻すことだけ決めている。[-core.md](DetailedDesign-core.md) §5） | **未定（ユーザーが決める）**。今の案は戻さない |
| 4 | **ヨーの届く範囲**（全周に向けられるか）と、届かない方向を頼まれたときの扱い（[-core.md](DetailedDesign-core.md) §3） | hve のカメラモジュールの口を待つ |
| 5 | 確からしさ（`pos_std_m`・`yaw_std_deg`）の式が推定の誤差に当てはまるか（[-core.md](DetailedDesign-core.md) §4） | `WP-EST-02` で合成データで確かめる |
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
