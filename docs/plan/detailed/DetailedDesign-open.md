# DetailedDesign-open — 未確定・申し送り・レビュー指摘

[DetailedDesign.md](DetailedDesign.md) の詳細。

---

## 1. 詳細設計側の未確定事項

| # | 問い | 状態 |
| --- | --- | --- |
| 1 | 盤の位置を探す口（[-locate.md](DetailedDesign-locate.md)）を fk-system が確かめる | **済み**（2026-11-05 fk-system が案のとおりと確かめた。fk は `phase` が `IDLE` でない間は機体も昇降機も動かさない。`cleanup_timeout_s` の 30 s より fk の待つ上限（60 s）が長いことを伝えた） |

## 2. 仕様書からの申し送り

| 仕様書の ID | 詳細設計で要ること |
| --- | --- |
| `SS-A1`・`SS-A2`・`SS-A4` | 依頼する側との口は [-locate.md](DetailedDesign-locate.md)（案）。中の構成は `SS-S1` と一緒に書く |

## 3. レビュー指摘管理表

| # | 回 | 指摘 | 対応 |
| --- | --- | --- | --- |
| — | — | — | — |
