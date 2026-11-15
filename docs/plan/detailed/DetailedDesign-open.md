# DetailedDesign-open — 未確定・申し送り・レビュー指摘

[DetailedDesign.md](DetailedDesign.md) の詳細。

---

## 1. 詳細設計側の未確定事項

| # | 問い | 状態 |
| --- | --- | --- |
| 1 | 盤の位置を探す口（[-locate.md](DetailedDesign-locate.md)）を fk-system が確かめる | **済み**（2026-11-05 fk-system が案のとおりと確かめた。fk は `phase` が `IDLE` でない間は機体も昇降機も動かさない。`cleanup_timeout_s` の 30 s より fk の待つ上限（60 s）が長いことを伝えた） |
| 2 | **金具が対角の 2 個だけの盤**（向き・幅・奥行が決まらない。試作は中心だけ出し、幅・奥行を仮に辺の長さ ÷ √2 にしている）を返すか（[-core.md](DetailedDesign-core.md) §4） | **済み**（2026-11-05 ユーザー決定: 案 B。返して `yaw_std_deg` を 90 にする。[-core.md](DetailedDesign-core.md) §4） |
| 3 | 後片付けで**カメラのヨー・ピッチを元に戻すか**（spec は昇降機を戻すことだけ決めている。[-core.md](DetailedDesign-core.md) §5） | **済み**（2026-11-05 ユーザー決定: ヨー・ピッチも戻す。`human_override` では戻さない。[-core.md](DetailedDesign-core.md) §5）。**元のヨーを知るにはヨーの向きが読める必要がある**（hve `H-A11`。それまでは偽のカメラの値） |
| 4 | **ヨーの届く範囲**（全周に向けられるか）と、届かない方向を頼まれたときの扱い（[-core.md](DetailedDesign-core.md) §3） | **回せる範囲は全周**（2026-11-16 hve の答え）。届かない方向は無い。向きが読めない（原点を合わせる前・再起動のあと）ときの扱いは、hve のカメラモジュールの口の案を待つ |
| 5 | 確からしさ（`pos_std_m`・`yaw_std_deg`）の式が推定の誤差に当てはまるか（[-core.md](DetailedDesign-core.md) §4） | **確かめた（2026-11-05。[source/std-check.md](../source/std-check.md)）: 位置は小さく見積もりすぎ**（誤差 ÷ `pos_std_m` の中央値 0.62・90% 5.46・1 以下 59%）。**向きは大きめ**（90° 周期で 90% 0.95・1 以下 91%）。金具 2 個の正面は 360° 周期で 90% 9.73（正面の取り違えが混じる）。**扱いは案 1**（2026-11-05 ユーザー決定: 今の式のまま進め、大きな誤差の出どころを `WP-EST-03` で調べてから式を直す）。**`WP-EST-03` の結果: 2σ 超えの 121 盤のうち 120 盤が金具 2 個の盤**（金具 2 個の 255 盤のうち 120。その誤差の中央値 0.07 m）。金具 3〜4 個はほぼ当てはまる。**直した**（2026-11-06。直し方はユーザーが任せた: 金具 2 個で正面が分かる盤に、中心をリング面の向きから決める分のばらつきを足す。[-core.md](DetailedDesign-core.md) §4） |
| 6 | カメラの取付の位置（`camera_mount_*`・`camera_forward_offset_m`・`camera_height_offset_m`）と `scan_pitch_down_deg` | 実機で測る・決める（[-names.md](DetailedDesign-names.md) §5） |
| 7 | **`WP-HVE-01`（昇降部とつなぐ）を、hve の偽の昇降部を相手に今から実装してよいか。**[-names.md](DetailedDesign-names.md) §1 は「hve が実機に入れたと知らせるまで、この名前で実装しない」としている。2026-11-15 時点で口は hve の `main` に入ったが、実機への書き込みと `goto` のファーム（hve `WP-LIFT-14`）がまだ。実機では上昇させないままにする | **済み**（2026-11-15 ユーザー決定: hve と話し合い、接続部を作る。hve 側の問題はユーザーが hve の側で解決する）。偽の昇降部を相手に `WP-HVE-01` を始める。**hve に確かめている最中のこと**（[Spec-open.md](../spec/Spec-open.md) §1.2 の 2026-11-15 の行）の答えで [-hve.md](DetailedDesign-hve.md) §3・§4 が変わりうる |
| 8 | **人でない操作元（fk-system・アームの制御器）に昇降機を取って代わられたときの `reason`。**案では人の割り込みと同じに扱い（すぐ止まり、戻さずに手を離す。奪い返さないため）、`reason` は `human_override` になる。名前が実態と合わない。新しい理由を足すなら口（[-locate.md](DetailedDesign-locate.md) §2.4）と spec の理由の一覧が変わり、fk-system に確かめ直す | **未定**（[-hve.md](DetailedDesign-hve.md) §4 の 3。fk-system は本システムの操作中に昇降機を動かさない決まりなので、起きるのは決まりが破れたときだけ） |
| 9 | **昇降の速さ `lift_speed_mm_s` と、後片付けの上限 `cleanup_timeout_s`（30 s）。**撮る高さは下端から約 1.05〜1.35 m。暫定の 40 mm/s だと戻るのに約 26〜34 s かかり、**30 s では戻り切る前に手を離して `lift_restored: false` になりうる**。fk-system が後片付けを待つ上限は 60 s（§1 の 1） | **速さは昇降部の設定から取る**（2026-11-16 hve の答え。[-hve.md](DetailedDesign-hve.md) §2.2。`lift_speed_mm_s` は無くした）。**後片付けの上限は未定**: hve の頼みは「30 s に固定せず、戻る距離 ÷ 速さ ＋ 余裕」。偽の昇降部の初期値 30 mm/s だと 1.35 m に約 45 s かかり、余裕を足すと **fk-system が待つ上限（60 s）に近づく・超えうる**。口（[-locate.md](DetailedDesign-locate.md) §3）の決まりが変わるので、fk-system に知らせてから決める |
| 10 | コンテナの中から `hve-lift.local` が引けるか（引けなければ `lift_host` に IP を与える） | 実機で確かめる |
| 11 | **天井の値が無くて上がれなかった（`CEILING_STALE`）ことを、天井が近い場合と分けて依頼する側へ返すか**（hve の勧め。今はどちらも `camera_too_low` で、ログでだけ分かる）。分けるなら理由の一覧（spec・口）が変わり、fk-system に確かめ直す | 未定 |
| 12 | **カメラの向きが読めないとき（ヨーの原点を人が合わせていない・カメラモジュールが再起動した）の返し方。**hve の口の案では、このときヨーを角度で向けられない。本システムは動かさずに（途中なら止めて）返すことになるが、**今の理由の一覧に当てはまるものが無い**。新しい理由を足すと spec と口が変わり、fk-system に確かめ直す。あわせて「再起動のあいだの画像は捨てて撮り直す」（[-core.md](DetailedDesign-core.md) §5）をやめる | **未定**（[-hve.md](DetailedDesign-hve.md) §6.1。ユーザーの判断待ち） |

## 2. 仕様書からの申し送り

| 仕様書の ID | 詳細設計で要ること |
| --- | --- |
| `SS-A1`・`SS-A2` | 依頼する側との口は [-locate.md](DetailedDesign-locate.md)（案）、中の構成は [-core.md](DetailedDesign-core.md)（案）。ヨーの測り方（hve `H-A11`）が決まったら [-core.md](DetailedDesign-core.md) §2・§6 を直す。hve-system とつなぐ部分は [-hve.md](DetailedDesign-hve.md)（昇降部は 2026-11-15 案。カメラモジュールは待ち） |
| `SS-V3` | `WP-DET-01` で測り、結果を spec に戻す |
| `SS-A10` | 依頼を手で試す道具の細部（[Spec-tester.md](../spec/Spec-tester.md) §3）がユーザーの確認待ち。変わったら [-tester.md](DetailedDesign-tester.md) と `WP-TEST-01` を直す |
| `SS-A7`（2026-11-16 解決） | 金具認識のリアルタイム表示で hve-system の静止画を入力にすること。**繰り返し要求してよい（1 秒に 5 枚まで）と hve が認めた。**カメラモジュールの口を待つ（[-demo.md](DetailedDesign-demo.md) §0） |

## 3. レビュー指摘管理表

| # | 回 | 指摘 | 対応 |
| --- | --- | --- | --- |
| — | — | — | — |
