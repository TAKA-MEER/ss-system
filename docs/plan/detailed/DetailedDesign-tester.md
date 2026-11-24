# 依頼を手で試す道具（案）

[DetailedDesign.md](DetailedDesign.md) の詳細。テスト用に、人が端末から依頼を出して結果を見るコマンド。
**状態: 2026-11-15 案。2026-11-17 に口の版 2 に合わせた（版の照合・後片付けの待ち方・理由 2 つ）。2026-11-19 にホストから起こすスクリプト（§7）を足した。**入力は [Spec-tester.md](../spec/Spec-tester.md)（§3 は 2026-11-17 決定）と [-locate.md](DetailedDesign-locate.md)（口）。名前の一覧は [-names.md](DetailedDesign-names.md) §8。

---

## 0. 結論

| 項目 | 案 |
| --- | --- |
| 形 | コンテナの中で打つコマンド `locate_cli`（`ros2 run ss_locator locate_cli`） |
| 通り道 | **[-locate.md](DetailedDesign-locate.md) の口だけを使う**（アクション `/ss/locate/panels`・`/ss/locate/cleanup_done`・`/ss/locate/status`）。`locator_node` の中は触らない・読み込まない |
| 層 | 純関数 `locate_view.py`（引数 → 依頼の値・結果 → 表・結果 → 図）と、つなぐ `locate_cli.py`（ROS 2 のクライアントと流れ）（`DD-2`） |
| 図 | OpenCV で描き、PNG に書き出す（画面は要らない。新しい依存を足さない） |
| 本番への影響 | `locator_node` と型は変えない |

## 1. 層とファイル

| 層 | ファイル | 中身 |
| --- | --- | --- |
| 純関数 | `locate_view.py` | `goal_from_args`・`format_status`・`format_result`・`format_cleanup`・`draw_top_view`・`result_to_dict`。ROS 2 を読み込まない（結果は dict で受ける） |
| つなぐ | `locate_cli.py` | `run_cli`・`main`。ROS 2 のメッセージを dict に直して `locate_view` に渡す。`setup.py` の `locate_cli` |

## 2. 引数

| 引数 | 既定 | 意味 |
| --- | --- | --- |
| `--from-deg` | −180 | 方向の範囲の開始 [°]（機体の前が 0・左が正） |
| `--span-deg` | 360 | 方向の範囲の幅 [°]（開始から反時計回り。0 より大きく 360 以下） |
| `--max-distance-m` | 0 | 探す最大の距離。0 は本システムの既定値 |
| `--time-limit-s` | 120 | 制限時間。**暫定**（偽の昇降機で全周が約 40 s。[ImplementationPlan.md](../ImplementationPlan.md) §4 `WP-LOC-02`） |
| `--yes` | — | 依頼の前の確認を省く |
| `--status` | — | 依頼を出さず、状態を 1 回出して終わる |
| `--plot <パス>` | — | 上から見た図を PNG で書き出す |
| `--json <パス>` | — | 結果と後片付けの完了を JSON で書き出す |
| `--wait-s` | 5 | 本システムの状態が届くのを待つ上限 |
| `--cleanup-margin-s` | 10 | 後片付けの完了を待つときに、結果の `cleanup_limit_s` に足す余裕。**待つ上限は `cleanup_limit_s` ＋ この値**（固定の秒数では待たない。[-locate.md](DetailedDesign-locate.md) §3 の勧め・[-open.md](DetailedDesign-open.md) §1 の 9。fk-system と同じ待ち方） |

`goal_from_args` は度を rad に直し、開始を −π〜π に直す。**範囲の外の値（幅が 0 以下・360 より大きい、距離が負、制限時間が 0 以下）は、依頼を出す前にこのコマンドが弾く**（終了コード 2 の使い方の誤り）。幅 360 は、口の全周（2π）にそのまま直す。

## 3. 流れ（`run_cli`）

| # | やること | 終わり方 |
| --- | --- | --- |
| 1 | `/ss/locate/status` を `--wait-s` まで待つ | 届かない → 「本システムが応えない」。終了コード 3 |
| 2 | `interface_version` を、このコマンドが知っている版（`"2"`）と照らす | 違う → 「口の版が合わない」。終了コード 3 |
| 3 | 状態を出す（`format_status`）。`--status` ならここで終わる | 終了コード 0 |
| 4 | `phase` が `IDLE` でなければ、依頼を出さない | 「別の依頼の最中」。終了コード 4 |
| 5 | 依頼の中身と「昇降機とカメラが動きます。よろしいですか？ [y/N]」を出す。`y`・`yes` 以外は出さない。`--yes` なら聞かない | 断った → 終了コード 0（依頼は出していない） |
| 6 | アクションで依頼する | 拒否された → 状態の `last_reject_reason` を出す。終了コード 4 |
| 7 | 途中経過（`phase`）が変わるたびに、経過時間と一緒に 1 行出す | — |
| 8 | 結果を受けたら、表を出す（`format_result`）。`--plot`・`--json` があれば書き出す。**「後片付けの完了まで、機体と昇降機を動かさないでください」を出す** | — |
| 9 | 依頼の ID が同じ `cleanup_done` を、**結果の `cleanup_limit_s` ＋ `--cleanup-margin-s`** まで待ち、出す（`format_cleanup`）。`cleanup_limit_s` が 0 の終わり方（`human_override` など）でも、`cleanup_done` は必ず 1 回出るので待つ（[-locate.md](DetailedDesign-locate.md) §3） | 届いた → 終了コード 0。届かない → 「後片付けの完了が届かない」。終了コード 5 |

**取り消し**（Ctrl+C）: 6〜7 の間に押されたら、アクションの取り消しを送り、**そのまま 8・9 へ進む**（取り消しの結果と後片付けの完了を待つ）。**2 回目の Ctrl+C で、待たずに終わる**（「後片付けの完了を見ていません」を出し、終了コード 130）。5 の確認の間の Ctrl+C は、依頼を出さずに終わる。

終了コード 0 は「結果と後片付けの完了を受けた」で、終わり方の理由（`reason`）は問わない。

`cleanup_done` は依頼の前から購読しておく（QoS は [-locate.md](DetailedDesign-locate.md) §3 と同じ）。前の依頼の完了が届いても、依頼の ID が違えば捨てる。

## 4. 出す文字

**表（`format_result`）**。角度はすべて度、長さは m。

```
結果: completed   盤 2 面   見た範囲: -180.0°〜+180.0°
 #    x[m]   y[m]  方位[°] 距離[m] 向き[°]  幅[m] 奥行[m] 位置σ[m] 向きσ[°] 金具 正面
 1   2.413  0.310    7.3    2.43   178.2   0.80   0.60    0.04     2.1    4   -
 2   2.398 -0.512  -12.1    2.45   179.0   0.80   0.60    0.07     3.4    2  181.5
```

| 欄 | 中身 |
| --- | --- |
| 方位・距離 | 中心の `atan2(y, x)`・`hypot(x, y)`（機体の原点から） |
| 向き | `yaw_rad`（基準の辺の向き）を度に |
| 奥行 | `depth_m`。**`depth_measured` が false なら数のうしろに `*` を付け、表の下に「* 奥行きは測っていない（既定値）」を 1 行出す**（版 3。2026-11-18） |
| 正面 | `front_known` なら `front_yaw_rad` を度に。そうでなければ `-` |
| 向きσ | `yaw_std_deg` が 90 以上なら数の代わりに `不明`（対角の吊り金具 2 個だけの盤。[-core.md](DetailedDesign-core.md) §4） |
| 見た範囲 | `seen_from_rad[i]`〜`seen_from_rad[i] + seen_span_rad[i]` を度で並べる。無ければ `なし` |

盤が 0 面のときは、表の代わりに理由の説明を 1 行出す（[-locate.md](DetailedDesign-locate.md) §2.4 の「spec の理由」の欄の言い方。**版 2 の `ceiling_unknown`・`camera_pose_unknown` も含め、§2.4 の 11 個すべてに説明を持つ**）。**`reason` の名前はそのまま出す**（言い換えた文だけにしない。記録と突き合わせるため）。知らない `reason` は名前だけ出す。

**状態（`format_status`）**: `interface_version`・`phase`・`last_reject_reason` を 1 行で。**後片付けの完了（`format_cleanup`）**: `reason` と、昇降機を元の高さに戻したか（`lift_restored`）を 1 行で。**結果の表には `cleanup_limit_s`（後片付けの上限）も出す。**

## 5. 上から見た図（`draw_top_view`）

| 項目 | 決まり |
| --- | --- |
| 大きさ | 800×800 px の画像（BGR の配列）を返す。書き出しは `locate_cli.py` が行う |
| 向き | 機体の原点が中央。**機体の前（+x）が上、左（+y）が左** |
| 縮尺 | 画像の半分の幅が「依頼の最大の距離（0 なら 5.0 m）＋ 0.5 m」 |
| 描くもの | 1 m ごとの同心円 ／ 機体（前向きの三角）／ 依頼された範囲（扇の輪郭）／ 見た範囲（薄く塗った扇）／ 盤の矩形（中心・`yaw_rad`・`width_m`・`depth_m` から）と番号 ／ **基準の辺を太く** ／ `front_known` の盤は正面の向きに短い線 |
| 文字 | 数字と英字だけ（OpenCV の文字は日本語を描けない） |

矩形の 4 隅: 基準の辺の法線 `n = (cos yaw, sin yaw)`、辺に沿う向き `t = (−sin yaw, cos yaw)` として、`中心 ± (depth/2) n ± (width/2) t`。基準の辺は `+n` の側の 2 隅を結ぶ辺。

## 6. 記録（`result_to_dict`）

`--json` で書き出す中身: 依頼の値（度と rad）・結果（`reason`・盤の一覧・見た範囲・`cleanup_limit_s`）・後片付けの完了（`reason`・`lift_restored`）・依頼を受け付けた時刻。**盤の値は口の値のまま（rad・m）で書き、丸めない。**

## 7. ホストから起こすスクリプト（`scripts/ss.sh`。`WP-TEST-02`。2026-11-19 案）

試験担当者が、ホスト（ノートパソコン）の端末から本システムを起こし、依頼を出し、止める。手順書は `docs/使い方.md`。

| 項目 | 決まり |
| --- | --- |
| 形 | 1 本の bash スクリプト `scripts/ss.sh <やること> [引数]`。どこから呼んでもリポジトリ直下を `/ws` に mount する |
| コンテナ | 本システムは名前 `ss-system` のコンテナで 1 つだけ動かす（`docker run -d`。`--rm` は付けない。落ちたあとも `logs` で読めるように）。`--network host`・`--ipc host`（実機のカメラ・昇降部に届き、同じ機械の ROS 2 からも見える）。ホストに `ROS_DOMAIN_ID` があれば渡す |
| 設定 | **`locator_node` には必ず `--params-file ss_ws/src/ss_locator/config/locator.yaml` を渡す**（渡さないと合成データの値で動く）。引数での上書きはそのうしろに `-p` で足す |
| コンテナの中のコマンド | **どれも `/ws` で動かす**（`locator.yaml` のモデル・材料のパスはリポジトリ直下からの相対）。`docker exec` で打つもの（`status`・`locate`・`stop` と `start` の待ち）は、`/opt/ros/humble/setup.bash` と `/ws/ss_ws/install/setup.bash` を読んでから `cd /ws` して `ros2 run ss_locator locate_cli …`。`locator_node` のコンテナには `PYTHONUNBUFFERED=1` を渡す（`logs` に出が溜まらないように） |
| 書き出し | 依頼の結果は `out/`（リポジトリ直下。gitignore）に時刻の名前で書く。コンテナは root で動くので、書いたあとにホストの利用者の持ち物に直す（`chown`） |
| docker の差し替え | 試験のため、呼ぶ docker を環境変数 `SS_DOCKER`（既定 `docker`）で差し替えられる。この名前は試験のためだけのもの |

| やること | 中身 | 終了コード |
| --- | --- | --- |
| `start [--camera-host <IP>] [--lift-host <名前かIP>] [--fakes]` | 1. `ss-system` が動いていれば起こさない。止まったまま残っていれば消してから起こす。2. `--fakes` でなければ `model/lug_v3_s.pt` が在るか見る。3. イメージ `ss-system:dev` が無ければ作る。4. コンテナの中で `colcon build` してから `locator_node` を起こす（`--camera-host` は `-p camera_host:=`、`--lift-host` は `-p lift_host:=`、`--fakes` は `-p use_fakes:=true`）。5. **状態が読めるまで待つ**（`locate_cli --status` を繰り返す。上限は環境変数 `SS_START_WAIT_S`、既定 180 s。ビルドを含むため）。読めたら状態の 1 行を出す | 0 起きた・1 起こせない（動いている・モデルが無い・コンテナが落ちた・上限まで読めない。落ちた・読めないときはコンテナの記録の最後の 50 行を出す） |
| `status` | `locate_cli --status` | `locate_cli` のまま。動いていなければ 1 |
| `locate [locate_cli の引数]` | 引数を `locate_cli` にそのまま渡す。**`--plot`・`--json` が無ければ `out/<YYYYmmdd-HHMMSS>.png`・`.json` を足す**。端末なら `docker exec -it`（確認の入力と Ctrl+C を通す）。終わったら書いたファイルの持ち主を直し、パスを出す | `locate_cli` のまま。動いていなければ 1 |
| `logs [-f]` | `docker logs`（`-f` は渡す） | docker のまま |
| `stop [--force]` | 状態の `phase` が `IDLE` でなければ止めない（**依頼の最中に止めると、昇降機を元の高さに戻さない**）。`--force` なら止める。状態が読めないときは止める。止めたらコンテナを消す | 0 止めた（動いていなかった）・4 依頼の最中 |
| `calib [mount_calib の引数]` | **`ss-system` が動いていれば撮らない**（どちらもカメラを動かすため。`--from` だけのときは動いていてもよい）。別のコンテナ（`docker run --rm`、`--network host`。端末なら `-it`）で `mount_calib` を起こす。**`colcon build` はしない**（動いている `locator_node` の `ss_ws/build`・`install` を書き換えないため）。`cd /ws && PYTHONPATH=ss_ws/src/ss_locator python3 -m ss_locator.mount_calib …` で起こす。**`--params-file` が無ければ `ss_ws/src/ss_locator/config/locator.yaml` を足す**。`--out`・`--from` がどちらも無ければ `--out out/calib-<時刻>` を足す。終わったら `out/` の持ち主を直す | `mount_calib` のまま。動いているので撮れない → 1 |
| それ以外・引数なし | 使い方を出す | 2 |

## 7a. 撮影の記録（`WP-TEST-03`。2026-11-19 案）

[Spec-tester.md](../spec/Spec-tester.md) §3「撮影の記録」。**記録するのは `locator_node`**（静止画と検出を持っているのはノードだけ）。道具（`locate_cli`）は変えない。

| 項目 | 決まり |
| --- | --- |
| パラメータ | `shot_log_dir`（文字列。既定 `""` = 記録しない。`/ws` からの相対でも絶対でもよい）・`shot_save_images`（真偽。既定 false。`shot_log_dir` が空なら効かない）。`locator.yaml` には既定のまま足す |
| 書く場所 | `<shot_log_dir>/shots.jsonl`（1 枚 1 行で足していく）。静止画は `<shot_log_dir>/shot_<6 桁の通し番号>.jpg`。フォルダが無ければ作る |
| 書く時 | `_frame_events` の本物の道で、検出と床への戻し（`assemble.lift`）が済んだ 1 枚ごと（`FrameDone(..., True)` を返す枚）。偽の道（`use_fakes`）でも書く（静止画は無いので `photo` は `null`。試験のため） |
| 1 行の中身 | `time`（UNIX 秒）・`shot`（通し番号。ノードを起こしてから 0 始まり）・`lift_height_m`（偽の道は `null`）・`camera_height_m`（偽の道は `null`）・`yaw_rad`・`pitch_down_rad`（偽の道は `null` でよい）・`keypoints`（検出器の出力のまま。金具ごとに 4 点の `[x, y]`。画素）・`confidences`（金具ごと）・`lugs`（床へ戻した金具。`[x, y]` の並び。ローカル座標・m）・`photo`（ファイル名か `null`）・あわせて当てはめ直しに要る値 `camera_pitch_offset_deg`・`camera_yaw_offset_deg`・`camera_roll_deg`・`camera_height_offset_m`・`top_height_default_m` |
| 失敗 | 書けない（フォルダが作れない・ディスク満杯など）ときは警告を 1 回ログに出し、**依頼の流れ（返す出来事）は変えない** |
| `ss.sh start` | `--record` で `-p shot_log_dir:=out/shots-<YYYYmmdd-HHMMSS>` を足し、そのフォルダ名を出す。`--save-images` は `--record` も兼ね、さらに `-p shot_save_images:=true` を足す |
| `ss.sh stop` | コンテナを止めたあと、`out/` があれば持ち主を直す（`chown`。`locate` と同じ） |

**`--param <名前>:=<値>`**（2026-11-25 ユーザー承認。fk の求め。`WP-TEST-04`）: `ss.sh start` に、任意のノードのパラメータの上書きを足す。繰り返し指定でき、`-p <名前>:=<値>` として `--params-file` より後ろ、ほかの上書き（`--camera-host` など）より後ろに、指定した順で付く。名前は英数字と `_` だけ（`[A-Za-z_][A-Za-z0-9_]*`）。`:=` が無い・名前が違う形なら、何も起こさずに 1 で終わる。値はコンテナの中のシェルで解釈されないよう引用する（空白や `;` を含んでも 1 つの値として渡る）。使い道: hve のピッチの改修までの `scan_view_down_max_deg`（実機 58・fk の sim 63）。

## 8. この案で決めないもの

| もの | どこで |
| --- | --- |
| 正解（巻き尺の値）と比べて誤差を出すこと | `SS-V1` の測り方を詳細にするとき |
