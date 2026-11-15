# 作業パケット（案）

[DetailedDesign.md](DetailedDesign.md) の詳細。**状態: 2026-11-05 案。**状態の追跡は [ImplementationPlan.md](../ImplementationPlan.md) §4。
各パケットの終わりに「動くもの」が残るように切った。**受け入れ条件はすべてコマンドで確かめる**（[README.md](README.md) §3 の約束 3）。

## 0. 一覧と順番

| パケット | 中身 | 前提 | 着手 |
| --- | --- | --- | --- |
| `WP-ENV-01` | 土台（Docker・型・ライセンス・試験の入口） | — | できる |
| `WP-EST-01` | 推定の移植（幾何・組み立て） | `WP-ENV-01` | できる |
| `WP-DET-01` | 検出と CPU の処理時間（`SS-V3`） | `WP-ENV-01` | できる（`WP-EST-01` と並べてよい） |
| `WP-EST-02` | 返す盤への変換・走査の計画 | `WP-EST-01` | できる |
| `WP-LOC-01` | 依頼の流れ（純関数）と偽の hve | `WP-EST-02`・`WP-DET-01` | できる |
| `WP-LOC-02` | ノード（ROS 2 の配線） | `WP-LOC-01` | できる |
| `WP-EST-03` | 位置の大きな誤差の出どころ（調べるだけ） | `WP-EST-02` | できる |
| `WP-HVE-01` | hve-system の昇降部とつなぐ（2026-11-15 に昇降部の分だけに絞った） | `WP-LOC-02` | 済み（2026-11-16 マージ。実機はまだ） |
| `WP-HVE-02` | カメラモジュールとつなぐ（純関数と `locate_core` の直し） | `WP-LOC-03` | 済み（2026-11-17 マージ） |
| `WP-HVE-03` | カメラモジュールとつなぐ（つなぐ層と、ノードの本物の道） | `WP-HVE-02` | できる（試験は小さなサーバが相手。**手での確認は hve の実装待ち**） |
| `WP-CAL-01` | カメラの内部パラメータの校正 | `WP-ENV-01` | **待ち**（hve のカメラで撮ったチェッカーボードの画像） |
| `WP-DEMO-01` | 金具認識のリアルタイム表示（本番外の道具） | `WP-DET-01` | できる |
| `WP-DEMO-02` | 金具認識のリアルタイム表示の起動スクリプト | `WP-DEMO-01` | できる |
| `WP-DEMO-03` | 金具認識のリアルタイム表示に速さと入力の解像度を出す | `WP-DEMO-01` | できる |
| `WP-LOC-03` | 口の版 2（理由 2 つ・後片付けの上限・`cleanup_limit_s`） | `WP-HVE-01` | 済み（2026-11-17 マージ） |
| `WP-TEST-01` | 依頼を手で試す道具（テスト用の端末のコマンド） | `WP-LOC-02` | **ユーザーの確認待ち**（[Spec-tester.md](../spec/Spec-tester.md) §3） |

先行試作は `../../lifting-lug-pose` の `9265d2d`。**試作のリポジトリは読むだけで、書き換えない。**

---

## `WP-ENV-01` 土台

**読む節**: [-core.md](DetailedDesign-core.md) §1、[-locate.md](DetailedDesign-locate.md) §2〜§4、[-names.md](DetailedDesign-names.md) §2〜§5。

| やること | 中身 |
| --- | --- |
| Docker | `docker/Dockerfile`。元は `ros:humble`（Ubuntu 22.04）。ultralytics `8.4.194`（試作と同じ版）と、PyTorch の **CPU 版**、pytest を入れる。入らない版があれば入る版にして、パケットの報告に書く |
| 型 | `ss_ws/src/ss_msgs` に [-locate.md](DetailedDesign-locate.md) §2〜§4 の 4 つの型 |
| パッケージ | `ss_ws/src/ss_locator`（ament_python）。中身は空のモジュールと `tests/` |
| ライセンス | リポジトリ直下に `LICENSE`（AGPL-3.0 の全文）（[Spec.md](../spec/Spec.md) §1.2） |
| git から外す | `.gitignore` に `model/`（[-names.md](DetailedDesign-names.md) §4） |
| 入口 | `scripts/test.sh`: コンテナの中で `colcon build` と `pytest` を回す |

**受け入れ条件**:

```bash
docker build -t ss-system:dev -f docker/Dockerfile .
docker run --rm -v "$PWD":/ws ss-system:dev bash scripts/test.sh          # 0 で終わる
docker run --rm -v "$PWD":/ws ss-system:dev bash -c \
  "cd /ws/ss_ws && colcon build && . install/setup.bash && ros2 interface show ss_msgs/action/LocatePanels"
                                                                          # -locate.md §2.1・§2.2 のフィールドが並ぶ
docker run --rm ss-system:dev python3 -c "import torch; assert not torch.cuda.is_available()"
git check-ignore model/x.pt                                               # model/x.pt と出る
```

## `WP-EST-01` 推定の移植

**読む節**: [-core.md](DetailedDesign-core.md) §0〜§2。

| やること | 中身 |
| --- | --- |
| 写す | 試作の `src/common.py` の `undistort_rays`（と、それが使うもの）→ `geometry.py`。`src/assemble_lugs.py` の `lift`・`fuse`・`group`・`fit` と、それが使う関数・定数 → `assemble.py`。**先頭に元のファイル名と `9265d2d` を書く** |
| 直す | 天板の高さを定数 `PANEL_H` から引数 `top_height_m` にする。`orient_rows` は写さない |
| 足す | `geometry.camera_T`（[-core.md](DetailedDesign-core.md) §2） |
| 試験の材料 | 試作の環境（Windows の `.venv`）で、`robot_v3_test` の 10 シーンについて、カメラの `K`・`dist`・`T`・検出（試作の `results/lug_v3_on_v3_test/preds.json`）と、**試作の `assemble` から `orient_rows` を抜いた出力**を JSON に書き出し、`ss_locator/tests/fixtures/proto_9265d2d.json` に置く。書き出す道具は `tools/make_proto_fixture.py`（試作の環境で回す。このリポジトリに置く） |

**受け入れ条件**（`scripts/test.sh` の中で回る）:

| 試験 | 確かめること |
| --- | --- |
| `test_assemble_matches_proto` | 材料の 10 シーンで、盤の数・中心（1 mm 以内）・正面の向き（0.01° 以内）・幅・奥行（1 mm 以内）・金具の数が試作と同じ |
| `test_camera_T_roundtrip` | `camera_T` で作った `T` で `base_link` の点を射影し、`undistort_rays` で戻すと、その点の水平面との交点が元の点に 1 mm 以内で戻る（ヨー −π〜π・見下ろし 0〜40° の格子） |
| `test_camera_T_axes` | ヨー 0・見下ろし 0 で、カメラの z が `base_link` の +x、x が −y、y が −z。ヨー +90° で z が +y |
| `test_top_height_param` | `top_height_m` を変えると、根元の z の平面が変わる（2.35 と 2.0 で結果が違い、2.0 では z = 2.0 の平面に乗る） |

## `WP-DET-01` 検出と CPU の処理時間

**読む節**: [-core.md](DetailedDesign-core.md) §0・§1、[-names.md](DetailedDesign-names.md) §4・§5。

| やること | 中身 |
| --- | --- |
| 検出 | `detector.py`。`model/` の重みを `model_sha256` と照合してから読む。違えば読まずに止まる。`imgsz` 1280・確信度 `detect_conf` で 1 枚を検出し、キーポイント (N,4,2) と確信度 (N,) を返す |
| 時間 | `tools/measure_cpu_time.py`。コンテナの中の CPU で、`robot_v3_test` の画像 50 枚の 1 枚あたりの時間（最初の 1 枚を除いた中央値・90%）を測り、`docs/plan/source/cpu-time.md` に機種・CPU・版・結果を書く |

**受け入れ条件**:

```bash
sha256sum model/lug_v3_s.pt                  # model_sha256 と同じ
docker run --rm -v "$PWD":/ws -v <試作>/data:/proto_data:ro ss-system:dev \
  python3 /ws/tools/measure_cpu_time.py --images /proto_data/synthetic/robot_v3_test --n 50
```

| 試験 | 確かめること |
| --- | --- |
| `test_detector_matches_proto` | 試作の `preds.json` の 5 枚で、確信度 0.25 以上の検出の数が同じで、キーポイントが 1 px 以内（CPU と GPU の差の分） |
| `test_detector_rejects_bad_hash` | 重みの SHA-256 が違うと例外を出し、モデルを読まない |

**結果は [Spec-open.md](../spec/Spec-open.md) `SS-V3` に記録し、目標の時間を fk-system と決める材料にする。**遅すぎれば、CPU 向けの形式への変換を別パケットに切る。

## `WP-EST-02` 返す盤への変換・走査の計画

**読む節**: [-core.md](DetailedDesign-core.md) §3・§4、[-locate.md](DetailedDesign-locate.md) §1・§2.3。

| やること | 中身 |
| --- | --- |
| `panels.py` | [-core.md](DetailedDesign-core.md) §4 の表のとおり。範囲と距離での絞り込みも |
| `scan_plan.py` | [-core.md](DetailedDesign-core.md) §3 のとおり。角度の正規化（−π〜π） |
| 確からしさの当てはまり | `tools/check_std.py`: 材料（`WP-EST-01`）と `robot_v3_test` の真値で、位置の誤差 ÷ `pos_std_m`、向きの誤差 ÷ `yaw_std_deg` の分布を出し、`docs/plan/source/std-check.md` に書く。**合わなければ式を変えず報告する**（式を変えるのは設計書の側） |

| 試験 | 確かめること |
| --- | --- |
| `test_base_edge_faces_robot` | 機体の前 3 m・左 3 m・後ろ 3 m に、向きを 0°・30°・90° に変えた矩形を置くと、`yaw_rad` が中心から原点へ向かう向きと 45° 以内で、`width_m` がその辺の長さ |
| `test_width_depth_swap` | 同じ矩形を 90° 回すと、`width_m` と `depth_m` が入れ替わらない（機体側の辺の長さが幅のまま） |
| `test_filter_range_distance` | 範囲の外・`max_distance_m` より遠い盤を返さない。0 をまたぐ範囲・全周でも |
| `test_scan_yaws` | 幅が `hfov` 以下で 1 枚。幅 90°・`hfov` 60°・重なり 10% で、両端が範囲に合い、隣どうしの間隔が `step` 以下。全周で等間隔 |
| `test_seen_ranges` | 撮れた画像だけから見た方向の範囲を作り、重なりをまとめ、依頼の範囲で切る |
| `test_no_lugs_vs_ungrouped` | 金具が 0 個なら `no_lugs`、1 個以上で盤が 0 面なら `lugs_ungrouped` |
| `test_diagonal_pair` | 対角の金具 2 個の盤を返し、`yaw_std_deg` が 90・`front_known` が false・中心が 2 個の中点 |

## `WP-LOC-01` 依頼の流れ（純関数）と偽の hve

**読む節**: [-core.md](DetailedDesign-core.md) §5・§6、[-locate.md](DetailedDesign-locate.md) §2・§3、[-names.md](DetailedDesign-names.md) §3・§5。

| やること | 中身 |
| --- | --- |
| `locate_core.py` | [-core.md](DetailedDesign-core.md) §5 の状態遷移。ROS 2・ultralytics を読み込まない。**時刻と出来事を受けて、命令の並びを返すだけ**（下の表）。画像の検出と金具を床に戻す処理は呼ばない（つなぐ層が行い、結果の金具を出来事として渡す） |
| `fakes.py` | 偽の昇降機・偽のカメラ・偽の検出器（下の表）。純 Python |

**`locate_core` の出来事と命令**（名前は Python の型名。ここに無いものは作らない）:

| 向き | 名前 | 中身 |
| --- | --- | --- |
| 出来事 | `GoalRequest` | 依頼 4 つの値。受け付けの判定（[-locate.md](DetailedDesign-locate.md) §2.1）は `accept(goal, now)` が返す（受けた／拒否の理由） |
| 出来事 | `Cancel` | 取り消し |
| 出来事 | `LiftState` | 昇降部の高さ（配信値）・目標に着いたか・止まった理由の種類（`none`／`ceiling`（天井などで上がれない）／`refused`／`human`） |
| 出来事 | `CameraState` | 今のヨー・ピッチの目標角、再起動した印 |
| 出来事 | `FrameDone` | 撮って検出し床に戻した結果: そのときのヨーと金具の観測（`assemble.lift` の出力）。失敗なら `ok: false` |
| 出来事 | `Tick` | 時刻だけ |
| 命令 | `LiftGoto(height_m)`・`LiftRelease` | 昇降部へ |
| 命令 | `CameraPoint(yaw_rad, pitch_down_rad)`・`CameraRelease`・`RequestFrame` | カメラモジュールへ（`RequestFrame` は向けて `settle_s` 待ったあと） |
| 命令 | `SendFeedback(phase)`・`SendResult(reason, panels, seen_from, seen_span)`・`SendCleanupDone(reason, lift_restored)` | 依頼する側へ。`panels` は `panels.to_panel_top` の dict |

**偽の hve と検出器**（`fakes.py`）:

| 名前 | 中身 |
| --- | --- |
| `FakeLift` | 目標の高さへ一定の速さ（既定 0.2 m/s）で動き、着いたら `arrived`。上限の高さ（既定 なし）を超える目標では上限で止まり `ceiling`。`refuse=True` なら `refused`。`take_over()` で `human` |
| `FakeCamera` | 材料 `proto_9265d2d.json` の 1 シーンを持つ。`point(yaw, pitch)` を受け、`capture()` で**そのシーンの視点のうち、カメラの向き（`T` の z 軸の方位）がヨーに最も近いもの**を返す（`K`・`dist`・`T`・視点の番号）。`fail=True` なら返さない。`restart()` で再起動の印を立て、ピッチの目標角を 0 にする |
| `FakeDetector` | 視点の番号を受け、材料のその視点の検出（`kp`・`conf`）を返す |

偽の外部パラメータは材料の `T` をそのまま使う（`base_link` = 試作のワールド）。**本物のカメラでは `geometry.camera_T` で作る。偽のときだけ材料の `T` を使う分岐は、つなぐ層（`WP-LOC-02`）の 1 か所に置く。**

| 試験 | 確かめること |
| --- | --- |
| `test_core_reject_*` | 拒否の 4 種（`bad_range`・`bad_distance`・`bad_time_limit`・`busy`（実行中と後片付け中の両方）） |
| `test_core_reason_*` | 9 種の `reason` それぞれへ至る道筋（偽の昇降機・カメラ・検出器と組んで回す） |
| `test_core_partial` | 時間切れ・取り消し・人の割り込みでも、それまでに撮った分の盤と見た範囲を返す |
| `test_core_cleanup_restores` | 後片付けで昇降機の高さ・ヨー・ピッチを受け付け時の値へ戻す命令を出し、`SendCleanupDone` は `SendResult` のあとに 1 回だけ |
| `test_core_human_no_restore` | `human_override` では `LiftGoto`・`CameraPoint` を出さずに手を離す |
| `test_core_cleanup_timeout` | 後片付けが `cleanup_timeout_s` を過ぎたら手を離し `lift_restored: false` |
| `test_core_restart_discards` | 再起動の印のあいだに来た `FrameDone` を捨て、そのヨーを撮り直す |
| `test_core_end_to_end_fixture` | 材料の 1 シーンで全周を依頼し、`completed` と 1 面以上の盤が返る |

**変異チェック（実装管理担当が自分で回す）**: `locate_core.py` の中で (1) `SendCleanupDone` を `SendResult` より先に出す、(2) `human_override` で昇降機を戻す、(3) 後片付けでヨーを戻す命令を外す、(4) 後片付け中に依頼を受け付ける、のそれぞれで試験が落ちること。

## `WP-LOC-02` ノード（ROS 2 の配線）

**読む節**: [-locate.md](DetailedDesign-locate.md) 全部、[-core.md](DetailedDesign-core.md) §1・§5、[-names.md](DetailedDesign-names.md) §3・§5。

| やること | 中身 |
| --- | --- |
| `locator_node.py` | アクション `/ss/locate/panels`・`/ss/locate/cleanup_done`・`/ss/locate/status` と `locate_core` の配線。`locate_core` の命令を実行し、出来事を渡す。`use_fakes` のときは `fakes.py` を使う（本物の hve は `WP-HVE-01`）。撮った画像の検出 → `assemble.lift` はここで行う |
| `config/locator.yaml`・`setup.py` | [-names.md](DetailedDesign-names.md) §3・§5 のパラメータを全部入れ、`ros2 run ss_locator locator_node` で起動できるようにする |

| 試験 | 確かめること |
| --- | --- |
| `test_node_cleanup_after_result` | ノードを起動し、依頼して、**結果より先に `cleanup_done` が来ない**・**受け付けた依頼に 1 回だけ来る** |
| `test_node_busy_reject` | 実行中・後片付け中の依頼は拒否され、`last_reject_reason` が `busy` |
| `test_node_status_period` | `status` が `status_period_ms` ごとに来る |

**受け入れ条件**（自分の手でも）:

```bash
docker run --rm -v "$PWD":/ws ss-system:dev bash scripts/test.sh
# 2 つ目の端末で同じコンテナに入り:
ros2 run ss_locator locator_node --ros-args -p use_fakes:=true
ros2 action send_goal /ss/locate/panels ss_msgs/action/LocatePanels   "{azimuth_from_rad: -3.14, azimuth_span_rad: 6.28, max_distance_m: 0.0, time_limit_s: 120.0}" --feedback
ros2 topic echo /ss/locate/cleanup_done
```

**変異チェック（実装管理担当が自分で回す）**: `locator_node.py` の中で (1) `cleanup_done` を出す呼び出しを結果を返す前に移す、(2) 実行中でも依頼を受け付ける、のそれぞれで試験が落ちること。

## `WP-EST-03` 位置の大きな誤差の出どころ（調べるだけ）

**読む節**: [source/std-check.md](../source/std-check.md)、[-core.md](DetailedDesign-core.md) §4。

`tools/check_std.py` の結果のうち、位置の誤差 ÷ `pos_std_m` が 2 を超える盤を、金具の数（2／3／4）・対角か・正面が分かるか・カメラからの距離・誤差の向き（基準の辺に沿う／直交）で分けて数え、`docs/plan/source/std-check.md` に足す。**式は変えない。**結果を見て、確からしさの式を直すかを決める（[-open.md](DetailedDesign-open.md) §1 の 5）。

## `WP-HVE-01` hve-system の昇降部とつなぐ

**状態: 2026-11-15 案。同日ユーザーが着手を決めた**（hve と話し合って接続部を作る）。**このパケットでは実機の昇降部に繋がない。**

**読む節**: [-hve.md](DetailedDesign-hve.md) §0〜§5、[-core.md](DetailedDesign-core.md) §5、[-names.md](DetailedDesign-names.md) §5・§7。相手の形は hve [DetailedDesign-protocol-modules.md](../../../../hve-system/docs/plan/detailed/DetailedDesign-protocol-modules.md) §1〜§3 と hve [DetailedDesign-protocol.md](../../../../hve-system/docs/plan/detailed/DetailedDesign-protocol.md) §2.2（`state` の今の欄）。**hve のリポジトリは読むだけで、書き換えない。**

| やること | 中身 |
| --- | --- |
| `hve_lift_core.py` | `LiftLink`・`hello_message`（[-hve.md](DetailedDesign-hve.md) §2〜§4）。WebSocket・ROS 2 を読み込まない |
| `hve_lift.py` | `HveLift`（[-hve.md](DetailedDesign-hve.md) §5） |
| `locate_core.py` | `LiftState.height_m` の `None` を受ける（[-hve.md](DetailedDesign-hve.md) §3）。`stopped_reason` の `ceiling_unknown` を `ceiling` と同じに扱う（理由を分けて返すのは `WP-LOC-03`）。ほかは変えない |
| `locator_node.py`・`config/locator.yaml` | [-names.md](DetailedDesign-names.md) §7 のパラメータ 4 つと、`hve_lift` が true のときの差し替え |
| Docker | `docker/Dockerfile` に `websockets` を足す。足した版をパケットの報告に書く |

**受け入れ条件**: `docker run --rm -v "$PWD":/ws ss-system:dev bash scripts/test.sh` が 0 で終わる（今までの試験も通る）。**実時間で待つ試験は、小さなサーバを相手にする `test_hve_lift.py` だけにし、純関数の試験は時刻を引数で渡す。**

| 試験 | 確かめること |
| --- | --- |
| `test_hello_fields` | `hello` が [-hve.md](DetailedDesign-hve.md) §2.1 の値で、`busy_label` が UTF-8 で 48 バイト以下。`ceiling_sensor` は false |
| `test_goto_repeats_same_press` | `LiftGoto` のあと、`poll` が `lift_cmd_period_ms` ごとに同じ `press`・`target_mm`・`speed` の `goto` を返し、`ceiling` を持たない。周期の途中では返さない |
| `test_new_goto_increments_press` | 2 つ目の `LiftGoto` で `press` が増える。繋ぎ直しても戻らない |
| `test_arrived_needs_same_goto` | `state.goto` が `ARRIVED` でも、`by`・`press`・`target_mm` のどれかが違えば `arrived` は false。**前の `press` で着いたままの `state` を、次の `LiftGoto` の直後に渡しても false** |
| `test_stops_sending_after_arrived` | 着いたあと `poll` は `goto` を返さない |
| `test_human_takeover_stops_sending` | [-hve.md](DetailedDesign-hve.md) §4 の 1・2・3 のそれぞれで `stopped_reason` が `human` になり、**その次の `poll` から `goto` を返さない**。3 は着いたあと（送っていない間）でも |
| `test_idle_ignores_owner` | 命令が無い間は、持ち主が人でも `stopped_reason` は `none` |
| `test_reason_table` | [-hve.md](DetailedDesign-hve.md) §4 の 4・4a・5・6 の名前を 1 つずつ（`CEILING_STALE` は `ceiling_unknown`）。知らない名前は `refused`。上昇の `BOTTOM` は `refused` |
| `test_speed_from_settings` | `set_speeds(30, 20)` のあと、上昇の `goto` の `speed` が 30、下降が 20。**`set_speeds` の前の `LiftGoto` は `goto` を返さず `refused`** |
| `test_max_run_retries` | `MAX_RUN` で `press` が増えた `goto` を返し続け、`LIFT_MAX_RUN_RETRY` 回を超えると `refused` |
| `test_target_range_down_refused` | 下降の `TARGET_RANGE` は `refused`（上昇は `ceiling`） |
| `test_hve_lift_reads_settings` | 小さなサーバの `/api/settings` の `lift_up`・`lift_down` の `init` を読み、`min`〜`max` に収めた値が `goto` の `speed` に入る。設定が取れないと `goto` が届かない |
| `test_height_unknown` | `height_ok` が false・`state` をまだ受けていない・`lift_state_timeout_ms` より古い、のそれぞれで `height_m` が `None` |
| `test_stale_requests_reconnect` | 最後の `state` から `lift_state_timeout_ms` を過ぎると `poll` が「繋ぎ直す」を返し、命令の途中なら `refused`。繋ぎ直して新しい `state` が来ても、その命令の `goto` は返さない |
| `test_release_once` | `LiftRelease` で `release` が 1 回だけ出て、`press` は最後の `goto` と同じ。命令が無ければ出ない |
| `test_core_height_none_refused` | `locate_core`: 受け付けのとき `height_m` が `None` なら、`LiftGoto` を出さずに `lift_refused`・`lift_restored: false` |
| `test_hve_lift_hello_first` | 小さなサーバに繋ぐと、最初に届く文が `hello`。切って繋ぎ直させても、また `hello` から |
| `test_hve_lift_goto_and_release` | `apply(LiftGoto)` で `goto` が続けて届き、サーバが `ARRIVED` を送ると `advance` が `arrived` を返して `goto` が止まる。`apply(LiftRelease)` で `release` が届く |
| `test_hve_lift_reconnects_when_silent` | サーバが `state` を送るのをやめると、`HveLift` が自分で閉じて繋ぎ直す |
| `test_node_uses_hve_lift` | `use_fakes`・`hve_lift` を true にしてノードを起こし、小さなサーバを昇降部に見立てて依頼すると、`goto` が届き、サーバが人の持ち主を送ると結果が `human_override` で、**そのあと `goto` が届かない** |

**変異チェック（実装管理担当が自分で回す）**: (1) `hve_lift_core.py` で `press` を照らさずに `ARRIVED` を着いたとみなす、(2) 人の割り込みのあとも `goto` を返し続ける、(3) [-hve.md](DetailedDesign-hve.md) §4 の 3 を `refused` にする、(4) `hve_lift.py` で `poll` が返した文を送らない、(5) `locator_node.py` で `hve_lift` が true でも `FakeLift` を使う、のそれぞれで試験が落ちること。

**実装管理担当が手で確かめる**（hve の偽の昇降部を相手に。hve の道具の動かし方は hve の `docs/使い方.md` に従う）:

```bash
# hve のリポジトリで偽の昇降部を起こす（ポートは例）
python tools/fake_lift_server.py --port 18080
# ss のコンテナ（--network host）の中で
ros2 run ss_locator locator_node --ros-args -p use_fakes:=true -p hve_lift:=true \
  -p lift_host:=127.0.0.1:18080 -p camera_height_offset_m:=1.9
ros2 action send_goal /ss/locate/panels ss_msgs/action/LocatePanels \
  "{azimuth_from_rad: -3.14, azimuth_span_rad: 6.28, max_distance_m: 0.0, time_limit_s: 120.0}" --feedback
```

| 場面 | 見ること |
| --- | --- |
| 天井の値の出どころが繋がっていない | 昇降機が上がらず、結果が `camera_too_low`（ログに `CEILING_STALE`） |
| hve の `python tools/lift_probe.py ws://127.0.0.1:18080/ws/module --name hve-cam` を動かし、`c` キーで天井を遠い値にした（2026-11-16 hve の答え。キーを押さなければ持ち主にならない） | 上がって `completed`、終わると元の高さへ戻り、`cleanup_done` の `lift_restored` が true |
| 上がっている途中に、偽の昇降部の画面から人が操作した | すぐ `goto` が止まり、結果が `human_override`、昇降機は戻らない |

`camera_height_offset_m` の 1.9 は、この確認のためだけの値（hve「下端のとき機体は約 1900 mm」から。光学中心の実測値ではない）。

## `WP-HVE-02` カメラモジュールとつなぐ（純関数と `locate_core` の直し）

**状態: 2026-11-16 案。**`WP-LOC-03` のあとに着手する（同じ `locate_core.py` を触るため）。

**読む節**: [-hve.md](DetailedDesign-hve.md) §7.0〜§7.3、[-core.md](DetailedDesign-core.md) §5、[-names.md](DetailedDesign-names.md) §10。相手の形は hve [DetailedDesign-protocol-ss.md](../../../../hve-system/docs/plan/detailed/DetailedDesign-protocol-ss.md)（読むだけ）。

| やること | 中身 |
| --- | --- |
| `hve_camera_core.py` | `CameraLink`・`frame_verdict`・`camera_hello_message`（[-hve.md](DetailedDesign-hve.md) §7.1・§7.2）。WebSocket・HTTP・ROS 2 を読み込まない |
| `locate_core.py` | [-hve.md](DetailedDesign-hve.md) §7.3 の表 |
| `fakes.py` | `FakeCamera` が `arrived` を返す。`take_over()`・`fail_aim` を足す（試験用） |
| `locator_node.py`・`config/locator.yaml` | `aim_timeout_s` |

| 試験 | 確かめること |
| --- | --- |
| `test_camera_hello_fields` | `name` が `ss`・`operator` が `auto`・`ceiling_sensor` の欄が無い |
| `test_aim_converts_angles` | `CameraPoint(radians(90), radians(20))` で `aim` の `yaw_deg` が 90・**`pitch_deg` が −20**。ヨー 190° は −170° に直る |
| `test_aim_without_yaw` | `yaw_rad` が `None` なら `aim` に `yaw_deg` が無く、`pitch_deg` はある |
| `test_camera_state_converts_back` | `state` の `pitch_deg` −20・`yaw_deg` 88.5 が、見下ろし 20°・ヨー 88.5° になる。`yaw_deg` が `null` なら `yaw_rad` は `None` |
| `test_camera_arrived_needs_same_aim` | `press` が違う `ARRIVED` は着いたとみなさない |
| `test_camera_taken_over` | [-hve.md](DetailedDesign-hve.md) §7.1 の 1 の 3 つそれぞれで `taken_over` になり、次の `poll` から `aim` を返さない。着いたあとでも |
| `test_camera_pose_unknown` | 2 の 3 つそれぞれで `pose_known` が false |
| `test_camera_failed` | 3 の名前を 1 つずつと、知らない名前・`state` が古い |
| `test_frame_verdict_table` | [-hve.md](DetailedDesign-hve.md) §7.2 の表の行を 1 つずつ。**`X-Still-Ms` が `X-Frame-Age-Ms` より小さいと `retry`、等しい・大きいと `use`** |
| `test_core_settle_after_arrived` | `CameraPoint` のあと、`arrived` が来るまで `RequestFrame` を出さない。`arrived` から `settle_s` たって初めて出す |
| `test_core_aim_timeout` | `aim_timeout_s` たっても着かなければ `no_image` |
| `test_core_camera_taken_over` | 走査の途中でカメラを取って代わられると `human_override`。昇降機もカメラも戻さない |
| `test_core_camera_failed` | `failed` で `no_image`。後片付けはする |
| `test_core_cleanup_waits_camera` | 昇降機が先に着いても、カメラが着くまで `SendCleanupDone` を出さない |
| `test_core_cleanup_limit_includes_yaw` | 昇降機の分 20 s・ヨーを戻す分 36 s（180° を 5°/s）・余裕 10 s で `cleanup_limit_s` が 46 |

**変異チェック（実装管理担当が自分で回す）**: (1) ピッチの符号を逆にしない、(2) `arrived` を待たずに `settle_s` を数える、(3) カメラを取って代わられても続ける、(4) 後片付けでカメラを待たない、(5) `X-Still-Ms` の比べ方を逆にする、のそれぞれで試験が落ちること。

## `WP-HVE-03` カメラモジュールとつなぐ（つなぐ層と、ノードの本物の道）

**状態: 2026-11-16 案。**

**読む節**: [-hve.md](DetailedDesign-hve.md) §5・§7.4、[-core.md](DetailedDesign-core.md) §2、[-names.md](DetailedDesign-names.md) §5・§10。

| やること | 中身 |
| --- | --- |
| `hve_camera.py` | `HveCamera`（[-hve.md](DetailedDesign-hve.md) §7.4） |
| `locator_node.py`・`config/locator.yaml` | `use_fakes` が false のときの本物の道（`HveLift`・`HveCamera`・`Detector`・`geometry.camera_T`）。パラメータ 4 つ |
| `tests/camera_stub_server.py` | 試験の中だけで使う小さなサーバ。`/ws/module` で受けた文を覚えて `state` を送り、`/api/snapshot` で渡された JPEG とヘッダを返す |

| 試験 | 確かめること |
| --- | --- |
| `test_hve_camera_hello_first` | 繋ぐと最初に `hello`。繋ぎ直してもまた `hello` から |
| `test_hve_camera_aim_and_release` | `point` で `aim` が続けて届き、`ARRIVED` で止まる。`release` が届く |
| `test_hve_camera_capture_uses_headers` | 返った向きが、`state` ではなく応答ヘッダの値（ピッチは符号を逆に） |
| `test_hve_camera_capture_retries` | 1 回目が 503、2 回目が動いている間のコマ、3 回目が使えるコマのとき、3 回目を返す。取り直しの間隔が `snapshot_min_interval_ms` 以上 |
| `test_hve_camera_capture_times_out` | 503 が続くと `image_timeout_s` で失敗を返す |
| `test_hve_camera_host_from_lift` | `camera_host` が空のとき、昇降部の `state.modules` の `hve-cam` の `ip` に繋ぐ |
| `test_node_real_path_uses_camera_T` | `use_fakes` を false にし、小さなサーバ 2 つ（昇降部・カメラ）と偽の検出器で依頼すると、金具を床に戻すのに使った外部パラメータが、応答のヨー・ピッチと昇降部の高さから `geometry.camera_T` で作ったものと同じ |
| `test_node_real_path_no_camera` | カメラモジュールに繋がらないと、動かさずに `camera_pose_unknown` |

**受け入れ条件**: `scripts/test.sh` が 0 で終わる。**実装管理担当が手で確かめるのは、hve の偽のカメラモジュールが新しい口に対応してから**（hve の知らせ待ち）。モデルを使う本物の検出は、`model/` を置いた本体で実装管理担当が回す。

**変異チェック（実装管理担当が自分で回す）**: (1) 外部パラメータに `state` の向きを使う、(2) 取り直しの間隔を空けない、(3) `use_fakes` が false でも偽のカメラを使う、のそれぞれで試験が落ちること。

## `WP-CAL-01` カメラの内部パラメータの校正（待ち）

hve のカメラで 1280×720 のチェッカーボードの静止画が撮れるようになったら書く。OpenCV で `K`・`k1`・`k2` を求め、パラメータファイルに書く。

## `WP-DEMO-01` 金具認識のリアルタイム表示（本番外の道具）

**読む節**: [-demo.md](DetailedDesign-demo.md) 全部、[-names.md](DetailedDesign-names.md) §5・§6。

| やること | 中身 |
| --- | --- |
| 検出 | `detector.py` に `detect_with_boxes` を足し、`detect` をそれの最初の 2 つを返す形にする（`detect` の結果は変えない） |
| 純関数 | `demo_view.py`（[-demo.md](DetailedDesign-demo.md) §2） |
| つなぐ | `lug_demo.py`（§1・§3）と、`setup.py` の `lug_demo` |
| Docker | `cv2.imshow` がコンテナから X に出せるよう、要る apt パッケージと、画面の無いところで画面ありの経路を試すための `xvfb` を `docker/Dockerfile` に足す。**OpenCV の wheel は 1 つだけにする**（`opencv-python-headless` が並ぶと `imshow` が黙って動かない）。足したものをパケットの報告に書く |
| 試験 | `tests/test_demo_view.py`・`tests/test_lug_demo.py`（下の表）。偽の検出器とメモリの中のコマで回す。**学習済みモデルは使わない**（`main` を通す試験では `Detector` を偽物に差し替え、SHA-256 の照合に当てない）。**時間に頼る試験を作らない**（実時間の待ちではなく、偽の検出器が呼ばれるたびに入力へコマを足す形で捨て方を確かめる） |

**受け入れ条件**:

```bash
docker build -t ss-system:dev -f docker/Dockerfile .
docker run --rm -v "$PWD":/ws ss-system:dev bash scripts/test.sh          # 0 で終わる（今までの試験も通る）
# 合成のフォルダを --headless で回す（モデルがあるときだけ。行が 3 行出て 0 で終わる）
docker run --rm -v "$PWD":/ws ss-system:dev bash -c \
  "cd /ws/ss_ws && colcon build && . install/setup.bash && cd /ws && \
   mkdir -p .briefs/tmp/demo && python3 -c 'import cv2,numpy as n; [cv2.imwrite(f\".briefs/tmp/demo/{i}.png\", n.zeros((480,640,3),n.uint8)) for i in range(3)]' && \
   ros2 run ss_locator lug_demo --source .briefs/tmp/demo --headless"
# 画面ありの経路（Xvfb の上で。モデルがあるときだけ。0 で終わる）
docker run --rm -v "$PWD":/ws ss-system:dev bash -c \
  "cd /ws/ss_ws && colcon build && . install/setup.bash && cd /ws && \
   xvfb-run -a ros2 run ss_locator lug_demo --source .briefs/tmp/demo --max-frames 3"
docker run --rm ss-system:dev python3 -c \
  "import cv2; assert 'GTK' in cv2.getBuildInformation() or 'QT' in cv2.getBuildInformation().upper()"
docker run --rm ss-system:dev bash -c "python3 -m pip list 2>/dev/null | grep -ci '^opencv'"   # 1
```

**モデルが要る 2 つのコマンドは、実装管理担当が回す**（`model/` は git に入らず、エージェントの worktree には無い。`model/lug_v3_s.pt` を `model_sha256` と照合して置いてから）。

| 試験 | 確かめること |
| --- | --- |
| `test_fit_to_canvas_sizes` | 640×480 → 倍率 1.5・余白 x 160 ／ 1920×1080 → 倍率 2/3・余白なし ／ 1280×720 → そのまま ／ 縦長 720×1280 → 余白 x。どれも出力は 1280×720 |
| `test_fit_to_canvas_keeps_aspect` | 入力に描いた正方形が、出力でも縦横の差 1 px 以内の正方形 |
| `test_draw_lugs_does_not_modify_input` | `draw_lugs` は渡した `canvas` を書き換えない。金具 0 個でも描ける |
| `test_draw_lugs_draws_marks` | 一様な灰色の `canvas` に金具 1 個を描くと、枠の辺の上・キーポイント 4 点の位置・左上の `info` の領域の画素が入力と違う（**何も描かずに写しを返す変異を落とす**） |
| `test_latest_frame_keeps_newest` | `put` を 1・2・3 の順 → `take` は 3、`dropped` は 2。空の `take` は `timeout_s` 後に `None`。`close` 後は `None` |
| `test_run_demo_skips_stale_frames` | 偽の検出器が呼ばれるたびに入力へコマを 3 つ足す形で、推定したコマの通し番号が増える一方で飛びがあり（推定したのは足した 3 つのうち最後のもの）、`dropped` が 0 より大きい |
| `test_run_demo_draws_on_inferred_frame` | 偽の検出器が受け取ったコマと、`show` に渡った画像の下地（描いていない部分）が同じコマ |
| `test_folder_source_in_order_no_drop` | フォルダの画像（大文字の拡張子・画像でないファイルを混ぜる）を名前順に全部推定し、画像でないものは読まない。`dropped` は 0 |
| `test_conf_reaches_detector` | `main` に `--conf 0.6` を渡すと `Detector` の `conf` が 0.6。渡さないと `locator.yaml` の `detect_conf` と同じ |
| `test_main_writes_no_files` | 一時ディレクトリを作業場所にしてフォルダを `--headless` で回すと、一時ディレクトリにも入力のフォルダにも新しいファイルができない |
| `test_detect_with_boxes_matches_detect` | 偽の YOLO の結果で、`detect` の結果が `detect_with_boxes` の最初の 2 つと同じ。検出 0 個で枠は (0,4) |

## `WP-DEMO-02` 金具認識のリアルタイム表示の起動スクリプト

**読む節**: [-demo.md](DetailedDesign-demo.md) §4・§4.1、[-names.md](DetailedDesign-names.md) §6。

| やること | 中身 |
| --- | --- |
| スクリプト | `scripts/demo.sh`（実行権限を付ける） |
| 試験 | `ss_ws/src/ss_locator/tests/test_demo_script.py`。`--dry-run` で `subprocess` から呼び、出た引数を確かめる（下の表）。**`docker` も `xhost` も呼ばない**ので、コンテナの中の `scripts/test.sh` で回る。`/dev/video<N>` の有無はスクリプトの外から差し替えられるようにする（環境変数 `SS_DEMO_DEV_DIR`。既定 `/dev`）。この名前は試験のためだけのもの |

**受け入れ条件**:

```bash
docker run --rm -v "$PWD":/ws ss-system:dev bash scripts/test.sh   # 0 で終わる
bash -n scripts/demo.sh && scripts/demo.sh --dry-run --source usb:0 --headless   # 引数が出て 0 で終わる
# 実装管理担当がホストで（モデルを置いて）: 
scripts/demo.sh --source <金具の写っていない画像フォルダ> --headless --max-frames 2   # 2 行出て 0 で終わる
```

| 試験 | 確かめること |
| --- | --- |
| `test_default_source_usb0` | `--source` が無いと `--source usb:0` と `--device /dev/video0` が付く |
| `test_usb_device_index` | `--source usb:2` で `--device /dev/video2`。`/dev/video2` が無いと 1 で終わり、標準出力に `docker run` の引数が出ない |
| `test_folder_mounted_readonly` | 一時フォルダを相対パスで渡すと、その絶対パスが `/input` に `:ro` で mount され、`--source /input` になる。`--device` は付かない |
| `test_video_file_mounts_parent` | ファイルを渡すと親が `/input` に mount され、`--source /input/<ファイル名>` になる |
| `test_missing_path_fails` | 在らないパスで 1 で終わる |
| `test_headless_has_no_display` | `--headless` で `DISPLAY`・`/tmp/.X11-unix` が付かず、`--headless` は `lug_demo` に渡る。付けないと両方付く |
| `test_args_passed_through` | `--conf 0.4 --max-frames 3` がそのまま `lug_demo` に渡り、`--dry-run` は渡らない |
| `test_repo_root_from_anywhere` | 別のフォルダを作業場所にして呼んでも、`/ws` に mount されるのはリポジトリ直下 |

## `WP-DEMO-03` 金具認識のリアルタイム表示に速さと入力の解像度を出す

**読む節**: [-demo.md](DetailedDesign-demo.md) §2・§3、[-names.md](DetailedDesign-names.md) §6。

| やること | 中身 |
| --- | --- |
| 純関数 | `demo_view.py` に `RateMeter` を足し、`LatestFrame` に `received` を足す |
| つなぐ | `lug_demo.py` の `run_demo` に `clock` を足し、左上の行と `show` に渡す結果に速さと解像度を足す。USB・動画の入力に `received` を持たせる |
| 試験 | `tests/test_demo_view.py`・`tests/test_lug_demo.py` に足す（下の表）。**時計は差し替え、実時間で待たない** |

**受け入れ条件**: `docker run --rm -v "$PWD":/ws ss-system:dev bash scripts/test.sh` が 0 で終わる（今までの試験も通る）。実装管理担当がホストで `scripts/demo.sh --source <画像フォルダ> --headless --max-frames 2` を回し、行の形が変わっていないこと。

| 試験 | 確かめること |
| --- | --- |
| `test_rate_meter` | 0.1 秒おきに `tick` すると、1 秒たつまで `None`、そのあと 10.0（±0.5）。`tick` を止めると 1 秒後に 0 |
| `test_latest_frame_counts_received` | `put` を 3 回 → `received` は 3（`take` しても減らない） |
| `test_run_demo_reports_rates` | 偽の時計（推定のたびに 0.25 秒進む）と、推定のたびに入力へ 3 コマ足す偽の検出器で、十分回したあとの結果の `fps` が 4.0（±0.5）、`in_fps` が 12.0（±1） |
| `test_run_demo_reports_src_size` | 640×480 のコマで `src_size` が (640, 480)、左上の行に `src: 640x480` がある（`draw_lugs` に渡った `info` で確かめる） |
| `test_folder_has_no_input_rate` | フォルダの入力で `in_fps` が `None`、左上の行が `in fps: -` |

## `WP-TEST-01` 依頼を手で試す道具

**状態: 2026-11-15 案。着手は [Spec-tester.md](../spec/Spec-tester.md) §3 の確認のあと。**

**読む節**: [-tester.md](DetailedDesign-tester.md) 全部、[-locate.md](DetailedDesign-locate.md) §1〜§4、[-names.md](DetailedDesign-names.md) §3・§8。

| やること | 中身 |
| --- | --- |
| 純関数 | `locate_view.py`（[-tester.md](DetailedDesign-tester.md) §2・§4〜§6） |
| つなぐ | `locate_cli.py`（§3）と、`setup.py` の `locate_cli` |
| 変えないもの | `locator_node.py`・`locate_core.py`・`ss_msgs`。**`locate_cli.py` は `locator_node`・`locate_core` を読み込まない**（口だけを使う） |
| 試験 | `tests/test_locate_view.py`（ROS 2 なし）・`tests/test_locate_cli.py`（`use_fakes` の `locator_node` を同じプロセスで起こして相手にする）。**確認の入力と Ctrl+C は、`run_cli` に渡す関数・印で差し替える**（端末と実際のシグナルに頼らない） |

**受け入れ条件**:

```bash
docker run --rm -v "$PWD":/ws ss-system:dev bash scripts/test.sh          # 0 で終わる（今までの試験も通る）
# 実装管理担当が手で。1 つ目の端末:
ros2 run ss_locator locator_node --ros-args -p use_fakes:=true
# 2 つ目の端末（同じコンテナ）:
ros2 run ss_locator locate_cli --status                                   # 状態が 1 行出て 0
ros2 run ss_locator locate_cli --plot .briefs/tmp/top.png --json .briefs/tmp/result.json
                                                                          # 確認に y → 段・表・後片付けの完了が出て 0。図と JSON ができる
```

| 試験 | 確かめること |
| --- | --- |
| `test_goal_from_args_defaults` | 引数なしで開始 −π・幅 2π・距離 0・制限時間 120 |
| `test_goal_from_args_degrees` | `--from-deg 170 --span-deg 40` で開始が rad の 170°・幅が rad の 40°。`--from-deg 270` は −90° に直る |
| `test_goal_from_args_rejects` | 幅 0・幅 361・距離 −1・制限時間 0 のそれぞれで誤りになる |
| `test_format_result_table` | 盤 2 面の結果で、行が 2 つ、方位・距離が中心から計算した値、角度が度、`reason` の名前が出る |
| `test_format_result_unknown_yaw` | `yaw_std_deg` が 90 の盤の向きσ が `不明`。`front_known` が false の正面は `-` |
| `test_format_result_empty` | 盤 0 面で、表の代わりに理由の説明と `reason` の名前が出る。知らない `reason` でも落ちない |
| `test_format_seen_ranges` | 見た範囲が 2 つのとき両方出る。無ければ `なし` |
| `test_draw_top_view_axes` | 機体の前 3 m の盤だけを描くと、画像の上半分の中央の列の近くの画素が変わり、下半分は盤なしの図と同じ。**左 3 m の盤は左半分**（左右を逆にする変異を落とす） |
| `test_draw_top_view_base_edge` | 基準の辺の中点の画素が、反対の辺の中点より太く描かれている（盤を 180° 回すと入れ替わる） |
| `test_result_to_dict_keeps_values` | 書き出した盤の値が、渡した値と同じ（丸めない） |
| `test_cli_completes` | 偽の hve のノードを相手に `--yes` で回すと、段 → 表 → 後片付けの完了の順に出て 0。**後片付けの完了の行は表より後** |
| `test_cli_confirm_no_sends_nothing` | 確認に `n` と答えると 0 で終わり、ノードの `phase` が `IDLE` のまま（依頼が届いていない） |
| `test_cli_cancel_waits_cleanup` | 途中で取り消しの印を立てると、結果の `reason` が `canceled` で、**後片付けの完了を出してから** 0 で終わる |
| `test_cli_second_interrupt` | 取り消しの印を 2 回立てると、待たずに 130 で終わる |
| `test_cli_no_server` | ノードが無いと、`--wait-s` のあと 3 で終わり、依頼を出さない |
| `test_cli_busy` | 別の依頼の最中に起こすと 4 で終わる |
| `test_cli_status_only` | `--status` で状態を出して 0。依頼を出さない |
| `test_cli_ignores_other_cleanup` | 依頼の ID が違う `cleanup_done` が先に届いても、それで終わらない |
| `test_cli_writes_plot_and_json` | `--plot`・`--json` のファイルができ、PNG が 800×800、JSON の `reason` が結果と同じ。付けなければファイルができない |

**変異チェック（実装管理担当が自分で回す）**: (1) `locate_cli.py` で確認に `n` でも依頼を送る、(2) 取り消したあと後片付けの完了を待たずに終わる、(3) `cleanup_done` の依頼の ID を照らさない、(4) `locate_view.py` で度を rad に直さない、(5) 図の左右を逆にする、のそれぞれで試験が落ちること。

## `WP-LOC-03` 口の版 2

**状態: 2026-11-16 案。同日 fk-system が確かめた**（案のとおり）。**マージしたら fk-system に知らせる**（fk はそのコミットの `ss_msgs` を取り込んで照合する）。

**読む節**: [-locate.md](DetailedDesign-locate.md) §2.2・§2.4・§3・§4（「版 2」と書いた所）、[-core.md](DetailedDesign-core.md) §5、[-hve.md](DetailedDesign-hve.md) §4、[-names.md](DetailedDesign-names.md) §3・§9。

| やること | 中身 |
| --- | --- |
| 型 | `LocatePanels.action` の結果に `float32 cleanup_limit_s` を足す |
| `locate_core.py` | 理由 `ceiling_unknown`・`camera_pose_unknown`（受け付けのときと途中の両方。**再起動で撮り直すのをやめる**）。後片付けの上限を距離 ÷ 速さ ＋ `cleanup_margin_s` にし、`SendResult.cleanup_limit_s` に入れる。速さが `None`・0 以下なら `cleanup_timeout_s`。`cleanup_limit_max_s` を超えたら切り詰める |
| `hve_lift_core.py`・`fakes.py` | `LiftState` に速さを入れる。`FakeCamera.restart()` は `pose_known` を false にする |
| `locator_node.py`・`config/locator.yaml` | `cleanup_limit_s` を結果に入れる。`INTERFACE_VERSION` を `"2"`。`cleanup_margin_s` |

| 試験 | 確かめること |
| --- | --- |
| `test_core_ceiling_unknown` | 上昇が `ceiling_unknown` で止まり高さが足りないと、結果が `ceiling_unknown`（`ceiling` なら `camera_too_low` のまま）。昇降機は戻す |
| `test_core_pose_unknown_at_accept` | 受け付けのとき向きが読めなければ、`LiftGoto`・`CameraPoint` を出さずに `camera_pose_unknown` |
| `test_core_restart_ends_request` | 走査の途中で再起動すると、撮り直さずに `camera_pose_unknown` で終わり、それまでの盤と見た範囲を返す。**後片付けで `CameraPoint` のヨーを戻す命令を出さない**。昇降機は戻す（今の `test_core_restart_discards` を置き換える） |
| `test_core_cleanup_limit_from_speed` | 高さの差 1.2 m・下降 0.03 m/s・余裕 10 s で `cleanup_limit_s` が 50。その時間までは待ち、過ぎたら手を離して `lift_restored: false` |
| `test_core_cleanup_limit_fallback` | 速さが `None`・0 のとき `cleanup_limit_s` は `cleanup_timeout_s` |
| `test_core_cleanup_limit_capped` | 高さの差 1.8 m・下降 0.005 m/s（370 s になる）で、`cleanup_limit_s` が `cleanup_limit_max_s`。その時間を過ぎたら手を離して `lift_restored: false` |
| `test_core_cleanup_limit_zero_when_no_restore` | `human_override` の結果の `cleanup_limit_s` は 0 |
| `test_node_result_has_cleanup_limit` | ノードの結果に `cleanup_limit_s` が入り、`cleanup_done` がその時間のうちに来る |
| `test_node_interface_version` | `status` の `interface_version` が `"2"` |

**変異チェック（実装管理担当が自分で回す）**: (1) 上限を速さに関係なく `cleanup_timeout_s` にする、(2) 再起動のあと撮り直す、(3) 向きが読めないのに `LiftGoto` を出す、(4) `ceiling_unknown` を `camera_too_low` にまとめる、のそれぞれで試験が落ちること。
