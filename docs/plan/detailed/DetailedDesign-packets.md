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
| `WP-LOC-01` | 依頼の流れとノード（偽の hve で動く） | `WP-EST-02`・`WP-DET-01` | できる |
| `WP-HVE-01` | hve-system とつなぐ | `WP-LOC-01` | **待ち**（[-core.md](DetailedDesign-core.md) §6） |
| `WP-CAL-01` | カメラの内部パラメータの校正 | `WP-ENV-01` | **待ち**（hve のカメラで撮ったチェッカーボードの画像） |

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

## `WP-LOC-01` 依頼の流れとノード

**読む節**: [-core.md](DetailedDesign-core.md) §5、[-locate.md](DetailedDesign-locate.md) 全部。

| やること | 中身 |
| --- | --- |
| `locate_core.py` | [-core.md](DetailedDesign-core.md) §5 の状態遷移。ROS 2 を読み込まない |
| `fakes.py` | 偽の昇降機（目標の高さへ一定の速さで動く・上限の高さ・拒否・人の割り込みを起こせる）、偽のカメラ（ヨー・ピッチを受け、`robot_v3_test` の 1 シーンの、そのヨーに最も近い画像と姿勢を返す・画像を返さない・再起動を起こせる） |
| `locator_node.py` | アクション・`cleanup_done`・`status` と `locate_core`・`detector`・`fakes` の配線。起動の引数で偽を選ぶ（本物は `WP-HVE-01`） |

| 試験 | 確かめること |
| --- | --- |
| `test_core_*` | 拒否の 4 種、9 種の `reason` それぞれへ至る道筋、途中で終わってもそれまでの盤を返す、`human_override` で昇降機を戻さない、後片付けの上限、再起動のあいだの画像を捨てる |
| `test_node_cleanup_after_result` | ノードを起動し、依頼して、**結果より先に `cleanup_done` が来ない**・**受け付けた依頼に 1 回だけ来る** |
| `test_node_busy_reject` | 実行中・後片付け中の依頼は拒否され、`last_reject_reason` が `busy` |
| `test_node_status_period` | `status` が `status_period_ms` ごとに来る |

**受け入れ条件**（自分の手でも）:

```bash
docker run --rm -v "$PWD":/ws ss-system:dev bash scripts/test.sh
# 2 つ目の端末で同じコンテナに入り:
ros2 run ss_locator locator_node --ros-args -p use_fakes:=true
ros2 action send_goal /ss/locate/panels ss_msgs/action/LocatePanels \
  "{azimuth_from_rad: -0.8, azimuth_span_rad: 1.6, max_distance_m: 0.0, time_limit_s: 120.0}" --feedback
ros2 topic echo /ss/locate/cleanup_done
```

**変異チェック（実装管理担当が自分で回す）**: `locator_node.py` の中で (1) `cleanup_done` を出す呼び出しを結果を返す前に移す、(2) `human_override` で昇降機を戻す、(3) 実行中でも依頼を受け付ける、のそれぞれで試験が落ちること。

## `WP-HVE-01` hve-system とつなぐ（待ち）

[-core.md](DetailedDesign-core.md) §6 の待ちが解けたら書く。hve の止めた理由と `reason` の対応表を含める。

## `WP-CAL-01` カメラの内部パラメータの校正（待ち）

hve のカメラで 1280×720 のチェッカーボードの静止画が撮れるようになったら書く。OpenCV で `K`・`k1`・`k2` を求め、パラメータファイルに書く。
