# 名前辞書

[DetailedDesign.md](DetailedDesign.md) の詳細。**この文書に無い名前を実装で作ってはいけない。**

> **`DD-1`**: 名前を発明する余地をゼロにする。必要な名前が無いと分かったら、
> **実装する前にこのファイルへ行を足す。**


---

## 0. 命名規則

**案（2026-11-05）。**th-system [DetailedDesign-names.md](../../../../th-system/docs/plan/detailed/DetailedDesign-names.md) §0 に揃え、fk-system [DetailedDesign-names.md](../../../../fk-system/docs/plan/detailed/DetailedDesign-names.md) §0 と同じく名前空間を前に付ける。

| 対象 | 規則 | 例 |
| --- | --- | --- |
| パッケージ | `ss_<領域>` | `ss_msgs` |
| トピック・サービス・アクション | **`/ss/<領域>/<名前>`**（th・fk と同じ ROS 2 のドメインに居るので、`/ss/` を前に付ける） | `/ss/locate/status` |
| 状態 | 大文字スネーク | `RUNNING` |
| 理由 | 小文字スネーク | `camera_too_low` |
| パラメータ | 小文字スネーク。単位を接尾辞に付ける | `status_period_ms` |
| 作業パケット | `WP-<領域>-<番号>` | `WP-LOC-01` |

## 1. 関連システム側の名前（参照のみ・こちらでは発明しない）

結合に関わる相手側の名前の出どころを載せる。**正は相手の文書とコード。名前は写さない。**

| 相手 | 出どころ | 状態 |
| --- | --- | --- |
| hve-system（昇降部） | hve [DetailedDesign-protocol-modules.md](../../../../hve-system/docs/plan/detailed/DetailedDesign-protocol-modules.md) §1〜§4。本システムは名前 `ss`・`operator: auto`・`ceiling_sensor: false`・`busy_label`「画像認識で操作中」で繋ぐ | 2026-11-05 本システムが確かめた。**hve が実機に入れたと知らせるまで、この名前で実装しない・今の昇降部で上昇させない** |
| hve-system（カメラモジュール） | hve がこれから書く（昇降部の口のあと） | 未着 |

## 2. パッケージ

| パッケージ | 役割 |
| --- | --- |
| `ss_msgs` | 本システムの型（§3）。依頼する側が読み取り専用で取り込む（`ss_ws/src/ss_msgs`） |
| `ss_locator` | 依頼を受けて盤の位置を返すノードと推定（[-core.md](DetailedDesign-core.md)。`ss_ws/src/ss_locator`） |

## 3. 盤の位置を探す口（[-locate.md](DetailedDesign-locate.md)。案）

| 名前 | 種類と型 | 向き | QoS・周期 |
| --- | --- | --- | --- |
| `/ss/locate/panels` | アクション `ss_msgs/action/LocatePanels` | 依頼する側 → 本システム | アクションの既定 |
| `/ss/locate/cleanup_done` | トピック `ss_msgs/msg/CleanupDone` | 本システム → 依頼する側 | reliable・transient_local・depth 10。依頼ごとに 1 回 |
| `/ss/locate/status` | トピック `ss_msgs/msg/LocatorStatus` | 本システム → 依頼する側 | reliable・transient_local・depth 1。`status_period_ms` ごと＋変化時 |

| 型 | 中身 |
| --- | --- |
| `LocatePanels.action` | 依頼: `float32 azimuth_from_rad` / `float32 azimuth_span_rad` / `float32 max_distance_m` / `float32 time_limit_s` ― 結果: `std_msgs/Header header` / `string reason` / `PanelTop[] panels` / `float32[] seen_from_rad` / `float32[] seen_span_rad` ― 途中経過: `string phase` |
| `PanelTop.msg` | `float64 x_m` / `float64 y_m` / `float32 top_height_m` / `bool height_measured` / `float64 yaw_rad` / `float32 width_m` / `float32 depth_m` / `float32 pos_std_m` / `float32 yaw_std_deg` / `int32 lugs` / `bool front_known` / `float64 front_yaw_rad` |
| `CleanupDone.msg` | `std_msgs/Header header` / `unique_identifier_msgs/UUID goal_id` / `string reason` / `bool lift_restored` |
| `LocatorStatus.msg` | `std_msgs/Header header` / `string interface_version` / `string phase` / `unique_identifier_msgs/UUID goal_id` / `string last_reject_reason` |

| 種類 | 名前 |
| --- | --- |
| 終わり方（`reason`） | `completed` / `no_lugs` / `lugs_ungrouped` / `no_image` / `lift_refused` / `camera_too_low` / `canceled` / `timeout` / `human_override` |
| 状態（`phase`） | `IDLE` / `RUNNING` / `CLEANING` |
| 途中経過の段（feedback の `phase`） | `RAISING` / `SCANNING` / `ESTIMATING` |
| 拒否の理由（`last_reject_reason`） | `busy` / `bad_range` / `bad_distance` / `bad_time_limit` |

| パラメータ | 単位 | 既定 | 出どころ |
| --- | --- | --- | --- |
| `max_distance_default_m` | m | 5.0 | [Spec.md](../spec/Spec.md) §1.2「撮り方」（盤まで 5 m 以内） |
| `top_height_default_m` | m | 2.35 | [Spec.md](../spec/Spec.md) §1.1「高さ」 |
| `status_period_ms` | ms | 200 | 暫定 |
| `cleanup_timeout_s` | s | 30 | 暫定（後片付けの上限。過ぎたら手を離して `lift_restored: false`） |

## 4. ファイル・ディレクトリ

| パス（リポジトリ直下から） | 中身 | git |
| --- | --- | --- |
| `model/` | 学習済みモデル（重み）。どの重みを使うかは SHA-256 で指す（[Spec.md](../spec/Spec.md) §1.2。2026-11-05 ユーザー決定） | **入れない**（`.gitignore` に `model/` を書く） |
| `LICENSE` | AGPL-3.0 の全文 | 入れる |
| `docker/Dockerfile` | 開発・実機で使うイメージ `ss-system:dev`（`ros:humble` が元） | 入れる |
| `scripts/test.sh` | コンテナの中で `colcon build` と `pytest` | 入れる |
| `tools/` | `make_proto_fixture.py`（試作の環境で回す）・`measure_cpu_time.py`・`check_std.py` | 入れる |
| `ss_ws/src/ss_locator/ss_locator/` | `geometry.py`・`assemble.py`・`panels.py`・`scan_plan.py`・`locate_core.py`・`detector.py`・`fakes.py`・`locator_node.py`・（待ち）`hve_lift.py`・`hve_camera.py`（[-core.md](DetailedDesign-core.md) §1） | 入れる |
| `ss_ws/src/ss_locator/tests/fixtures/proto_9265d2d.json` | 試作の出力（回帰試験の正解。`WP-EST-01`） | 入れる |
| `docs/plan/source/cpu-time.md`・`std-check.md` | 測った結果（`WP-DET-01`・`WP-EST-02`） | 入れる |

## 5. 推定と流れのパラメータ（[-core.md](DetailedDesign-core.md)）

| パラメータ | 単位 | 既定 | 出どころ |
| --- | --- | --- | --- |
| `model_file` | — | `model/lug_v3_s.pt` | 試作の今のモデル（[source/lifting-lug-pose.md](../source/lifting-lug-pose.md) §1） |
| `model_sha256` | — | 試作の `models/lug_v3_s.pt` の SHA-256（`WP-DET-01` で書き込む） | 試作 |
| `detect_imgsz` | px | 1280 | 試作の評価（`eval_lug.py`） |
| `detect_conf` | — | 0.25 | 試作の評価（`eval_lug.py`。[source/lifting-lug-pose.md](../source/lifting-lug-pose.md) §4） |
| `scan_camera_height_m` | m | 3.10 | [Spec.md](../spec/Spec.md) §1.2（床から約 2.95〜3.25 m の中央） |
| `min_camera_height_m` | m | 2.95 | [Spec.md](../spec/Spec.md) §1.2（同じ範囲の下端。届かなければ `camera_too_low`） |
| `scan_pitch_down_deg` | ° | 20 | **暫定（逆算）**: 5 m 先の金具の見下ろしは約 10°、1 m 先は約 30〜40°。そのあいだが縦の画角に入る向き。実機で決める（`SS-V2`） |
| `scan_overlap_ratio` | — | 0.1 | **暫定**（[Spec.md](../spec/Spec.md) §1.2「少し重なる」） |
| `settle_s` | s | 0.5 | **暫定**（向けてから撮るまで待つ。静止画は最大約 0.1 s 前のコマ: 試作 E13） |
| `image_timeout_s` | s | 5 | **暫定** |
| `camera_mount_x_m`・`camera_mount_y_m` | m | 0 | **実測待ち**（ヨーの回転軸の `base_link` での位置） |
| `camera_forward_offset_m` | m | 0 | **実測待ち**（回転軸から光学中心まで） |
| `camera_height_offset_m` | m | 0 | **実測待ち**（昇降部の配信値から光学中心の床からの高さへ） |
| `camera_K`・`camera_dist` | — | 偽のカメラは合成データの値 | **`WP-CAL-01` で校正** |
