# 盤の位置を探す口（案）

[DetailedDesign.md](DetailedDesign.md) の詳細。依頼する側（fk-system ほか）と本システムのあいだの ROS 2 の口。
**状態: 2026-11-05 fk-system が確かめた（案のとおり）。**作業パケットに切るまで実装しない。名前の一覧は [-names.md](DetailedDesign-names.md) §2・§3。

- 入力は [Spec.md](../spec/Spec.md) §1.1（受け取るもの・返すもの・受け渡しの細部・見た方向の範囲）と §1.2（依頼の通り道）。
- **名前に fk・測定を入れない**（`SSD-4`・`SSD-6`。ほかのシステムからも同じ口で依頼できるようにする）。

---

## 0. 結論

| 項目 | 案 |
| --- | --- |
| 通り道 | ROS 2 Humble。依頼する側と同じ PC の別コンテナ。`--network host`・`ROS_DOMAIN_ID=10`・FastDDS は UDP だけ（[Spec.md](../spec/Spec.md) §1.2） |
| 依頼・取り消し・結果 | **アクション `/ss/locate/panels`**（型 `ss_msgs/action/LocatePanels`）。取り消しはアクションの取り消し |
| 後片付けの完了 | **トピック `/ss/locate/cleanup_done`**（`ss_msgs/msg/CleanupDone`）。依頼の ID（goal UUID）で結果と対応づける |
| 生存と状態 | **トピック `/ss/locate/status`**（`ss_msgs/msg/LocatorStatus`）。周期で出す。依頼する側はこれで「通信できない」「依頼を受けられない」を知る |
| 同時の依頼 | **1 件だけ。**実行中と後片付けが終わるまでは、新しい依頼を拒否する |
| 型の取り込み | 依頼する側は `ss_ws/src/ss_msgs` を読み取り専用でコンテナに入れる（写さない）。版は `LocatorStatus.interface_version` で照合する（fk-system が th-system の型を取り込むのと同じ形） |

## 1. 座標と角度の決まり

| 項目 | 決まり |
| --- | --- |
| 座標 | **依頼を受け付けた時点の機体**を基準にする（機体の中心・床・前向き。[Spec.md](../spec/Spec.md) §1.1）。REP-103: x が前、y が左、z が上。`header.frame_id` は `base_link`、`header.stamp` は依頼を受け付けた時刻 |
| 角度 | 単位は rad（確からしさの `yaw_std_deg` だけ °。spec どおり）。**機体の前が 0、反時計回り（左）が正** |
| 方向の範囲 | **開始 `azimuth_from_rad` と幅 `azimuth_span_rad`** で表す。開始から反時計回りに幅の分。幅は 0 より大きく 2π 以下。**全周は幅 2π**。0 をまたぐ範囲も同じ形で表せる（開始と終了の形にしないのは、全周と「同じ値」の区別が曖昧になるため） |
| 見た方向の範囲 | 同じ形の並んだ配列 `seen_from_rad[]`・`seen_span_rad[]`（i 番目どうしが 1 つの範囲）。重なりは本システムがまとめて返す |
| 開始の角度の値 | −π〜π に直して返す。受けるときはどの値でも受け、直して使う |

## 2. アクション `LocatePanels`

### 2.1 依頼（goal）

| フィールド | 型 | 意味 |
| --- | --- | --- |
| `azimuth_from_rad` | float32 | 方向の範囲の開始（§1） |
| `azimuth_span_rad` | float32 | 方向の範囲の幅（0 < 幅 ≤ 2π） |
| `max_distance_m` | float32 | 探す最大の距離。**0 なら既定値 `max_distance_default_m`**（5.0 m。[Spec.md](../spec/Spec.md) §1.2「撮り方」の「盤まで 5 m 以内」）。既定値より大きい値は既定値に切り詰める |
| `time_limit_s` | float32 | 制限時間（[Spec.md](../spec/Spec.md) §1.1）。0 より大きいこと |

**拒否する依頼**（アクションの goal を REJECT する。結果も後片付けの完了も出さない。何も動かしていないため）:
幅が範囲の外・`max_distance_m` が負・`time_limit_s` が 0 以下・**別の依頼が実行中か後片付け中**。
拒否した理由は `LocatorStatus` の `last_reject_reason` に出す（§4）。

### 2.2 結果（result）

**どの終わり方でも結果を返す**（取り消し・時間切れ・中断でも。アクションの状態は、取り消しなら CANCELED、それ以外は SUCCEEDED）。

| フィールド | 型 | 意味 |
| --- | --- | --- |
| `header` | std_msgs/Header | §1 |
| `reason` | string | **終わり方。常に入れる**（§2.4）。最後まで見終えて 1 面以上見つけたら `completed` |
| `panels` | `PanelTop[]` | 見つけた盤（§2.3）。**途中で終わっても、それまでに見つけた盤は返す**。見つからなければ空 |
| `seen_from_rad` | float32[] | 実際に撮って推定にかけた方向の範囲の開始（§1） |
| `seen_span_rad` | float32[] | 同じく幅 |

途中経過（feedback）は `string phase`（`RAISING`／`SCANNING`／`ESTIMATING`）だけ。表示用で、依頼する側が判断に使わなくてよい。

### 2.3 `PanelTop.msg`（盤 1 面の天面の矩形）

| フィールド | 型 | 意味 |
| --- | --- | --- |
| `x_m`・`y_m` | float64 | 天面の中心（§1 の座標） |
| `top_height_m` | float32 | 天面の高さ。**今は常に既定値 `top_height_default_m`**（2.35 m） |
| `height_measured` | bool | 高さを測ったか。**今は常に false**（カメラ 1 台では測れない。[Spec.md](../spec/Spec.md) §1.1） |
| `yaw_rad` | float64 | **基準の辺の向き。**基準の辺＝機体側に最も向いている辺。中心からその辺へ向かう向き（基準の辺の外向きの法線） |
| `width_m` | float32 | 基準の辺の長さ |
| `depth_m` | float32 | 基準の辺と直交する辺の長さ |
| `pos_std_m` | float32 | 位置の標準偏差 [m] |
| `yaw_std_deg` | float32 | 向きの標準偏差 [°] |
| `lugs` | int32 | 使った吊り金具の数 |
| `front_known` | bool | 正面が分かっている（背中合わせの盤など。追加の情報。使わなくてよい） |
| `front_yaw_rad` | float64 | `front_known` のとき、正面の向き（中心から正面の辺へ向かう向き）。そうでなければ 0 |

### 2.4 終わり方（`reason`）

[Spec.md](../spec/Spec.md) §1.1 の「見つからない理由」と 1 対 1。

| 名前 | spec の理由 | 後片付けで昇降機を戻すか |
| --- | --- | --- |
| `completed` | （最後まで見終え、1 面以上見つけた） | 戻す |
| `no_lugs` | 範囲に吊り金具が見えない（**見終えて 1 面も無く、吊り金具も 1 個も見えなかったとき**。`completed` の代わりに出す） | 戻す |
| `lugs_ungrouped` | 吊り金具は見えたが盤に組めない（見終えて 1 面も無いとき。`completed` の代わりに出す） | 戻す |
| `no_image` | 画像が取れない | 戻す |
| `lift_refused` | 昇降機が動かない・拒否された | 戻せれば戻す |
| `camera_too_low` | 必要な高さまで上がれない | 戻す |
| `canceled` | 取り消された | 戻す |
| `timeout` | 時間切れ | 戻す |
| `human_override` | 人の操作で中断 | **戻さない**（手を離すだけ） |

後片付けでは、昇降機と一緒に**カメラのヨー・ピッチも依頼を受ける前の向きに戻す**（`human_override` では戻さない。[Spec.md](../spec/Spec.md) §1.1。2026-11-05 ユーザー決定）。口の形は変えない。

## 3. 後片付けの完了 `CleanupDone.msg`

| フィールド | 型 | 意味 |
| --- | --- | --- |
| `header` | std_msgs/Header | 出した時刻 |
| `goal_id` | unique_identifier_msgs/UUID | 対応する依頼（アクションの goal UUID） |
| `reason` | string | その依頼の結果の `reason` と同じ値 |
| `lift_restored` | bool | 昇降機を元の高さに戻したか（`human_override` と、戻せなかった `lift_refused` は false）。**カメラの向きを戻せたかは含めない** |

- **受け付けた依頼には、必ず 1 回だけ出す。**結果を返したあとに出す（結果より先には出さない）。拒否した依頼には出さない。
- QoS: reliable・transient_local・depth 10（依頼する側が繋ぎ直しても、最近の完了を取りこぼさない）。
- 後片付けが `cleanup_timeout_s` を過ぎても終わらないときも、そこで手を離し、`lift_restored: false` で出す。

## 4. 生存と状態 `LocatorStatus.msg`

| フィールド | 型 | 意味 |
| --- | --- | --- |
| `header` | std_msgs/Header | 出した時刻 |
| `interface_version` | string | この口の版。今は `"1"`。依頼する側は自分が知っている版と照合する |
| `phase` | string | `IDLE`（受けられる）／ `RUNNING`（依頼を実行中）／ `CLEANING`（結果を返し、後片付け中） |
| `goal_id` | unique_identifier_msgs/UUID | 実行中・後片付け中の依頼。`IDLE` のときは全 0 |
| `last_reject_reason` | string | 最後に拒否した依頼の理由（`busy`／`bad_range`／`bad_distance`／`bad_time_limit`）。無ければ空 |

- `status_period_ms`（暫定 200 ms）ごと＋`phase` が変わったとき。QoS: reliable・transient_local・depth 1。
- **依頼する側は、`status_period_ms` の 5 倍（1 s）届かなければ「通信できない」とみなす**ことを勧める（判定と値は依頼する側が持つ）。

## 5. この案で決めないもの

| もの | どこで |
| --- | --- |
| ノードの中身（走査の刻み・推定の流れ） | 次の詳細設計（`SS-S1`・先行試作の取り込み） |
| hve-system の昇降部・カメラモジュールの口 | hve が書く（昇降部は hve DetailedDesign-protocol-modules.md、カメラモジュールは昇降部の口のあと） |
| 処理時間の目標 | CPU で測ってから fk-system と決める（`SS-V3`） |
