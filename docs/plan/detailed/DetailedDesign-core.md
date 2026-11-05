# 中の構成と推定（案）

[DetailedDesign.md](DetailedDesign.md) の詳細。依頼を受けてから結果を返すまでの、本システムの中身。
**状態: 2026-11-05 案。**名前の一覧は [-names.md](DetailedDesign-names.md) §4・§5。決まっていないことは [-open.md](DetailedDesign-open.md) §1。

- 入力は [Spec.md](../spec/Spec.md) §1.1・§1.2、口は [-locate.md](DetailedDesign-locate.md)、先行試作は [source/lifting-lug-pose.md](../source/lifting-lug-pose.md)。
- **先行試作をなるべく使う**（[Spec.md](../spec/Spec.md) §1.2）。写すのは `9265d2d` の `src/assemble_lugs.py` と `src/common.py` の射影の部分。**写したファイルの先頭に、元のファイル名と版 `9265d2d` を書く。**

---

## 0. 結論

| 項目 | 案 |
| --- | --- |
| 層 | **純関数の層**（幾何・組み立て・返す形への変換・走査の計画・依頼の状態遷移）と、**つなぐ層**（ROS 2・検出器・hve-system）に分ける（`DD-2`）。純関数の層は ROS 2 も ultralytics も読み込まない |
| 座標 | 試作の「ワールド」を **`base_link`（依頼を受け付けた時点の機体。床が z = 0）** に置き換える。試作も床が z = 0 なので、**組み立ての計算は変えずに、カメラの外部パラメータの作り方だけ変える**（§2） |
| 検出 | 試作の評価と同じ呼び方（`imgsz` 1280・確信度 0.25）。タイル分けはしない（試作でも評価の経路では使っていない） |
| 組み立て | 試作の `lift`・`fuse`・`group`・`fit` をそのまま使う。**`orient_rows`（並んだ盤で幅の向きを決め直す）は使わない**。幅は spec の「機体側に最も向いている辺」で決めるため（§4） |
| 流れ | 昇降機を上げる → ヨーを振って 1 枚ずつ撮り、撮るたびに検出して金具を床に戻す → 全部撮ったらまとめて組み立てる → 結果 → 後片付け（§5） |
| 途中で終わったとき | **それまでに撮った画像の金具だけで組み立てて返す**（組み立ては純関数で速い） |

## 1. 層とファイル

置き場所はすべて `ss_ws/src/ss_locator/ss_locator/`（パッケージ `ss_locator`）。

| 層 | ファイル | 中身 | 試作との関係 |
| --- | --- | --- | --- |
| 純関数 | `geometry.py` | カメラの姿勢 → 外部パラメータ（§2）、歪み補正つきの光線、水平面との交点 | `common.py` の `undistort_rays` を写す。外部パラメータの作り方は新しく書く |
| 純関数 | `assemble.py` | `lift`・`fuse`・`group`・`fit` | `assemble_lugs.py` を写す。天板の高さを引数にする（試作は定数 `PANEL_H`）。`orient_rows` は写さない |
| 純関数 | `panels.py` | 組み立ての結果 → 返す盤（`PanelTop` に入れる値）。基準の辺・確からしさ・範囲での絞り込み（§4） | 新しく書く |
| 純関数 | `scan_plan.py` | 方向の範囲 → 撮るヨーの並び、撮った画像 → 見た方向の範囲、角度の正規化（§3） | 新しく書く |
| 純関数 | `locate_core.py` | 依頼の受け付けの判定、依頼の状態遷移（§5）。**時刻と出来事を受けて、次にやることを返すだけ** | 新しく書く（th-system `follow_planner_core.py` と同じ二層構造） |
| つなぐ | `detector.py` | 学習済みモデルの読み込み（SHA-256 を照合）と検出。画像 → キーポイントと確信度 | `eval_lug.py` の推論の呼び方に揃える |
| つなぐ | `hve_lift.py`・`hve_camera.py` | hve-system の昇降部・カメラモジュールとの口 | **hve の口が実機に入る・書かれるまで作らない**（§6） |
| つなぐ | `fakes.py` | 試験用の偽の昇降機・カメラ（合成データの画像とカメラの姿勢を返す） | 新しく書く |
| つなぐ | `locator_node.py` | ROS 2 のノード。アクション・トピック（[-locate.md](DetailedDesign-locate.md)）と `locate_core` の配線 | 新しく書く |

## 2. カメラの外部パラメータ

試作は外部パラメータ（ワールド → カメラの 4×4 `T`）を既知として受ける。本システムは**昇降機の高さ・ヨー・ピッチから作る**（[Spec.md](../spec/Spec.md) §1.2）。

| 入力 | 出どころ |
| --- | --- |
| 高さ | hve-system の昇降部が配信する値（[Spec.md](../spec/Spec.md) §1.2）。光学中心の床からの高さ = 配信値 + `camera_height_offset_m` |
| ヨー | **hve-system の改良待ち**（hve `H-A11`）。それまでは偽のカメラが返す値 |
| ピッチ | **サーボの目標角を信頼する**（[Spec.md](../spec/Spec.md) §1.2）。本システムが送った目標角を使う |
| 取付の位置 | ヨーの回転軸の `base_link` での位置 `camera_mount_x_m`・`camera_mount_y_m`、回転軸から光学中心までの前向きの距離 `camera_forward_offset_m`（[-names.md](DetailedDesign-names.md) §5。**実測待ち**） |

計算（`geometry.camera_T(height_m, yaw_rad, pitch_down_rad, mount)`）:

- 向き: 前 `f = (cos ψ cos θ, sin ψ cos θ, −sin θ)`、右 `r = (sin ψ, −cos ψ, 0)`、下 `d = f × r`（ψ = ヨー、θ = 見下ろしの角。下向きが正）
- OpenCV のカメラ（x 右・y 下・z 前）から `base_link` への回転 `R = [r d f]`（列）
- 光学中心 `C = (mount_x + a cos ψ, mount_y + a sin ψ, height)`（a = `camera_forward_offset_m`）
- `T = [Rᵀ | −Rᵀ C]`

ヨー・ピッチの符号と原点は**本システムの決まり**（ヨーは機体の前が 0・左が正、ピッチは見下ろしが正。[-locate.md](DetailedDesign-locate.md) §1 と同じ）。hve-system の値との変換は `hve_camera.py` の中だけで行う。

内部パラメータ（`K`・歪み `k1`・`k2`）は 1280×720 でチェッカーボードを使って 1 回校正した値をパラメータファイルに持つ（[Spec.md](../spec/Spec.md) §1.2）。**歪みのモデルは試作と同じ k1・k2 だけ**（試作の射影がこの形のため）。

## 3. 走査の計画

| 項目 | 決まり |
| --- | --- |
| 水平の画角 | 内部パラメータから求める: `hfov = 2 atan(640 / fx)` |
| 刻み | `step = hfov × (1 − scan_overlap_ratio)`（[Spec.md](../spec/Spec.md) §1.2「画角が少し重なる刻み」） |
| 撮るヨー | 幅 `span ≤ hfov` なら範囲の中央で 1 枚。それより広ければ、最初の画像の左端を範囲の開始に、最後の画像の右端を範囲の終わりに合わせ、そのあいだを `step` 以下の等間隔で埋める。**全周（幅 2π）は `ceil(2π / step)` 枚を等間隔** |
| 並び | 範囲の開始から反時計回りの順に撮る |
| 見た方向の範囲 | 撮って検出まで終えた画像ごとに `[ヨー − hfov/2, ヨー + hfov/2]` を取り、依頼の範囲で切り、重なりをまとめる |
| ヨーが届かない方向 | **未定**（hve のヨーの届く範囲が分からない。[-open.md](DetailedDesign-open.md) §1） |

ピッチは `scan_pitch_down_deg` に固定する（[Spec.md](../spec/Spec.md) §1.2「ピッチを固定し」）。値は暫定（[-names.md](DetailedDesign-names.md) §5）。

## 4. 組み立ての結果から返す盤へ

試作の `fit` の出力（中心・正面の向き・幅・奥行・金具の数・正面が分かるか・対角か）を、[-locate.md](DetailedDesign-locate.md) §2.3 の `PanelTop` に直す。

| `PanelTop` | 求め方 |
| --- | --- |
| `x_m`・`y_m` | `fit` の中心 |
| `yaw_rad` | 矩形の 4 辺の外向きの法線のうち、**中心から機体の原点へ向かう向きに最も近いもの**（基準の辺の法線） |
| `width_m`・`depth_m` | 基準の辺の長さ・それと直交する辺の長さ。試作の幅・奥行（金具の位置から角までの `LUG_INSET` を足したもの）を、基準の辺に合わせて入れ替える |
| `top_height_m`・`height_measured` | `top_height_default_m`・false |
| `lugs` | 組に使った金具の数 |
| `front_known`・`front_yaw_rad` | 金具 2 個（背中合わせ）で正面が決まったとき true と、`fit` の正面の向き |
| `pos_std_m` | `sqrt( Σ s_posᵢ² / n² + e² )`。`s_posᵢ` は金具ごとの位置のばらつき（試作の `lift`・`fuse`）、`e` は中心からリング面の線までの距離の平均（組分けの残差） |
| `yaw_std_deg` | `degrees( atan( √2 × mean(s_posᵢ) / width_m ) )` |

確からしさの 2 つの式は**案**。合成データで、推定の誤差と比べて当てはまるかを確かめる（`WP-EST-02`）。

**返す盤の絞り込み**: 中心の方位角が依頼の範囲に入り、`base_link` の原点からの水平距離が `max_distance_m` 以下の盤だけを返す。
**金具が対角の 2 個だけの盤**（試作の `fit` が `diagonal` を付けたもの）も返す（[Spec.md](../spec/Spec.md) §1.1。2026-11-05 ユーザー決定）。中心は 2 個の中点、幅・奥行は試作どおり 2 個の距離 ÷ √2、`yaw_rad` は上の規則で選んだ辺、**`yaw_std_deg` は 90**、`front_known` は false。

`reason` の `no_lugs` と `lugs_ungrouped` の区別は、**範囲内で金具が 1 個でも床に戻せたか**で決める（[-locate.md](DetailedDesign-locate.md) §2.4）。

## 5. 依頼の流れ（`locate_core`）

`locate_core` は状態（`IDLE`／`RUNNING`／`CLEANING`）と、`RUNNING` の中の段（`RAISING`／`SCANNING`／`ESTIMATING`）を持つ。
**時刻・出来事（昇降機の状態・画像が届いた・検出が終わった・取り消し・人の割り込み）を受けて、次の命令（昇降機・カメラ・結果を返す・後片付けの完了）を返す。**ROS 2 と hve-system への送受信は `locator_node` が行う。

| 段 | やること | 次へ | 途中で終わる |
| --- | --- | --- | --- |
| 受け付け | 依頼を確かめる（[-locate.md](DetailedDesign-locate.md) §2.1）。**今の昇降機の高さ・カメラのヨー・ピッチの目標角を「元の高さ・向き」として覚える** | `RAISING` | 拒否（何も動かさない） |
| `RAISING` | 昇降機を `scan_camera_height_m` に合わせる高さ（`scan_camera_height_m − camera_height_offset_m`）へ動かす。**受け付けた時点で昇降部の高さがまだ届いていなければ、動かさずに `lift_refused`**（原点を取る前の hve は目標の高さへの移動を受け付けないのと同じ扱い。元の高さが分からないので `lift_restored: false`） | 着いた → `SCANNING` | 光学中心が `min_camera_height_m` に届かないまま止まった（天井など）→ `camera_too_low`。拒否・動かない → `lift_refused` |
| `SCANNING` | 撮るヨーの順に: ヨーとピッチを向ける → `settle_s` 待つ → 静止画を 1 枚要求 → 検出 → 金具を床に戻す | 全部撮った → `ESTIMATING` | 画像が `image_timeout_s` 来ない → `no_image`。**カメラモジュールが再起動したと知らせたあいだの画像は捨てて撮り直す**（ピッチの目標角が 0° に戻るため。hve の答え。[Spec-open.md](../spec/Spec-open.md) §3 2026-11-05） |
| `ESTIMATING` | 撮った画像の金具をまとめて組み立て、返す盤に直す（§4） | 結果を返す → `CLEANING` | — |
| `CLEANING` | 昇降機を元の高さへ、カメラのヨー・ピッチを元の向きへ戻す（`human_override` ではどれも戻さない）。カメラと昇降機から手を離す | 後片付けの完了を出す → `IDLE` | `cleanup_timeout_s` を過ぎた → 手を離して `lift_restored: false` |

**どの段にいても**、次のときはその場で止め、それまでの金具で組み立てて結果を返し、`CLEANING` へ行く:

| 出来事 | `reason` |
| --- | --- |
| 取り消された | `canceled` |
| 依頼を受け付けてから `time_limit_s` を過ぎた | `timeout` |
| hve-system が「人に取って代わられた・人の操作」を知らせた | `human_override`（昇降機を戻さない） |

元のヨーは、ヨーの向きが読めるようになるまで偽のカメラが返す値を使う（hve `H-A11`。[-open.md](DetailedDesign-open.md) §1）。

## 6. hve-system とつなぐ部分（待ち）

| もの | 待っていること | それまで |
| --- | --- | --- |
| 昇降部（`hve_lift.py`） | hve が実機に入れたと知らせる（[-names.md](DetailedDesign-names.md) §1）。名前 `ss`・自動・距離計なし・「画像認識で操作中」で繋ぐ。`goto` で高さを合わせる | `fakes.py` の偽の昇降機 |
| カメラモジュール（`hve_camera.py`） | hve が口を書き、本システムが確かめる（静止画・名乗り・ピッチの目標角と再起動・ヨーの欄） | `fakes.py` の偽のカメラ |
| ヨーの向き | hve `H-A11` | 偽のカメラが返す値 |

hve-system の止めた理由と本システムの `reason`（`lift_refused`・`camera_too_low`・`human_override`）の対応表は、口が決まってから `WP-HVE-01` で書く。
