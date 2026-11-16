# hve-system とつなぐ部分（案）

[DetailedDesign.md](DetailedDesign.md) の詳細。本システムが hve-system の昇降部・カメラモジュールとつなぐ所。
**状態: 2026-11-15 案。2026-11-16 に hve-system の答え（[Spec-open.md](../spec/Spec-open.md) §3）で直した。昇降部の分（§1〜§5）は実装済み（`WP-HVE-01`）。カメラモジュールの分は、口を確かめた経緯が §6、本システムの側の設計が §7（2026-11-16 案）。**名前の一覧は [-names.md](DetailedDesign-names.md) §7。決まっていないことは [-open.md](DetailedDesign-open.md) §1。

- 相手の口の正は hve [DetailedDesign-protocol-modules.md](../../../../hve-system/docs/plan/detailed/DetailedDesign-protocol-modules.md)（昇降部。2026-11-05 に本システムが確かめた）と hve [DetailedDesign-names.md](../../../../hve-system/docs/plan/detailed/DetailedDesign-names.md) §3（止めた理由）。**形は写さない。**ここには本システムの使い方だけ書く。
- 入力は [Spec.md](../spec/Spec.md) §1.1（人が操作に割り込んだとき・見つからない理由）・§1.2（高さの出どころ・撮り方）と [-core.md](DetailedDesign-core.md) §5・§6。

---

## 0. 結論

| 項目 | 案 |
| --- | --- |
| 相手の状態（2026-11-15） | 昇降部の口は hve の `main` に入った（`WP-LIFT-13`・`WP-LIFTUI-03`・`WP-CAM-10`。**実機はまだ**）。**`goto` のファームも 2026-11-16 に hve の `main` に入った**（hve `WP-LIFT-14`。`af1eb0f`。**昇降部への書き込みと実機での確認はまだ**）。偽の昇降部と違う点（hve の知らせ）: 実行中に高さが読めなくなると `HEIGHT_UNKNOWN` で止まる（下降も。§4 の 6 に入る）／ 目標が上端ちょうどなら受け付ける。**着く前に速さを落とさないので、速すぎると行き過ぎて往復しうる。実機で確かめるまでは設定の初期値で送る**（§2.2 のとおり）。カメラモジュールが本システムに開く口は未着手（hve [ImplementationPlan.md](../../../../hve-system/docs/plan/ImplementationPlan.md) §6 の行 8a） |
| 分け方 | **昇降部とカメラモジュールを別のパケットにする**（`WP-HVE-01` 昇降部・`WP-HVE-02` カメラモジュール）。昇降部の分は、偽の昇降部を相手に今から作って確かめられる |
| 層 | **純関数 `hve_lift_core.py`**（送る文を決める・`state` を `LiftState` に直す）と、**つなぐ `hve_lift.py`**（WebSocket の送受信と繋ぎ直し）（`DD-2`） |
| `locate_core` | **変えるのは 1 点だけ**: `LiftState.height_m` に「読めない」（`None`）を足す（§3） |
| 使う命令 | `hello`・`goto`・`release` だけ。`hold` は使わない |
| 人の割り込み | `goto` が取って代わられた・持ち主が人・持ち主がほかの上部モジュール、のどれかを見たら、**送るのをやめ、`human` として `locate_core` に渡す**（戻さずに手を離す。§4） |
| 実機で上昇させるのは | hve が「実機に入れた」と知らせ、`WP-LIFT-14` が入ってから（[-names.md](DetailedDesign-names.md) §1 の決まりのまま） |

## 1. 層とファイル

| 層 | ファイル | 中身 |
| --- | --- | --- |
| 純関数 | `hve_lift_core.py` | `LiftLink`（§2）。**時刻・受け取った `state`・`locate_core` の命令を受けて、送る文（dict）と今の `LiftState` を返すだけ。**WebSocket も ROS 2 も読み込まない |
| つなぐ | `hve_lift.py` | `HveLift`。自分のスレッドで WebSocket に繋ぎ、`LiftLink` が返した文を送り、受けた `state` を渡す。**`FakeLift` と同じ `apply(command)`・`advance(dt)` を持つ**（`locator_node` は差し替えるだけ） |
| つなぐ | `locator_node.py` | パラメータ `hve_lift` が true のとき、`FakeLift` の代わりに `HveLift` を使う（§5） |

WebSocket のライブラリは **`websockets`（pip）**（案。クライアントと、試験で使う小さなサーバの両方を持つ）。`docker/Dockerfile` に足す。入らない版があれば入る版にして、パケットの報告に書く。

## 2. `LiftLink`（純関数）

| 入口 | やること |
| --- | --- |
| `on_connected(now)` | 次に送る文の先頭に `hello` を積む（§2.1）。`state` をまだ受けていない扱いにする |
| `on_closed(now)` | 繋がっていない扱いにする。**命令の途中なら、その命令をやめる**（繋ぎ直しても自分から動かし直さない。§4 の `refused`） |
| `on_state(msg, now)` | 受けた `state`（dict）と時刻を覚える。`t` が `state` でないもの・dict でないものは捨てる |
| `on_command(command, now)` | `LiftGoto`: `press` を 1 増やし、目標 `target_mm = round(height_m × 1000)` を覚えて「移動中」にする。`LiftRelease`: 命令をやめ、`release` を 1 回積む（命令が無ければ何も積まない） |
| `poll(now)` | 送る文の並びと、繋ぎ直すかを返す。**「移動中」の間は `lift_cmd_period_ms` ごとに同じ `press` の `goto` を返す。**着いた・止まった（§3・§4）あとは返さない。最後の `state` から `lift_state_timeout_ms` を過ぎたら「繋ぎ直す」を返す |
| `lift_state(now)` | 今の `LiftState`（§3） |

`press` は起動してから増やし続ける（繋ぎ直しても戻さない）。

### 2.1 送る文

形の正は hve [DetailedDesign-protocol-modules.md](../../../../hve-system/docs/plan/detailed/DetailedDesign-protocol-modules.md) §1.1・§2.1・§2.2。本システムが入れる値だけ書く。

| 文 | 入れる値 |
| --- | --- |
| `hello` | `name`: `ss` ／ `role`: `control` ／ `operator`: `auto` ／ `ceiling_sensor`: `false` ／ `busy_label`: 「画像認識で操作中」（UTF-8 で 24 バイト。上限 48） ／ `fw`: パッケージ `ss_locator` の版 |
| `goto` | `press`・`target_mm`・`speed`（§2.2）。**`ceiling` は載せない**（距離計を持たない） |
| `release` | `press`（最後の `goto` と同じ値） |

繋ぐ先は `ws://<lift_host>/ws/module`。**`/ws/module` はコードの定数にし、パラメータにしない**（`/ws/ui` に繋ぐと天井の守りが外れる。hve と同じ扱い）。

### 2.2 速さ（2026-11-16 hve の答え）

`goto` の `speed` は、**昇降部の設定（`GET http://<lift_host>/api/settings`）から取る**: 上昇は `lift_up.init`、下降は `lift_down.init`。それぞれ `min`〜`max` に収める。昇降部は設定の範囲に丸めないので、本システムが収める。

- 取るのは `HveLift`（繋ぐたびに 1 回）。`LiftLink` には `set_speeds(up_mm_s, down_mm_s)` で渡す。
- **取れていない間に `LiftGoto` を受けたら、動かさずに `refused`**（速さを決め打ちで送らない）。
- 向きは、`LiftGoto` を受けたときの高さと目標を比べて決める。

## 3. `state` から `LiftState` へ

| `LiftState` | 求め方 |
| --- | --- |
| `height_m` | `height_ok` が true なら `height_mm ÷ 1000`。**`height_ok` が false・繋がっていない・最後の `state` が `lift_state_timeout_ms` より古いときは `None`（読めない）** |
| `arrived` | 「移動中」にした `goto` と**同じもの**（`state.goto` の `by` が `ss`・`press` と `target_mm` が送った値）の `status` が `ARRIVED` のとき true。**下降の `goto` が `BOTTOM` で止まったときも true**（下端より下へは戻せない。§4） |
| `stopped_reason` | §4 |

**`press` と `target_mm` まで照らす**ので、目標を変えた直後に、前の目標で着いたままの `state` が届いても着いたとみなさない（[-core.md](DetailedDesign-core.md) §6 の気をつけること）。

`locate_core` の直し（`WP-HVE-01` に含める）: 受け付けのとき `height_m` が `None` なら「元の高さが分からない」として扱う（今の「昇降部の高さがまだ届いていない」と同じ道。動かさずに `lift_refused`）。`RAISING` の天井の判定は、`height_m` が `None` なら `camera_too_low` にする（届いたと言えないため）。

高さの基準: hve の `height_mm` は**下端からの昇降量**。光学中心の床からの高さ = `height_m` + `camera_height_offset_m`（[-core.md](DetailedDesign-core.md) §2。**実測待ち**。下端で 0 のままだと、目標の 3.10 m は hve の範囲の外になり、必ず `camera_too_low` で返る）。

## 4. 止めた理由の対応

`LiftLink` は、命令の途中（`LiftGoto` を受けてから `LiftRelease` を受けるまで）に次を見たら、**`goto` を送るのをやめ**、`stopped_reason` を出す。上の行が先。

| # | 見たもの | `stopped_reason` | 依頼の `reason`（`locate_core` が決める） |
| --- | --- | --- | --- |
| 1 | `state.owner_operator` が `human`（人が昇降機を動かしている） | `human` | `human_override` |
| 2 | 自分の `goto` の `reason` が `PREEMPTED`（取って代わられた） | `human` | `human_override` |
| 3 | `state.owner` が `null` でなく、`owner_name` が `ss` でもない（ほかの上部モジュール・アームの制御器が動かしている） | `human` | `human_override`（**名前が実態と合わない**。[-open.md](DetailedDesign-open.md) §1 の 8） |
| 4 | **上昇の** `goto` が `NOT_ARRIVED` で、`reason` が `CEILING_NEAR`・`TOP`・`TARGET_RANGE` | `ceiling` | 光学中心が `min_camera_height_m` 以上なら続ける。届かなければ `camera_too_low` |
| 4a | **上昇の** `goto` が `NOT_ARRIVED` で、`reason` が `CEILING_STALE`（天井の値が無い） | `ceiling_unknown` | 光学中心が `min_camera_height_m` 以上なら続ける。届かなければ `ceiling_unknown`（**版 2**。`WP-LOC-03` が入るまでは `camera_too_low` にまとめる） |
| 5 | 下降の `goto` が `NOT_ARRIVED` で、`reason` が `BOTTOM` | `none`（`arrived` を true） | （着いた扱い） |
| 5a | 自分の `goto` が `NOT_ARRIVED` で、`reason` が `MAX_RUN`（連続駆動の上限） | `none`（**`press` を 1 増やして同じ目標を送り直す**。1 つの `LiftGoto` につき `LIFT_MAX_RUN_RETRY` 回まで。超えたら `refused`） | （続ける） |
| 6 | 自分の `goto` が `NOT_ARRIVED` で、上のどれでもない（`HEIGHT_UNKNOWN`・`ENCODER_STALL`・`ARM_LINK`・`CMD_TIMEOUT`・`OWNER_GONE`・`CMD_STOP`・**知らない名前**） | `refused` | `lift_refused` |
| 7 | 繋がっていない・`state` が `lift_state_timeout_ms` より古い | `refused` | `lift_refused` |

- 1〜3 は 2026-11-05 に hve へ答えた使い方のとおり（「`goto.status` が `PREEMPTED`、または `owner_operator` が `human` を見たら、すぐ止まり、戻さずに手を離す」）。**3 を足したのは、取って代わった相手を後片付けの `goto` で奪い返さないため**（`refused` にすると、`locate_core` が元の高さへ戻す `goto` を出す）。
- 1・3 は、着いて `goto` を送っていない間（`SCANNING`）も見る。**命令が無い間（`IDLE`）は見ない**（`stopped_reason` は `none`）。
- **2026-11-16 に hve が分け方を確かめた**（合っている）。hve の補足: `CEILING_STALE` は天井が近いのではなく「天井の値が無い」（カメラモジュールが繋がっていない・値が古い）。**2026-11-16 ユーザー決定で、理由を分けて返す**（4a。`HveLift` は hve の理由の名前もログに出す）。`TARGET_RANGE` は目標が 0 未満のときにも出るので、下降では 6 に入れる。`MAX_RUN` は新しい `press` で続けられる（5a）。
- 着いた周期で持ち主は `null` に戻る（hve の答え）。昇降部の画面で人が操作している間は `owner` が `ui`・`owner_name` が `null` なので、1 と 3 の両方に当たる。
- 知らない名前を `refused` に倒すのは、hve が止めた理由を足しても、動かない側に倒れるようにするため。
- 7 のあと繋ぎ直せても、その命令は続けない（`LiftRelease` を受けるまで `refused` のまま）。

## 5. `HveLift`（つなぐ）と `locator_node`

| 項目 | 決まり |
| --- | --- |
| スレッド | `HveLift` が自分のスレッドを 1 本持ち、繋ぐ → `on_connected` → `lift_cmd_period_ms` ごとに `poll` して送る ／ 受けたら `on_state`。`LiftLink` への出入りは 1 つのロックで守る |
| 繋ぎ直し | 繋げない・切れた・`poll` が「繋ぎ直す」を返したら、閉じて `lift_cmd_period_ms` の 10 倍（1 s）待って繋ぎ直す。繋ぎ直したら `hello` から |
| `apply(command)` | `LiftLink.on_command` を呼ぶ。**`LiftRelease` は、その場で `release` を送る**（周期を待たない） |
| `advance(dt)` | `dt` は使わず、`LiftLink.lift_state(now)` を返す |
| `close()` | 命令の途中なら `release` を送ってから閉じる。`locator_node.destroy_node` から呼ぶ |
| 時計 | `time.monotonic()`。`LiftLink` には引数で渡す |
| `locator_node` | `use_fakes` が true で `hve_lift` も true のとき、昇降機だけ `HveLift`（カメラと検出器は偽のまま）。**`use_fakes` が false のときに止まるのは今のまま**（カメラが本物になる `WP-HVE-02` まで） |

`locator_node` は状態の周期（`status_period_ms`。200 ms）で `advance` を呼んで `LiftState` を `locate_core` に渡す。人の割り込みで**送るのをやめるのは `LiftLink` の中**（`poll`）なので、止まるまでの遅れは `lift_cmd_period_ms` で決まる（ノードの周期を待たない）。

**偽の天井の値を送る道具は作らない。**本システムが `ceiling_sensor: true` を名乗る接続を持つと、実機に向けたとき天井の守りが外れる。手で確かめるときの天井の値は、hve の道具 `tools/lift_probe.py`（`--name hve-cam`）から出す（2026-11-16 hve の答え。手順は [-packets.md](DetailedDesign-packets.md) `WP-HVE-01`）。

## 6. カメラモジュール（hve の案を確かめている最中）

**2026-11-16 に hve が口の案を書いた**（hve [DetailedDesign-protocol-ss.md](../../../../hve-system/docs/plan/detailed/DetailedDesign-protocol-ss.md)。hve `9c4af5f`）。実装管理担当の見解は §6.1。**2026-11-16 ユーザーの承認を得て「確かめた」と返した。**本システムの側の設計（`hve_camera.py`・`WP-HVE-02`）は、確かめ終えてから書く。

以下は、案が来る前に並べた「見ること」（照らした結果は §6.1）。

hve が口を書いたら、本システムが確かめる（[Spec-open.md](../spec/Spec-open.md) §3 2026-11-05）。**確かめるときに見ること**（出どころのあるものだけ）:

| 見ること | 出どころ |
| --- | --- |
| 静止画: 1280×720・倍率に関係なく全体が写る 1 枚・要るときだけ要求 | [Spec.md](../spec/Spec.md) §1.1 |
| ピッチ: 目標角を指定して向けられる。今の目標角と、カメラモジュールが再起動したことが読める | [Spec-open.md](../spec/Spec-open.md) §3 2026-11-05（hve の答え） |
| ヨー: 向きを指定して向けられるか・今の向きが読めるか・届く範囲（[-open.md](DetailedDesign-open.md) §1 の 4）。**2026-11-16 hve の決定**: 向きはモータを回した量から数え、原点は人がカメラを正面に合わせて画面のボタンで 0° にする。原点を合わせる前と再起動のあとは向きが `null`（案）。角度を指示して向ける口を作る。回せる範囲は全周。**向きが `null` のときの本システムの扱いは、口の案が来てから決める** | hve `H-A11`・hve [DetailedDesign-protocol.md](../../../../hve-system/docs/plan/detailed/DetailedDesign-protocol.md) §5 |
| 人の割り込み: カメラ（ヨー・ピッチ）を人が動かしたことが分かるか | [-core.md](DetailedDesign-core.md) §6 |
| 省電力: 操作が 30 分無いと映像が止まり、ピッチの力が抜ける。本システムの操作・静止画の要求で戻るか、戻るまで何秒待つか | hve [DetailedDesign-power.md](../../../../hve-system/docs/plan/detailed/DetailedDesign-power.md)（本システムの口の案に `touch()` を書く、とある） |
| 元の向きに戻す: 依頼を受ける前のヨー・ピッチが読めるか | [Spec.md](../spec/Spec.md) §1.1 |

### 6.1 hve の案への見解（2026-11-16。ユーザーが承認し、hve へ返した）

| hve が確かめてほしいこと | 見解 |
| --- | --- |
| 1. 口（WS `/ws/module` と `GET /api/snapshot`）と名乗り | よい（昇降部と同じ書き方で扱える） |
| 2. `aim`（両軸を 1 つの命令で・片方だけ可・送り続ける・着いたら持ち主を離す・ピッチは渡した周期で着いた扱い） | よい。ピッチが動き終わるのを待つのは `settle_s` |
| 3. ヨーの原点を合わせる前は向きが `null`・`aim` のヨーは受け付けない・原点合わせは人だけ | よい。本システムは、向きが読めないときは動かさずに `camera_pose_unknown` で返す（2026-11-16 ユーザー決定） |
| 4. 再起動は `boot_id` の変化で知る | よい。**再起動するとヨーが `null` に戻るので、「捨てて撮り直す」（[-core.md](DetailedDesign-core.md) §5）はできなくなる。**その依頼を `camera_pose_unknown` で終える（2026-11-16 ユーザー決定） |
| 5. 撮ってからの経過を `X-Frame-Age-Ms` で返す・200 ms に 1 枚まで | よい。向け終えて `settle_s` 待ったあとより前のコマ（経過が長いもの）は捨てて取り直す |
| 6. 省電力は `aim`・静止画の要求で戻る | よい |

**hve に送った問いと頼みへの答え**（2026-11-16。hve のユーザーの承認。hve `d448eda`・`ff790fb`。形の正は hve の文書）:

| 問い・頼み | hve の答え | 本システムの使い方 |
| --- | --- | --- |
| 符号 | ヨーは正が左（上から見て反時計回り）。ピッチは正が見上げ・負が見下ろし | ヨーはそのまま。**ピッチだけ符号を逆にする**（本システムは見下ろしが正。変換は `hve_camera.py` の中だけ。[-core.md](DetailedDesign-core.md) §2） |
| ピッチの可動範囲 | `state` に下限・上限を載せる（動かす前に読める）。今は仮の −45°〜45° | 受け付けのとき、`scan_pitch_down_deg` が範囲に入るかを見る |
| ヨーの速さ | 設定の初期値で回す（既定 10°/s。1〜30）。`state` で読める。全周で 12〜36 s の見込み（実機では未測定） | 依頼の制限時間の目安に使う。偽の昇降機だけで約 40 s だったので、実機の全周は昇降とヨーで 1 分を超えうる |
| 静止画にそのときの向きを付ける | 応答に、返す時点のピッチの目標角・ヨーの向き・**向きの目標が最後に変わってからの経過**を付ける | **「向きが変わってからの経過」が「撮ってからの経過」より小さい 1 枚は使わずに撮り直す**（動いている間か、動く前のコマ）。推定には応答に付いたピッチ・ヨーを使う |
| 0° は機体の前 | hve の spec を「人がカメラを機体の前に合わせて 0° にする」に直した。画面と使い方にも書く | — |

**本システムの側に残る危険**: ヨーの 0° は人が目で合わせる。**1° ずれると 5 m 先の盤が約 9 cm ずれる**（精度の暫定の目標は ±15 cm）。モータの脱調でもずれる。合わせ方と確かめ方は実機で決める（`SS-V2`）。

## 7. カメラモジュールとつなぐ（2026-11-16 案）

相手の口の正は hve [DetailedDesign-protocol-ss.md](../../../../hve-system/docs/plan/detailed/DetailedDesign-protocol-ss.md)（2026-11-16 に本システムが確かめた。**hve の実装はまだ**）。形は写さない。昇降部（§1〜§5）と同じ作りにそろえる。

### 7.0 結論

| 項目 | 案 |
| --- | --- |
| 層 | 純関数 `hve_camera_core.py`（`CameraLink`・`frame_verdict`）と、つなぐ `hve_camera.py`（`HveCamera`: WebSocket と静止画の HTTP） |
| パケット | **2 つに分ける**: `WP-HVE-02`（純関数と、`locate_core` がカメラの「着いた」「取って代わられた」を受ける直し）・`WP-HVE-03`（つなぐ層と、ノードの本物の道） |
| `locate_core` の直し | (1) **向け終えてから** `settle_s` を数える（今は向ける命令を出した時刻から数える）。(2) カメラを取って代わられたら `human_override`。(3) 後片付けは昇降機とカメラの両方が着くのを待ち、上限にヨーを戻す時間を入れる |
| 相手がいない間 | 試験の中の小さなサーバ（昇降部と同じやり方）を相手にする。hve の偽のカメラモジュールが新しい口に対応したら、手で確かめる |

### 7.1 `CameraLink`（純関数）

`LiftLink`（§2）と同じ入口を持つ: `on_connected`・`on_closed`・`on_state`・`on_command`・`poll`・`camera_state`。違う所だけ書く。

| 項目 | 決まり |
| --- | --- |
| 命令 | `CameraPoint(yaw_rad, pitch_down_rad)` → `press` を 1 増やし、`aim` を `camera_cmd_period_ms` ごとに返す。`CameraRelease` → 命令の途中なら `release` を 1 回 |
| 角度の変換 | `aim` の `yaw_deg` = `degrees(yaw_rad)` を −180 以上 180 未満に直した値。`pitch_deg` = **`−degrees(pitch_down_rad)`**（hve は見上げが正）。**この変換はここだけ**（[-core.md](DetailedDesign-core.md) §2） |
| ヨーを動かさない命令 | `CameraPoint` の `yaw_rad` が `None` のとき、`aim` に `yaw_deg` を載せない（ピッチだけ向ける。`camera_pose_unknown` の後片付け） |
| 着いた | 送った `aim` と同じもの（`state.aim` の `by` が `ss`・`press` が同じ）の `status` が `ARRIVED` |
| `CameraState.yaw_rad`・`pitch_down_rad` | `state` の `yaw_deg`・`pitch_deg` を上の変換の逆で直す。`yaw_deg` が `null` なら `yaw_rad` は `None` |
| `CameraState.pose_known` | **`yaw_zeroed` が true・`yaw_deg` が `null` でない・`boot_id` が命令を始めてから変わっていない・`state` が古くない**、の全部が成り立つとき true |
| `CameraState.yaw_speed_rad_s` | `state` の `yaw_dps` から。無ければ `None` |

**止めた理由**（命令の途中に見たら `aim` を送るのをやめる。上の行が先）:

| # | 見たもの | `CameraState` | 依頼の `reason`（`locate_core` が決める） |
| --- | --- | --- | --- |
| 1 | `owner_operator` が `human` ／ 自分の `aim` が `PREEMPTED` ／ 持ち主が `null` でなく `owner_name` が `ss` でない | `taken_over: true` | `human_override` |
| 2 | `boot_id` が変わった ／ `yaw_zeroed` が false ／ 自分の `aim` が `NOT_ARRIVED` で `reason` が `YAW_UNKNOWN` | `pose_known: false` | `camera_pose_unknown` |
| 3 | 自分の `aim` が `NOT_ARRIVED` で、上のどれでもない（`TARGET_RANGE`・`IO_LOST`・`CMD_TIMEOUT`・`CMD_STOP`・`OWNER_GONE`・知らない名前）／ 繋がっていない・`state` が古い | `failed: true` | `no_image`（**カメラを向けられない場合の理由が一覧に無いので、`no_image` に入れる案**。[-open.md](DetailedDesign-open.md) §1 の 13） |

1 は、着いて `aim` を送っていない間（撮っている間）も見る。命令が無い間は見ない。

**`WP-HVE-02` で決めたこと**（2026-11-17。実装のとき設計書に無かった所）:

- **命令の途中で `state` が古くなる・切れる**と、`pose_known` が false と `failed` が true の両方になる。`locate_core` は `failed` を先に見るので `no_image`。**受け付けの時点で繋がっていない**ときは `camera_pose_unknown`。
- **ヨーを向けない命令**（`yaw_rad` が `None`）では、`yaw_zeroed` と `YAW_UNKNOWN` を見ない（見ると、再起動のあとのピッチだけの後片付けが止まる）。`boot_id` の変化はどの命令でも見る。
- `frame_verdict` は、200 でも `X-Still-Ms`・`X-Frame-Age-Ms`・`X-Pitch-Deg` のどれかが無い・読めないときは `fail`。ヘッダの名前は大文字と小文字を区別しない。**引数 `since_arrived_ms` は受けるが使っていない**（`X-Still-Ms` が同じ役目をする）。
- カメラモジュールへの名乗りの名前と表示は、昇降部と同じ定数（`LIFT_NAME`・`LIFT_BUSY_LABEL`）を使う。

### 7.2 静止画（`frame_verdict`。純関数）

`HveCamera.capture()` が応答を受けるたびに、`frame_verdict(status, headers, boot_id, since_arrived_ms)` で扱いを決める。

| 応答 | 扱い |
| --- | --- |
| 200 で、`X-Boot-Id` が命令を始めたときと違う ／ `X-Yaw-Deg` が無い | `pose_unknown`（→ `camera_pose_unknown`） |
| 200 で、`X-Still-Ms` が `X-Frame-Age-Ms` より小さい（動いている間か、動く前のコマ） | `retry` |
| 200 で、上のどれでもない | `use`。**推定には `X-Pitch-Deg`・`X-Yaw-Deg` を使う**（`state` の値ではなく。§7.1 の変換で直す） |
| 429・503 | `retry`（`snapshot_min_interval_ms` 空けて取り直す） |
| それ以外・繋がらない・JPEG が読めない・1280×720 でない | `fail`（→ `no_image`） |

`retry` は、`RequestFrame` を受けてから `image_timeout_s` を過ぎるまで繰り返す。過ぎたら `fail`。取り直しの間隔は `snapshot_min_interval_ms` より短くしない。

### 7.3 `locate_core` の直し

| 項目 | 今 | 直す |
| --- | --- | --- |
| 撮るまでの待ち | `CameraPoint` を出した時刻から `settle_s` | **`CameraState.arrived` を受けた時刻から `settle_s`**（ヨーは 10°/s 前後でゆっくり回るので、命令の時刻からでは回っている途中で撮る）。着くのを待つ上限は `aim_timeout_s`（過ぎたら `no_image`） |
| 人の割り込み | 昇降部（`LiftState` の `human`）だけ | `CameraState.taken_over` でも `human_override`（どの段でも。後片付け中なら戻すのをやめて手を離す） |
| カメラを向けられない | — | `CameraState.failed` → `no_image` |
| 後片付けの終わり | 昇降機が着いたら終わり | **昇降機とカメラの両方が着いたら終わり**（カメラを戻す命令を出さない終わり方では昇降機だけ） |
| 後片付けの上限 | 昇降機を戻す距離 ÷ 速さ ＋ 余裕（`WP-LOC-03`） | `max(昇降機の分, ヨーを戻す角度 ÷ ヨーの速さ) ＋ cleanup_margin_s`。ヨーの速さが `None` なら昇降機の分だけ |
| `lift_restored` | 昇降機を戻せたか | 変えない（カメラの向きを戻せたかは含めない。[-locate.md](DetailedDesign-locate.md) §3） |

偽のカメラ（`FakeCamera`）は、向けた次の `state()` で `arrived` を返す（今の試験の時間の流れを大きく変えない）。

**`WP-HVE-02` で決めたこと**（2026-11-17）: 後片付け中にカメラが `failed` になったら、カメラは待たずに終える（向けられないものを上限まで待たない）。後片付けが上限で終わる・カメラを取って代わられたときの `lift_restored` は、昇降機が着いていれば true。**ノードは、カメラの状態を周期ごとに `locate_core` へ渡す**（昇降機の分を処理したあと。本物のカメラの「着いた」「取って代わられた」「向けられない」はこの道でだけ届く）。

### 7.4 `HveCamera`（つなぐ）と `locator_node`（`WP-HVE-03`）

| 項目 | 決まり |
| --- | --- |
| 形 | `HveLift`（§5）と同じ: スレッド 1 本・繋ぎ直し・`close()`。`point(yaw_rad, pitch_down_rad)`・`release()`・`state()`・`capture()` を持つ（`FakeCamera` と差し替えられる形） |
| 繋ぐ先 | `ws://<camera_host>/ws/module` と `http://<camera_host>/api/snapshot?by=ss`。**`camera_host` が空なら、昇降部の `state.modules` にある `hve-cam` の `ip` を使う**（hve の勧め）。どちらも無ければ繋がない（`pose_known: false`） |
| `capture()` | 静止画を取り、§7.2 で扱いを決め、`use` なら JPEG を OpenCV で読んで `{image, yaw_rad, pitch_down_rad}` を返す。ノードの周期を止めないよう、別スレッドで取り、結果を出来事（`FrameDone`）として渡す |
| ノードの本物の道 | `use_fakes` が false のとき: 昇降機は `HveLift`、カメラは `HveCamera`、検出は `Detector`（学習済みモデル）。外部パラメータは `geometry.camera_T(昇降部の高さ ＋ camera_height_offset_m, 応答のヨー, 応答のピッチ, 取付の位置)`。**`use_fakes` が false で起動時に止まるのをやめる** |
| 受け付けの前 | 昇降部・カメラモジュールの `state` がまだ来ていなければ、今までどおり動かさずに返す（昇降部は `lift_refused`、カメラは `camera_pose_unknown`） |

**ピッチの可動範囲**: `scan_pitch_down_deg` が hve の `state` の範囲の外でも、起動は止めない。`aim` が `TARGET_RANGE` で返り、`no_image` になる（ログに hve の理由の名前を出す）。

**本システムが持たないもの**: ヨーの原点合わせ（人だけ）・ズーム・設定の変更。カメラモジュールへは `ceiling_sensor` を名乗らない。

**`WP-HVE-03` で決めたこと**（2026-11-17。実装のとき設計書に無かった所）:

- **静止画の結果の渡し方**: `HveCamera.capture(on_done)` はすぐ戻り、別スレッドで取って `on_done(verdict, result)` を呼ぶ。ノードは同じスレッドで検出と金具を床に戻すことまで行い、出来事を積む（ノードの周期は止めない）。`pose_unknown` は `pose_known: false` の `CameraState`、`fail` と昇降部の高さが読めないときは `FrameDone(ok: false)` にする。**この処理の中の例外はログに出し、`FrameDone(ok: false)` にする。**
- **カメラモジュールの居場所**: 昇降部の `state.modules` を読む入口として `HveLift.modules()` を足した（最後に受けた `state` の `modules`。切れると `None`）。読む形は hve の文書の一覧の形（`name`・`ip` を持つ要素の並び）。名前を鍵にした形も読む。**実機の形と照らす**（[docs/試験項目.md](../../試験項目.md)）。
- 静止画の取り直しは、前の要求を始めてから `snapshot_min_interval_ms` 以上空ける（要求をまたいで効く）。`frame_verdict` に渡す起動の番号は、向けたときに見ていた `state` のもの。
- 偽と本物で、ノードの命令の呼び方（`CameraRelease`・`RequestFrame`）をそろえた。**これまでノードは `CameraRelease` を実行していなかった**（偽のカメラでは何もしないので害は無かった）。
- `use_fakes` が false の起動は、`Detector` が学習済みモデルを照らす所で止まりうる（`model/` が無い・SHA-256 が違う）。
