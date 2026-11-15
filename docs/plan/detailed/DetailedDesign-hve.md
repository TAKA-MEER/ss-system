# hve-system とつなぐ部分（案）

[DetailedDesign.md](DetailedDesign.md) の詳細。本システムが hve-system の昇降部・カメラモジュールとつなぐ所。
**状態: 2026-11-15 案。昇降部の分（§1〜§5）だけ書いた。カメラモジュールの分（§6）は hve の口を待つ。**名前の一覧は [-names.md](DetailedDesign-names.md) §7。決まっていないことは [-open.md](DetailedDesign-open.md) §1。

- 相手の口の正は hve [DetailedDesign-protocol-modules.md](../../../../hve-system/docs/plan/detailed/DetailedDesign-protocol-modules.md)（昇降部。2026-11-05 に本システムが確かめた）と hve [DetailedDesign-names.md](../../../../hve-system/docs/plan/detailed/DetailedDesign-names.md) §3（止めた理由）。**形は写さない。**ここには本システムの使い方だけ書く。
- 入力は [Spec.md](../spec/Spec.md) §1.1（人が操作に割り込んだとき・見つからない理由）・§1.2（高さの出どころ・撮り方）と [-core.md](DetailedDesign-core.md) §5・§6。

---

## 0. 結論

| 項目 | 案 |
| --- | --- |
| 相手の状態（2026-11-15） | 昇降部の口は hve の `main` に入った（`WP-LIFT-13`・`WP-LIFTUI-03`・`WP-CAM-10`。**実機はまだ**）。**`goto` は偽の昇降部（hve `tools/fake_lift_server.py`）にだけあり、ファームは未着手**（hve `WP-LIFT-14`）。カメラモジュールが本システムに開く口は未着手（hve [ImplementationPlan.md](../../../../hve-system/docs/plan/ImplementationPlan.md) §6 の行 8a） |
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
| `goto` | `press`・`target_mm`・`speed`（`lift_speed_mm_s`）。**`ceiling` は載せない**（距離計を持たない） |
| `release` | `press`（最後の `goto` と同じ値） |

繋ぐ先は `ws://<lift_host>/ws/module`。**`/ws/module` はコードの定数にし、パラメータにしない**（`/ws/ui` に繋ぐと天井の守りが外れる。hve と同じ扱い）。

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
| 4 | 自分の `goto` が `NOT_ARRIVED` で、`reason` が `CEILING_NEAR`・`CEILING_STALE`・`TOP`・`TARGET_RANGE` | `ceiling` | 光学中心が `min_camera_height_m` 以上なら続ける。届かなければ `camera_too_low` |
| 5 | 下降の `goto` が `NOT_ARRIVED` で、`reason` が `BOTTOM` | `none`（`arrived` を true） | （着いた扱い） |
| 6 | 自分の `goto` が `NOT_ARRIVED` で、上のどれでもない（`HEIGHT_UNKNOWN`・`ENCODER_STALL`・`ARM_LINK`・`MAX_RUN`・`CMD_TIMEOUT`・`OWNER_GONE`・`CMD_STOP`・**知らない名前**） | `refused` | `lift_refused` |
| 7 | 繋がっていない・`state` が `lift_state_timeout_ms` より古い | `refused` | `lift_refused` |

- 1〜3 は 2026-11-05 に hve へ答えた使い方のとおり（「`goto.status` が `PREEMPTED`、または `owner_operator` が `human` を見たら、すぐ止まり、戻さずに手を離す」）。**3 を足したのは、取って代わった相手を後片付けの `goto` で奪い返さないため**（`refused` にすると、`locate_core` が元の高さへ戻す `goto` を出す）。
- 1・3 は、着いて `goto` を送っていない間（`SCANNING`）も見る。**命令が無い間（`IDLE`）は見ない**（`stopped_reason` は `none`）。
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

**偽の天井の値を送る道具は作らない。**本システムが `ceiling_sensor: true` を名乗る接続を持つと、実機に向けたとき天井の守りが外れる。手で確かめるときの天井の値は、hve の偽のカメラモジュールから出す。

## 6. カメラモジュール（待ち）

hve が口を書いたら、本システムが確かめる（[Spec-open.md](../spec/Spec-open.md) §3 2026-11-05）。**確かめるときに見ること**（出どころのあるものだけ）:

| 見ること | 出どころ |
| --- | --- |
| 静止画: 1280×720・倍率に関係なく全体が写る 1 枚・要るときだけ要求 | [Spec.md](../spec/Spec.md) §1.1 |
| ピッチ: 目標角を指定して向けられる。今の目標角と、カメラモジュールが再起動したことが読める | [Spec-open.md](../spec/Spec-open.md) §3 2026-11-05（hve の答え） |
| ヨー: 向きを指定して向けられるか・今の向きが読めるか・届く範囲（[-open.md](DetailedDesign-open.md) §1 の 4）。hve の今の制御はヨーを速さで指示し、角度を持たない | hve `H-A11`・hve [DetailedDesign-protocol.md](../../../../hve-system/docs/plan/detailed/DetailedDesign-protocol.md) §5 |
| 人の割り込み: カメラ（ヨー・ピッチ）を人が動かしたことが分かるか | [-core.md](DetailedDesign-core.md) §6 |
| 省電力: 操作が 30 分無いと映像が止まり、ピッチの力が抜ける。本システムの操作・静止画の要求で戻るか、戻るまで何秒待つか | hve [DetailedDesign-power.md](../../../../hve-system/docs/plan/detailed/DetailedDesign-power.md)（本システムの口の案に `touch()` を書く、とある） |
| 元の向きに戻す: 依頼を受ける前のヨー・ピッチが読めるか | [Spec.md](../spec/Spec.md) §1.1 |
