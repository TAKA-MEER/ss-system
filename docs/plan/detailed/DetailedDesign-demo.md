# 金具認識のリアルタイム表示（本番外の道具）

[DetailedDesign.md](DetailedDesign.md) の詳細。**状態: 2026-11-08 案。**入力は [Spec-demo.md](../spec/Spec-demo.md)。
名前は [-names.md](DetailedDesign-names.md) §6、作業パケットは [-packets.md](DetailedDesign-packets.md) `WP-DEMO-01`。

## 0. 結論

| 項目 | 案 |
| --- | --- |
| 置き場所 | パッケージ `ss_locator` の中に、純関数の `demo_view.py` とつなぐ層の `lug_demo.py` を足す（`DD-2`）。本番のノード（`locator_node.py`）からは読み込まない |
| 起動 | コンテナの中で `ros2 run ss_locator lug_demo --source <入力>`（ROS 2 の通信は使わない。起動の入口として使うだけ） |
| 画面 | OpenCV のウィンドウ（`cv2.imshow`）。ホストの X に出す（§4）。**`--headless` では画面を出さず、1 コマごとの結果を標準出力に 1 行ずつ出す**（受け入れ検査と、画面の無いところでの確認に使う） |
| 検出 | 本番と同じ `Detector`（`detector.py`）。重みの SHA-256 の照合もそのまま。**枠を出すため、`Detector` に枠も返す `detect_with_boxes` を足す**（`detect` の結果は変えない） |
| 既定値 | 重み・SHA-256・`imgsz`・確信度のしきい値は、**本番のパラメータファイル（`config/locator.yaml` の `locator_node` の下）から読む**。デモの側に値を書かない |
| hve-system の静止画 | **作らない**（[Spec-demo.md](../spec/Spec-demo.md) §2 の待ち。`SS-A7`）。`--source` に `hve` を受け付ける形も今は作らない |

## 1. 入力（`--source`）

| 書き方 | 入力 | 捨て方 |
| --- | --- | --- |
| `usb:<番号>`（例 `usb:0`） | USB カメラ（`cv2.VideoCapture(<番号>)`）。読む側を別のスレッドで回し続け、**最新の 1 コマだけを持つ**（§2 `LatestFrame`） | 推定しているあいだに来たコマは、最新の 1 コマを残して捨てる |
| 動画のファイル | `cv2.VideoCapture(<パス>)`。**元の fps の速さで読み進める**（実時間で流したときと同じ捨て方にする）。最後まで来たら終わる | USB カメラと同じ |
| 画像のフォルダ | フォルダ直下の `.jpg`・`.jpeg`・`.png`（大文字小文字を問わない）を**名前順に 1 枚ずつ** | **捨てない**（推定が終わったら次の 1 枚）。最後まで来たら終わる |

どの入力も、推定の前に `fit_to_canvas`（§2）で 1280×720 に合わせる。読めなかったコマ・画像は飛ばし、標準エラーに 1 行出す。
USB カメラが開けないときは、理由を標準エラーに出して 1 で終わる。

## 2. 純関数（`demo_view.py`）

ROS 2・ultralytics を読み込まない。numpy と cv2（描画と縮小・拡大だけ）を使う。

| 名前 | 中身 |
| --- | --- |
| `fit_to_canvas(image, width=1280, height=720)` | 倍率 `s = min(width / w, height / h)`（**1 より大きくてもよい。小さい映像も拡大する**）で縦横比を保って縮小・拡大し、`width`×`height` の黒い画像の中央に置く。返すのは `(canvas, s, (pad_x, pad_y))` |
| `draw_lugs(canvas, boxes, keypoints, confidences, info)` | `canvas` の写しに描く。金具ごとに枠（`boxes` (N,4) の xyxy）・キーポイント 4 点（根元・リング中心・両端の順に色を変えた円。両端は線で結ぶ）・確信度（枠の上に小数 2 桁）。左上に `info` の行（下の表）。文字は ASCII だけ（`cv2.putText` は日本語を描けない） |
| `LatestFrame` | スレッドをまたいでコマを 1 つだけ持つ箱。`put(frame)` は前のコマを上書きし、上書きで捨てた数を `dropped` に足す。`take(timeout_s)` は持っているコマを返して空にする。空なら `timeout_s` まで待ち、来なければ `None`。`close()` のあとの `take` は `None` |

左上の行（`info`）:

| 項目 | 例 |
| --- | --- |
| 金具の数 | `lugs: 3` |
| 1 コマの推定にかかった時間（`Detector` の呼び出しの前後） | `infer: 183 ms` |
| しきい値 | `conf >= 0.25` |
| 捨てたコマの数（起動からの合計。フォルダでは 0） | `dropped: 12` |

## 3. つなぐ層（`lug_demo.py`）

| 名前 | 中身 |
| --- | --- |
| `load_defaults(yaml_path)` | `locator.yaml` の `locator_node.ros__parameters` から `model_file`・`model_sha256`・`detect_imgsz`・`detect_conf` を読んで返す。`yaml_path` の既定は `ament_index_python` で引いた `share/ss_locator/config/locator.yaml` |
| `run_demo(source, detector, show, conf, max_frames=None)` | 推定の繰り返し。`source` からコマを取り → `fit_to_canvas` → `detector.detect_with_boxes` → `draw_lugs` → `show(画像, 結果)`。**描くのは、推定したそのコマの上**（推定のあいだに来た新しいコマの上に古い結果を描かない）。`show` が `False` を返したら終わる。`max_frames` 枚推定したら終わる。`detector` と `show` は差し替えられる（試験で偽物を入れる） |
| `main(argv)` | 引数（下の表）を読み、`Detector` を作って `run_demo` を回す。画面ありなら `show` は `cv2.imshow` と `cv2.waitKey(1)`（`q` か Esc で `False`）。`--headless` なら `show` は 1 行を標準出力に出すだけ |

| 引数 | 既定 | 中身 |
| --- | --- | --- |
| `--source` | （必須） | §1 |
| `--conf` | `locator.yaml` の `detect_conf` | 確信度のしきい値（[Spec-demo.md](../spec/Spec-demo.md) §2「起動するときに変えられる」） |
| `--model` | `locator.yaml` の `model_file`（`/ws` からの相対） | 重みのファイル |
| `--headless` | なし | 画面を出さない |
| `--max-frames` | なし（終わりまで） | 推定する枚数の上限 |

`--headless` で 1 コマごとに出す行: `frame=<推定した通し番号> lugs=<数> infer_ms=<整数> dropped=<合計>`。

**保存しない**（[Spec-demo.md](../spec/Spec-demo.md) §2）: `cv2.imwrite`・`cv2.VideoWriter` を使わない。ファイルに書くものは何も無い。

## 4. 動かし方

**ふつうは起動スクリプト `scripts/demo.sh` を使う**（§4.1。2026-11-08 ユーザー依頼）。下は中でやっていること。
ホスト（ノートパソコンの Ubuntu 22.04）で X の接続をコンテナに許してから動かす。

```bash
xhost +local:
docker run --rm -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device /dev/video0 -v "$PWD":/ws ss-system:dev bash -c \
  "cd /ws/ss_ws && colcon build && . install/setup.bash && cd /ws && ros2 run ss_locator lug_demo --source usb:0"
```

- 動画・フォルダのときは `--device` を外し、`--source /ws/<パス>` にする。
- Windows（WSL）では WSLg が `DISPLAY` を用意する。USB カメラを WSL に渡すには `usbipd` が要る（未確認）。
- 画面を出すのに要るライブラリ（Qt の X 用）は Dockerfile の apt に足す（`WP-DEMO-01`）。

## 5. 試験の考え方

推定の速さはパソコンで変わるので、**試験は偽の検出器（決まった時間だけ待って決まった結果を返す）と、メモリの中で作ったコマで回す**。学習済みモデルが要る試験は、モデルが無いとき skip する（`test_detector.py` と同じ）。
**呼び出し側を縛る**: 「最新のコマだけを推定する」「推定したコマの上に描く」「しきい値が `Detector` まで届く」「何も書かない」は、`run_demo`・`main` を通して確かめる（純関数の試験だけにしない）。

### 4.1 起動スクリプト（`scripts/demo.sh`）

ホストで、リポジトリのどこからでも `scripts/demo.sh [lug_demo の引数…]` で起動する。bash で書く。

| 振る舞い | 中身 |
| --- | --- |
| 作業場所 | スクリプトの置き場所からリポジトリ直下を求め、そこを `/ws` に bind mount する（呼んだ場所に依らない） |
| 引数 | `--dry-run` だけはスクリプトのもの（取り除く）。**ほかはそのまま `lug_demo` に渡す**。`--source` が無ければ `--source usb:0` を足す |
| 事前の確かめ | `model/lug_v3_s.pt` が無ければ、置き場所を書いて 1 で終わる（SHA-256 の照合は `lug_demo` に任せる）。イメージ `ss-system:dev` が無ければ `docker/Dockerfile` から作る |
| USB カメラ | `--source usb:<N>` なら `--device /dev/video<N>` を付ける。`/dev/video<N>` が無ければ、そう書いて 1 で終わる |
| 動画・フォルダ | `--source` がホストに在るパスなら、**絶対パスにして、その場所を読み取り専用で `/input` に bind mount し、`--source` を `/input` 側に書き換える**（ファイルなら親のフォルダを mount してファイル名を付ける）。在らなければ、そう書いて 1 で終わる |
| 画面 | `--headless` が無いときだけ: `DISPLAY` が空なら、そう書いて 1 で終わる。`xhost +local:docker` を打ち（`xhost` が無ければ飛ばす）、`-e DISPLAY` と `/tmp/.X11-unix` の mount を付ける |
| 端末 | 標準入力が端末なら `-it`、そうでなければ `-i` も `-t` も付けない |
| コンテナの中 | `colcon build`（出力は捨て、失敗したら止まる）→ `install/setup.bash` → `/ws` で `ros2 run ss_locator lug_demo <引数>` |
| `--dry-run` | 上の確かめのうちモデル・イメージ・`DISPLAY`・`xhost` を飛ばし、**打つはずの `docker run` の引数を 1 行に 1 つずつ標準出力に出して 0 で終わる**（試験用。`docker` も `xhost` も呼ばない） |

終了コードは `lug_demo` のものをそのまま返す。
