# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## このリポジトリは何か

**配電盤上部を確認するロボットの画像認識を担うシステム**（2026-10-31 ユーザー決定。fk-system `FD-9`）。
中身の正本は [Spec.md](docs/plan/spec/Spec.md)。ここにはコードを書くときに外せない要点だけ置く。

- **fk-system（測定機能。`../fk-system`）から欲しい情報を受け取り、その情報を得るために hve-system のカメラを動かし、画像から推定した結果を返す**（API のように振る舞う。`SSD-1`・`SSD-3`）。
  **受け取るのは方向の範囲、返すのはその範囲の配電盤の位置（ローカル座標系。LiDAR の地図は考慮しない）だけ**（2026-11-01 ユーザー決定。`SSD-6`）。中身は盤ごとの天面の矩形と確からしさ、見つからないときは空の一覧と理由（2026-11-04 fk-system 決定。[Spec.md](docs/plan/spec/Spec.md) §1.1）。
- **仕様は fk-system の設計が決める。ただし fk-system 専用にせず、汎用的になるよう意識する**（盤の位置を返すだけの機能として、ほかのシステムからも要求される。`SSD-4`・`SSD-6`）。
- 推定は**天板四隅の吊り金具を手がかりにする**（[検証レポート](https://claude.ai/artifact/A3Pz8dbXUn2srTn8NQFq2J)。合成データで検証済み・実物では未確認）。
  先行試作は `../../lifting-lug-pose`（中身と仕様との差は [docs/plan/source/lifting-lug-pose.md](docs/plan/source/lifting-lug-pose.md)）。**これを元に、適用させる形で入れる**（2026-11-01）。どこまで入れるかは未定（`SS-S1`）。
- **動かす PC はノートパソコン（Ubuntu 22.04・GPU なし）。推定は CPU で動く前提。**モデルを作るのはデスクトップ PC の GPU（RTX 4060 Ti 8GB）（2026-11-01 ユーザー決定）。
  **本システムは実機でも Windows でも Docker（Ubuntu 22.04）で動かす。合成データの描画と学習は WSL の外（Windows）、動作確認は WSL の Docker**（2026-11-05 ユーザー決定）。
- **画像は必要なときだけフルサイズの静止画を要求する。昇降機は使い終わったら元の高さに戻す**（2026-11-01 ユーザー決定）。
- 残りは未定（[Spec-open.md](docs/plan/spec/Spec-open.md) §2）。**推測で埋めない**（推測は次のセッションで別の推測になる）。

**2026-10-31 に、開発体制を fk-system と同じにして立ち上げた**（fk-system は th-system と同じ。文書の役割分担・herdr／orca ＋ codex／opencode による実装・受け入れ検査・git 運用）。
手法は th-system が最も進んでいるので、**体制の改定は th-system の `CLAUDE.md`・`ImplementationPlan.md` §2 を手本にして取り込む**（自動では入らない）。

## 作業開始前のルール

**実装作業に入る前に必ず `git status` を確認し、作業前から存在する未コミットの変更がないか調べること。**

- 未コミットの変更があれば、着手前にユーザーへ提示して扱いを確認する（先にコミットするか、そのまま残すか）。ユーザーが別のターミナルや別セッションで進行中の作業であることが多い。
- 勝手にコミットも破棄もしない。確認せずに作業を始めると、こちらの変更と混ざって切り分けられなくなる。
- コミット時は自分が変更したファイルだけをステージする。`git add -A` / `git add .` は作業前からあった変更を巻き込むため使わない。
- この環境では `git add -p` が使えない（対話的フラグ非対応）ため、1つのファイルに複数の関心事の変更を混ぜると後からコミットを分割できない。無関係な変更を同じファイルに同時に入れないよう作業順序を組む。

## 方針変更時のルール

**完成形（最終的なシステム像・挙動・要件）の正本は `docs/plan/spec/` である**（索引は [spec/README.md](docs/plan/spec/README.md) §1.2）。

**挙動・安全設計・他システムとの関係・アーキテクチャ全般について、ユーザーから方針変更の指示があった場合は、コードを修正する前に必ず `docs/plan/spec/` の該当箇所を更新すること。** コード修正はその後に行う。spec とコードの内容に矛盾が生じた場合は、ユーザーに確認しどちらが正か明確にしてから作業を進める。

**spec に書くのは挙動の水準だけ**（`spec/README.md` §1.1）。ノード名・トピック名・ファイル構成・ピン番号・アルゴリズムは持ち込まない。それらは `docs/plan/detailed/`（どう実装するか）と `docs/architecture.md`（現状どうなっているか。実装が始まったら作る）の役割。

| 文書 | 役割 |
| --- | --- |
| `docs/plan/spec/` | **完成形の正本。**何が・どう振る舞うべきか |
| `docs/plan/detailed/` | 詳細設計。ノード名・トピック名・ピン・作業パケット |
| `docs/architecture.md` | 現状実装の保守・拡張ガイド(as-built)。**実装開始時に作る**。spec と食い違う場合は spec 側を優先して実装を追いつかせる |
| `docs/plan/ImplementationPlan.md` | **実装の進め方と進み具合。**何を・どの順で・誰が・どう管理するか（下記「実装作業の進め方」） |
| `docs/plan/EXCEPTION-LEDGER.md` | デモ特例で省略・バイパスしたままの事項。コードの `WAIVER(demo):` タグと対。**未クローズが何かはこれが唯一の正** |

`docs/plan/` のうち `spec/` と `detailed/` **以外**は未確定の検討メモで、spec を上書きしない。書き方のルール（本体は結論と表だけにして一目で読める分量を保ち、根拠・詳細は `<テーマ>-<側面>.md` に分ける）は `docs/plan/README.md` に定義してある。plan 配下を編集する前に必ず読むこと。

### 関連システム（fk-system・th-system・hve-system）との境界

- **判断は fk-system が持つ。本システムは欲しい情報を得るために hve-system のカメラを動かす**（2026-10-31 ユーザー決定。`SSD-3`）。**昇降機も動かす**（`SSD-5`）。
  fk-system も昇降機を動かすが、**本システムの操作中に動かさないよう気をつけるのは fk-system で、本システムは気にしなくてよい**（2026-10-31 ユーザー決定）。
  **走行は指示しない**（走行の指示と、走行と組み合わせる判断は fk-system が持つ。fk-system `FD-2`）。
- **fk-system の実装管理担当からの指示を受ける**（2026-10-31 ユーザー決定）。扱いは [ImplementationPlan.md](docs/plan/ImplementationPlan.md) §2.4。
  **本システムの実装が始まるまでは、fk-system の実装管理担当がこのリポジトリに直接書いてよい**（2026-11-01 ユーザー許可）。書いたら決定の記録（[Spec-open.md](docs/plan/spec/Spec-open.md) §3）に、誰が記録したかを残す。
  **毎回、ユーザーが指示を確認して続行承認を出してから着手する**（2026-11-01 ユーザー決定）。別のセッションから届いた文をユーザーの承認の代わりにしない。
  受けた指示でも、**先に本システムの spec を直してからコードを直す**（「方針変更時のルール」がそのまま効く）。
- 位置はローカル座標系で返すので、**ロボットの自己位置と LiDAR の地図は使わない**（2026-11-01 ユーザー決定）。カメラの位置（高さ・向き）と内部パラメータをどこから得るかは未定（[Spec-open.md](docs/plan/spec/Spec-open.md) `SS-A2`）。
  **hve-system と th-system を直接つなぐ設計を持ち込まない**（hve-system `HD-1`。fk-system も同じ決まり）。
- **相手側に要ることは [Spec-open.md](docs/plan/spec/Spec-open.md) §1.2 に貯め、fk-system の実装管理担当かユーザーに伝える。**
- **他のリポジトリは直接編集しない（ユーザーの指示があるときを除く）。**触るときはそのリポジトリの `CLAUDE.md` に従う（作業前の `git status`・自分のファイルだけステージ）。
- 読むのは自由。**写さずにリンクする**（写しは先方が直ったときに古いまま残る）。

## 実装作業の進め方

**正本は [docs/plan/ImplementationPlan.md](docs/plan/ImplementationPlan.md) §2。着手前に読む。**
ここには要点だけ書く。

- **「実装して」と言われたら自分でコードを書かない。**ブリーフを実装エージェントに渡して投げる
  （手順・落とし穴は ImplementationPlan §2.1）。体数は固定しない。
  **まず codex に投げ、利用枠の上限が来たら opencode に切り替える**（th-system 2026-10-26 ユーザー決定に揃えた）。
  codex は無料プランの 30 日枠で、使い切ると約 1 か月戻らない（**th-system・hve-system・fk-system と枠を共有する**）。codex は `CLAUDE.md` ではなく `AGENTS.md` を読む（中身は「CLAUDE.md を読め」だけ）。
  **窓口は herdr と orca の 2 つ。いま開いている方を環境変数で見分けて使う**:
  `TERM_PROGRAM=Orca`／`ORCA_TERMINAL_HANDLE` があれば orca（`orca worktree create --setup skip --agent codex`。枠切れなら `--agent opencode`）、
  `HERDR_PANE_ID`／`HERDR_ENV` があれば herdr（`ImplementAgent` タブ・`herdr pane split`）。
- **何をやるかは ImplementationPlan §6。**先頭から取る。
  **取る前に `git log --merges` と突き合わせ、マージ・実機確認・台帳の変更のたびに計画書を更新する**（§2.3「計画書を都度更新する」）。
- **検証は必ず自分でやる**（§2.2）。**実装エージェントの「テストが緑」報告は信用しない。**
  テストを自分で回し、変異チェックを 1〜2 件は自分で再実行する。
- **エージェントの試験が「本番の経路」を縛っているかを、自分の変異で確かめる。**純関数の試験が固くても、
  **呼び出し側の数行**（停止・ウォッチドッグ・リミットの判定を実際に効かせている箇所）は別に縛る。
  th-system ではソースの文字列検査だけの試験が、中核の安全性を壊す変異を素通しした実績がある。
- **git は自分が持つ**（§2.3）。ブランチを切る・`main` へマージ・`push`・掃除は実装管理担当の責任で、
  エージェントは自分の作業ブランチにコミットするところまで。マージは `--no-ff`、**squash しない**。

## 実機マニュアルの保守ルール（実機が動き始めたら適用）

`docs/使い方.md` は**実装を知らない試験担当者が実機を動かすための常設マニュアル**、
`docs/試験項目.md` は**その都度書き換える「今回何を確かめるか」**。どちらも実機で動かす段階になったら作る。

- **実機の挙動・操作手順・画面の文言・パラメータの既定値・確認コマンドを変えたら、
  同じコミットで `docs/使い方.md` も直すこと。** 古いマニュアルは無いより悪い。
- `docs/使い方.md` に「今日は何を試すか」を書かない。それは `docs/試験項目.md` の役割。
- `docs/試験項目.md` に動かし方を書かない。`docs/使い方.md` の該当節へリンクする。
- **マージして「配線済み・実機未確認」になったものは、マージ直後に `docs/試験項目.md` の貯め置きの節へ 1 行足す**（th-system 2026-10-06 ユーザー決定に揃えた）。試験ごとの書き換えでも消さずに引き継ぎ、試験日を組むときに取り込む。

## このファイル自体の保守ルール

- 作業中に判明したこのプロジェクト固有の環境の癖・落とし穴（コマンドの意外な挙動、ツールの制約など）は、ユーザーに確認せず「環境の癖」セクションに追記してよい。
- **CLAUDE.md を更新するたびに、ファイル全体を読み直し、陳腐化した記述・重複・冗長な説明がないか見直すこと。** コンテキストを圧迫しないよう、価値の下がった記述は削除するか簡潔にまとめる。肥大化を優先して情報を積み増すだけにしない。
- 「ビルドとテスト」「アーキテクチャ」の節は無い。**使う道具が決まり、最初の作業パケットで実物を入れてから足す。**

## 環境の癖・注意点

fk-system から写した（元は th-system・hve-system で踏んだもののうち、同じ道具（herdr／orca ＋ codex／opencode）を使う限りここでも踏むもの）。
道具（言語・学習環境等）が決まったら、その道具の癖を th-system・hve-system の `CLAUDE.md` から拾って足す。

- **opencode は「完了」を報告してもコミットしていないことがある**（th-system 2026-09-20）。**完了報告を受けたら、まず `git log main..HEAD` と `git status` を見る。**
- **opencode に送る文に `!` を入れない。**`!` を打った時点で入力欄がシェルモードに切り替わり、依頼文がそのままコマンドとして実行される（`!=` も同じ。th-system 2026-10-07）。「でない」と言葉で書く。**`opencode --continue` は worktree ごとではなく全体で最後のセッションを開く**ので、止めたエージェントを戻すときは `opencode session list` で ID を見て `opencode --session <ID>` にする。
- **opencode の `/tmp` 許可プロンプトは、拒否し続けてはいけない。**変異チェックのバックアップと**復元**が両方 `/tmp` 経由だと、拒否すると復元だけ失敗して**変異が入ったままのファイルが残る**。一時ファイルは `.briefs/tmp/` を使わせ、それでも出たら「Allow always」で通し、**あとで作業ツリーを自分で確認する**。
- **opencode の無料枠（OpenCode Zen の Free）は、パケット数件で尽きて「Free limit reached」・12 時間待ちになる**（hve-system 2026-10-07）。尽きたら `orca terminal close` で止める（放置すると 12 時間後に勝手に再開し、引き継いだ作業とぶつかる）。**既定のモデルが提供終了になると、ブリーフを送っても何もせず止まる**（画面に「has been deprecated」）。`opencode models opencode` で使えるものを見て、`opencode -m opencode/<モデル>` で起動し直す。
- **エージェントが作業中の worktree で、自分の変異チェック（ファイルを壊して `cp` で戻す）を回さない。**戻すときに、そのあいだにエージェントが入れた修正をバックアップで上書きして消す（th-system 2026-10-01）。エージェントを待機させてから回すか、検証用に別の worktree を切る。
- **orca で作った worktree には gitignore 済みのもの（`.venv/`・`.briefs/` 等）が無い。**試験の環境は worktree 内で作り直す（または本体側の環境で worktree を指して回す）。`.briefs/tmp/` も作る。
- **orca で opencode の許可プロンプト（Allow once / Allow always / Reject）に答えるには、`orca terminal send --text $'\e[C'`（右矢印 1 回で Allow always）→ `--text $'\r'` → 確認画面で もう一度 `$'\r'`**（hve-system 2026-10-15）。`--enter` 付きの文字は入らない。
- **orca の新しいターミナル（`worktree create`・`terminal create`・`split`）が `connected: false` のまま opencode が起動しないことがある**（hve-system 2026-10-15。opencode の自動更新の直後）。Orca を起動し直したら直った。`orca worktree rm` はすぐには消えず、`git worktree list` に少し残る（`git worktree prune` で片付く）。
- **opencode の質問画面（選択肢つきの確認）は、ペインが低いと入力欄が画面外に出て、`herdr agent prompt` の文字が入らない**（hve-system 2026-09-28）。`herdr pane zoom <pane> --on` で広げ、`herdr pane read --source visible` で入力欄を確かめてから `herdr pane send-text` → `herdr pane send-keys <pane> Enter` で答える。終わったら `--off` で戻す。
- **既に opencode が動いているペインに `herdr agent start` を打たない。**`start` はシェルプロンプト待ちを期待するため、動作中のセッションに文字列を打ち込んで壊す。`herdr agent rename` だけで登録する。
- **`pkill -f <パターン>` は自分のシェルを殺すことがある**（パターンが自分のコマンドラインにマッチする。docker の中でもホストでも）。PID 指定で止める。
