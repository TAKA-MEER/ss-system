# ss-system

配電盤上部を確認するロボットの**画像認識**を担うシステム。
**fk-system（測定機能）から欲しい情報を受け取り、その情報を得るために hve-system のカメラを動かし、画像から推定した結果（盤の配置図・天面の位置など）を返す。**
仕様は fk-system の設計が決めるが、fk-system 専用にせず汎用的になるよう意識する（将来作るかもしれない th-system の前日準備の自動化にも）。
推定は天板四隅の吊り金具を手がかりにする（[検証レポート](https://claude.ai/artifact/A3Pz8dbXUn2srTn8NQFq2J)）。
開発体制は fk-system・th-system と同じにしてある。

**現在は設計段階。**コードはまだ無い。

| 知りたいこと | 見る文書 |
| --- | --- |
| 何を作るのか・どう振る舞うべきか | [docs/plan/spec/README.md](docs/plan/spec/README.md)（完成形の正本） |
| 何が決まっていないか | [docs/plan/spec/Spec-open.md](docs/plan/spec/Spec-open.md) |
| どう実装するか | [docs/plan/detailed/README.md](docs/plan/detailed/README.md) |
| 実装の順番・進み具合 | [docs/plan/ImplementationPlan.md](docs/plan/ImplementationPlan.md) |
| 作業のしかた（人・AI 共通） | [CLAUDE.md](CLAUDE.md)（codex は [AGENTS.md](AGENTS.md) から入る） |

関連リポジトリ（同じ `TM/` 配下）:

| リポジトリ | 中身 | 本システムとの関係 |
| --- | --- | --- |
| `../fk-system` | 測定機能（膜厚・板厚）。走行系・昇降機・カメラを組み合わせて、測定を自動で行う | **欲しい情報を渡してくる相手。本システムの仕様を決める。**実装管理担当からの指示を受ける |
| `../th-system` | 移動システム（走行部・WebUI・安全・試験場内の移動） | ロボットの自己位置を持つ。開発の手法はここが最も進んでいる |
| `../hve-system` | th-system の走行部に載る昇降機と上部カメラ | **本システムがカメラと昇降機を動かして画像を得る**（昇降機は fk-system も使う） |
| `../../lifting-lug-pose`（git 管理外） | 吊り金具による配置推定の先行試作（検証レポートのコード・モデル） | 引き継ぐかは未定 |
