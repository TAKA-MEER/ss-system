# docs/plan/source — 原典・先行試作・関連リポジトリへの参照

spec に統合する前の材料。**仕様の正ではない。**spec に取り込んだら、取り込んだ先を §3 に書く。

## 1. 先行試作・原典

別リポジトリ・別フォルダにあるものは**写さずに**場所だけ書く（複製すると、元が直ったときに古い写しが残る）。

| 材料 | 場所 | 中身 |
| --- | --- | --- |
| 吊り金具による配電盤配置推定 検証レポート（2026-10-31） | [https://claude.ai/artifact/A3Pz8dbXUn2srTn8NQFq2J](https://claude.ai/artifact/A3Pz8dbXUn2srTn8NQFq2J) | 合成データでの検証結果（方法・精度・失敗の傾向・実物向けに要る画像と枚数・課題） |
| 先行試作 `lifting-lug-pose` | `../../lifting-lug-pose`（git 管理外。`TM/` の 1 つ上） | 合成データの生成・金具の検出（2 段）・配置の当てはめ・評価のコードと、学習済みモデル・合成データ。使い方はそのフォルダの `README.md` |

## 2. 関連リポジトリ

| リポジトリ | 入口 | 中身 |
| --- | --- | --- |
| fk-system | `../fk-system/docs/plan/spec/README.md` | 測定機能の完全設計書（正本）。本システムに情報を要求する側 |
| th-system | `../th-system/docs/plan/spec/README.md` | 移動システムの完全設計書（正本） |
| hve-system | `../hve-system/docs/plan/spec/README.md` | 昇降機・上部カメラの完全設計書（正本） |

パスはリポジトリ直下（`ss-system/`）基準。

## 3. spec への取り込み状況

| 材料 | 取り込み先 |
| --- | --- |
| 検証レポート | 方法の手がかり（吊り金具）と前提だけ [Spec.md](../spec/Spec.md) §1.2（2026-10-31）。数値は写さずリンク |
