# 詳細設計書 — ss-system

[仕様書](../spec/Spec.md) を実装できる形に落とす。読み方は [README.md](README.md)。

> **2026-11-05: 盤の位置を探す口（[-locate.md](DetailedDesign-locate.md)）の案を書き、同日 fk-system が確かめた。**ほかは未定。

---

## 1. 設計方針

| ID | 方針 |
| --- | --- |
| **`DD-1`** | **名前を発明する余地をゼロにする。**必要な名前が無いと分かったら、実装する前に [-names.md](DetailedDesign-names.md) へ行を足す |
| **`DD-2`** | **判定ロジックはハードウェア・通信から切り離した純関数にする。**ホストで試験できるようにし、呼び出し側の配線は別の試験で縛る（th-system `follow_planner_core.py` と同じ二層構造） |
| **`DD-3`** | **裸の数値を書かない。**値には出どころ（実測・先方・逆算・暫定）を付ける |

## 2. 全体構成

| 項目 | 案 | 出どころ |
| --- | --- | --- |
| 依頼する側との口 | ROS 2 のアクション＋後片付けの完了＋生存と状態（[-locate.md](DetailedDesign-locate.md)。**案**） | [Spec.md](../spec/Spec.md) §1.1・§1.2 |
| hve-system との口 | hve が書いたもの（[-names.md](DetailedDesign-names.md) §1） | hve `DetailedDesign-protocol-modules.md` |
| 中の構成 | 未定（`SS-S1`。先行試作の取り込みと一緒に書く） | — |

## 3. 段階と作業パケット

未定。仕様書の章ができたら、th-system と同じく**各段階の終わりに必ず「動くもの」が残る**ように切り、
`WP-<領域>-<番号>` で番号を振る。一覧と状態は [ImplementationPlan.md](../ImplementationPlan.md) §4 に置く。
