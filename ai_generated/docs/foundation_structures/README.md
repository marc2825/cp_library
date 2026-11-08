# 共通基盤 F-08〜F-10

2026-10-19。残作業一覧の次の3項目を、5つの公開ヘッダ群へ整理しました。Hash Mapと複合hashは3.1、CSR Graphは共有graph（3.7の入口も提供）、Monoid／作用は3.3へ配置します。

| ID | 機能・説明 | 実装 | 操作 |
|---|---|---|---|
| F-08 | [seed付き複合キーハッシュ](../basic_utility/seeded_hash.md) | [hpp](../../basic_utility/seeded_hash.hpp) | [図解](../../visualizations/foundation_structures/index.html#seeded_hash) |
| F-08 | [Open Addressing Hash Map](../basic_utility/hash_map.md) | [hpp](../../basic_utility/hash_map.hpp) | [図解](../../visualizations/foundation_structures/index.html#hash_map) |
| F-09 | [CSR Graph・静的隣接リスト](../graph_basic/csr_graph.md) | [hpp](../../graph/csr_graph.hpp) | [図解](../../visualizations/foundation_structures/index.html#csr_graph) |
| F-10 | [Monoid／ActedMonoidカタログ](../range_query/monoid_catalog.md) | [hpp](../../range_query/monoid_catalog.hpp) | [図解](../../visualizations/foundation_structures/index.html#monoid_catalog) |
| F-10 | [等差数列・多項式加算と区間和](../range_query/polynomial_add_sum.md) | [hpp](../../range_query/polynomial_add_sum.hpp) | [図解](../../visualizations/foundation_structures/index.html#polynomial_add_sum) |

## 接続と契約

SeededHashは3.25のRandomを共有し、HashMapは独自のopen addressing実装です。固定seedで再現可能、既定seedはrandom_deviceから取得します。キー／値はコピー可能な型に限定し、再構築時の例外保証と参照の無効化を明示します。

CSRGraphは辺ID・入力順・無向の逆向きarcを保持します。有向arcのrevは-1。既存Graphとの相互変換はコピーで、既存のGraph専用アルゴリズムへのゼロコピー接続は提供しません。

MonoidSegtree／ActedMonoidLazySegtreeは既存Segtree／LazySegtreeの別名です。和・add/assign/affine作用、行列型と積は既存実装を共有します。追加のmin/max＋個数・最大部分配列・関数合成・一次分数・hash・等差／多項式加算について、結合順序と使用条件を個別説明に記録しています。

## 検証

```sh
make verify-foundation-structures
python3 verify/run_foundation_structures.py --sanitize
node verify/visualizations/foundation_structures_model_test.js
node verify/visualizations/foundation_structures_browser_test.mjs --browser /path/to/chromium
```

- 共通入口を含む6ヘッダの単独／重複include、全カテゴリall.hppとの混在、複数翻訳単位を検証します。5つの説明内usageはexamplesと内容・出力を照合します。
- HashMapはstd::mapとの60,000操作、定数hashによる2,000衝突キー、削除／再挿入・コピー／ムーブ・非既定構築の値・rehash時のコピー例外・容量超過・20万キーを確認します。
- CSRは500グラフをGraphと照合し、逆arcの双方向対応、自己ループ・多重辺・孤立点・空、往復変換、20万頂点を確認します。
- 1,093個の三値配列で全区間の最大部分配列和を直接探索。500列で関数の逐次評価、行列の列ベクトル作用、一次分数の無限遠点を含む合成を照合します。
- 多項式作用は次数0/1/4、31サイズ×400操作、分配・合成・点更新・空区間を検証します。既存affine作用の非可換な更新順も確認します。
- Python独立比較はhash1,500入力、CSR300グラフ、多項式8,000操作の合計4,470出力。Debug/Releaseでそれぞれ比較します。
- Associative ArrayとPoint Set Range Compositeの2ジャッジ形式を各24ケースでローカル検証。オンライン提出・ACは未確認です。
- 図解は5モード、monoidカタログ内の7種類を扱い、C++参照値と照合します。詳細は[図の範囲](../../visualizations/foundation_structures/README.md)。

[前回F-01〜F-07](../basic_utility/foundation.md) / [一次資料](references.md)
