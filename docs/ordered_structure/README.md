# 3.5 平衡二分探索木・永続データ構造

今回の S 項目は、標準連想コンテナの典型操作と、座標圧縮＋BIT による順位管理です。C++17、`namespace cp`、0 始まりの順位を使います。必要なヘッダだけ include でき、まとめる場合は `ordered_structure/all.hpp` を指定します。

| やりたいこと | 入口 | 説明 | 図解 |
|---|---|---|---|
| set/map/multiset の挿入・検索・削除、前後の要素 | [`ordered_associative.hpp`](../../ordered_structure/ordered_associative.hpp) | [典型操作・計算量・usage](ordered_associative.md) | [標準コンテナ](../../visualizations/ordered_structure/index.html#set) |
| 元の値を残して座標圧縮、値域の境界を探す | [`CoordinateCompression<T,Compare>`](../../ordered_structure/coordinate_compression.hpp) | [説明・usage](coordinate_compression.md) | [座標と添字](../../visualizations/ordered_structure/index.html#compression) |
| 重複を含む個数・順位・k 番目・前後検索 | [`CompressedOrderStatistics<T,Compare>`](../../ordered_structure/compressed_order_statistics.hpp) | [説明・usage](compressed_order_statistics.md) | [頻度・BIT・k 番目](../../visualizations/ordered_structure/index.html#statistics) |

値が逐次増える前後検索には std::set/map/multiset を使います。挿入される値を先に集められ、順位や k 番目も必要なら圧縮＋BIT を選べます。標準コンテナをラップし直さず、そのまま使える補助関数と使用例を用意しています。

## 共通の契約

- 順序・同値判定は比較器に従います。同値は `!comp(x,y) && !comp(y,x)` であり、T の `==` は不要です。
- 既定は `std::less<T>`。greater なら降順です。「前／後」「未満／以下」は比較器が定める順序で解釈します。
- 標準コンテナの補助検索は、見つからない場合にそのコンテナの end() を返します。圧縮版の k 番目・前後検索は optional を返します。-1 の値そのものも格納できます。
- 比較・値のコピーが O(1) の場合の計算量を記載します。文字列等ではその分増えます。
- 圧縮版は登録済みの座標に対してのみ挿入できます。**問い合わせの閾値は未登録でも使えます。**
- BIT の頻度は非負の long long。個数不足・負の個数・総数の上限超過は false を返し、内容を変更しません。

## 検証

```sh
python3 verify/run_ordered_structure.py
python3 verify/run_ordered_structure.py --sanitize
node verify/visualizations/ordered_structure_model_test.js
```

4 ヘッダの単独／重複 include（debug/release）、逆順 include、複数翻訳単位、3 使用例、4 ジャッジ用プログラムを確認します。5 座標の頻度 0/1/2 を昇順・降順で全探索した 486 状態、400 ランダム履歴・152,000 更新操作と素朴な列を比較します。

空集合、未登録値、同値、個数不足、k の範囲外、const 検索、異種キー検索、コピーできない mapped 型での検索・削除、カスタム比較器、デフォルト構築／大小・等値演算子のない値型、LLONG_MAX の個数、20 万座標を含みます。オンライン提出による AC を表すものではありません。

[動く使用例](../../examples/ordered_structure/) / [検証プログラム](../../verify/ordered_structure/) / [参考資料・設計判断](references.md) / [すべての図解](../../visualizations/index.html)
