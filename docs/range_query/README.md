# 3.3 セグメント木・Fenwick Tree

0 始まり・半開区間 `[l,r)` に統一した、C++17 の区間クエリ用ライブラリです。必要なヘッダだけ include できます。まとめて使う場合は `range_query/all.hpp` を指定します。

| やりたいこと | 型・実装 | 詳細 | 図解 |
|---|---|---|---|
| 一点加算・区間和、頻度の k 番目 | [`cp::FenwickTree<T>`](../../range_query/fenwick_tree.hpp) | [API・usage](fenwick_tree.md) | [BIT と lower_bound](../../visualizations/range_query/index.html#fenwick) |
| 一点代入・区間の集約・境界探索 | [`cp::Segtree<S,op,e>`](../../range_query/segtree.hpp) | [API・usage](segtree.md) | [区間の分解](../../visualizations/range_query/index.html#segtree) |
| 区間更新・区間の集約・境界探索 | [`cp::LazySegtree<S,op,e,F,mapping,composition,id>`](../../range_query/lazy_segtree.hpp) | [API・usage](lazy_segtree.md) | [遅延・伝播](../../visualizations/range_query/index.html#lazy) |
| 区間更新・一点取得 | [`cp::DualSegtree<S,F,mapping,composition,id>`](../../range_query/dual_segtree.hpp) | [API・usage](dual_segtree.md) | [更新の合成](../../visualizations/range_query/index.html#dual) |
| 区間加算・区間和に特化 | [`cp::RangeAddRangeSumFenwickTree<T>`](../../range_query/range_add_range_sum_fenwick_tree.hpp) | [API・usage](range_add_range_sum_fenwick_tree.md) | [2 本の BIT](../../visualizations/range_query/index.html#range-bit) |

和だけなら Fenwick Tree、逆演算のない min/max や順序のある関数合成なら Segtree を選びます。区間更新が必要なら Lazy、取得が一点だけなら Dual を選べます。加算と和だけの区間操作には 2 本の BIT が簡潔です。

## 共通の契約

- `size()` は論理要素数です。長さは `0 <= n <= 100000000`、実際の上限は使用できるメモリによります。
- `0 <= p < n`、`0 <= l <= r <= n` を呼び出し側で守ります。デバッグ時は `assert` で検査します。
- 空配列・空区間に対応します。空区間の集約は単位元、更新は何もしません。一点操作は空配列には使えません。
- 演算・コピーが O(1) の場合の計算量を記載しています。文字列連結など、サイズに依存する演算ではその分増えます。
- 整数の中間値も型に収まる必要があります。signed 整数のオーバーフローを処理する機能はありません。
- Lazy と Dual の `composition(f,g)` は「古い `g` を適用してから、新しい `f`」です。

## 検証

```sh
python3 verify/run_range_query.py
python3 verify/run_range_query.py --sanitize
node verify/visualizations/range_query_model_test.js
```

8 ヘッダの単独 include と再 include（debug/release）、逆順 include、複数翻訳単位、3 使用例、5 ジャッジ形式の入出力、720 ランダム系列・154,400 操作と素朴配列の比較、20 万要素を検証します。境界探索、0 の並ぶ累積和、負数、剰余型、文字列の順序、加算／代入／アフィン変換の合成を含みます。

ジャッジ用プログラムは [`verify/range_query/`](../../verify/range_query/) にあります。Library Checker / AOJ の形式でローカル比較しています。オンライン提出による AC を表すものではありません。

旧 `data_structure/segment_tree.hpp` の `segtree` は新実装への別名です。旧 `BIT` は公開メンバ `n/bit/val` の互換性のため保持しています。新規コードはこのカテゴリの API を使ってください。

[主要資料と設計判断](references.md) / [動く C++ 使用例](../../examples/range_query/) / [説明一覧](../README.md) / [すべての図解](../../visualizations/index.html)

## 高度なアルゴリズムの追加

- [Segment Tree Beats](segment_tree_beats.md) / [操作できる図解](../../visualizations/advanced/index.html#beats)

追加分の検証は `python3 verify/run_advanced.py`（`--sanitize`対応）。[全追加項目と既存項目の対応表](../advanced/README.md)。
