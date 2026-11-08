---
title: Segtree
documentation_of: //range_query/segtree.hpp
---

# Segtree

一点代入と区間の集約、木の上での境界探索を扱います。min/max、和、行列積、関数合成などに使えます。

[実装](../../range_query/segtree.hpp) / [区間分解の図解](../../visualizations/range_query/index.html#segtree) / [一覧](README.md)

## API・条件

`cp::Segtree<S,op,e>`。`S op(S,S)` は結合律を満たし、`S e()` は左右の単位元を返す必要があります。可換性は不要で、左から右の順序を保ちます。関数はテンプレート引数として渡します。

| API | 意味 | 計算量 |
|---|---|---|
| `Segtree(n=0)` / `Segtree(vector<S>)` | e() で初期化／配列から構築 | O(n) |
| `size()` | 要素数 | O(1) |
| `set(p,x)` | 一点代入 | O(log n) |
| `get(p)` | 一点取得 | O(1) |
| `prod(l,r)` | `[l,r)` の左から右への総積、空なら e() | O(log n) |
| `all_prod()` | 全体の積、空配列なら e() | O(1) |
| `max_right(l,pred)` | `pred(prod(l,r))` を満たす最大の右端 r | O(log n) |
| `min_left(r,pred)` | `pred(prod(l,r))` を満たす最小の左端 l | O(log n) |

メモリ O(n)。境界探索には `pred(e())==true`、副作用なし、区間を伸ばすと true から false へ単調に変化することが必要です。`max_right<pred>(l)` / `min_left<pred>(r)` で関数ポインタも渡せます。

## usage

```cpp
#include "range_query/segtree.hpp"
#include <vector>
long long op(long long a, long long b) { return a + b; }
long long e() { return 0; }
// 以下は関数内:
cp::Segtree<long long, op, e> seg(std::vector<long long>{2, 3, 4, 5});
seg.set(1, 6);
auto value = seg.prod(1, 3); // 10
auto r = seg.max_right(0, [](long long s) { return s <= 8; }); // 2
```

和での境界探索には非負要素などの単調性が必要です。[max を使う動く使用例](../../examples/range_query/segtree.cpp) もあります。

## 原理・注意点

葉の数を 2 の冪へ切り上げ、各親に左右の積を保持します。区間取得は左右から O(log n) 個のノードを選びます。左側の結果には後ろへ、右側の結果には前へ連結し、非可換な演算の順序を守ります。

min/max の単位元には、取り得る値の範囲を満たす十分大きい／小さい値を使ってください。0 が単位元とは限りません。旧 `data_structure/segment_tree.hpp` のグローバル `segtree<S,op,e>` はこの型への別名で、`ll` の事前定義は不要になりました。

## 使用する問題・検証

- [Library Checker: Point Set Range Composite](https://judge.yosupo.jp/problem/point_set_range_composite) — [検証プログラム](../../verify/range_query/point_set_range_composite.test.cpp)。
- [AtCoder practice2 J](https://atcoder.jp/contests/practice2/tasks/practice2_j) — 最大値と境界探索。

素朴な列との比較で更新・全積・区間積・左右の探索、文字列連結の順序、旧 API を検証します。オンライン AC の記録ではありません。

API と反復実装は [ACL Segtree](https://atcoder.github.io/ac-library/production/document_ja/segtree.html) を基に調整しています（CC0）。[出典・変更点](references.md)。
