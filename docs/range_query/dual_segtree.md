---
title: Dual Segtree
documentation_of: //range_query/dual_segtree.hpp
---

# Dual Segtree

区間へ更新をかけ、必要な一点の値を取得します。区間の集約は不要な問題で Lazy Segtree より定義する関数を減らせます。

[実装](../../range_query/dual_segtree.hpp) / [更新合成の図解](../../visualizations/range_query/index.html#dual) / [一覧](README.md)

## API・条件

`cp::DualSegtree<S,F,mapping,composition,id>`。値の型 S と更新 F を分離します。`mapping(F,S)` が値への適用、`composition(f,g)` が g の後の f、`id()` が恒等更新です。

composition は結合律と単位元を持ち、`mapping(composition(f,g),x)==mapping(f,mapping(g,x))`、`mapping(id(),x)==x` を満たす必要があります。可換性・F の等値比較は不要です。S は通常のコピー・初期化が可能な型です。

| API | 意味 | 計算量 |
|---|---|---|
| `DualSegtree(n=0,initial=S{})` | 全要素を initial に初期化 | O(n) |
| `DualSegtree(vector<S>)` | 配列から構築 | O(n) |
| `size()` | 要素数 | O(1) |
| `apply(l,r,f)` / `apply(p,f)` | 区間／一点へ更新を適用 | O(log n) |
| `get(p)` | 更新を反映した一点取得 | O(log n) |
| `set(p,x)` | それまでの更新を反映した後で一点代入 | O(log n) |

メモリ O(n)。区間 prod・全体 prod・境界探索は提供しません。get も内部で伝播するため非 const です。

## usage

```cpp
#include "range_query/dual_segtree.hpp"
#include <vector>
struct F { long long a, b; };
long long mapping(F f, long long x) { return f.a*x+f.b; }
F composition(F f, F g) { return {f.a*g.a, f.a*g.b+f.b}; }
F id() { return {1, 0}; }
// 以下は関数内:
cp::DualSegtree<long long,F,mapping,composition,id> seg(
    std::vector<long long>{1,2,3,4});
seg.apply(0, 4, {1, 5}); // +5
seg.apply(1, 3, {0, 2}); // 2 に代入
auto value = seg.get(2); // 2
seg.set(2, 9);
seg.apply(2, {2, 1});    // 2*9+1
```

[Lazy と比較できる動く例](../../examples/range_query/range_actions.cpp)。

## 原理・注意点

内部ノードは区間の値の集約を持たず、未伝播の更新だけを持ちます。更新前に左右の境界の祖先を根から伝播し、古い更新が新しい更新より後に適用されるのを防ぎます。get/set は対象の葉まで伝播して処理します。

この境界伝播は加算だけだと省略できてしまうため、不具合を見落としやすい箇所です。代入と加算やアフィン変換など、順序に意味のある更新でも使える設計にしています。`F == id()` の比較に頼らず恒等更新も伝播します。

## 使用する問題・検証

- [Library Checker: Range Affine Point Get](https://judge.yosupo.jp/problem/range_affine_point_get) — [検証プログラム](../../verify/range_query/range_affine_point_get.test.cpp)。
- [AOJ DSL_2_D](https://onlinejudge.u-aizu.ac.jp/problems/DSL_2_D) — 区間代入・一点取得。初期値や閉区間を問題に合わせて変換します。

素朴配列との比較で、重複区間の非可換更新、境界伝播、get/set/一点 apply、空区間、20 万要素を検証します。オンライン AC の記録ではありません。

設計資料: [Luzhiled Dual Segment Tree](https://ei1333.github.io/library/structure/segment-tree/dual-segment-tree.hpp.html)。本実装は値 S を持つ形式で、composition の向きはこのカテゴリの Lazy と揃えています。
