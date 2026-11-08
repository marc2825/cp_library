---
title: Lazy Segtree
documentation_of: //range_query/lazy_segtree.hpp
---

# Lazy Segtree

区間全体への更新と区間の集約を扱います。区間加算・区間和、区間代入・最小値、アフィン変換・区間和などに使えます。

[実装](../../range_query/lazy_segtree.hpp) / [遅延伝播の図解](../../visualizations/range_query/index.html#lazy) / [一覧](README.md)

## API・条件

`cp::LazySegtree<S,op,e,F,mapping,composition,id>`。`S,op,e` は Segtree と同じモノイドです。更新の型 `F` と、次の関数を定義します。

- `S mapping(F f,S x)`: 更新 f を集約値 x に適用する。
- `F composition(F f,F g)`: **g を先に、f を後に**適用した更新。
- `F id()`: 何も変更しない更新。

`mapping(id(),x)==x`、`mapping(composition(f,g),x)==mapping(f,mapping(g,x))`、`mapping(f,op(x,y))==op(mapping(f,x),mapping(f,y))` が必要です。本ライブラリの利用例では `mapping(f,e())==e()` も満たし、長さ 0 の区間を保持します。`F` の `==` は不要です。

| API | 意味 | 計算量 |
|---|---|---|
| `LazySegtree(n=0)` / `LazySegtree(vector<S>)` | e() で初期化／配列から構築 | O(n) |
| `size()` / `all_prod()` | 要素数／全体の積 | O(1) |
| `set(p,x)` / `get(p)` | 一点代入／取得 | O(log n) |
| `apply(p,f)` / `apply(l,r,f)` | 一点／区間更新 | O(log n) |
| `prod(l,r)` | 区間の左から右への積 | O(log n) |
| `max_right(l,pred)` / `min_left(r,pred)` | 条件を満たす境界探索 | O(log n) |

メモリ O(n)。探索は Segtree と同じ単調性・`pred(e())==true` が必要です。関数ポインタの探索形式も使えます。get/prod/探索は内部で伝播するため非 const です。

## usage: 区間加算・代入・区間和

```cpp
#include "range_query/lazy_segtree.hpp"
#include <vector>
struct S { long long sum; int len; };
struct F { long long a, b; }; // x -> a*x+b
S op(S x, S y) { return {x.sum+y.sum, x.len+y.len}; }
S e() { return {0, 0}; }
S mapping(F f, S x) { return {f.a*x.sum+f.b*x.len, x.len}; }
F composition(F f, F g) { return {f.a*g.a, f.a*g.b+f.b}; }
F id() { return {1, 0}; }
// 以下は関数内:
cp::LazySegtree<S,op,e,F,mapping,composition,id> seg(
    std::vector<S>{{1,1},{2,1},{3,1},{4,1}});
seg.apply(0, 4, {1, 5}); // 全要素 +5
seg.apply(1, 3, {0, 2}); // [1,3) を 2 に代入
auto answer = seg.prod(0, 4).sum; // 19
```

[動く使用例](../../examples/range_query/range_actions.cpp)。和への加算には区間長が必要なので S に長さを持たせます。**`LazySegtree(n)` は `{0,0}` の n 個であり、長さ 1 のゼロ要素列ではありません。** ゼロ配列にも `vector<S>(n,{0,1})` を渡してください。

## 原理・注意点

区間全体を覆うノードでは、そのノードの集約を直ちに変更し、子へまだ伝えていない更新を合成します。部分的に入るときは先に子へ伝播します。更新後は必要な祖先だけを再計算し、遅延のあるノードを古い子の値で上書きしません。

加算 +5 の後の代入 2 は 2、代入 2 の後の加算 +5 は 7 です。composition の引数順序を逆にすると壊れます。代入の「更新なし」を 0 で表す実装も、0 の代入と区別できないため避けてください。アフィン表現では id={1,0}、代入={0,value} で区別できます。

## 使用する問題・検証

- [Library Checker: Range Affine Range Sum](https://judge.yosupo.jp/problem/range_affine_range_sum) — [剰余付き検証プログラム](../../verify/range_query/range_affine_range_sum.test.cpp)。
- [AtCoder practice2 K](https://atcoder.jp/contests/practice2/tasks/practice2_k) — アフィン更新と区間和。

素朴配列との比較で、重複する更新、代入・加算の合成、更新後の一点 set、非可換集約、空区間、左右の探索を検証します。オンライン AC の記録ではありません。

[ACL Lazy Segtree](https://github.com/atcoder/ac-library/blob/master/document_ja/lazysegtree.md) の API と反復実装を基にしています（CC0）。[出典・変更点](references.md)。
