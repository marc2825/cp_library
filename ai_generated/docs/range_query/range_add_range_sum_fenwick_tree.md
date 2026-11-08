---
title: 区間加算・区間和 BIT
documentation_of: //range_query/range_add_range_sum_fenwick_tree.hpp
---

# 区間加算・区間和 BIT

2 本の Fenwick Tree を使い、区間加算と区間和を両方 O(log n) で扱います。和と加算に絞る場合、Lazy Segtree の演算定義が不要です。

[実装](../../range_query/range_add_range_sum_fenwick_tree.hpp) / [2 本の BIT の図解](../../visualizations/range_query/index.html#range-bit) / [一覧](README.md)

## API・条件

`cp::RangeAddRangeSumFenwickTree<T>`。T は可換な加算・減算、`T(0)`、整数 k との `T * int` が必要です。整数倍は加算と整合し、分配律を満たす必要があります。整数・modint などを使えます。符号付き整数では差分・`x*k`・prefix 計算の中間値も収まるようにします。

| API | 意味 | 計算量 |
|---|---|---|
| `RangeAddRangeSumFenwickTree(n=0)` | ゼロ配列を構築 | O(n) |
| `RangeAddRangeSumFenwickTree(vector<T>)` | 配列から構築 | O(n) |
| `size()` | 要素数 | O(1) |
| `add(l,r,x)` | `[l,r)` の全要素に x を加算 | O(log n) |
| `sum(r)` / `sum(l,r)` | prefix／区間の和 | O(log n) |
| `get(p)` / `set(p,x)` | 一点取得／代入 | O(log n) |

メモリ O(n)、内部の配列は 2 本です。この型には lower_bound を設けません。

## usage

```cpp
#include "range_query/range_add_range_sum_fenwick_tree.hpp"
#include <vector>
cp::RangeAddRangeSumFenwickTree<long long> bit(
    std::vector<long long>{1,2,3,4});
bit.add(1, 4, 5);       // {1,7,8,9}
auto s = bit.sum(0, 4); // 25
auto x = bit.get(2);    // 8
bit.set(2, -1);         // {1,7,-1,9}
```

[コンパイルできる使用例](../../examples/range_query/fenwick.cpp)。

## 原理・注意点

差分 `D[i]=A[i]-A[i-1]`（`A[-1]=0`）を B1、`i*D[i]` を B2 に入れます。

```text
prefix(r) = Σ(i<r) (r-i)*D[i]
          = r*B1.sum(r) - B2.sum(r)
sum(l,r)  = prefix(r) - prefix(l)
```

`add(l,r,x)` は差分に `D[l]+=x` と `D[r]-=x` を施します。r==n の差分は問い合わせに影響しないため保存しません。空区間は何もしません。初期配列は差分を O(n) で作り、各 BIT を線形構築します。

## 使用する問題・検証

- [AOJ DSL_2_G: RSQ and RAQ](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DSL_2_G) — [検証プログラム](../../verify/range_query/range_add_range_sum_aoj.test.cpp)。問題の 1 始まり閉区間 `[s,t]` は `add(s-1,t,x)` / `sum(s-1,t)` に変換します。

素朴配列との比較で、負の加算、非ゼロ配列の初期化、重複区間、n に接する右端、一点代入、空区間・空配列、剰余型を検証します。オンライン AC の記録ではありません。

参考: [CP-Algorithms: Fenwick Tree / Range operations](https://cp-algorithms.com/data_structures/fenwick.html#range-operations)。数式はこの API の 0 始まり・半開区間に合わせています。
