---
title: Sparse Table
documentation_of: //static_query/sparse_table.hpp
---

# Sparse Table

変更しない配列の min/max/gcd/bitwise AND・OR などを、前計算 O(n log n)、区間取得 O(1) で扱います。

[実装](../../static_query/sparse_table.hpp) / [操作できる図解](../../visualizations/static_query/index.html#sparse) / [一覧](README.md)

## API・条件

`cp::SparseTable<S,op,e>`。op は結合律と **`op(x,x)==x`（冪等性）** を満たし、e() が左右の単位元である必要があります。可換性は不要です。通常の加算・関数合成はこの O(1) 方式では使えません。

| API | 意味 | 計算量 |
|---|---|---|
| `SparseTable()` | 空で初期化 | O(1) |
| `SparseTable(vector<S>)` / `build(vector<S>)` | 配列から構築／全体を再構築 | O(n log n) |
| `size()` / `empty()` | 要素数／空判定 | O(1) |
| `get(p)` | 元の要素 | O(1) |
| `prod(l,r)` | `[l,r)` の積。空区間は e() | O(1) |
| `all_prod()` | 全体の積。空配列は e() | O(1) |

メモリ O(n log n)。長さは `0 <= n <= 100000000` で、実用上の上限はメモリによります。一点更新はありません。build は以前の内容を置き換えます。

## usage

```cpp
#include "static_query/sparse_table.hpp"
#include <algorithm>
#include <limits>
#include <vector>
int op(int x, int y) { return std::min(x, y); }
int e() { return std::numeric_limits<int>::max(); }
int main() {
    cp::SparseTable<int, op, e> table(std::vector<int>{3, 1, 1, 2});
    int minimum = table.prod(1, 4); // 1
    return minimum != 1;
}
```

[動く使用例](../../examples/static_query/sparse_and_nearest.cpp)。gcd なら op を `std::gcd` のラッパー、e を 0 にします。

## 原理・注意点

`table[k][i]` に `[i,i+2^k)` の積を持ちます。長さ d>0 の問い合わせでは `k=floor(log2(d))` とし、`[l,l+2^k)` と `[r-2^k,r)` の 2 つを合わせます。

中央が重複しますが、重なる部分の積を v とすると `op(v,v)==v` のため結果が変わりません。加算では二重計上になるので使えません。空区間は先に処理し、log2(0) を計算しません。各段には配列内に収まる区間だけを保持します。

## 使用する問題・検証

[Library Checker: Static RMQ](https://judge.yosupo.jp/problem/staticrmq) — [ローカル検証プログラム](../../verify/static_query/static_rmq.test.cpp)。全探索配列の全区間、空配列・空区間、再構築、非可換な「最初の有効要素を選ぶ」演算、20 万要素を確認します。オンライン AC の記録ではありません。

参考: [Luzhiled Sparse Table](https://ei1333.github.io/library/structure/others/sparse-table.hpp.html)、[CP-Algorithms Sparse Table](https://cp-algorithms.com/data_structures/sparse-table.html)。本 API は単位元を要求し、空区間にも対応しています。
