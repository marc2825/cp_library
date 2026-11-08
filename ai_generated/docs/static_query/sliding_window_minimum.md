---
title: Sliding Window Minimum
documentation_of: //static_query/sliding_window_minimum.hpp
---

# Sliding Window Minimum

配列の長さ k のすべての窓 `[i,i+k)` の最小値を O(n) で計算します。最良値の位置が必要なら argmin を使います。

[実装](../../static_query/sliding_window_minimum.hpp) / [窓を動かす図解](../../visualizations/static_query/index.html#window) / [一覧](README.md)

## API・条件

| API | 戻り値 |
|---|---|
| `sliding_window_minimum(a,k,compare={})` | 各窓の最良値 `vector<T>` |
| `sliding_window_argmin(a,k,compare={})` | 各窓の最良値の**元配列の添字** `vector<int>` |

比較器の既定は less。同値は最も左の添字を返します。greater を渡すと最大値／argmax を取得できます。**k>0 が必要**です。k>n、または空配列には空の vector を返します。k=0 の「空の窓」は対象外です。

時間 O(n)、補助メモリ O(k)（結果の O(n-k+1) を除く）。比較・コピー O(1) を仮定します。元配列の要素は変更しません。

## usage

```cpp
#include "static_query/sliding_window_minimum.hpp"
#include <functional>
#include <vector>
int main() {
    std::vector<int> a{4, 2, 2, 5, 1};
    auto minimums = cp::sliding_window_minimum(a, 3); // {2,2,1}
    auto indices = cp::sliding_window_argmin(a, 3);  // {1,1,4}
    auto maximums = cp::sliding_window_minimum(a, 3, std::greater<int>{}); // {4,5,5}
    return minimums[0] != 2 || indices[1] != 1 || maximums[2] != 5;
}
```

[動く使用例](../../examples/static_query/window_minimum.cpp)。

## 原理・注意点

窓が k 要素になったら最良候補を出力します。次の追加の前に左端の論理要素を 1 つ除きます。内部では [Monotonic Queue](monotonic_queue.md) を使い、候補を単調に保つため各要素の出入りは高々 1 回です。

返す添字は窓の中での相対位置ではありません。相対位置が必要なら、結果の i 番目から i を引きます。変動する窓幅には単調キューを直接使ってください。和・積・関数合成などには [SWAG](swag.md) が使えます。

## 使用する問題・検証

[AOJ DSL_3_D](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DSL_3_D) — [検証プログラム](../../verify/static_query/sliding_window_minimum_aoj.test.cpp)。

全探索配列の全窓幅を素朴な min/max/最左添字と比較します。k=1/n/n+1、空配列、重複値、カスタム比較器と 20 万要素を含みます。オンライン AC の記録ではありません。

参考: [CP-Algorithms 固定長部分配列の最小値](https://cp-algorithms.com/data_structures/stack_queue_modification.html#finding-the-minimum-for-all-subarrays-of-fixed-length)。
