---
title: Persistent Order Statistics
documentation_of: //ordered_structure/persistent_order_statistics.hpp
---

# Persistent Order Statistics

[カテゴリ索引](README.md) / [図解](../../visualizations/advanced/index.html#persistent-kth) / [実装](../../ordered_structure/persistent_order_statistics.hpp) / [一次資料・設計判断](../advanced/references.md)

## usage

```cpp
#include <iostream>
#include "ordered_structure/persistent_order_statistics.hpp"
int main() {
    cp::PersistentOrderStatistics<int> s({5, 1, 4, 1, 3});
    std::cout << *s.kth_smallest(1, 5, 2) << ' ' << s.count_less(0, 5, 4) << '\n';
}
```

[実行できる使用例](../../examples/ordered_structure/persistent_order_statistics.cpp)。C++17、namespace cp。ライブラリルートをincludeパスに追加してください。

## API・使用条件

`PersistentOrderStatistics<T>(vector<T>)`、`size()`、`kth_smallest(l,r,k)->optional<T>`（0始まり）、`count_less(l,r,x)`、`range_freq(l,r,low,high)`。静的な列の各prefixを自動で永続化し、任意区間に問い合わせます。k範囲外はnullopt、空入力可。`<`とコピーが必要。更新版を自分で管理する用途は [Persistent Segment Tree](persistent_segment_tree.md) を使います。

## 計算量・原理

構築O(n log(n+1))、検索O(log(σ+1))、格納O(n log(σ+1)+σ)。σは異なる値の数。

各prefixの値の頻度を永続木に保存し、prefix rの頻度からprefix lを引いて[l,r)の頻度を得ます。左の子の頻度差とkを比較して降ります。一般モノイドの2つのprodを引いてk番目を求めるAPIではありません。

## 使用する問題・検証

[Range Kth Smallest](https://judge.yosupo.jp/problem/range_kth_smallest)は`kth_smallest`だけで解けます（`verify/advanced/persistent_kth.test.cpp`）。

[追加分の検証入口](../../verify/run_advanced.py)で、長さ100未満・`LLONG_MIN`/`LLONG_MAX`を含む150列について`kth_smallest`・`count_less`・`range_freq`を整列した部分列と比較します（`structures_test.cpp`の`static_order()`）。実際の範囲は[検証一覧](../advanced/README.md)を参照。オンライン提出のACは未確認です。

図解は学習用の小さい入力で内部状態・候補・論理的な操作を追えます。[簡略化の範囲](../../visualizations/advanced/README.md)を参照してください。
