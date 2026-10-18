---
title: Maximum Weight Independent Set
documentation_of: //combinatorial_optimization/maximum_weight_independent_set.hpp
---

# Maximum Weight Independent Set

マトロイドの重み付き貪欲法と、別問題である一般グラフの重み付き独立集合の両方を提供します。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#maximum_weight_independent_set) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`maximum_weight_matroid_independent_set(weights,oracle)`、`maximum_weight_independent_set(Graph<T>,weights)`→`{weight,selected}`。weightはOptimizationWide。

重みはlong long。空集合を許し、負・零の重みは選びません。マトロイド版は真の独立性オラクルが必要。グラフ版は無向・最大64頂点で、自己辺の頂点は除外、平行辺は統合。既存の重みなし版とoverloadを共存させます。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../combinatorial_optimization/maximum_weight_independent_set.hpp"
#include "../../combinatorial_optimization/partition_matroid.hpp"

int main() {
    cp::PartitionMatroid matroid({0, 0, 1}, {1, 1});
    auto result = cp::maximum_weight_matroid_independent_set(std::vector<long long>{2, 5, 3}, matroid);
    std::cout << cp::optimization_to_string(result.weight) << "\n";
}
```

出力:

```text
8
```

[コンパイルして動く使用例](../../examples/combinatorial_optimization/maximum_weight_independent_set_b.cpp)。

## 計算量

マトロイド版O(n log n+n Q+n²)時間、一般グラフ版は64頂点までの厳密分枝限定で最悪O(2^n poly(n))時間。領域はいずれもO(n²)以内の保守的上界。

## 使用する問題・検証

[Maximum Independent Set](https://judge.yosupo.jp/problem/maximum_independent_set)の重み付き拡張、1マトロイドの重み付き選択。

両版を全subsetと比較。64頂点の空グラフ、負重み・自己辺も検証。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
