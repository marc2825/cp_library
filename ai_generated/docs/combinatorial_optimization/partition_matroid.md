---
title: Partition Matroid
documentation_of: //combinatorial_optimization/partition_matroid.hpp
---

# Partition Matroid

要素のグループごとの採用数上限を独立性オラクルとして表します。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#partition_matroid) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`PartitionMatroid(groups,capacities)`、`size()`、`independent(elements)` / `operator()`。

groups[e]は0..g-1、capacityは非負。重複要素指定はfalse、範囲外IDは例外。空グループや容量0可。各問い合わせでカウンタを作り直します。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../combinatorial_optimization/partition_matroid.hpp"

int main() {
    cp::PartitionMatroid matroid({0, 0, 1}, {1, 2});
    std::cout << matroid.independent({0, 2}) << " " << matroid.independent({0, 1}) << "\n";
}
```

出力:

```text
1 0
```

[コンパイルして動く使用例](../../examples/combinatorial_optimization/partition_matroid_b.cpp)。

## 計算量

構築O(n+g)、1判定O(n+g+|elements|)時間、O(n+g)作業領域。

## 使用する問題・検証

色制約、カテゴリごとの採用数上限、二部マッチングの両端制約。

全subsetのグループ個数を直接数えて比較。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
