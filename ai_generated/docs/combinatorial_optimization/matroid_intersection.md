---
title: Matroid Intersection
documentation_of: //combinatorial_optimization/matroid_intersection.hpp
---

# Matroid Intersection

二つのマトロイドに共通な独立集合の最大要素数を、交換グラフの最短増加路で求めます。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#matroid_intersection) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`matroid_intersection(n,first,second)`→`{selected,oracle_calls}`。oracleは異なる要素IDのvector<int>を受けるbool関数。

oracleは真のマトロイドを表し、入力不変・純粋な判定であること。空集合の独立性は確認しますが、交換公理の検査は行いません。0始まり、n≥0、非重み付き。重み付き交差はCに保持。selectedは昇順。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../combinatorial_optimization/matroid_intersection.hpp"

int main() {
    auto first = [](const std::vector<int>& s) { return s.size() <= 2; };
    auto second = [](const std::vector<int>& s) { int even = 0; for (int x : s) even += x % 2 == 0; return even <= 1; };
    std::cout << cp::matroid_intersection(4, first, second).selected.size() << "\n";
}
```

出力:

```text
2
```

[コンパイルして動く使用例](../../examples/combinatorial_optimization/matroid_intersection_b.cpp)。

## 計算量

最大共通rank r、要素数n、独立性判定費用QとしてO(r n² Q+r²n²)の保守的時間上界、O(n)作業領域。各oracle引数の構築費用も含む。

## 使用する問題・検証

色ごとの容量を満たす最大森、共通独立集合、二部マッチングのマトロイド表現。

全subsetのGraphic×Partition共通独立性と最大サイズを比較。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
