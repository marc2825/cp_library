---
title: 最小費用循環流
documentation_of: //graph_flow/min_cost_circulation.hpp
---

# 最小費用循環流

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#min_cost_circulation) / [実装](../../graph_flow/min_cost_circulation.hpp) / [一次資料と設計](../b_7_11/references.md)

全頂点で流出=流入となる最小費用の循環流を求めます。Min-Cost b-Flowのbalance=0版を共有します。

## usage

```cpp
#include <iostream>
#include "graph_flow/min_cost_circulation.hpp"
int main() {
    cp::MinCostCirculation<> f(2); f.add_edge(0,1,0,3,-4); f.add_edge(1,0,0,2,1);
    std::cout << cp::optimization_to_string(f.solve()->cost) << '\n';
}
```

出力:

```text
-6
```

## API・使用条件

`MinCostCirculation<Cap,Cost>(n)`、`add_edge(u,v,lower,upper,cost)`、`solve()`。Resultと整数条件はMinCostBFlowと共通。

頂点番号は assert による使用条件、lower>upper は`invalid_argument`です。

## 仕組みと境界

下限付き、負費用閉路、負の流量範囲を扱います。需要がなくても負費用閉路を見落とさず、有限容量まで最適化します。無限制約を表す記号的infinityはありません。

## 計算量

Min-Cost b-Flowと同じ時間・領域。 Vは頂点数、Eは論理辺数。

## 使用する問題・検証

想定問題: 下限付きの巡回スケジュールや、負費用の閉路を上限まで回す利益最大化。 [実行できる小問題](../../examples/graph_flow/min_cost_circulation_b.cpp)。

関連する出題: [Minimum Cost b-flowのbalance=0の場合](https://judge.yosupo.jp/problem/min_cost_b_flow)。入出力仕様の参照先であり、オンラインACの主張ではありません。

検証では、b-flow と同じ1,000例で循環流の最小費用（実行不能なら nullopt）を全列挙と比較します（流量と potential の検査は b-flow 側）。`LLONG_MAX` 容量×`-LLONG_MAX` 費用・`LLONG_MIN` 費用の自己ループと、lower>upper の`invalid_argument`も確認します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/flow_test.cpp)。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
