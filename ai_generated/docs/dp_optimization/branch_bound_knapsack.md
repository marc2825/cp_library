---
title: 分枝限定ナップサック
documentation_of: //dp_optimization/branch_bound_knapsack.hpp
---

# 分枝限定ナップサック

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#branch_bound_knapsack) / [実装](../../dp_optimization/branch_bound_knapsack.hpp) / [一次資料と設計](../b_12_15/references.md)

価値密度順の分数ナップサック上界を使い、0-1選択の探索を枝刈りします。浮動小数点を使わず上界を評価します。

## usage

```cpp
#include <iostream>
#include "dp_optimization/branch_bound_knapsack.hpp"
int main() {
    auto r=cp::branch_bound_knapsack(7,{{3,4},{4,5},{5,9}});
    std::cout << r.value << '\n';
}
```

出力:

```text
9
```

## API・使用条件

`branch_bound_knapsack(long long capacity,vector<KnapsackItem>)`→{value,weight,items,visited_nodes}。capacity>=0、weight>=0、valueは負も可。重さ0の正価値は採用し、非正価値は除外。itemsは元の添字の昇順。

## 仕組み・境界条件

比較・費用・分数上界はsigned128で評価し、整数目的値に対する上界を切り下げます。prefix重さから詰められる個数を二分探索。探索は非再帰で復元選択をarenaへ保存します。答えがlong longを超えるとoverflow_error。一般の入力で高速に解ける保証はありません。

## 計算量

前処理O(N log N)、探索状態数Sに対しO(S log N)、領域O(N+S)。最悪指数時間。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

容量が大きく通常の容量DPを置けない0-1ナップサック。 [動く小問題](../../examples/dp_optimization/branch_bound_knapsack_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
