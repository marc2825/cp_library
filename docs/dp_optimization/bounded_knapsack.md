---
title: 個数制限ナップサック
documentation_of: //dp_optimization/bounded_knapsack.hpp
---

# 個数制限ナップサック

各種類の使用個数に上限があるナップサックです。個数ぶん複製せず、剰余類と単調キューで O(NW) にします。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/dp_optimization/index.html#bounded) / [実装](../../dp_optimization/bounded_knapsack.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "dp_optimization/bounded_knapsack.hpp"
int main() {
    auto result = cp::bounded_knapsack(7, {{3,4,1}, {2,3,2}}); // {重さ,価値,個数}
    std::cout << result.best_value() << '\n';
}
```

[動く使用例](../../examples/dp_optimization/bounded_knapsack.cpp)。リポジトリのルートをincludeパスへ通します。

## API・使用条件

`bounded_knapsack(int capacity,const vector<BoundedKnapsackItem>& items) -> KnapsackResult`。

品物は `{int weight,long long value,long long count}`。capacity,weight,countは非負。count=0や重さ0にも対応します。重さ0は価値が正なら上限まで、0以下なら0個を選びます。

結果は重さちょうどのdpと容量以下のbest_value。[共通型](knapsack_common.md)の算術条件を引き継ぎます。countがLLONG_MAXでも、正の重さでは容量で使える個数が制限されます。

## 原理・計算量・注意点

時間 O(NW)、空間 O(W)。正の重さ w について、各剰余 r の重さを r+tw と並べます。

```text
dp[r+tw] = max_j(old[r+jw] + (t-j)×value)
t-count <= j <= t
```

有効な候補を単調キューで保持します。候補jの得点はtが1増えるたびに全て同じvalueだけ増えるため、順序を保てます。古すぎる先頭を除き、劣る末尾も除きます。

変形した `old[j]-j×value` を保存せず、実際に足す個数の得点を評価します。それでも `(t-j)×value` と、その加算を含む全中間値がlong longに収まることが条件です。重さ0ではcount×valueの積にも必要です。

## 使用する問題・検証範囲

[AOJ DPL_1_G: Knapsack Problem with Limitations](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_1_G)。

小さな品物列6,175通りを各容量0〜6で全個数列挙、400ランダム系列を他2種類のナップサックと併せて比較。重さ0・count=0/MAX・負値・演算超過・20万容量。

[検証入口](../../verify/run_dp_optimization.py)は通常／ASan・UBSan、debug/release、単独include、混在include、複数翻訳単位、使用例、独立オラクル、ジャッジ形式の入出力を確認します。オンラインACは未確認です。
