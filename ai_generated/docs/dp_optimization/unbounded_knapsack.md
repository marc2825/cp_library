---
title: 完全ナップサック
documentation_of: //dp_optimization/unbounded_knapsack.hpp
---

# 完全ナップサック

各種類を何個でも選べるナップサックです。0-1版と更新式が似ていますが、重さを走査する向きが異なります。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/dp_optimization/index.html#unbounded) / [実装](../../dp_optimization/unbounded_knapsack.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "dp_optimization/unbounded_knapsack.hpp"
int main() {
    auto result = cp::unbounded_knapsack(7, {{3,4}, {4,5}, {2,3}});
    std::cout << result.best_value() << '\n';
}
```

[動く使用例](../../examples/dp_optimization/unbounded_knapsack.cpp)。リポジトリのルートをincludeパスへ通します。

## API・使用条件

`unbounded_knapsack(int capacity, const vector<KnapsackItem>& items) -> KnapsackResult`。

capacityは非負、**各weightは正**。価値は負でも可。重さ0は前提違反としてassertで検査します。空・容量0は許容します。重さ0・正の価値を何個でも選べる場合の無限大を表すAPIではありません。

dp[w]は重さちょうど、未到達はnullopt。容量以下の最大値はbest_value。[共通型](knapsack_common.md)を参照。品物の復元は提供しません。

## 原理・計算量・注意点

時間 O(NW)、空間 O(W)。各品物で小さい重さから更新します。すでに今回の品物を使った状態を参照できるので、複数個選べます。0-1版の降順走査とは使い分けてください。

負の価値も、重さちょうどを作る状態では必要になる場合があります。未到達を数値の極小値と同一視しません。

価値の加算の全中間値がlong longに収まることが条件で、超過はoverflow_error。最大値を未到達の番兵に予約しません。

## 使用する問題・検証範囲

[AOJ DPL_1_C: Knapsack Problem](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_1_C)。

容量から各品物の最大個数を求めて全列挙し、全dpセルを比較。負値、未到達、空・容量0、加算超過、20万容量。

[検証入口](../../verify/run_dp_optimization.py)は通常／ASan・UBSan、debug/release、単独include、混在include、複数翻訳単位、使用例、独立オラクル、ジャッジ形式の入出力を確認します。オンラインACは未確認です。
