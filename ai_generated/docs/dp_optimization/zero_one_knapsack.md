---
title: 0-1ナップサック
documentation_of: //dp_optimization/zero_one_knapsack.hpp
---

# 0-1ナップサック

各品物を高々1個ずつ選び、重さの和を制限した価値の最大化を行います。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/dp_optimization/index.html#zero-one) / [実装](../../dp_optimization/zero_one_knapsack.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "dp_optimization/zero_one_knapsack.hpp"
int main() {
    auto result = cp::zero_one_knapsack(7, {{3,4}, {4,5}, {2,3}}); // {重さ,価値}
    std::cout << result.best_value() << '\n';
    if (result.dp[7]) std::cout << *result.dp[7] << '\n'; // 重さちょうど7。
}
```

[動く使用例](../../examples/dp_optimization/zero_one_knapsack.cpp)。リポジトリのルートをincludeパスへ通します。

## API・使用条件

`zero_one_knapsack(int capacity, const vector<KnapsackItem>& items) -> KnapsackResult`。

品物は `{int weight,long long value}`。capacityとweightは非負。重さ0も使えます。価値は負でも可。戻り値の `dp[w]` は**重さちょうどw**の最大価値（未到達はnullopt）、`best_value()` は容量以下の最大値です。[共通型](knapsack_common.md)を参照。

空の品物・容量0にも対応。dp[0]の初期値は0で、空集合を許容します。選んだ品物の復元は提供しません。

## 原理・計算量・注意点

時間 O(NW)、空間 O(W)、best_value は O(W)。品物ごとに容量を大きい側から走査するため、参照元は今回の品物でまだ更新されていません。重さ0では同じセルを1回だけ更新します。

全てを0で初期化すると未到達を表せず、重さちょうどの問い合わせや負の価値で誤りになります。到達状態は optional で分離し、LLONG_MIN/MAXも有限な値として使えます。

価値の加算の全中間値がlong longに収まることが条件で、超過はoverflow_error。最大値を未到達の番兵に予約しません。

## 使用する問題・検証範囲

[AOJ DPL_1_B: 0-1 Knapsack Problem](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_1_B)。

全品物個数の列挙と全dpセルを比較。負の価値、重さ0、容量0、未到達、整数極値・加算超過、20万容量。

[検証入口](../../verify/run_dp_optimization.py)は通常／ASan・UBSan、debug/release、単独include、混在include、複数翻訳単位、使用例、独立オラクル、ジャッジ形式の入出力を確認します。オンラインACは未確認です。
