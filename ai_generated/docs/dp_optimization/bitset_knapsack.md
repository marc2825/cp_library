---
title: Bitset Knapsack・0-1部分和
documentation_of: //dp_optimization/bitset_knapsack.hpp
---

# Bitset Knapsack・0-1部分和

重さの和を作れるかという到達可能性だけを、64ビット単位でまとめて更新します。価値の最大化を行うナップサックとは別のAPIです。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/dp_optimization/index.html#bitset) / [実装](../../dp_optimization/bitset_knapsack.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "dp_optimization/bitset_knapsack.hpp"
int main() {
    cp::BitsetKnapsack sums(12);
    for (int weight : {3,5,6}) sums.add(weight);
    for (int sum : sums.reachable_sums()) std::cout << sum << ' ';
    std::cout << '\n';
    std::cout << sums.contains(11) << '\n';
}
```

[動く使用例](../../examples/dp_optimization/bitset_knapsack.cpp)。リポジトリのルートをincludeパスへ通します。

## API・使用条件

| API | 意味 |
|---|---|
| `BitsetKnapsack(int max_sum)` | 0〜max_sumを管理。初期状態は0だけ到達可能 |
| `add(int weight)` | 非負の重さを1個追加。0や上限超過では集合は変わらない |
| `contains(int sum)` | 到達可能か。範囲外（負も含む）はfalse |
| `reachable_sums()` | 到達可能な和を昇順のvector<int>で返す。0を含む |
| `words()` | 内部のconst vector<uint64_t>&。低い和の語から順に保持 |
| `max_sum()` | 上限 |
| `reset()` | 上限を保持し、0だけに戻す |

max_sumとweightは非負が前提、assertで検査。空の品物集合でもcontains(0)はtrue。

## 原理・計算量・注意点

L=floor(max_sum/64)+1として、構築・resetはO(L)、addは最悪O(L)、containsはO(1)、reachable_sumsはO(max_sum+1)、空間O(L)。N個の構築はO(NL)。

`bits |= bits << weight` を語の大きい側からin-placeで処理します。語をまたぐ持ち越しを加え、シフト量が64の倍数なら64ビットシフトを行いません。最後の語の余分なビットは消します。同じ品物が繰り返し使われない順序が重要です。復元・個数カウントは提供しません。

## 使用する問題・検証範囲

[CSES Money Sums](https://cses.fi/problemset/task/1745/)。

全追加後のビット・語全体・和一覧を素朴な配列／部分集合列挙と比較。0/1/63/64/65/127/128/129と最終語の余白、reset、20万上限の64の倍数。

[検証入口](../../verify/run_dp_optimization.py)は通常／ASan・UBSan、debug/release、単独include、混在include、複数翻訳単位、使用例、独立オラクル、ジャッジ形式の入出力を確認します。オンラインACは未確認です。
