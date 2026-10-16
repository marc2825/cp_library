---
title: 局所探索
documentation_of: //heuristic_search/local_search.hpp
---

# 局所探索

[カテゴリ](README.md) / [実装](../../heuristic_search/local_search.hpp) / [使用例](../../examples/heuristic_search/local_search.cpp) / [図解](../../visualizations/heuristic_search/index.html#local) / [一次資料](references.md)

現在解の近傍を列挙し、厳密に改善する候補へ移ります。最初の改善と最良改善を選択できます。同点へは移動せず、有限状態空間での同点巡回を防ぎます。局所最適は指定した近傍に対する結果で、大域最適を意味しません。

## usage

```cpp
#include <iostream>
#include <vector>
#include "heuristic_search/local_search.hpp"
int main() {
    cp::LocalSearchOptions options;
    options.mode = cp::LocalSearchMode::best_improvement;
    const auto result = cp::local_search(8,
        [](int x) { return (x - 3) * (x - 3); },
        [](int x) { return std::vector<int>{x - 1, x + 1}; }, options);
    std::cout << result.best_state << ' ' << result.best_score << '\n';
}
```

`local_search(initial,evaluate,neighbors,options={})`。neighbors(const State&)は近傍の範囲を値で返します。vector<State>を推奨します。元の状態を変更せず、コピーを独立に保持できる必要があります。

LocalSearchOptionsは共通SearchOptionsとmode。既定はLocalSearchMode::best_improvement（全近傍から最良改善）、first_improvementは列挙順で最初の改善を見つけて走査を終了します。最良候補が同点なら先の候補を保持します。firstでも近傍vectorの生成自体は全て行うため、短絡するのは評価だけです。

返値はSearchResult。currentとbestは同じ状態・評価です。iterationsは完了した近傍走査数で、改善なしの最後の走査も含みます。acceptedは実際に移動した回数、evaluationsは初期評価と評価した近傍の数。改善なしならlocal_optimum、上限ならiteration_limit。最終反復で最適位置へ着いても、次の走査を行う予算がなければiteration_limitです。

stopを走査開始前と各候補の評価前に確認。最良改善の途中で中断した場合は、その不完全な走査で見つけた候補を採用せず、前の状態をstoppedで返します。済んだ評価はevaluationsに含め、iterationsは増やしません。コールバックの途中には割り込めません。

## 計算量・使いどころ・検証

I走査、最大B近傍ならO(IB)回の評価と最大I回の更新。近傍生成、評価、状態コピーが定数ならO(IB)時間。近傍vectorと選択候補を保持するO(B)個の状態が必要です。乱択近傍を使う場合のlocal_optimumはその回に列挙した近傍だけに対する意味になります。

問題例は[AtCoder Introduction to Heuristics](https://atcoder.jp/contests/intro-heuristics/tasks/intro_heuristics_a)の1日変更・交換。経路のswap、2-opt、割当の変更などにも使えます。評価の差分更新やタブー法、リスタートは利用側で組み合わせます。

6位置までの3値評価の全列を使い、全開始位置・両目的・両モードの24,060探索を独立歩行と比較。平坦な近傍、空近傍、反復0・中断・非有限評価、20万回の改善、デフォルト構築できない状態を検証。Pythonの任意精度評価でも照合します。
