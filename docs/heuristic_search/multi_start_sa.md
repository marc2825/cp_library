---
title: Multi-Start SA
documentation_of: //heuristic_search/multi_start_sa.hpp
---

# Multi-Start SA

独立した乱数streamで初期解を生成し、複数回のSAを実行して最良runを選びます。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#multi_start_sa) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`multi_start_sa(starts,factory,evaluate,propose,seed=20261018,options={})`→`{runs,best_run_index,best_run()}`。factory(run_index,Random&)→State。

starts>0。seed生成器から各run用seedを取り、初期解生成と探索はそのrunのRNGを使用。options.stopはrun途中と次の開始前に確認。同点は先のrun、minimize/maximize共通。状態の独立コピー・有限評価等は既存SAの条件を共有します。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../heuristic_search/multi_start_sa.hpp"

int main() {
    cp::AnnealingOptions options; options.max_iterations = 0;
    auto result = cp::multi_start_sa(3, [](std::size_t i, cp::Random&) { return int(i); }, [](int x) { return x; }, [](int x, cp::Random&) { return x; }, 1, options);
    std::cout << result.runs.size() << " " << result.best_run().best_score << "\n";
}
```

出力:

```text
3 0
```

[コンパイルして動く使用例](../../examples/heuristic_search/multi_start_sa_b.cpp)。

## 計算量

run数Rと各runの反復Iに対し、既存SAの費用の和。全run結果を保持するためO(R*Stateの大きさ)。

## 使用する問題・検証

AHC等の複数初期解からの探索。解品質の改善保証は返しません。

独立なseed分割・初期解・温度・採否を直接シミュレーションして全runと最良選択を比較。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
