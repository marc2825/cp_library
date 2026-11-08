---
title: 多腕バンディット
documentation_of: //heuristic_search/multi_armed_bandit.hpp
---

# 多腕バンディット

各armの平均報酬を保持し、UCB1またはepsilon-greedyで次のarmを選びます。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#multi_armed_bandit) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`MultiArmedBandit(arms,exploration=sqrt(2))`、`select_ucb1()`、`select_epsilon_greedy(Random&,epsilon)`、`update(arm,reward)`、`pulls/means/total_pulls/size`。

報酬は有限な[0,1]。未観測armは小さい番号から優先。selectは記録を変更せず、選んだarmの報酬をupdateで確定します。同点は先のarm。UCB1の定常・独立報酬の理論条件と、非定常な近傍の実用的選択は区別してください。epsilon∈[0,1]、探索係数は有限・非負、回数超過は例外。大きな探索係数では全indexを同じ係数で割って比較し、積のoverflowを避けます。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../heuristic_search/multi_armed_bandit.hpp"

int main() {
    cp::MultiArmedBandit bandit(2);
    std::size_t first = bandit.select_ucb1(); bandit.update(first, 1);
    std::cout << first << " " << bandit.select_ucb1() << "\n";
}
```

出力:

```text
0 1
```

[コンパイルして動く使用例](../../examples/heuristic_search/multi_armed_bandit_b.cpp)。

## 計算量

選択O(arms)、更新O(1)、保存O(arms)。

## 使用する問題・検証

探索近傍や初期解生成器への予算配分、定常報酬のオンライン選択。

直接の累積報酬からのUCB indexと5000観測を比較し、未観測・greedy・不正報酬を検査。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
