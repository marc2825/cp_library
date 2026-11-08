---
title: パラメータチューニング
documentation_of: //heuristic_search/parameter_tuning.hpp
---

# パラメータチューニング

設定のCartesian gridを作り、全設定に同じseed列を適用して平均スコアを比較します。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#parameter_tuning) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`parameter_grid(vector<vector<T>> axes)`、`tune_parameters(configs,seeds,evaluate,goal=minimize)`→`{best_index,mean_scores,scores}`。evaluate(config,seed)→有限な数値。

空の軸列は空設定1つ、空の軸を含めば0設定。tuningは設定・seedとも非空が必要。目標と同点規約は既存探索と共通。平均はlong doubleの逐次平均で、非常に近い平均値の順位は丸めに依存します。訓練seed以外の品質・汎化を保証しません。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../heuristic_search/parameter_tuning.hpp"

int main() {
    auto result = cp::tune_parameters(std::vector<int>{3, 1, 2}, std::vector<std::uint64_t>{1, 2}, [](int setting, std::uint64_t seed) { return setting * setting + int(seed); });
    std::cout << result.best_index << "\n";
}
```

出力:

```text
1
```

[コンパイルして動く使用例](../../examples/heuristic_search/parameter_tuning_b.cpp)。

## 計算量

K設定・S seedでK*S回の評価、O(KS)保存。gridは出力要素数に比例した時間・領域。

## 使用する問題・検証

温度・近傍比率・beam幅のローカル比較。外部への自動送信や提出は行いません。

同じseedの呼出し順・全スコア・平均・最良設定、grid順と空軸を検証。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
