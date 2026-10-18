---
title: 焼きなましの近傍管理
documentation_of: //heuristic_search/neighborhood_manager.hpp
---

# 焼きなましの近傍管理

近傍ごとの重みを管理して選択し、採用数・最良改善数を記録します。SAへ直接接続する版も提供します。

[カテゴリ索引](README.md) / [28項目の一覧](../b_21_25/README.md) / [操作できる図解](../../visualizations/b_21_25/index.html#neighborhood_manager) / [一次資料と設計判断](../b_21_25/references.md)

## API・使用条件

`NeighborhoodManager(weights)`、`set_weight/select/observe/adapt_weights`、`weights/statistics/total_weight/size`。`managed_simulated_annealing(initial,evaluate,propose,manager,Random&,options)`。propose(current,id,rng)→候補。

非負の有限重み、0は無効化。和はsegment treeで正の子同士から再計算し、大きい重みの削除による差し引きの桁落ちを避けます。抽選座標は各部分木の重み未満に丸め直し、0重みを選びません。和がoverflowする更新は変更前に例外を投げます。全無効ならselectはdomain_error、managed SAはexhausted。浮動小数点の比率は厳密な有理確率ではありません。改善は採用された最良更新を指し、improvedかつrejectedは拒否。adaptは無効近傍を維持し平滑化した改善率へ重みを変更。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../heuristic_search/neighborhood_manager.hpp"

int main() {
    cp::NeighborhoodManager manager({0, 1}); cp::Random random(1);
    std::cout << manager.select(random) << "\n";
}
```

出力:

```text
1
```

[コンパイルして動く使用例](../../examples/heuristic_search/neighborhood_manager_b.cpp)。

## 計算量

重み更新・選択O(log K)、observe O(1)、adapt O(K log K)、保存O(K)。SAの各反復は既存費用にO(log K)を加える。

## 使用する問題・検証

複数近傍の選択・無効化・オンラインの改善統計、AHCのSA。

線形累積和による独立な重み抽選、managed SAの全採否、統計・無効化・巨大重み削除の境界を比較。 `python3 verify/run_b_21_25.py` / `--sanitize`。単独・重複include、debug/release、使用例と文書usageを検証します。オンライン提出・AC、ヒューリスティックのスコア改善は未確認です。ジャッジの最大制約での時間内完了は、記載した計算量とは別に判断してください。
