---
title: Functional Graph分解の共通入口
documentation_of: //graph_connectivity/functional_graph.hpp
---

# Functional Graph分解の共通入口

3.8から3.7のFunctional Graphを共有する入口。

## API・使用条件

FunctionalGraphのAPIはgraph_basic/functional_graph.hppと同じ。

実体を複製しない。詳細は3.7の個別説明を参照。

C++17、namespace cp、0始まり。頂点・辺IDと結果の公開配列は読み取り用です。入力を変えたら再構築/再計算してください。前提条件は呼出し側が守り、添字・符号などはdebug assertで検査します。整数の全中間値は使用型に収まる必要があります。

## Usage

```cpp
#include <iostream>
#include "graph_connectivity/functional_graph.hpp"
int main() {
    cp::FunctionalGraph f({1,2,1});std::cout<<f.component_count()<<"\n";
}
```

出力: `1`。

## 原理・計算量

forward includeのみ。

O(64N)前処理、O(64)照会。

## 問題例・検証・参考資料

[一次資料・対象ごとの問題例](../graph_a/references.md)。`python3 verify/run_graph_a.py` / `--sanitize` で単独/重複include、複数翻訳単位、使用例、独立した全探索/素朴解と比較します。[検証範囲](../graph_a/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/graph_a/index.html#functional_graph) / [図の範囲](../../visualizations/graph_a/README.md) / [動く例](../../examples/graph_connectivity/functional_graph.cpp) / [カテゴリ索引](README.md)
