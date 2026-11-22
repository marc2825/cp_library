---
title: Project Selection Problem
documentation_of: //graph_flow/project_selection.hpp
---

# Project Selection Problem

選ぶ/選ばないの費用、含意、罰金、submodularな二変数費用を最小カットで最小化。

## API・使用条件

ProjectSelection<Cost=long long>(n)、add_cost(v,c0,c1)、add_profit(v,profit)、add_penalty(i,j,c)、imply(i,j)、force(v,bool)、add_pairwise(i,j,e00,e01,e10,e11)、solve()->optional<ProjectSelectionResult{cost,selected}>。

Costは符号付き整数、selected=trueが選ぶ側。penaltyはi=trueかつj=falseの非負罰金。imply/forceは必須条件、不成立nullopt。pairwiseはe00+e11<=e01+e10（違反invalid_argument）。i=jなら対角のe00/e11だけ使用。全費用と変換途中が型内、soft容量総和が型内、必須条件があるならsoft総和+1も型内。超過overflow_error。

C++17、namespace cp、0始まり。頂点番号と罰金が非負であることは assert による使用条件です。非 submodular な二変数費用は`invalid_argument`、費用・容量があふれるときは`overflow_error`です。solve は入力を変えないので、制約を足して解き直せます。

## Usage

```cpp
#include <iostream>
#include "graph_flow/project_selection.hpp"
int main() {
    cp::ProjectSelection<> p(2);p.add_profit(0,7);p.add_profit(1,-2);p.imply(0,1);
    auto a=p.solve();std::cout<<a->cost<<" ";for(bool b:a->selected)std::cout<<b;std::cout<<"\n";
}
```

出力: `-5 11`。

## 原理・計算量

単項費用の小さい方を定数へ移し、差をsource/ sinkの辺にする。true→falseの罰金はi→j。必須条件には全soft容量総和+1を使い、そこまで流れたら不成立。一般の二変数submodular費用も定数・単項・罰金へ分解する。

構築O(N)、制約追加O(1)、solveはDinicでO((N+2)²(N+M))、領域O(N+M)。

## 問題例・検証・参考資料

[一次資料・対象ごとの問題例](../graph_a/references.md)。`python3 verify/run_graph_a.py` / `--sanitize`。検証では、頂点1〜7の700例で、単項費用・submodular な二変数費用10個・含意3個・強制（矛盾を含む）を全 2^n 通りの割当と比較し、最小費用と、返す `selected` の費用・制約の充足を確かめます。非 submodular の`invalid_argument`、矛盾する強制の nullopt、`LLONG_MIN` 費用、n=0、20万頂点の含意の鎖も確認します。[検証範囲](../graph_a/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/graph_a/index.html#project_selection) / [図の範囲](../../visualizations/graph_a/README.md) / [動く例](../../examples/graph_flow/project_selection.cpp) / [カテゴリ索引](README.md)
