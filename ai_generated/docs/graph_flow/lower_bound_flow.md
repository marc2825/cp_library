---
title: 下限制約付きフロー
documentation_of: //graph_flow/lower_bound_flow.hpp
---

# 下限制約付きフロー

各辺に下限/上限を設け、実行可能な循環/符号付きs-t流と最大流を求める。

## API・使用条件

LowerBoundFlow<Cap=long long>(n)、add_edge(u,v,lower,upper)->ID、feasible_circulation()->optional<vector<Cap>>、feasible_flow(s,t)、maximum_flow(s,t)->optional<BoundedFlowResult{value,flow}>。

Capは符号付き整数。0<=lower<=upper、s!=t。流量は元辺ID順。s-tのvalueはsから出る正味量で負も可。解なしnullopt。各solveは独立で再solve/追記可。下限の頂点balance、正のdemandの総和、value計算の途中値が型内。最大流が型を超える場合はoverflow_error。

C++17、namespace cp、0始まり。頂点番号・0≤lower≤upper・s≠t は assert による使用条件です。balance・需要・value・最大流があふれるときは`overflow_error`です。

## Usage

```cpp
#include <iostream>
#include "graph_flow/lower_bound_flow.hpp"
int main() {
    cp::LowerBoundFlow<> f(3);f.add_edge(0,1,1,4);f.add_edge(1,2,1,3);f.add_edge(0,2,0,2);
    auto a=f.maximum_flow(0,2);std::cout<<a->value;for(auto x:a->flow)std::cout<<" "<<x;std::cout<<"\n";
}
```

出力: `5 3 3 2`。

## 原理・計算量

下限分を固定してbalanceを作り、超始点/超終点間の要求を全て満たせるか調べる。s-tでは両向きの人工辺で負のvalueも許す。実行可能解から上限-f/f-下限の残余網を作り、sからtへ追加する。Cap最大値を含む容量を扱い、巨大な合計を無条件にINFへ足さない。

Dinicを定数回使い、保守的にO((N+2)²(M+N))時間、O(N+M)領域。

## 問題例・検証・参考資料

[一次資料・対象ごとの問題例](../graph_a/references.md)。`python3 verify/run_graph_a.py` / `--sanitize`。検証では、頂点2〜5・辺6本以下・下限0〜2・幅2以下の700例で全ての流量割当を列挙し、循環／s-t の実行可能性と最大流の値を比較し、返す流量の上下限・保存則・value を確かめます。負の value、`LLONG_MAX` 容量と`overflow_error`、`LLONG_MAX` 下限の自己ループも確認します。[検証範囲](../graph_a/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/graph_a/index.html#lower_bound_flow) / [図の範囲](../../visualizations/graph_a/README.md) / [動く例](../../examples/graph_flow/lower_bound_flow.cpp) / [カテゴリ索引](README.md)
