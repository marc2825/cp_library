---
title: K-th Shortest Path
documentation_of: //graph_basic/k_shortest_paths.hpp
---

# K-th Shortest Path

頂点が重複しない経路を、コスト昇順で高々k個返す。

## API・使用条件

k_shortest_paths(Graph<T>,s,t,k) -> vector<WeightedPath<T>>。k>=0、配列[k-1]がk番目（1始まり）。

Tは非負整数、bool以外。有向/無向・多重辺可。同じ頂点列でも辺ID列が異なれば別の経路。同コストの順序は未保証。k=0は空、s=tは空経路1個だけ。到達不能/経路不足なら短い列。全ての距離/接頭辞の途中和は型の最大値未満。

C++17、namespace cp、0始まり。頂点・辺IDと結果の公開配列は読み取り用です。入力を変えたら再構築/再計算してください。前提条件は呼出し側が守り、添字・符号などはdebug assertで検査します。整数の全中間値は使用型に収まる必要があります。

## Usage

```cpp
#include <iostream>
#include "graph_basic/k_shortest_paths.hpp"
int main() {
    cp::Graph<long long> g(4,cp::GraphKind::Directed); g.add_edge(0,1,1);g.add_edge(1,3,2);g.add_edge(0,2,2);g.add_edge(2,3,2);
    for(auto p:cp::k_shortest_paths(g,0,3,3)) { std::cout<<p.cost<<" "; } std::cout<<"\n";
}
```

出力: `3 4`。

## 原理・計算量

Yen法。最後に採用した経路の各位置で分岐し、接頭辞の頂点を禁止、同じ接頭辞を持つ採用済み経路の次の辺を禁止してDijkstraする。接頭辞trieで禁止辺を共有し、全候補をcost/辺列順の集合に保存する。

保守的にO(KN((N+M)log(N+M+1)+N log(KN+1)))時間、候補経路の保存も含めO(N+M+KN²)領域。KやNが大きい場合は重い。

## 問題例・検証・参考資料

[一次資料・対象ごとの問題例](../graph_a/references.md)。`python3 verify/run_graph_a.py` / `--sanitize` で単独/重複include、複数翻訳単位、使用例、独立した全探索/素朴解と比較します。[検証範囲](../graph_a/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/graph_a/index.html#k_shortest_paths) / [図の範囲](../../visualizations/graph_a/README.md) / [動く例](../../examples/graph_basic/k_shortest_paths.cpp) / [カテゴリ索引](README.md)
